#ifndef DEVICE_TTS_CLIENT_H
#define DEVICE_TTS_CLIENT_H

/**
 * @file device_tts_client.h
 * @brief TTS tren thiet bi: robot tu mo WebSocket toi nha cung cap va phat tieng.
 *
 * Thay cho edge_tts_client cu (go o commit 93d6a52). Khac biet ve nguyen tac:
 *
 *  1. LOP NAY KHONG BIET EDGE LA GI. URL, header, khung `speech.config` va ca
 *     noi dung SSML deu do SERVER render san va gui xuong qua message
 *     `tts_config` / truong `tts_body`. Doi nha cung cap = doi JSON o backend,
 *     KHONG phai OTA lai robot ngoai hien truong.
 *
 *  2. MOI CAU MOT SOCKET, NHUNG MO TRUOC SONG SONG. Edge tren robot da do duoc
 *     co luc ngung cap audio neu tai su dung cung socket. Vi vay ta van xoay
 *     socket moi cau, nhung bat tay cau N+1 ngay khi dang tong hop/phat cau N.
 *
 *  3. KE THUA AudioStreamPlayer (dung cho radio internet) thay vi tu viet duong
 *     giai ma. Edge da BO moi dinh dang tho -- xin raw PCM bi dong socket 1007 --
 *     nen robot buoc phai giai ma MP3. AudioStreamPlayer da co san: buffer PSRAM,
 *     esp_audio_codec, task ghim core, PCM ra thang loa.
 *
 * Luong: Configure(tts_config) -> Enqueue(tts_body) moi cau -> Abort() khi be chen.
 */

#include <atomic>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <cJSON.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include "features/music/audio_stream_player.h"
#include "web_socket.h"

class AudioCodec;

class DeviceTtsClient : public AudioStreamPlayer {
public:
  DeviceTtsClient();
  ~DeviceTtsClient() override;

  /**
   * Nap cau hinh phien tu message `tts_config` cua server.
   * Goi duoc nhieu lan: server lam tuoi URL truoc moi luot vi token chong bot
   * cua Edge xoay theo khoi 5 phut. Cau hinh moi chi ap dung cho lan KET NOI
   * sau; socket dang mo khong bi cat.
   * @return false neu thieu truong bat buoc (url / config_frame).
   */
  bool Configure(const cJSON *cfg);
  bool IsConfigured() const { return configured_.load(); }

  /** Codec de phat PCM. Goi mot lan luc khoi tao. */
  void SetCodec(AudioCodec *codec) { SetAudioCodec(codec); }

  /**
   * Them mot cau vao hang doi. `tts_body` la khung text `ssml` server render san
   * (ke ca X-RequestId va X-Timestamp) -- lop nay gui NGUYEN VAN, khong sua gi.
   * Tu bat player neu no chua chay.
   */
  void Enqueue(const std::string &tts_body);
  void Enqueue(const std::string &tts_body, const AudioTraceContext &trace);

  /**
   * Huy cau dang doc + xoa hang doi + tat tieng ngay (be chen ngang / bam nut).
   * An toan goi tu task khac.
   */
  void Abort();

  /** true neu con cau cho, dang tong hop, hoac con audio chua phat het. */
  bool IsBusy();

  /**
   * Ban ra tu task nguon MOI KHI hang doi can va tieng da phat het.
   *
   * Day la "hop dong chong ket trang thai": server gui `tts:stop` NGAY sau cau
   * cuoi (no khong con nhin thay tieng nua), nen Application phai tu hoan viec
   * chuyen Speaking->Listening cho toi khi nhan tin hieu nay. Thieu no thi cau
   * cuoi bi cat cut va be phai hoi lai.
   */
  void OnIdle(std::function<void()> callback) { on_idle_ = std::move(callback); }

  /**
   * T1 — mot cau tong hop hong, be se khong nghe gi neu khong ai doc bu.
   *
   * Do 09/08 tren robot COM5: 34 cau gui di, 27 cau ra tieng. Truoc day firmware
   * chi ESP_LOGW roi di tiep nen server khong he biet.
   *
   * ⚠️ Callback no tren TASK NGUON (hoac task cua Application voi
   * `stream_start_failed`), KHONG phai main task -- nguoi nhan phai tu hop ve
   * main task truoc khi dung WebSocket.
   *
   * @param reason mot trong 4 chuoi tinh: "connect_failed", "no_audio_timeout",
   *        "stream_start_failed", "synthesis_failed". Phai khop allowlist cua
   *        backend (`_TTS_SEGMENT_FAILURE_REASONS`), lech chinh ta thi server vut
   *        im lang.
   */
  void OnSegmentFailed(
      std::function<void(const std::string &turn_id,
                         const std::string &segment_id, const char *reason)>
          callback) {
    on_segment_failed_ = std::move(callback);
  }

  /** Dung han, dong socket. Goi khi ket thuc phien / tat may. */
  void Shutdown();

protected:
  /** Ghi de: nguon du lieu la WebSocket chu khong phai HTTP. */
  void SourceDataLoop(const std::string &source) override;

private:
  /**
   * Nguong bat dau phat. Mot cau ~17KB MP3 48kbps, con mac dinh cua lop cha la
   * 32KB -> cau dau se KHONG BAO GIO phat neu khong ha xuong. Chon ~4KB vi do
   * la moc "du du lieu de bat dau phat" do duoc 06/08 (158-237ms).
   */
  static constexpr size_t kTtsMinBufferBytes = 4 * 1024;

  /** Tran hang doi -- LLM khong bao gio nha nhieu the nay trong mot luot. */
  static constexpr size_t kMaxQueuedSentences = 16;

  // Bat tay Edge tren ESP32 do duoc 1,39-1,51s. Worker warm-up co stack rieng
  // vi TLS can nhieu stack; delay ngan de cau hien tai co quyen gui SSML truoc.
  static constexpr int kPreconnectDelayMs = 150;
  static constexpr int kPreconnectWaitMs = 12000;
  static constexpr int kPreconnectTaskStack = 10 * 1024;
  static constexpr EventBits_t kPreconnectWakeBit = BIT0;

  /** Khong nhan them byte nao trong ngan nay -> coi nhu chet (half-open TCP). */
  static constexpr int kNoAudioTimeoutMs = 7000;

  /** Toi da cho mot cau, ty le do dai text de cau dai khong bi cat oan. */
  static constexpr int kTotalTimeoutMinMs = 15000;

  /**
   * Cho them chung nay truoc khi bao "het viec". GetBufferSize()==0 chi noi
   * buffer nen da can, CHUA noi loa da keu xong: PCM cuoi con nam trong DMA cua
   * I2S. Bao som thi cau cuoi cut vai chu.
   */
  static constexpr int kDrainGraceMs = 300;

  /**
   * Provider da gui turn.end ma decoder van doi them byte cua frame cuoi.
   * Sau mot khoang pending khong doi, bo tail loi tren task playback de robot
   * khong ket Speaking vo han. Cau sau chua duoc synth trong luc cho nay.
   */
  static constexpr int kDecoderTailStallMs = 500;

  /**
   * Tuoi toi da cua socket warm chua dung. Do 09/08 cho thay socket warm song
   * qua ranh gioi turn tiet kiem ~1,6s cho cau dau moi turn, nhung giu qua lau
   * thi gap TCP nua song: `IsConnected()` van true, gui SSML vao do khong bao
   * gio co byte tra ve va phai an tron `kNoAudioTimeoutMs`. 120s du cho khoang
   * nghi giua hai luot va van nam trong khoi token 5 phut cua nha cung cap.
   */
  static constexpr int64_t kReadySocketMaxAgeMs = 120 * 1000;

  bool EnsureStarted();
  /**
   * @param segment_generation `turn_generation_` luc segment duoc xep hang.
   *        Moi vong cho/bat tay phai bo ngay khi generation hien tai da khac:
   *        `abort_` KHONG dung duoc lam tin hieu huy vi `Enqueue()` xoa no.
   */
  bool EnsureConnected(uint32_t segment_generation);
  bool HasQueuedSentence();
  void CloseSocket();
  void CloseReadySocket();
  bool SynthesizeOne(const std::string &body, uint32_t synthesis_generation);
  void HandleData(const char *data, size_t len, bool binary);
  void RequestPreconnect();
  void PreconnectTaskRoutine();

  struct ConnectionSnapshot {
    std::string url;
    std::string config_frame;
    std::vector<std::pair<std::string, std::string>> headers;
    uint32_t config_generation = 0;
    uint32_t socket_generation = 0;
  };

  bool CaptureConnectionSnapshot(ConnectionSnapshot &snapshot);
  std::unique_ptr<WebSocket>
  OpenConfiguredSocket(const ConnectionSnapshot &snapshot, bool preconnect);
  bool PromoteReadySocket();

  struct QueuedTtsSegment {
    std::string body;
    AudioTraceContext trace;
    /** `turn_generation_` luc xep hang -- cau cua turn da abort khong duoc doc. */
    uint32_t turn_generation = 0;
  };

  AudioTraceContext GetActiveTrace();
  void SetActiveTrace(const AudioTraceContext &trace);
  void ClearActiveTrace();

  /* --- cau hinh tu server (bao ve boi config_mutex_) --- */
  mutable std::mutex config_mutex_;
  std::string url_;
  std::string config_frame_;
  std::vector<std::pair<std::string, std::string>> headers_;
  std::atomic<bool> configured_{false};
  std::atomic<uint32_t> config_generation_{0};

  /* --- hang doi cau --- */
  std::mutex queue_mutex_;
  std::deque<QueuedTtsSegment> queue_;
  std::atomic<uint32_t> next_trace_sequence_{1};

  /* --- correlation cua segment dang tong hop/nhan byte --- */
  std::mutex active_trace_mutex_;
  AudioTraceContext active_trace_;
  std::atomic<uint32_t> active_stream_generation_{0};
  std::atomic<bool> first_provider_byte_seen_{false};

  /* --- trang thai --- */
  std::atomic<bool> running_{true};
  std::atomic<bool> abort_{false};
  std::atomic<bool> synthesizing_{false};
  // Tang moi lan Abort. Day la tin hieu HUY duy nhat dang tin cay cho segment
  // dang cho/dang tong hop (`abort_` bi `Enqueue()` xoa ngay sau do). KHONG
  // dung no de danh gia socket warm -- xem chu thich o `ready_websocket_`.
  std::atomic<uint32_t> turn_generation_{1};
  std::atomic<uint32_t> next_socket_generation_{1};
  std::atomic<uint32_t> active_socket_generation_{0};
  /** Ghi tu task nhan cua WebSocket, doc tu task nguon. */
  std::atomic<bool> turn_end_{false};
  std::atomic<TickType_t> last_data_tick_{0};

  /**
   * Socket CHI duoc task nguon tao/dong. Abort() tu task khac chi set co;
   * neu khong se dua nhau giai phong con tro (BUG-4 cua ban cu).
   */
  std::unique_ptr<WebSocket> websocket_;

  // Worker chi ghi slot ready; source task chi lay/move slot nay. Socket active
  // van chi do source task dong/mo, giu nguyen bat bien chong use-after-free.
  //
  // Socket warm SONG QUA RANH GIOI TURN co chu dich: no chua gui SSML va chua
  // gan OnData nen khong mang danh tinh turn nao. Chi `config_generation_`
  // (server doi URL/token) va tuoi socket moi duoc phep vo hieu hoa no. Ban cu
  // vut socket theo `turn_generation_` -> cau dau moi turn tra lai ~1,6s TLS.
  std::mutex ready_socket_mutex_;
  std::unique_ptr<WebSocket> ready_websocket_;
  uint32_t ready_config_generation_ = 0;
  uint32_t ready_socket_generation_ = 0;
  int64_t ready_socket_opened_us_ = 0;
  std::atomic<bool> preconnect_requested_{false};
  std::atomic<bool> preconnect_inflight_{false};
  EventGroupHandle_t preconnect_events_ = nullptr;
  SemaphoreHandle_t preconnect_exited_ = nullptr;
  TaskHandle_t preconnect_task_handle_ = nullptr;
  std::atomic<bool> preconnect_task_exited_{true};

  std::function<void()> on_idle_;
  std::function<void(const std::string &turn_id, const std::string &segment_id,
                     const char *reason)>
      on_segment_failed_;

  /**
   * T1 — nguyen nhan hong cua cau DANG tong hop, hoac nullptr = chua hong.
   *
   * Chi chua con tro toi chuoi hang (immortal) nen doc/ghi cheo task an toan ma
   * khong can khoa: `stream_start_failed` duoc ghi tu task nhan cua WebSocket,
   * cac ly do con lai tu task nguon.
   *
   * nullptr sau khi `SynthesizeOne` tra false = cau bi HUY CO Y (abort / doi
   * generation / tat may), KHONG phai hong -> khong duoc bao len server.
   */
  std::atomic<const char *> failure_reason_{nullptr};
  /** Nguyen nhan dau tien thang: watchdog no sau khong duoc de len that bai goc. */
  void SetFailureReasonIfUnset(const char *reason) {
    const char *expected = nullptr;
    failure_reason_.compare_exchange_strong(expected, reason);
  }
};

#endif // DEVICE_TTS_CLIENT_H
