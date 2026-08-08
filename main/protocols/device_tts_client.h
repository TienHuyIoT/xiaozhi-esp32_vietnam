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
 *  2. GIU MOT SOCKET MO SUOT PHIEN. Ban cu mo socket MOI cho tung cau -> tra
 *     gia bat tay 316-421ms moi cau (do that 06/08). Giu socket thi chi cau dau
 *     tra gia do; cac cau sau ~200ms la co tieng.
 *
 *  3. KE THUA AudioStreamPlayer (dung cho radio internet) thay vi tu viet duong
 *     giai ma. Edge da BO moi dinh dang tho -- xin raw PCM bi dong socket 1007 --
 *     nen robot buoc phai giai ma MP3. AudioStreamPlayer da co san: buffer PSRAM,
 *     esp_audio_codec, task ghim core, PCM ra thang loa.
 *
 * Luong: Configure(tts_config) -> Enqueue(tts_body) moi cau -> Abort() khi be chen.
 */

#include <atomic>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <cJSON.h>

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

  /*
   * So cau toi da tren MOT socket, roi chu dong dong di.
   *
   * Nguoc voi truc giac, va nguoc voi chinh ghi chep cu cua file nay: GIU socket
   * lau KHONG nhanh hon ma CHAM di ro ret. Do 08/08 tu mang dan dung VN, cung
   * mot chuoi 6 cau, tinh tu luc gui SSML den byte tieng dau:
   *
   *   giu mot socket suot     : 1690 1561 1819 2111 2143 2570 ms  (te dan)
   *   dong+mo lai moi 2 cau   :  201  124  192  185  160  207 ms
   *
   * Muoi lan chenh. Bat tay ton ~400ms nhung roi vao luc cau truoc CON DANG PHAT
   * (buffer PSRAM chua vai giay tieng) nen be khong nghe thay do tre do.
   *
   * Neu ai do sau nay thay "dong socket moi 2 cau" la phi pham va bo di: hay do
   * lai truoc, dung suy luan. Con so o tren la do that, khong phai uoc luong.
   */
  static constexpr int kSentencesPerConnection = 2;

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

  bool EnsureStarted();
  bool EnsureConnected();
  void CloseSocket();
  bool SynthesizeOne(const std::string &body);
  void HandleData(const char *data, size_t len, bool binary);

  /* --- cau hinh tu server (bao ve boi config_mutex_) --- */
  mutable std::mutex config_mutex_;
  std::string url_;
  std::string config_frame_;
  std::vector<std::pair<std::string, std::string>> headers_;
  std::atomic<bool> configured_{false};

  /* --- hang doi cau --- */
  std::mutex queue_mutex_;
  std::deque<std::string> queue_;

  /* --- trang thai --- */
  std::atomic<bool> running_{true};
  std::atomic<bool> abort_{false};
  std::atomic<bool> synthesizing_{false};
  /** So cau da tong hop tren socket hien tai; ve 0 moi lan bat tay lai. */
  int cau_tren_socket_ = 0;
  /** Ghi tu task nhan cua WebSocket, doc tu task nguon. */
  std::atomic<bool> turn_end_{false};
  std::atomic<TickType_t> last_data_tick_{0};

  /**
   * Socket CHI duoc task nguon tao/dong. Abort() tu task khac chi set co;
   * neu khong se dua nhau giai phong con tro (BUG-4 cua ban cu).
   */
  std::unique_ptr<WebSocket> websocket_;

  std::function<void()> on_idle_;
};

#endif // DEVICE_TTS_CLIENT_H
