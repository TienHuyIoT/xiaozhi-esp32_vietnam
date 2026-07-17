#ifndef EDGE_TTS_CLIENT_H
#define EDGE_TTS_CLIENT_H

// Fixed version — addresses:
//   BUG-1  : moi sentence_start huy cau truoc  -> FIFO queue, Enqueue() khong
//   bao gio cat cau dang doc BUG-4  : race
//   is_speaking_/pending_text_/&connection_ok/destructor -> atomics + mutex +
//   join semaphore,
//            websocket_ CHI duoc worker task dong/mo (Abort() chi set co)
//   BUG-5  : khong co watchdog -> no-audio timeout + total timeout trong
//   SynthesizeOne BUG-7/8: parse binary sign-extension + fragment >8KB ->
//   uint8_t + heuristic "Path:audio" + carry byte BUG-9  : sample rate co dinh
//   16k -> SetOutputSampleRate() de khop codec, khoi resample BUG-11 : stack
//   6KB khong pin core -> 10KB, pin core 0 (cung WiFi/lwIP, giu core 1 cho
//   audio)

#include <atomic>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include "web_socket.h"

class EdgeTtsClient {
public:
  EdgeTtsClient();
  ~EdgeTtsClient();

  // Them mot cau vao hang doi. KHONG huy cau dang doc / dang cho (khac ban cu).
  void Enqueue(const std::string &text,
               const std::string &voice = "vi-VN-HoaiMyNeural");

  // Huy cau dang doc + xoa toan bo hang doi. An toan goi tu task khac:
  // chi set co atomic, worker tu dong socket trong <=50ms roi bao OnIdle.
  void Abort();

  // true neu dang doc hoac con cau trong hang doi.
  bool IsBusy();

  // Goi truoc khi Enqueue lan dau. Chi nhan 16000 / 24000 / 48000.
  // Nen set = codec->output_sample_rate() de khoi resample tung chunk (BUG-9).
  void SetOutputSampleRate(int sample_rate);

  void
  OnAudioData(std::function<void(const std::vector<int16_t> &, int)> callback) {
    on_audio_data_ = callback;
  }
  // Ban ra (tu worker task) MOI KHI hang doi can va cau cuoi da xong (ke ca sau
  // Abort/loi). Day la "hop dong chong ket trang thai": worker LUON ket thuc
  // mot dot bang OnIdle.
  void OnIdle(std::function<void()> callback) { on_idle_ = callback; }
  // Thong bao loi tung lan synthesize that bai (khong phai ket thuc dot).
  void OnError(std::function<void(const std::string &)> callback) {
    on_error_ = callback;
  }

private:
  struct Request {
    std::string text;
    std::string voice;
  };

  static constexpr EventBits_t WAKE_BIT = BIT0;
  static constexpr size_t kMaxQueuedSentences = 16;
  static constexpr int kMaxConsecutiveFailures = 3;

  // --- hang doi & trang thai (BUG-1, BUG-4) ---
  std::mutex mutex_; // bao ve queue_
  std::deque<Request> queue_;
  std::atomic<bool> running_{true};
  std::atomic<bool> abort_{false};
  std::atomic<bool> synthesizing_{false};
  std::atomic<int> consecutive_failures_{0};

  // --- trang thai 1 phien synthesize, ghi tu task nhan cua WebSocket (khong
  // capture stack) ---
  std::atomic<bool> turn_end_{false};
  std::atomic<TickType_t> last_audio_tick_{0};
  // carry byte khi PCM bi cat le giua 2 fragment (BUG-8); chi task WS cham vao
  // trong 1 phien
  bool carry_valid_ = false;
  uint8_t carry_byte_ = 0;

  int sample_rate_ = 16000;

  std::unique_ptr<WebSocket> websocket_; // CHI worker task duoc tao/dong/reset

  std::function<void(const std::vector<int16_t> &, int)> on_audio_data_;
  std::function<void()> on_idle_;
  std::function<void(const std::string &)> on_error_;

  TaskHandle_t task_handle_ = nullptr;
  EventGroupHandle_t event_group_ = nullptr;
  SemaphoreHandle_t exited_ =
      nullptr; // worker give truoc khi tu xoa -> destructor join duoc

  void TaskRoutine();
  bool
  SynthesizeOne(const Request &req); // true = doc xong binh thuong (turn.end)
  void HandleData(const char *data, size_t len, bool binary);

  std::string GenerateRequestId();
  std::string GetDateHeader();
  void SendSpeechConfig();
  void SendSsml(const std::string &request_id, const Request &req);
  static std::string EscapeXml(const std::string &in);
};

#endif // EDGE_TTS_CLIENT_H