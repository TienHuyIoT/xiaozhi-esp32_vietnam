#include "edge_tts_client.h"

#include <algorithm>
#include <cstdio>
#include <esp_log.h>
#include <esp_random.h>

#include "board.h"

#define TAG "EdgeTts"

EdgeTtsClient::EdgeTtsClient() {
  event_group_ = xEventGroupCreate();
  exited_ = xSemaphoreCreateBinary();
  // BUG-11: 10KB stack (handshake TLS co the chay trong task nay), pin core 0
  // cung WiFi/lwIP de giu core 1 sach cho pipeline audio.
  xTaskCreatePinnedToCore(
      [](void *arg) { static_cast<EdgeTtsClient *>(arg)->TaskRoutine(); },
      "edge_tts", 10 * 1024, this, 5, &task_handle_, 0);
}

EdgeTtsClient::~EdgeTtsClient() {
  running_ = false;
  Abort(); // xoa queue + set abort + WAKE de worker thoat du dang ngu hay dang
           // doc
  if (task_handle_ && exited_) {
    // BUG-4: join that su thay vi vTaskDelay(50) roi xoa event group duoi chan
    // task
    if (xSemaphoreTake(exited_, pdMS_TO_TICKS(3000)) != pdTRUE) {
      ESP_LOGE(TAG, "worker did not exit in time");
    }
  }
  if (event_group_)
    vEventGroupDelete(event_group_);
  if (exited_)
    vSemaphoreDelete(exited_);
}

void EdgeTtsClient::SetOutputSampleRate(int sample_rate) {
  if (sample_rate == 16000 || sample_rate == 24000 || sample_rate == 48000) {
    sample_rate_ = sample_rate;
  } else {
    ESP_LOGW(TAG, "unsupported sample rate %d, keep %d", sample_rate,
             sample_rate_);
  }
}

void EdgeTtsClient::Enqueue(const std::string &text, const std::string &voice) {
  if (text.empty())
    return;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (queue_.size() >= kMaxQueuedSentences) {
      // Chan bo nho: tu choi cau moi thay vi pop cau cu (giu lien mach noi
      // dung).
      ESP_LOGW(TAG, "queue full (%u), dropping new sentence",
               (unsigned)queue_.size());
      return;
    }
    queue_.push_back({text, voice});
  }
  xEventGroupSetBits(event_group_, WAKE_BIT);
}

void EdgeTtsClient::Abort() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.clear();
  }
  abort_ = true;
  // Danh thuc neu worker dang ngu de no ban OnIdle -> app khong bao gio ket
  // trang thai.
  xEventGroupSetBits(event_group_, WAKE_BIT);
  // Luu y: KHONG dong websocket_ o day (BUG-4). Worker poll abort_ moi 50ms va
  // tu dong. Gia dinh: sau Abort() phai co >=1 vong round-trip mang truoc khi
  // Enqueue cau cua turn moi, nen worker luon kip xu ly abort truoc khi cau moi
  // vao queue.
}

bool EdgeTtsClient::IsBusy() {
  std::lock_guard<std::mutex> lock(mutex_);
  return synthesizing_.load() || !queue_.empty();
}

void EdgeTtsClient::TaskRoutine() {
  while (true) {
    xEventGroupWaitBits(event_group_, WAKE_BIT, pdTRUE, pdFALSE, portMAX_DELAY);
    if (!running_)
      break;

    while (running_) {
      if (abort_) {
        std::lock_guard<std::mutex> lock(mutex_);
        queue_.clear();
        break;
      }
      Request req;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.empty())
          break;
        req = std::move(queue_.front());
        queue_.pop_front();
        // Set trong cung lock voi thao tac pop de IsBusy() khong bao gio
        // thay "queue rong + khong doc" trong khi thuc ra dang cam 1 cau
        // (BUG-4).
        synthesizing_ = true;
      }

      bool ok = SynthesizeOne(req);
      synthesizing_ = false;

      if (ok) {
        consecutive_failures_ = 0;
      } else if (!abort_) {
        int failures = ++consecutive_failures_;
        ESP_LOGE(TAG, "synthesize failed (%d consecutive)", failures);
        if (on_error_)
          on_error_("Edge TTS synthesize failed");
        if (failures >= kMaxConsecutiveFailures) {
          // Bo cuoc turn nay de khong giam chan state machine cua app (BUG-2).
          ESP_LOGE(TAG, "too many failures, dropping remaining queue");
          std::lock_guard<std::mutex> lock(mutex_);
          queue_.clear();
          consecutive_failures_ = 0;
        }
      }
    }

    abort_ = false;
    if (!running_)
      break;
    // Hop dong: moi dot xu ly LUON ket thuc bang OnIdle (ke ca abort/loi)
    // -> app doi chieu voi co "server da gui tts stop" de dong turn (BUG-2).
    if (on_idle_)
      on_idle_();
  }

  xSemaphoreGive(exited_);
  task_handle_ = nullptr;
  vTaskDelete(NULL);
}

bool EdgeTtsClient::SynthesizeOne(const Request &req) {
  auto network = Board::GetInstance().GetNetwork();
  websocket_ = network->CreateWebSocket(2); // ID 2: kenh Edge TTS
  if (!websocket_) {
    ESP_LOGE(TAG, "CreateWebSocket failed");
    return false;
  }

  turn_end_ = false;
  carry_valid_ = false;
  last_audio_tick_ = xTaskGetTickCount();

  websocket_->SetReceiveBufferSize(8192);
  websocket_->SetHeader("Origin",
                        "chrome-extension://jdiccldimpdaibmpdkjnbmckianbfold");
  websocket_->SetHeader("User-Agent",
                        "Mozilla/5.0 (Windows NT 10.0; Win64; x64) "
                        "AppleWebKit/537.36 (KHTML, like Gecko) "
                        "Chrome/114.0.0.0 Safari/537.36 Edg/114.0.1823.51");

  websocket_->OnConnected([]() { ESP_LOGI(TAG, "connected"); });
  websocket_->OnDisconnected([]() { ESP_LOGI(TAG, "disconnected"); });
  websocket_->OnError([](int err) { ESP_LOGE(TAG, "socket error %d", err); });
  // Chi capture `this` — khong capture bien stack theo tham chieu (BUG-4
  // &connection_ok cu).
  websocket_->OnData([this](const char *data, size_t len, bool binary) {
    HandleData(data, len, binary);
  });

  const std::string url =
      "wss://speech.platform.bing.com/consumer/speech/synthesize/readaloud/"
      "edge/v1"
      "?TrustedClientToken=6A5AA1D4EAFF4E9FB37E23D68491D6F4";

  if (!websocket_->Connect(url.c_str())) {
    ESP_LOGE(TAG, "connect failed");
    websocket_.reset();
    return false;
  }

  SendSpeechConfig();
  SendSsml(GenerateRequestId(), req);

  // BUG-5: watchdog kep.
  //  - no-audio: 7s khong nhan them du lieu -> coi nhu chet (half-open TCP,
  //  server treo).
  //  - total: tran cho ca cau, ty le theo do dai text de cau dai khong bi cat
  //  oan.
  const TickType_t kNoAudioTimeout = pdMS_TO_TICKS(7000);
  const TickType_t kTotalTimeout =
      pdMS_TO_TICKS(std::max<size_t>(15000, req.text.size() * 500));
  const TickType_t start = xTaskGetTickCount();

  bool ok = false;
  while (running_ && !abort_) {
    if (turn_end_) {
      ok = true;
      break;
    }
    if (!websocket_->IsConnected()) {
      ESP_LOGW(TAG, "connection dropped mid-sentence");
      break;
    }
    const TickType_t now = xTaskGetTickCount();
    if ((now - last_audio_tick_.load()) > kNoAudioTimeout) {
      ESP_LOGE(TAG, "no-audio watchdog fired");
      break;
    }
    if ((now - start) > kTotalTimeout) {
      ESP_LOGE(TAG, "total watchdog fired");
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }

  websocket_->Close();
  websocket_.reset();
  return ok;
}

void EdgeTtsClient::HandleData(const char *data, size_t len, bool binary) {
  // Chay tren task nhan cua WebSocket. Sau khi abort thi bo het.
  if (abort_.load() || !running_.load())
    return;

  if (!binary) {
    std::string msg(data, len);
    if (msg.find("Path:turn.end") != std::string::npos) {
      carry_valid_ = false;
      turn_end_ = true;
    }
    return;
  }
  if (len == 0)
    return;
  last_audio_tick_ = xTaskGetTickCount();

  const uint8_t *p = reinterpret_cast<const uint8_t *>(data);
  size_t off = 0;

  // Mot message binary moi cua Edge = [2 byte BE header-len][header ASCII co
  // "Path:audio"][PCM LE]. Fragment noi tiep cua message >8KB KHONG co header
  // -> nhan dien va coi toan bo la PCM (BUG-8).
  if (len > 2) {
    // BUG-7: ep ve uint8_t truoc khi shift, tranh sign-extension cua char.
    const uint16_t header_len = static_cast<uint16_t>((p[0] << 8) | p[1]);
    if (static_cast<size_t>(header_len) + 2 <= len) {
      std::string headers(reinterpret_cast<const char *>(p) + 2,
                          std::min<size_t>(header_len, 160));
      if (headers.find("Path:audio") != std::string::npos) {
        off = 2 + header_len;
        carry_valid_ = false; // message moi: carry cu (neu co) vo nghia
      }
    }
  }

  const uint8_t *payload = p + off;
  const size_t n = len - off;
  if (n == 0)
    return;

  std::vector<int16_t> pcm;
  pcm.reserve(n / 2 + 1);
  size_t i = 0;
  if (carry_valid_) { // ghep not sample bi cat doi giua 2 fragment (PCM
                      // little-endian)
    pcm.push_back(static_cast<int16_t>(carry_byte_ | (payload[0] << 8)));
    i = 1;
    carry_valid_ = false;
  }
  for (; i + 1 < n; i += 2) {
    pcm.push_back(static_cast<int16_t>(payload[i] | (payload[i + 1] << 8)));
  }
  if (i < n) {
    carry_byte_ = payload[i];
    carry_valid_ = true;
  }

  if (!pcm.empty() && on_audio_data_) {
    on_audio_data_(pcm, sample_rate_);
  }
}

std::string EdgeTtsClient::GenerateRequestId() {
  static const char *kHex = "0123456789abcdef";
  std::string id(32, '0');
  for (int i = 0; i < 32; ++i)
    id[i] = kHex[esp_random() & 0xF];
  return id;
}

std::string EdgeTtsClient::GetDateHeader() {
  // Edge khong kiem tra gia tri nay; neu sau nay them Sec-MS-GEC thi can gio
  // that (SNTP).
  return "Thu Oct 24 2024 13:42:04 GMT+0700 (Indochina Time)";
}

void EdgeTtsClient::SendSpeechConfig() {
  char fmt[40];
  snprintf(fmt, sizeof(fmt), "raw-%dkhz-16bit-mono-pcm", sample_rate_ / 1000);
  std::string config =
      "X-Timestamp:" + GetDateHeader() +
      "\r\n"
      "Content-Type:application/json; charset=utf-8\r\n"
      "Path:speech.config\r\n\r\n"
      "{\"context\":{\"synthesis\":{\"audio\":{\"metadataOptions\":{"
      "\"sentenceBoundaryEnabled\":false,\"wordBoundaryEnabled\":false},"
      "\"outputFormat\":\"" +
      std::string(fmt) + "\"}}}}";
  websocket_->Send(config);
}

void EdgeTtsClient::SendSsml(const std::string &request_id,
                             const Request &req) {
  const std::string lang =
      req.voice.size() >= 5 ? req.voice.substr(0, 5) : "vi-VN";
  const std::string ssml =
      "<ssml version='1.0' xmlns='http://www.w3.org/2001/10/synthesis' "
      "xmlns:mstts='https://www.w3.org/2001/mstts' xml:lang='" +
      lang +
      "'>"
      "<voice name='" +
      req.voice +
      "'>"
      "<prosody rate='+0%'>" +
      EscapeXml(req.text) +
      "</prosody>"
      "</voice></ssml>";

  const std::string msg = "X-RequestId:" + request_id +
                          "\r\n"
                          "Content-Type:application/ssml+xml\r\n"
                          "X-Timestamp:" +
                          GetDateHeader() +
                          "\r\n"
                          "Path:ssml\r\n\r\n" +
                          ssml;
  websocket_->Send(msg);
}

std::string EdgeTtsClient::EscapeXml(const std::string &in) {
  std::string out;
  out.reserve(in.size() + 16);
  for (char c : in) {
    switch (c) {
    case '&':
      out += "&amp;";
      break;
    case '<':
      out += "&lt;";
      break;
    case '>':
      out += "&gt;";
      break;
    default:
      out += c;
      break;
    }
  }
  return out;
}