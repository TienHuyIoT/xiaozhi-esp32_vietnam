#include "device_tts_client.h"

#include <algorithm>

#include <esp_log.h>
#include <esp_timer.h>

#include "board.h"

#define TAG "DeviceTTS"

DeviceTtsClient::DeviceTtsClient() {
  // Mot cau ~17KB MP3; nguong 32KB cua lop cha se treo cau dau mai mai.
  SetMinBufferSize(kTtsMinBufferBytes);
}

DeviceTtsClient::~DeviceTtsClient() { Shutdown(); }

/* ------------------------------------------------------------------ */
/*  Cau hinh tu server                                                */
/* ------------------------------------------------------------------ */

bool DeviceTtsClient::Configure(const cJSON *cfg) {
  if (cfg == nullptr) {
    return false;
  }

  const cJSON *url = cJSON_GetObjectItem(cfg, "url");
  if (!cJSON_IsString(url) || url->valuestring == nullptr ||
      url->valuestring[0] == '\0') {
    ESP_LOGE(TAG, "tts_config thieu 'url' -> bo qua, dung TTS cua server");
    return false;
  }

  // Chi nhan dinh dang ma duong giai ma nay thuc su phat duoc. Im lang chap nhan
  // codec la roi phat ra tieng nhieu thi kho lan ra nguyen nhan hon nhieu.
  const cJSON *codec = cJSON_GetObjectItem(cfg, "codec");
  const char *codec_name =
      (cJSON_IsString(codec) && codec->valuestring) ? codec->valuestring : "mp3";
  if (std::string(codec_name) != "mp3") {
    ESP_LOGE(TAG, "codec '%s' chua duoc ho tro (chi mp3) -> bo qua cau hinh",
             codec_name);
    return false;
  }

  const cJSON *frame = cJSON_GetObjectItem(cfg, "config_frame");
  const cJSON *hs = cJSON_GetObjectItem(cfg, "headers");

  {
    std::lock_guard<std::mutex> lock(config_mutex_);
    url_ = url->valuestring;
    config_frame_ = (cJSON_IsString(frame) && frame->valuestring)
                        ? frame->valuestring
                        : std::string();
    headers_.clear();
    if (cJSON_IsObject(hs)) {
      const cJSON *h = nullptr;
      cJSON_ArrayForEach(h, hs) {
        if (h->string != nullptr && cJSON_IsString(h) &&
            h->valuestring != nullptr) {
          headers_.emplace_back(h->string, h->valuestring);
        }
      }
    }
  }
  configured_ = true;

  // KHONG log url_: no chua token chong bot cua nha cung cap.
  ESP_LOGI(TAG, "nap cau hinh TTS thiet bi (%d header, codec mp3)",
           static_cast<int>(headers_.size()));
  return true;
}

/* ------------------------------------------------------------------ */
/*  Hang doi cau                                                      */
/* ------------------------------------------------------------------ */

void DeviceTtsClient::Enqueue(const std::string &tts_body) {
  if (tts_body.empty()) {
    return;
  }
  if (!configured_.load()) {
    ESP_LOGW(TAG, "chua co tts_config ma da bao doc -> bo qua cau nay");
    return;
  }

  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (queue_.size() >= kMaxQueuedSentences) {
      ESP_LOGW(TAG, "hang doi day (%d cau), bo cau moi",
               static_cast<int>(queue_.size()));
      return;
    }
    queue_.push_back(tts_body);
  }

  // Enqueue va Abort deu duoc goi tu task cua Application (mot luong) nen khong
  // co dua nhau o day. Neu sau nay co task khac goi thi phai xem lai cho nay.
  abort_ = false;
  if (!EnsureStarted()) {
    ESP_LOGE(TAG, "khong bat duoc duong phat -> be se khong nghe gi");
  }
}

void DeviceTtsClient::Abort() {
  abort_ = true;
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    queue_.clear();
  }
  // Tat tieng NGAY: StopStream() xa buffer + dung task giai ma. Cau sau tu bat
  // lai qua EnsureStarted(). Doi lai la lan do tra gia bat tay them mot lan --
  // chap nhan duoc vi be chen ngang la viec hiem, con tieng cu keo dai sau khi
  // be da noi thi rat kho chiu.
  StopStream();
}

bool DeviceTtsClient::IsBusy() {
  if (synthesizing_.load()) {
    return true;
  }
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (!queue_.empty()) {
      return true;
    }
  }
  // Con byte trong buffer nghia la con tieng chua phat het.
  return GetBufferSize() > 0;
}

void DeviceTtsClient::Shutdown() {
  running_ = false;
  Abort();
}

bool DeviceTtsClient::EnsureStarted() {
  if (IsPlaying()) {
    return true;
  }
  if (!configured_.load()) {
    return false;
  }
  running_ = true;
  abort_ = false;
  SetMinBufferSize(kTtsMinBufferBytes);
  // Nguon la WebSocket nen tham so nay khong duoc dung -- xem SourceDataLoop().
  return StartStream("device-tts", AudioDecoderType::MP3);
}

/* ------------------------------------------------------------------ */
/*  Vong lay du lieu (chay tren task nguon cua lop cha, core 0)        */
/* ------------------------------------------------------------------ */

void DeviceTtsClient::SourceDataLoop(const std::string & /*source*/) {
  ESP_LOGI(TAG, "vong nguon bat dau");

  while (IsSourceActive() && running_.load()) {
    std::string body;
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      if (!queue_.empty()) {
        body = std::move(queue_.front());
        queue_.pop_front();
      }
    }
    if (body.empty()) {
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }
    if (abort_.load()) {
      continue; // Abort() da xoa hang doi; vong sau se thay het viec
    }

    synthesizing_ = true;
    const bool ok = SynthesizeOne(body);
    synthesizing_ = false;

    if (!ok) {
      // Socket dang nghi ngo (dut, watchdog no, hoac bi tu choi): dong han de
      // cau sau bat tay lai bang URL moi nhat, thay vi keo dai ket noi hong.
      CloseSocket();
    } else if (++cau_tren_socket_ >= kSentencesPerConnection) {
      // Xoay socket -- KHONG phai don dep cho gon ma la don bay do tre: giu mot
      // socket qua 2 cau thi tieng dau tut tu ~190ms xuong ~2000ms. Xem con so
      // do duoc o kSentencesPerConnection (device_tts_client.h).
      //
      // Dong o DAY, ngay sau khi cau vua roi gui xong, la co y: tieng cua no van
      // dang nam trong buffer PSRAM va con phat vai giay nua, nen ~400ms bat tay
      // cua cau ke tiep nap hoan toan vao khoang do -- be khong nghe thay.
      ESP_LOGD(TAG, "da %d cau tren socket nay -> xoay socket", cau_tren_socket_);
      CloseSocket();
    }

    bool con_cau_khac;
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      con_cau_khac = !queue_.empty();
    }
    if (!con_cau_khac) {
      // Cho tieng phat het roi moi bao "het viec" -- Application dua vao day de
      // biet luc nao duoc chuyen Speaking->Listening (xem OnIdle o header).
      while (IsSourceActive() && running_.load() && !abort_.load() &&
             GetBufferSize() > 0) {
        vTaskDelay(pdMS_TO_TICKS(20));
      }
      vTaskDelay(pdMS_TO_TICKS(kDrainGraceMs));
      if (on_idle_ && !abort_.load()) {
        on_idle_();
      }
    }
  }

  CloseSocket();
  ESP_LOGI(TAG, "vong nguon ket thuc");
}

bool DeviceTtsClient::EnsureConnected() {
  if (websocket_ && websocket_->IsConnected()) {
    // Giu socket giua cac cau -- nhung chi trong pham vi kSentencesPerConnection.
    // Ghi chep cu o day tung noi giu cang lau cang tot; DO 08/08 BAC BO dieu do.
    return true;
  }
  CloseSocket();
  cau_tren_socket_ = 0;

  std::string url;
  std::string config_frame;
  std::vector<std::pair<std::string, std::string>> headers;
  {
    std::lock_guard<std::mutex> lock(config_mutex_);
    url = url_;
    config_frame = config_frame_;
    headers = headers_;
  }
  if (url.empty()) {
    return false;
  }

  auto network = Board::GetInstance().GetNetwork();
  websocket_ = network->CreateWebSocket(2); // ID 2: kenh TTS thiet bi
  if (!websocket_) {
    ESP_LOGE(TAG, "CreateWebSocket that bai");
    return false;
  }

  websocket_->SetReceiveBufferSize(8192);
  for (const auto &h : headers) {
    websocket_->SetHeader(h.first.c_str(), h.second.c_str());
  }
  // Chi capture `this` -- khong capture bien stack theo tham chieu.
  websocket_->OnData([this](const char *data, size_t len, bool binary) {
    HandleData(data, len, binary);
  });
  websocket_->OnError([](int err) { ESP_LOGE(TAG, "loi socket %d", err); });
  websocket_->OnDisconnected([]() { ESP_LOGI(TAG, "socket dong"); });

  const int64_t t0 = esp_timer_get_time();
  if (!websocket_->Connect(url.c_str())) {
    // Nguyen nhan hay gap nhat, theo thu tu: URL het han (token xoay 5 phut),
    // server chua kip gui tts_config moi, hoac mat mang.
    ESP_LOGE(TAG, "bat tay that bai -- URL het han hay mat mang?");
    websocket_.reset();
    return false;
  }
  ESP_LOGI(TAG, "bat tay xong sau %lldms", (esp_timer_get_time() - t0) / 1000);

  // Khung cau hinh gui MOT lan cho moi ket noi.
  if (!config_frame.empty() && !websocket_->Send(config_frame)) {
    ESP_LOGE(TAG, "gui config_frame that bai");
    CloseSocket();
    return false;
  }
  return true;
}

void DeviceTtsClient::CloseSocket() {
  if (websocket_) {
    websocket_->Close();
    websocket_.reset();
  }
}

bool DeviceTtsClient::SynthesizeOne(const std::string &body) {
  if (!EnsureConnected()) {
    return false;
  }

  turn_end_ = false;
  last_data_tick_ = xTaskGetTickCount();

  // Gui NGUYEN VAN khung text server da render (co san X-RequestId, X-Timestamp,
  // Path:ssml). Robot khong duoc tu sinh timestamp: dong ho no lech -7 gio vi
  // ota.cc cong timezone_offset vao epoch truoc khi settimeofday().
  if (!websocket_->Send(body)) {
    ESP_LOGE(TAG, "gui cau that bai");
    return false;
  }

  const TickType_t no_data_timeout = pdMS_TO_TICKS(kNoAudioTimeoutMs);
  const TickType_t total_timeout =
      pdMS_TO_TICKS(std::max<size_t>(kTotalTimeoutMinMs, body.size() * 60));
  const TickType_t started = xTaskGetTickCount();

  while (running_.load() && !abort_.load()) {
    if (turn_end_.load()) {
      return true;
    }
    if (!websocket_->IsConnected()) {
      ESP_LOGW(TAG, "socket dut giua cau");
      return false;
    }
    const TickType_t now = xTaskGetTickCount();
    if ((now - last_data_tick_.load()) > no_data_timeout) {
      ESP_LOGE(TAG, "watchdog: %dms khong nhan them du lieu", kNoAudioTimeoutMs);
      return false;
    }
    if ((now - started) > total_timeout) {
      ESP_LOGE(TAG, "watchdog: qua han ca cau");
      return false;
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
  return false;
}

/* ------------------------------------------------------------------ */
/*  Nhan du lieu (chay tren task nhan cua WebSocket)                  */
/* ------------------------------------------------------------------ */

void DeviceTtsClient::HandleData(const char *data, size_t len, bool binary) {
  if (abort_.load() || !running_.load()) {
    return; // sau khi be chen ngang thi bo het, khong bom vao buffer nua
  }

  if (!binary) {
    if (std::string(data, len).find("Path:turn.end") != std::string::npos) {
      turn_end_ = true;
    }
    return;
  }
  if (len == 0) {
    return;
  }
  last_data_tick_ = xTaskGetTickCount();

  // Khung nhi phan: [2 byte BE do dai header][header ASCII co "Path:audio"][audio].
  // Manh noi tiep cua mot message lon KHONG co header -> coi toan bo la audio.
  const uint8_t *p = reinterpret_cast<const uint8_t *>(data);
  size_t off = 0;
  if (len > 2) {
    // Ep ve uint8_t truoc khi dich, tranh sign-extension cua char.
    const uint16_t header_len = static_cast<uint16_t>((p[0] << 8) | p[1]);
    if (static_cast<size_t>(header_len) + 2 <= len) {
      const std::string headers(reinterpret_cast<const char *>(p) + 2,
                                std::min<size_t>(header_len, 160));
      if (headers.find("Path:audio") != std::string::npos) {
        off = 2 + static_cast<size_t>(header_len);
      }
    }
  }
  if (len <= off) {
    return;
  }

  // MP3 la dong BYTE nen khong can carry byte giua hai manh (ban PCM cu phai
  // giu, vi mot mau chiem 2 byte va co the bi cat doi).
  PushToBuffer(p + off, len - off);
}
