#include "device_tts_client.h"

#include <algorithm>

#include <esp_log.h>
#include <esp_system.h>
#include <esp_timer.h>

#include "board.h"

#define TAG "DeviceTTS"

DeviceTtsClient::DeviceTtsClient() {
  // Mot cau ~17KB MP3; nguong 32KB cua lop cha se treo cau dau mai mai.
  SetMinBufferSize(kTtsMinBufferBytes);

  preconnect_events_ = xEventGroupCreate();
  preconnect_exited_ = xSemaphoreCreateBinary();
  if (preconnect_events_ == nullptr || preconnect_exited_ == nullptr) {
    ESP_LOGE(TAG, "khong tao duoc dong bo preconnect; se bat tay dong bo");
    return;
  }

  preconnect_task_exited_ = false;
  const BaseType_t created = xTaskCreatePinnedToCore(
      [](void *arg) {
        static_cast<DeviceTtsClient *>(arg)->PreconnectTaskRoutine();
      },
      "tts_preconn", kPreconnectTaskStack, this, 4, &preconnect_task_handle_, 0);
  if (created != pdPASS) {
    preconnect_task_handle_ = nullptr;
    preconnect_task_exited_ = true;
    ESP_LOGE(TAG, "khong tao duoc task preconnect; se bat tay dong bo");
  }
}

DeviceTtsClient::~DeviceTtsClient() {
  Shutdown();
  // Join khi member cua DeviceTtsClient con song. Cho destructor lop cha la
  // qua muon: SourceDataLoop van co the cham queue_/socket_/mutex_.
  if (!StopStream()) {
    ESP_LOGE(TAG, "worker TTS khong thoat khi teardown; restart fail-safe");
    esp_restart();
  }
  // Source worker da dung, bay gio moi duoc phep cham active socket.
  CloseSocket();
}

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
    config_generation_.fetch_add(1);
  }
  configured_ = true;

  // URL moi lam socket warm cu vo nghia. Mo san ngay luc nhan tts_config --
  // thuong la trong khi be con dang noi -- de cau dau cung khong tra gia TLS.
  CloseReadySocket();
  RequestPreconnect();

  // KHONG log url_: no chua token chong bot cua nha cung cap.
  ESP_LOGI(TAG, "nap cau hinh TTS thiet bi (%d header, codec mp3)",
           static_cast<int>(headers_.size()));
  return true;
}

/* ------------------------------------------------------------------ */
/*  Hang doi cau                                                      */
/* ------------------------------------------------------------------ */

void DeviceTtsClient::Enqueue(const std::string &tts_body) {
  AudioTraceContext trace;
  trace.audio_source = "device_tts";
  Enqueue(tts_body, trace);
}

void DeviceTtsClient::Enqueue(const std::string &tts_body,
                              const AudioTraceContext &trace) {
  if (tts_body.empty()) {
    return;
  }
  if (!configured_.load()) {
    ESP_LOGW(TAG, "chua co tts_config ma da bao doc -> bo qua cau nay");
    return;
  }

  AudioTraceContext queued_trace = trace;
  queued_trace.trace_sequence = next_trace_sequence_.fetch_add(1);
  queued_trace.turn_id = SanitizeAudioTraceField(trace.turn_id);
  queued_trace.segment_id = SanitizeAudioTraceField(trace.segment_id);
  queued_trace.audio_source = SanitizeAudioTraceField(trace.audio_source);
  if (queued_trace.audio_source == "-" ||
      queued_trace.audio_source == "unknown") {
    queued_trace.audio_source = "device_tts";
  }

  size_t queue_depth = 0;
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (queue_.size() >= kMaxQueuedSentences) {
      ESP_LOGW(TAG, "hang doi day (%d cau), bo cau moi",
               static_cast<int>(queue_.size()));
      return;
    }
    // Dong dau generation ngay luc xep hang: neu Abort xay ra sau khi source
    // task da dequeue cau nay thi no van nhan ra minh thuoc turn da chet.
    queue_.push_back(
        QueuedTtsSegment{tts_body, queued_trace, turn_generation_.load()});
    queue_depth = queue_.size();
  }
  LogAudioTraceQueue("queue", "enqueue", queued_trace, queue_depth);

  // Enqueue va Abort deu duoc goi tu task cua Application (mot luong) nen khong
  // co dua nhau o day. Neu sau nay co task khac goi thi phai xem lai cho nay.
  abort_ = false;
  // Neu turn truoc vua abort thi worker co the chua kip mo lai socket warm.
  RequestPreconnect();
  if (!EnsureStarted()) {
    ESP_LOGE(TAG, "khong bat duoc duong phat -> be se khong nghe gi");
  }
}

void DeviceTtsClient::Abort() {
  abort_ = true;
  turn_generation_.fetch_add(1);
  const AudioTraceContext trace = GetActiveTrace();
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    queue_.clear();
  }
  LogAudioTraceQueue("queue", "clear", trace, 0);
  // Chi doi generation + xa byte dang cho. Hai worker va decoder se tu cat o
  // ranh gioi an toan; tuyet doi khong StopStream tu task Application.
  InterruptStream();
  ClearActiveTrace();
  // Danh thuc worker de no mo lai socket warm neu cai cu da hong/qua gia.
  // KHONG con dong socket warm o day: no khong thuoc turn vua bi huy, va vut
  // no lam cau dau turn ke tiep phai bat tay TLS lai tu dau (~1,6s do 09/08).
  if (preconnect_events_ != nullptr) {
    xEventGroupSetBits(preconnect_events_, kPreconnectWakeBit);
  }
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
  // Buffer co the da can nhung decoder van giu compressed input hoac dang ghi
  // PCM cuoi ra I2S. Bao idle som o day se mo mic giua cau.
  return GetBufferSize() > 0 || HasPendingDecoderPlayback();
}

void DeviceTtsClient::Shutdown() {
  running_ = false;
  Abort();

  if (preconnect_events_ != nullptr) {
    xEventGroupSetBits(preconnect_events_, kPreconnectWakeBit);
  }
  if (preconnect_task_handle_ != nullptr && preconnect_exited_ != nullptr) {
    if (xSemaphoreTake(preconnect_exited_, pdMS_TO_TICKS(15000)) != pdTRUE) {
      ESP_LOGE(TAG, "task preconnect khong thoat sau 15s; restart fail-safe");
      esp_restart();
    }
  }
  if (preconnect_task_handle_ != nullptr) {
    bool suspended = false;
    for (int i = 0; i < 50; ++i) {
      if (preconnect_task_exited_.load() &&
          eTaskGetState(preconnect_task_handle_) == eSuspended) {
        suspended = true;
        break;
      }
      vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (!suspended) {
      ESP_LOGE(TAG, "task preconnect chua suspend; restart fail-safe");
      esp_restart();
    }
    vTaskDelete(preconnect_task_handle_);
    preconnect_task_handle_ = nullptr;
  }
  CloseReadySocket();
  if (preconnect_events_ != nullptr) {
    vEventGroupDelete(preconnect_events_);
    preconnect_events_ = nullptr;
  }
  if (preconnect_exited_ != nullptr) {
    vSemaphoreDelete(preconnect_exited_);
    preconnect_exited_ = nullptr;
  }
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
    QueuedTtsSegment segment;
    size_t queue_depth = 0;
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      if (!queue_.empty()) {
        segment = std::move(queue_.front());
        queue_.pop_front();
        queue_depth = queue_.size();
      }
    }
    if (segment.body.empty()) {
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }
    LogAudioTraceQueue("queue", "dequeue", segment.trace, queue_depth);
    if (abort_.load() ||
        segment.turn_generation != turn_generation_.load()) {
      // Abort() da xoa hang doi, hoac cau nay thuoc turn cu va lot qua khe hep
      // giua dequeue va cho nay. Khong duoc mo socket cho no.
      LogAudioTraceEvent("drop_stale", segment.trace);
      continue;
    }

    RegisterAudioTraceSegment(segment.trace);
    SetActiveTrace(segment.trace);
    synthesizing_ = true;
    const bool ok = SynthesizeOne(segment.body, segment.turn_generation);
    synthesizing_ = false;

    // Edge tren robot khong on dinh khi tai su dung socket. Dong socket active
    // sau dung mot cau; socket moi da duoc worker mo SONG SONG tu luc gui SSML.
    CloseSocket();
    if (!ok) {
      ESP_LOGW(TAG, "tong hop cau that bai; cau sau se dung socket warm/fallback");
    }

    if (!HasQueuedSentence()) {
      // Race cu: neu LLM day cau N+1 trong luc vong nay dang doi buffer cau N
      // can ve 0, source task van doi den het + grace 300ms moi quay lai queue.
      // Ket qua loa im ro ret. Nay queue co cau moi la thoat NGAY de synth cau
      // tiep trong khi PCM cu van con phat o task core 1.
      int decoder_tail_stall_ms = 0;
      size_t last_pending_decoder_bytes = 0;
      while (IsSourceActive() && running_.load() && !abort_.load() &&
             (GetBufferSize() > 0 || HasPendingDecoderPlayback()) &&
             !HasQueuedSentence()) {
        const size_t pending_decoder_bytes = GetPendingDecoderInputBytes();
        if (GetBufferSize() == 0 && pending_decoder_bytes > 0 &&
            !IsDecoderOutputInFlight()) {
          if (pending_decoder_bytes == last_pending_decoder_bytes) {
            decoder_tail_stall_ms += 20;
          } else {
            last_pending_decoder_bytes = pending_decoder_bytes;
            decoder_tail_stall_ms = 0;
          }
          if (decoder_tail_stall_ms >= kDecoderTailStallMs) {
            ESP_LOGW(TAG,
                     "decoder tail dung o %zu byte; yeu cau bo frame loi",
                     pending_decoder_bytes);
            RequestIncompleteDecoderTailDiscard();
            decoder_tail_stall_ms = 0;
          }
        } else {
          last_pending_decoder_bytes = pending_decoder_bytes;
          decoder_tail_stall_ms = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
      }

      // Chi bao idle sau khi queue, decoder input va write PCM cung rong lien
      // tuc het grace. Pending cua chunk duoc cong bo truoc khi queue ve 0;
      // kiem tra lap o day van giu an toan neu co preemption bat thuong.
      int drained_ms = 0;
      while (IsSourceActive() && running_.load() && !abort_.load() &&
             !HasQueuedSentence()) {
        if (!IsPlaybackDrained()) {
          drained_ms = 0;
          vTaskDelay(pdMS_TO_TICKS(20));
          continue;
        }
        if (drained_ms >= kDrainGraceMs) {
          break;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
        drained_ms += 20;
      }
      if (!HasQueuedSentence() && !abort_.load() && IsPlaybackDrained()) {
        RequestAudioTraceSegmentFinish(segment.trace.trace_sequence);
        if (on_idle_) {
          on_idle_();
        }
      }
    }
  }

  CloseSocket();
  ClearActiveTrace();
  ESP_LOGI(TAG, "vong nguon ket thuc");
}

AudioTraceContext DeviceTtsClient::GetActiveTrace() {
  std::lock_guard<std::mutex> lock(active_trace_mutex_);
  return active_trace_;
}

void DeviceTtsClient::SetActiveTrace(const AudioTraceContext &trace) {
  {
    std::lock_guard<std::mutex> lock(active_trace_mutex_);
    active_trace_ = trace;
    active_stream_generation_ = GetStreamGeneration();
  }
  first_provider_byte_seen_ = false;
}

void DeviceTtsClient::ClearActiveTrace() {
  std::lock_guard<std::mutex> lock(active_trace_mutex_);
  active_stream_generation_ = 0;
  active_trace_ = AudioTraceContext{};
  first_provider_byte_seen_ = false;
}

bool DeviceTtsClient::HasQueuedSentence() {
  std::lock_guard<std::mutex> lock(queue_mutex_);
  return !queue_.empty();
}

bool DeviceTtsClient::EnsureConnected(uint32_t segment_generation) {
  if (websocket_ && websocket_->IsConnected()) {
    return true;
  }
  CloseSocket();

  if (PromoteReadySocket()) {
    return true;
  }

  // Worker co the dang o 100-200ms cuoi cua TLS. Doi dung ket qua do thay vi
  // mo them mot ket noi trung lap. Trong luc doi, task phat core 1 van tieu thu
  // PCM cau truoc, nen day van la overlap chu khong khoa loa.
  //
  // Dieu kien thoat dung `turn_generation_` chu KHONG dung `abort_`: sau Abort
  // thi `Enqueue()` cua turn moi xoa `abort_` ngay, nen co do se lam vong nay
  // tiep tuc cho ho mot segment da chet.
  const int64_t wait_started = esp_timer_get_time();
  while (running_.load() &&
         turn_generation_.load() == segment_generation &&
         (preconnect_requested_.load() || preconnect_inflight_.load()) &&
         (esp_timer_get_time() - wait_started) / 1000 < kPreconnectWaitMs) {
    if (PromoteReadySocket()) {
      return true;
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
  if (PromoteReadySocket()) {
    return true;
  }

  // Segment da bi huy trong luc cho: KHONG duoc tra them mot lan bat tay dong
  // bo ~1,5 giay cho no. Do 09/08: dung cho nay giu source worker 1.230ms roi
  // moi dequeue cau dau cua turn moi.
  if (!running_.load() || abort_.load() ||
      turn_generation_.load() != segment_generation) {
    return false;
  }

  // Worker vang/loi/qua han: fallback bat tay dong bo de khong lam mat cau.
  ConnectionSnapshot snapshot;
  if (!CaptureConnectionSnapshot(snapshot)) {
    return false;
  }
  websocket_ = OpenConfiguredSocket(snapshot, false);
  if (!websocket_) {
    return false;
  }
  active_socket_generation_ = snapshot.socket_generation;
  websocket_->OnData(
      [this, socket_generation = snapshot.socket_generation](
          const char *data, size_t len, bool binary) {
        if (active_socket_generation_.load() == socket_generation) {
          HandleData(data, len, binary);
        }
      });
  return true;
}

void DeviceTtsClient::CloseSocket() {
  active_socket_generation_ = 0;
  if (websocket_) {
    websocket_->Close();
    websocket_.reset();
  }
}

void DeviceTtsClient::CloseReadySocket() {
  std::unique_ptr<WebSocket> stale;
  {
    std::lock_guard<std::mutex> lock(ready_socket_mutex_);
    stale = std::move(ready_websocket_);
    ready_config_generation_ = 0;
    ready_socket_generation_ = 0;
    ready_socket_opened_us_ = 0;
  }
  if (stale) {
    stale->Close();
  }
}

bool DeviceTtsClient::CaptureConnectionSnapshot(
    ConnectionSnapshot &snapshot) {
  std::lock_guard<std::mutex> lock(config_mutex_);
  if (!configured_.load() || url_.empty()) {
    return false;
  }
  snapshot.url = url_;
  snapshot.config_frame = config_frame_;
  snapshot.headers = headers_;
  snapshot.config_generation = config_generation_.load();
  snapshot.socket_generation = next_socket_generation_.fetch_add(1);
  return true;
}

std::unique_ptr<WebSocket> DeviceTtsClient::OpenConfiguredSocket(
    const ConnectionSnapshot &snapshot, bool preconnect) {
  auto network = Board::GetInstance().GetNetwork();
  if (!network) {
    ESP_LOGE(TAG, "khong co network de mo TTS socket");
    return nullptr;
  }

  // Backend dang dung ID 1. Tach ID 2/3 de modem nao co slot ket noi cung khong
  // cho socket active va socket warm de len nhau; Wi-Fi bo qua ID nhung van an toan.
  auto socket = network->CreateWebSocket(preconnect ? 3 : 2);
  if (!socket) {
    ESP_LOGE(TAG, "CreateWebSocket that bai");
    return nullptr;
  }

  socket->SetReceiveBufferSize(8192);
  for (const auto &h : snapshot.headers) {
    socket->SetHeader(h.first.c_str(), h.second.c_str());
  }

  const uint32_t socket_generation = snapshot.socket_generation;
  // Socket warm KHONG gan OnData. Truoc khi duoc promote no khong gui SSML va
  // khong duoc phep co payload vao decoder; cach nay cung tranh cho hai TTS
  // socket dong thoi cham bo ghep frame cua dependency WebSocket.
  socket->OnError([socket_generation](int err) {
    ESP_LOGE(TAG, "loi socket #%lu: %d",
             static_cast<unsigned long>(socket_generation), err);
  });
  socket->OnDisconnected([socket_generation]() {
    ESP_LOGI(TAG, "socket #%lu dong",
             static_cast<unsigned long>(socket_generation));
  });

  const int64_t t0 = esp_timer_get_time();
  if (!socket->Connect(snapshot.url.c_str())) {
    ESP_LOGE(TAG, "%s bat tay that bai -- URL het han hay mat mang?",
             preconnect ? "preconnect" : "fallback");
    return nullptr;
  }
  const int64_t elapsed_ms = (esp_timer_get_time() - t0) / 1000;

  if (!snapshot.config_frame.empty() &&
      !socket->Send(snapshot.config_frame)) {
    ESP_LOGE(TAG, "gui config_frame that bai");
    socket->Close();
    return nullptr;
  }

  ESP_LOGI(TAG, "%s socket #%lu xong sau %lldms",
           preconnect ? "mo truoc" : "bat tay truc tiep",
           static_cast<unsigned long>(socket_generation), elapsed_ms);
  return socket;
}

bool DeviceTtsClient::PromoteReadySocket() {
  std::unique_ptr<WebSocket> stale;
  uint32_t promoted_generation = 0;
  {
    std::lock_guard<std::mutex> lock(ready_socket_mutex_);
    if (!ready_websocket_) {
      return false;
    }

    // Co tinh KHONG xet turn: socket warm chua gui SSML va chua gan OnData nen
    // no khong thuoc turn nao. Chi URL/token moi hoac socket qua gia moi bo.
    const int64_t age_ms =
        (esp_timer_get_time() - ready_socket_opened_us_) / 1000;
    const bool valid = ready_websocket_->IsConnected() &&
                       ready_config_generation_ == config_generation_.load() &&
                       age_ms < kReadySocketMaxAgeMs;
    if (!valid) {
      stale = std::move(ready_websocket_);
    } else {
      websocket_ = std::move(ready_websocket_);
      promoted_generation = ready_socket_generation_;
      active_socket_generation_ = promoted_generation;
      websocket_->OnData(
          [this, socket_generation = promoted_generation](
              const char *data, size_t len, bool binary) {
            if (active_socket_generation_.load() == socket_generation) {
              HandleData(data, len, binary);
            }
          });
    }
    ready_config_generation_ = 0;
    ready_socket_generation_ = 0;
    ready_socket_opened_us_ = 0;
  }

  if (stale) {
    stale->Close();
    return false;
  }
  if (promoted_generation != 0) {
    ESP_LOGI(TAG, "dung socket #%lu da mo truoc -- khong cho TLS",
             static_cast<unsigned long>(promoted_generation));
    return true;
  }
  return false;
}

void DeviceTtsClient::RequestPreconnect() {
  if (!running_.load() || abort_.load() || !configured_.load() ||
      preconnect_events_ == nullptr || preconnect_task_handle_ == nullptr) {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(ready_socket_mutex_);
    const int64_t age_ms =
        (esp_timer_get_time() - ready_socket_opened_us_) / 1000;
    if (ready_websocket_ && ready_websocket_->IsConnected() &&
        ready_config_generation_ == config_generation_.load() &&
        age_ms < kReadySocketMaxAgeMs) {
      return; // da co socket dung duoc -- ke ca khi turn vua doi
    }
  }

  bool expected = false;
  if (preconnect_requested_.compare_exchange_strong(expected, true)) {
    xEventGroupSetBits(preconnect_events_, kPreconnectWakeBit);
  }
}

void DeviceTtsClient::PreconnectTaskRoutine() {
  while (running_.load()) {
    xEventGroupWaitBits(preconnect_events_, kPreconnectWakeBit, pdTRUE, pdFALSE,
                        portMAX_DELAY);
    if (!running_.load()) {
      break;
    }

    preconnect_inflight_ = true;
    preconnect_requested_ = false;
    vTaskDelay(pdMS_TO_TICKS(kPreconnectDelayMs));

    bool already_ready = false;
    std::unique_ptr<WebSocket> stale;
    {
      std::lock_guard<std::mutex> lock(ready_socket_mutex_);
      const int64_t age_ms =
          (esp_timer_get_time() - ready_socket_opened_us_) / 1000;
      already_ready = ready_websocket_ && ready_websocket_->IsConnected() &&
                      ready_config_generation_ == config_generation_.load() &&
                      age_ms < kReadySocketMaxAgeMs;
      if (ready_websocket_ && !already_ready) {
        stale = std::move(ready_websocket_);
        ready_config_generation_ = 0;
        ready_socket_generation_ = 0;
        ready_socket_opened_us_ = 0;
      }
    }
    if (stale) {
      stale->Close();
    }

    ConnectionSnapshot snapshot;
    std::unique_ptr<WebSocket> socket;
    if (!already_ready && running_.load() && !abort_.load() &&
        CaptureConnectionSnapshot(snapshot)) {
      socket = OpenConfiguredSocket(snapshot, true);
    }

    if (socket) {
      std::lock_guard<std::mutex> lock(ready_socket_mutex_);
      // Khong xet turn: mot socket vua bat tay xong van dung duoc cho turn moi.
      if (running_.load() && !ready_websocket_ &&
          snapshot.config_generation == config_generation_.load()) {
        ready_websocket_ = std::move(socket);
        ready_config_generation_ = snapshot.config_generation;
        ready_socket_generation_ = snapshot.socket_generation;
        ready_socket_opened_us_ = esp_timer_get_time();
      }
    }
    if (socket) {
      socket->Close();
    }
    preconnect_inflight_ = false;
  }

  preconnect_inflight_ = false;
  preconnect_requested_ = false;
  if (preconnect_exited_ != nullptr) {
    xSemaphoreGive(preconnect_exited_);
  }
  preconnect_task_exited_ = true;
  vTaskSuspend(nullptr);
  vTaskDelete(nullptr);
}

bool DeviceTtsClient::SynthesizeOne(const std::string &body,
                                    uint32_t synthesis_generation) {
  const AudioTraceContext trace = GetActiveTrace();
  LogAudioTraceEvent("connect_begin", trace);
  if (!EnsureConnected(synthesis_generation)) {
    return false;
  }
  if (abort_.load() ||
      turn_generation_.load() != synthesis_generation) {
    return false;
  }
  LogAudioTraceEvent("connect_ready", trace);

  turn_end_ = false;
  last_data_tick_ = xTaskGetTickCount();

  // Gui NGUYEN VAN khung text server da render (co san X-RequestId, X-Timestamp,
  // Path:ssml). Robot khong duoc tu sinh timestamp: dong ho no lech -7 gio vi
  // ota.cc cong timezone_offset vao epoch truoc khi settimeofday().
  if (!websocket_->Send(body)) {
    ESP_LOGE(TAG, "gui cau that bai");
    return false;
  }

  // Bat tay cau ke tiep NGAY trong khi Edge dang tong hop/stream cau nay. Worker
  // co delay ngan de viec gui SSML va byte audio dau khong bi chen boi TLS.
  RequestPreconnect();

  const TickType_t no_data_timeout = pdMS_TO_TICKS(kNoAudioTimeoutMs);
  const TickType_t total_timeout =
      pdMS_TO_TICKS(std::max<size_t>(kTotalTimeoutMinMs, body.size() * 60));
  const TickType_t started = xTaskGetTickCount();

  while (running_.load() && !abort_.load()) {
    if (turn_generation_.load() != synthesis_generation) {
      return false;
    }
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
  const uint32_t stream_generation = active_stream_generation_.load();
  if (abort_.load() || !running_.load() || stream_generation == 0) {
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

  const AudioTraceContext trace = GetActiveTrace();
  if (!first_provider_byte_seen_.exchange(true)) {
    LogAudioTraceEvent("first_provider_byte", trace);
  }

  // MP3 la dong BYTE nen khong can carry byte giua hai manh (ban PCM cu phai
  // giu, vi mot mau chiem 2 byte va co the bi cat doi).
  PushToBuffer(p + off, len - off, trace.trace_sequence,
               stream_generation);
}
