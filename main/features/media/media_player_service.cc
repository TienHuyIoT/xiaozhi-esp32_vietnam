/**
 * @file media_player_service.cc
 * @brief Implementation of MediaPlayerService wrapper.
 *
 * All public methods enqueue commands to a FreeRTOS queue processed by a
 * dedicated worker task.  This guarantees thread-safe access to the
 * underlying media_player handle from any calling context.
 */

#include "media_player_service.h"

#include <cstring>
#include <esp_log.h>

/* Third-party media_player headers */
extern "C" {
#include "player.h"
#include "av_render.h"
}

static const char* TAG = "MediaPlayerSvc";

/* ================================================================== */
/*  Singleton                                                         */
/* ================================================================== */

MediaPlayerService& MediaPlayerService::GetInstance() {
    static MediaPlayerService instance;
    return instance;
}

MediaPlayerService::MediaPlayerService() = default;

MediaPlayerService::~MediaPlayerService() {
    Deinit();
}

/* ================================================================== */
/*  Lifecycle                                                         */
/* ================================================================== */

bool MediaPlayerService::Init(const MediaPlayerConfig& config) {
    if (initialized_.load()) {
        ESP_LOGW(TAG, "Already initialized");
        return true;
    }

    config_ = config;

    /* Create command queue */
    cmd_queue_ = xQueueCreate(kCmdQueueLen, sizeof(Command));
    if (!cmd_queue_) {
        ESP_LOGE(TAG, "Failed to create command queue");
        return false;
    }

    /* Create callback mutex */
    callback_mutex_ = xSemaphoreCreateMutex();
    if (!callback_mutex_) {
        ESP_LOGE(TAG, "Failed to create callback mutex");
        vQueueDelete(cmd_queue_);
        cmd_queue_ = nullptr;
        return false;
    }

    /* Configure player */
    uint8_t play_mask = 0;
    if (config_.enable_audio) {
        play_mask |= PLAY_MASK_AUDIO;
    }
    if (config_.enable_video) {
        play_mask |= PLAY_MASK_VIDEO;
    }

    player_cfg_t cfg = {};
    cfg.play_mask = play_mask;
    cfg.no_accurate_seek = config_.fast_seek;

    /* TODO: Initialize audio_render and video_render when renderers are wired.
     * For now, leave them as nullptr -- media_player may still parse/decode
     * but won't produce output until renderers are connected. */
    cfg.audio_render = nullptr;
    cfg.video_render = nullptr;

    player_ = media_player_open(&cfg);
    if (!player_) {
        ESP_LOGE(TAG, "media_player_open failed");
        vSemaphoreDelete(callback_mutex_);
        vQueueDelete(cmd_queue_);
        callback_mutex_ = nullptr;
        cmd_queue_ = nullptr;
        return false;
    }

    /* Register event callback */
    media_player_set_callback(player_, PlayerCallbackTrampoline, this);

    /* Apply FIFO configuration */
    player_fifo_cfg_t fifo_cfg = {};
    fifo_cfg.adec_fifo_size     = config_.fifo.adec_fifo_size;
    fifo_cfg.vdec_fifo_size     = config_.fifo.vdec_fifo_size;
    fifo_cfg.arender_fifo_size  = config_.fifo.arender_fifo_size;
    fifo_cfg.vrender_fifo_size  = config_.fifo.vrender_fifo_size;
    fifo_cfg.extractor_pool_size = config_.fifo.extractor_pool_size;
    media_player_set_fifo_size(player_, &fifo_cfg);

    /* Start worker task */
    BaseType_t ret = xTaskCreatePinnedToCore(
        WorkerTaskEntry, "media_svc", kWorkerStackSize,
        this, kWorkerPriority, &worker_task_, kWorkerCore);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create worker task");
        media_player_close(player_);
        player_ = nullptr;
        vSemaphoreDelete(callback_mutex_);
        vQueueDelete(cmd_queue_);
        callback_mutex_ = nullptr;
        cmd_queue_ = nullptr;
        return false;
    }

    initialized_.store(true);
    SetState(MediaPlayerState::kStopped);
    ESP_LOGI(TAG, "Initialized (audio=%d video=%d fast_seek=%d)",
             config_.enable_audio, config_.enable_video, config_.fast_seek);
    return true;
}

void MediaPlayerService::Deinit() {
    if (!initialized_.load()) return;

    /* Send deinit command and wait for worker to exit */
    Command cmd = {};
    cmd.type = CmdType::kDeinit;
    EnqueueCommand(cmd);

    /* Wait for worker task to finish */
    if (worker_task_) {
        /* Give the worker a reasonable time to process kDeinit */
        for (int i = 0; i < 50 && eTaskGetState(worker_task_) != eDeleted; ++i) {
            vTaskDelay(pdMS_TO_TICKS(20));
        }
        worker_task_ = nullptr;
    }

    if (player_) {
        media_player_close(player_);
        player_ = nullptr;
    }

    if (cmd_queue_) {
        vQueueDelete(cmd_queue_);
        cmd_queue_ = nullptr;
    }

    if (callback_mutex_) {
        vSemaphoreDelete(callback_mutex_);
        callback_mutex_ = nullptr;
    }

    initialized_.store(false);
    SetState(MediaPlayerState::kIdle);
    ESP_LOGI(TAG, "Deinitialized");
}

/* ================================================================== */
/*  Source                                                             */
/* ================================================================== */

bool MediaPlayerService::SetSource(MediaSourceType type, const std::string& uri) {
    if (!initialized_.load()) return false;

    Command cmd = {};
    cmd.type = CmdType::kSetSource;
    cmd.params.source.src_type = type;
    /* Safe copy with truncation */
    strncpy(cmd.uri, uri.c_str(), sizeof(cmd.uri) - 1);
    cmd.uri[sizeof(cmd.uri) - 1] = '\0';
    return EnqueueCommand(cmd);
}

/* ================================================================== */
/*  Playback control                                                  */
/* ================================================================== */

bool MediaPlayerService::Play() {
    Command cmd = {};
    cmd.type = CmdType::kPlay;
    return EnqueueCommand(cmd);
}

bool MediaPlayerService::Pause() {
    Command cmd = {};
    cmd.type = CmdType::kPause;
    return EnqueueCommand(cmd);
}

bool MediaPlayerService::Resume() {
    Command cmd = {};
    cmd.type = CmdType::kResume;
    return EnqueueCommand(cmd);
}

bool MediaPlayerService::Stop() {
    Command cmd = {};
    cmd.type = CmdType::kStop;
    return EnqueueCommand(cmd);
}

bool MediaPlayerService::Seek(uint32_t time_ms) {
    Command cmd = {};
    cmd.type = CmdType::kSeek;
    cmd.params.seek.time_ms = time_ms;
    return EnqueueCommand(cmd);
}

bool MediaPlayerService::SetSpeed(float speed) {
    Command cmd = {};
    cmd.type = CmdType::kSetSpeed;
    cmd.params.speed.speed = speed;
    return EnqueueCommand(cmd);
}

bool MediaPlayerService::SetLoop(bool enable) {
    Command cmd = {};
    cmd.type = CmdType::kSetLoop;
    cmd.params.loop.enable = enable;
    return EnqueueCommand(cmd);
}

/* ================================================================== */
/*  Status queries                                                    */
/* ================================================================== */

int64_t MediaPlayerService::GetPosition() const {
    if (!player_) return -1;
    int position = 0;
    if (media_player_get_position(player_, &position) != 0) return -1;
    return static_cast<int64_t>(position);
}

int64_t MediaPlayerService::GetDuration() const {
    if (!player_) return -1;
    int duration = 0;
    if (media_player_get_duration(player_, &duration) != 0) return -1;
    return static_cast<int64_t>(duration);
}

/* ================================================================== */
/*  Event listener                                                    */
/* ================================================================== */

void MediaPlayerService::SetEventCallback(MediaPlayerEventCallback cb) {
    if (callback_mutex_) {
        xSemaphoreTake(callback_mutex_, portMAX_DELAY);
    }
    event_callback_ = std::move(cb);
    if (callback_mutex_) {
        xSemaphoreGive(callback_mutex_);
    }
}

/* ================================================================== */
/*  Internal: command queue                                           */
/* ================================================================== */

bool MediaPlayerService::EnqueueCommand(const Command& cmd) {
    if (!cmd_queue_) return false;
    if (xQueueSend(cmd_queue_, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Command queue full, dropping cmd=%d", (int)cmd.type);
        return false;
    }
    return true;
}

void MediaPlayerService::ProcessCommand(const Command& cmd) {
    if (!player_ && cmd.type != CmdType::kDeinit) return;

    switch (cmd.type) {
        case CmdType::kSetSource: {
            int src_type = (cmd.params.source.src_type == MediaSourceType::kFile)
                           ? MEDIA_SRC_TYPE_FILE
                           : MEDIA_SRC_TYPE_HTTP;
            int ret = media_player_set_source(player_, src_type, cmd.uri);
            if (ret != 0) {
                ESP_LOGE(TAG, "set_source failed: %d uri=%s", ret, cmd.uri);
                SetState(MediaPlayerState::kError);
            }
            break;
        }
        case CmdType::kPlay:
            media_player_play(player_);
            break;
        case CmdType::kPause:
            media_player_set_speed(player_, 0.0f);
            break;
        case CmdType::kResume:
            media_player_set_speed(player_, 1.0f);
            break;
        case CmdType::kStop:
            media_player_stop(player_);
            SetState(MediaPlayerState::kStopped);
            break;
        case CmdType::kSeek:
            media_player_seek_time(player_, cmd.params.seek.time_ms);
            break;
        case CmdType::kSetSpeed:
            media_player_set_speed(player_, cmd.params.speed.speed);
            break;
        case CmdType::kSetLoop:
            media_player_set_loop(player_, cmd.params.loop.enable);
            break;
        case CmdType::kDeinit:
            /* Handled in WorkerLoop exit path */
            break;
    }
}

/* ================================================================== */
/*  Internal: worker task                                             */
/* ================================================================== */

void MediaPlayerService::WorkerTaskEntry(void* arg) {
    auto* self = static_cast<MediaPlayerService*>(arg);
    self->WorkerLoop();
    vTaskDelete(nullptr);
}

void MediaPlayerService::WorkerLoop() {
    Command cmd;
    while (true) {
        if (xQueueReceive(cmd_queue_, &cmd, portMAX_DELAY) == pdTRUE) {
            if (cmd.type == CmdType::kDeinit) {
                ESP_LOGI(TAG, "Worker received deinit, exiting");
                break;
            }
            ProcessCommand(cmd);
        }
    }
}

/* ================================================================== */
/*  Internal: player event callback                                   */
/* ================================================================== */

int MediaPlayerService::PlayerCallbackTrampoline(int event, void* ctx) {
    auto* self = static_cast<MediaPlayerService*>(ctx);
    self->HandlePlayerEvent(event);
    return 0;
}

void MediaPlayerService::HandlePlayerEvent(int event) {
    MediaPlayerEvent mapped_event;
    MediaPlayerState new_state = state_.load();

    switch (event) {
        case PLAYER_EVENT_SRC_CONNECTING:
            mapped_event = MediaPlayerEvent::kSourceConnecting;
            new_state = MediaPlayerState::kConnecting;
            break;
        case PLAYER_EVENT_SRC_CONNECTED:
            mapped_event = MediaPlayerEvent::kSourceConnected;
            break;
        case PLAYER_EVENT_PREPARED:
            mapped_event = MediaPlayerEvent::kPrepared;
            new_state = MediaPlayerState::kPrepared;
            break;
        case PLAYER_EVENT_PLAYED_DONE:
            mapped_event = MediaPlayerEvent::kPlayStarted;
            new_state = MediaPlayerState::kPlaying;
            break;
        case PLAYER_EVENT_SEEK_DONE:
            mapped_event = MediaPlayerEvent::kSeekDone;
            break;
        case PLAYER_EVENT_EOS:
            mapped_event = MediaPlayerEvent::kEndOfStream;
            new_state = MediaPlayerState::kStopped;
            break;
        case PLAYER_EVENT_PLAY_ERROR:
            mapped_event = MediaPlayerEvent::kPlayError;
            new_state = MediaPlayerState::kError;
            ESP_LOGE(TAG, "Playback error reported by media_player");
            break;
        default:
            ESP_LOGD(TAG, "Unhandled player event: %d", event);
            return;
    }

    SetState(new_state);

    /* Dispatch to registered callback (non-blocking, mutex-protected read) */
    if (callback_mutex_) {
        xSemaphoreTake(callback_mutex_, portMAX_DELAY);
    }
    if (event_callback_) {
        event_callback_(mapped_event, new_state);
    }
    if (callback_mutex_) {
        xSemaphoreGive(callback_mutex_);
    }
}

/* ================================================================== */
/*  Internal: state management                                        */
/* ================================================================== */

void MediaPlayerService::SetState(MediaPlayerState new_state) {
    MediaPlayerState old = state_.exchange(new_state);
    if (old != new_state) {
        ESP_LOGI(TAG, "State: %d -> %d", (int)old, (int)new_state);
    }
}
