#include "mp4_player.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <dirent.h>
#include <sys/stat.h>

#include <esp_heap_caps.h>
#include <esp_log.h>

extern "C" {
#include "esp_audio_dec_default.h"
#include "esp_extractor_defaults.h"
#include "esp_video_dec_default.h"
#include "extractor_helper.h"
}

#include "audio_codec.h"
#include "display.h"
#include "sd_card.h"

static const char* TAG = "Mp4Player";

namespace {
constexpr uint32_t kAudioRawFifoSize = 64 * 1024;
constexpr uint32_t kAudioRenderFifoSize = 16 * 1024;
constexpr uint32_t kVideoRawFifoSize = 512 * 1024;
constexpr uint32_t kVideoRenderFifoSize = 64 * 1024;

constexpr uint32_t kAacSampleRateTable[] = {
    96000, 88200, 64000, 48000, 44100, 32000,
    24000, 22050, 16000, 12000, 11025, 8000,
    7350,
};

bool ParseAacAsc(const uint8_t* data, size_t len, uint32_t& sample_rate, uint8_t& channels) {
    if (!data || len < 2) {
        return false;
    }

    uint8_t sf_idx = static_cast<uint8_t>(((data[0] & 0x07) << 1) | ((data[1] >> 7) & 0x01));
    uint8_t ch_cfg = static_cast<uint8_t>((data[1] >> 3) & 0x0F);

    if (sf_idx == 0x0F) {
        if (len < 5) {
            return false;
        }
        sample_rate = (static_cast<uint32_t>(data[1] & 0x7F) << 17) |
                      (static_cast<uint32_t>(data[2]) << 9) |
                      (static_cast<uint32_t>(data[3]) << 1) |
                      ((data[4] >> 7) & 0x01);
    } else if (sf_idx < (sizeof(kAacSampleRateTable) / sizeof(kAacSampleRateTable[0]))) {
        sample_rate = kAacSampleRateTable[sf_idx];
    }

    if (ch_cfg > 0 && ch_cfg <= 7) {
        channels = ch_cfg;
    }

    return (sample_rate > 0 && channels > 0);
}

inline int16_t ClampToInt16(float value) {
    if (value > 32767.0f) return 32767;
    if (value < -32768.0f) return -32768;
    return static_cast<int16_t>(value);
}
}  // namespace

struct Mp4Player::AudioRenderCtx {
    Mp4Player* owner = nullptr;
    av_render_audio_frame_info_t info{};
};

struct Mp4Player::VideoRenderCtx {
    Mp4Player* owner = nullptr;
    av_render_video_frame_info_t info{};
};

Mp4Player& Mp4Player::GetInstance() {
    static Mp4Player instance;
    return instance;
}

Mp4Player::Mp4Player() = default;

Mp4Player::~Mp4Player() {
    Deinitialize();
}

bool Mp4Player::Initialize(esp_lcd_panel_handle_t lcd_panel,
                           uint16_t lcd_width, uint16_t lcd_height,
                           AudioCodec* codec,
                           SdCard* sd_card,
                           Display* display,
                           Mp4RenderMode mode) {
    if (initialized_.load()) {
        ESP_LOGW(TAG, "Already initialized");
        return true;
    }
    if (!lcd_panel || !codec || !sd_card) {
        ESP_LOGE(TAG, "Invalid init args: panel=%p codec=%p sd=%p", lcd_panel, codec, sd_card);
        return false;
    }

    lcd_panel_ = lcd_panel;
    lcd_width_ = lcd_width;
    lcd_height_ = lcd_height;
    audio_codec_ = codec;
    sd_card_ = sd_card;
    mount_point_ = sd_card_->GetMountPoint();
    display_ = display;
    render_mode_ = mode;

    esp_extractor_register_default();
    esp_audio_dec_register_default();
    esp_video_dec_register_default();

    renderer_init_cfg_.owner = this;
    if (!InitializeAvRender()) {
        Deinitialize();
        return false;
    }

    initialized_.store(true);
    SetState(Mp4PlayerState::Idle);
    ESP_LOGI(TAG, "Initialized av_render pipeline: LCD %ux%u mount=%s mode=%s",
             lcd_width_, lcd_height_, mount_point_.c_str(),
             render_mode_ == Mp4RenderMode::DirectLcd ? "DirectLcd" : "LvglCanvas");
    return true;
}

void Mp4Player::Deinitialize() {
    Stop();
    CleanupPlaybackResources();
    DestroyAvRender();
    DestroyVideoCanvas();

    if (video_draw_buf_) {
        heap_caps_free(video_draw_buf_);
        video_draw_buf_ = nullptr;
        video_draw_buf_size_ = 0;
    }

#if MP4_PLAYER_STATIC_TASK_CREATION == 1
    if (playback_task_buffer_) {
        heap_caps_free(playback_task_buffer_);
        playback_task_buffer_ = nullptr;
    }
    if (playback_task_stack_) {
        heap_caps_free(playback_task_stack_);
        playback_task_stack_ = nullptr;
    }
#endif

    initialized_.store(false);
    SetState(Mp4PlayerState::Idle);
    ESP_LOGI(TAG, "Deinitialized");
}

bool Mp4Player::InitializeAvRender() {
    audio_render_cfg_t audio_cfg = {};
    audio_cfg.ops.init = AudioRenderInit;
    audio_cfg.ops.open = AudioRenderOpen;
    audio_cfg.ops.write = AudioRenderWrite;
    audio_cfg.ops.get_latency = AudioRenderGetLatency;
    audio_cfg.ops.get_frame_info = AudioRenderGetFrameInfo;
    audio_cfg.ops.set_speed = AudioRenderSetSpeed;
    audio_cfg.ops.close = AudioRenderClose;
    audio_cfg.ops.deinit = AudioRenderDeinit;
    audio_cfg.cfg = &renderer_init_cfg_;
    audio_cfg.cfg_size = sizeof(renderer_init_cfg_);

    audio_render_ = audio_render_alloc_handle(&audio_cfg);
    if (!audio_render_) {
        ESP_LOGE(TAG, "Failed to alloc audio renderer");
        return false;
    }

    video_render_cfg_t video_cfg = {};
    video_cfg.ops.open = VideoRenderOpen;
    video_cfg.ops.format_support = VideoRenderFormatSupported;
    video_cfg.ops.set_frame_info = VideoRenderSetFrameInfo;
    video_cfg.ops.get_frame_buffer = VideoRenderGetFrameBuffer;
    video_cfg.ops.write = VideoRenderWrite;
    video_cfg.ops.get_latency = VideoRenderGetLatency;
    video_cfg.ops.get_frame_info = VideoRenderGetFrameInfo;
    video_cfg.ops.clear = VideoRenderClear;
    video_cfg.ops.close = VideoRenderClose;
    video_cfg.cfg = &renderer_init_cfg_;
    video_cfg.cfg_size = sizeof(renderer_init_cfg_);

    video_render_ = video_render_alloc_handle(&video_cfg);
    if (!video_render_) {
        ESP_LOGE(TAG, "Failed to alloc video renderer");
        return false;
    }

    av_render_cfg_t render_cfg = {};
    render_cfg.audio_render = audio_render_;
    render_cfg.video_render = video_render_;
    render_cfg.sync_mode = AV_RENDER_SYNC_FOLLOW_AUDIO;
    render_cfg.audio_raw_fifo_size = kAudioRawFifoSize;
    render_cfg.video_raw_fifo_size = kVideoRawFifoSize;
    render_cfg.audio_render_fifo_size = kAudioRenderFifoSize;
    render_cfg.video_render_fifo_size = 0; // Use default in render implementation
    render_cfg.quit_when_eos = false;
    render_cfg.allow_drop_data = false;
    render_cfg.pause_render_only = true;

    av_render_ = av_render_open(&render_cfg);
    if (!av_render_) {
        ESP_LOGE(TAG, "Failed to open av_render");
        return false;
    }
    av_render_set_event_cb(av_render_, AvRenderEventCb, this);
    return true;
}

void Mp4Player::DestroyAvRender() {
    if (av_render_) {
        av_render_close(av_render_);
        av_render_ = nullptr;
    }
    if (video_render_) {
        video_render_free_handle(video_render_);
        video_render_ = nullptr;
    }
    if (audio_render_) {
        audio_render_free_handle(audio_render_);
        audio_render_ = nullptr;
    }
}

bool Mp4Player::Play(const std::string& file_path) {
    if (!initialized_.load()) {
        ESP_LOGE(TAG, "Not initialized");
        return false;
    }

    if (state_.load() != Mp4PlayerState::Idle) {
        Stop();
    }

    SetState(Mp4PlayerState::Loading);
    stop_requested_.store(false);
    paused_.store(false);
    current_file_path_ = file_path;

    {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        stats_ = Mp4PlaybackStats{};
    }

    if (!PreparePlayback(file_path)) {
        CleanupPlaybackResources();
        SetState(Mp4PlayerState::Error);
        return false;
    }

    playback_task_running_.store(true);
#if MP4_PLAYER_STATIC_TASK_CREATION == 1
    if (playback_task_stack_ == nullptr) {
        playback_task_stack_ = static_cast<StackType_t*>(
            heap_caps_malloc(MP4_TASK_STACK_SIZE, MALLOC_CAP_SPIRAM));
    }
    if (playback_task_buffer_ == nullptr) {
        playback_task_buffer_ = static_cast<StaticTask_t*>(
            heap_caps_malloc(sizeof(StaticTask_t), MALLOC_CAP_INTERNAL));
    }
    playback_task_handle_ = xTaskCreateStaticPinnedToCore(
        PlaybackTaskEntry, "mp4_play",
        MP4_TASK_STACK_SIZE, this,
        MP4_TASK_PRIORITY,
        playback_task_stack_, playback_task_buffer_, MP4_TASK_CORE);
#else
    BaseType_t ret = xTaskCreatePinnedToCore(
        PlaybackTaskEntry, "mp4_play",
        MP4_TASK_STACK_SIZE, this,
        MP4_TASK_PRIORITY,
        &playback_task_handle_, MP4_TASK_CORE);
    if (ret != pdPASS) {
        playback_task_handle_ = nullptr;
    }
#endif

    if (!playback_task_handle_) {
        playback_task_running_.store(false);
        ESP_LOGE(TAG, "Failed to create playback task");
        CleanupPlaybackResources();
        SetState(Mp4PlayerState::Error);
        return false;
    }

    SetState(Mp4PlayerState::Playing);
    ESP_LOGI(TAG, "Playing: %s", file_path.c_str());
    return true;
}

bool Mp4Player::PlayFile(const std::string& file_name) {
    return Play(BuildFullPath(file_name));
}

void Mp4Player::Stop() {
    if (state_.load() == Mp4PlayerState::Idle) {
        return;
    }

    stop_requested_.store(true);
    SetState(Mp4PlayerState::Stopping);

    int guard = 100;
    while (playback_task_running_.load() && --guard >= 0) {
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    if (av_render_) {
        av_render_pause(av_render_, false);
        av_render_reset(av_render_);
    }
    CleanupPlaybackResources();

    stop_requested_.store(false);
    paused_.store(false);
    playback_task_running_.store(false);
    current_file_path_.clear();
    SetState(Mp4PlayerState::Idle);
    ESP_LOGI(TAG, "Stopped");
}

void Mp4Player::Pause() {
    if (state_.load() != Mp4PlayerState::Playing) return;
    paused_.store(true);
    if (av_render_) {
        av_render_pause(av_render_, true);
    }
    SetState(Mp4PlayerState::Paused);
}

void Mp4Player::Resume() {
    if (state_.load() != Mp4PlayerState::Paused) return;
    paused_.store(false);
    if (av_render_) {
        av_render_pause(av_render_, false);
    }
    SetState(Mp4PlayerState::Playing);
}

bool Mp4Player::Next() {
    if (playlist_.empty()) return false;
    int next = current_index_ + 1;
    if (next >= static_cast<int>(playlist_.size())) next = 0;
    current_index_ = next;
    return Play(playlist_[current_index_].path);
}

bool Mp4Player::Prev() {
    if (playlist_.empty()) return false;
    int prev = current_index_ - 1;
    if (prev < 0) prev = static_cast<int>(playlist_.size()) - 1;
    current_index_ = prev;
    return Play(playlist_[current_index_].path);
}

void Mp4Player::SetDirectory(const std::string& dir) {
    mp4_directory_ = dir;
    playlist_.clear();
    current_index_ = -1;
}

size_t Mp4Player::ScanDirectory() {
    playlist_.clear();
    current_index_ = -1;

    std::string dir_path = mount_point_ + "/" + mp4_directory_;
    DIR* dir = opendir(dir_path.c_str());
    if (!dir) {
        ESP_LOGW(TAG, "Cannot open directory: %s", dir_path.c_str());
        return 0;
    }

    struct dirent* entry = nullptr;
    while ((entry = readdir(dir)) != nullptr && playlist_.size() < MP4_MAX_FILES) {
        if (entry->d_type != DT_REG) continue;
        std::string name(entry->d_name);
        if (!IsSupportedMp4Name(name)) continue;

        Mp4FileInfo info;
        info.name = name;
        info.path = dir_path + "/" + name;

        struct stat st;
        if (stat(info.path.c_str(), &st) == 0) {
            info.file_size = static_cast<uint32_t>(st.st_size);
        }
        playlist_.push_back(std::move(info));
    }
    closedir(dir);

    std::sort(playlist_.begin(), playlist_.end(),
              [](const Mp4FileInfo& a, const Mp4FileInfo& b) { return a.name < b.name; });

    ESP_LOGI(TAG, "Scanned %zu MP4 files in %s", playlist_.size(), dir_path.c_str());
    return playlist_.size();
}

Mp4PlaybackStats Mp4Player::GetStats() const {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    return stats_;
}

void Mp4Player::PlaybackTaskEntry(void* arg) {
    auto* self = static_cast<Mp4Player*>(arg);
    if (self) self->PlaybackTaskLoop();
}

int Mp4Player::FileReadCb(void* buffer, uint32_t size, void* ctx) {
    auto* fc = static_cast<FileContext*>(ctx);
    if (!fc || !fc->fp) return -1;
    return static_cast<int>(fread(buffer, 1, size, fc->fp));
}

int Mp4Player::FileSeekCb(uint32_t position, void* ctx) {
    auto* fc = static_cast<FileContext*>(ctx);
    if (!fc || !fc->fp) return -1;
    return fseek(fc->fp, static_cast<long>(position), SEEK_SET);
}

uint32_t Mp4Player::FileSizeCb(void* ctx) {
    auto* fc = static_cast<FileContext*>(ctx);
    if (!fc || !fc->fp) return 0;
    long cur = ftell(fc->fp);
    if (cur < 0) return 0;
    fseek(fc->fp, 0, SEEK_END);
    long end = ftell(fc->fp);
    fseek(fc->fp, cur, SEEK_SET);
    return end < 0 ? 0 : static_cast<uint32_t>(end);
}

bool Mp4Player::PreparePlayback(const std::string& file_path) {
    if (file_path.empty()) {
        ESP_LOGE(TAG, "Empty file path");
        return false;
    }

    CleanupPlaybackResources();
    ResetRenderEosFlags();

    file_ctx_.fp = fopen(file_path.c_str(), "rb");
    if (!file_ctx_.fp) {
        ESP_LOGE(TAG, "Failed to open: %s", file_path.c_str());
        return false;
    }

    memset(&extractor_cfg_, 0, sizeof(extractor_cfg_));
    extractor_cfg_.type = ESP_EXTRACTOR_TYPE_MP4;
    extractor_cfg_.extract_mask = ESP_EXTRACT_MASK_AV;
    extractor_cfg_.in_read_cb = FileReadCb;
    extractor_cfg_.in_seek_cb = FileSeekCb;
    extractor_cfg_.in_size_cb = FileSizeCb;
    extractor_cfg_.in_ctx = &file_ctx_;
    extractor_cfg_.out_pool_size = MP4_EXTRACTOR_POOL_SIZE;
    extractor_cfg_.out_align = 16;

    esp_extractor_err_t err = esp_extractor_open(&extractor_cfg_, &extractor_);
    if (err != ESP_EXTRACTOR_ERR_OK) {
        ESP_LOGE(TAG, "esp_extractor_open failed: %d", err);
        return false;
    }

    err = esp_extractor_parse_stream(extractor_);
    if (err != ESP_EXTRACTOR_ERR_OK) {
        ESP_LOGE(TAG, "esp_extractor_parse_stream failed: %d", err);
        return false;
    }

    uint16_t audio_num = 0;
    uint16_t video_num = 0;
    err = esp_extractor_get_stream_num(extractor_, ESP_EXTRACTOR_STREAM_TYPE_AUDIO, &audio_num);
    if (err != ESP_EXTRACTOR_ERR_OK || audio_num == 0) {
        ESP_LOGE(TAG, "No audio stream in %s", file_path.c_str());
        return false;
    }
    err = esp_extractor_get_stream_num(extractor_, ESP_EXTRACTOR_STREAM_TYPE_VIDEO, &video_num);
    if (err != ESP_EXTRACTOR_ERR_OK || video_num == 0) {
        ESP_LOGE(TAG, "No video stream in %s", file_path.c_str());
        return false;
    }

    memset(&audio_stream_info_, 0, sizeof(audio_stream_info_));
    memset(&video_stream_info_, 0, sizeof(video_stream_info_));

    err = esp_extractor_get_stream_info(extractor_, ESP_EXTRACTOR_STREAM_TYPE_AUDIO, 0, &audio_stream_info_);
    if (err != ESP_EXTRACTOR_ERR_OK) {
        ESP_LOGE(TAG, "Failed to get audio info: %d", err);
        return false;
    }

    err = esp_extractor_get_stream_info(extractor_, ESP_EXTRACTOR_STREAM_TYPE_VIDEO, 0, &video_stream_info_);
    if (err != ESP_EXTRACTOR_ERR_OK) {
        ESP_LOGE(TAG, "Failed to get video info: %d", err);
        return false;
    }

    audio_codec_type_ = MapAudioCodec(audio_stream_info_.audio_info.format);
    video_codec_type_ = MapVideoCodec(video_stream_info_.video_info.format);
    if (audio_codec_type_ == AV_RENDER_AUDIO_CODEC_NONE) {
        ESP_LOGE(TAG, "Unsupported audio fmt: 0x%lx", (unsigned long)audio_stream_info_.audio_info.format);
        return false;
    }
    if (video_codec_type_ == AV_RENDER_VIDEO_CODEC_NONE) {
        ESP_LOGE(TAG, "Unsupported video fmt: 0x%lx", (unsigned long)video_stream_info_.video_info.format);
        return false;
    }

    ESP_LOGI(TAG, "Audio format=%s sample_rate=%lu channel=%u bits=%u duration=%lu",
             esp_extractor_get_format_name(audio_stream_info_.audio_info.format),
             (unsigned long)audio_stream_info_.audio_info.sample_rate,
             (unsigned)audio_stream_info_.audio_info.channel,
             (unsigned)audio_stream_info_.audio_info.bits_per_sample,
             (unsigned long)audio_stream_info_.duration);

    ESP_LOGI(TAG, "Video format=%s resolution=%ux%u fps=%u duration=%lu",
             esp_extractor_get_format_name(video_stream_info_.video_info.format),
             (unsigned)video_stream_info_.video_info.width,
             (unsigned)video_stream_info_.video_info.height,
             (unsigned)video_stream_info_.video_info.fps,
             (unsigned long)video_stream_info_.duration);

    return ConfigureRenderStreams();
}

bool Mp4Player::ConfigureRenderStreams() {
    if (!av_render_) {
        ESP_LOGE(TAG, "av_render not initialized");
        return false;
    }

    if (av_render_reset(av_render_) != 0) {
        ESP_LOGW(TAG, "av_render_reset failed, continue with stream setup");
    }

    av_render_audio_info_t audio_info = {};
    audio_info.codec = audio_codec_type_;
    audio_info.channel = audio_stream_info_.audio_info.channel;
    audio_info.bits_per_sample = audio_stream_info_.audio_info.bits_per_sample;
    audio_info.sample_rate = audio_stream_info_.audio_info.sample_rate;
    audio_info.codec_spec_info = audio_stream_info_.spec_info;
    audio_info.spec_info_len = static_cast<int>(audio_stream_info_.spec_info_len);

    if (audio_info.bits_per_sample == 0) {
        audio_info.bits_per_sample = 16;
    }

    if (audio_codec_type_ == AV_RENDER_AUDIO_CODEC_AAC) {
        uint32_t asc_rate = audio_info.sample_rate;
        uint8_t asc_channels = audio_info.channel;
        if (ParseAacAsc(audio_stream_info_.spec_info,
                        audio_stream_info_.spec_info_len,
                        asc_rate,
                        asc_channels)) {
            audio_info.sample_rate = asc_rate;
            audio_info.channel = asc_channels;
        }
        if (audio_info.sample_rate == 0) {
            audio_info.sample_rate = 44100;
        }
        if (audio_info.channel == 0) {
            audio_info.channel = 2;
        }

        // MP4 usually carries raw AAC frames (no ADTS) with AudioSpecificConfig.
        audio_info.aac_no_adts = (audio_info.spec_info_len > 0) ? 1 : 0;
    }

    if (audio_codec_type_ == AV_RENDER_AUDIO_CODEC_AAC) {
        ESP_LOGI(TAG,
                 "AAC cfg: sr=%lu ch=%u bits=%u spec_len=%d no_adts=%u",
                 (unsigned long)audio_info.sample_rate,
                 static_cast<unsigned>(audio_info.channel),
                 static_cast<unsigned>(audio_info.bits_per_sample),
                 audio_info.spec_info_len,
                 static_cast<unsigned>(audio_info.aac_no_adts));
    }

    int ret = av_render_add_audio_stream(av_render_, &audio_info);
    if (ret != 0) {
        ESP_LOGE(TAG, "av_render_add_audio_stream failed: %d", ret);
        return false;
    }

    av_render_video_info_t video_info = {};
    video_info.codec = video_codec_type_;
    video_info.width = video_stream_info_.video_info.width;
    video_info.height = video_stream_info_.video_info.height;
    video_info.fps = static_cast<uint8_t>(video_stream_info_.video_info.fps);
    video_info.codec_spec_info = video_stream_info_.spec_info;
    video_info.spec_info_len = static_cast<int>(video_stream_info_.spec_info_len);

    ret = av_render_add_video_stream(av_render_, &video_info);
    if (ret != 0) {
        ESP_LOGE(TAG, "av_render_add_video_stream failed: %d", ret);
        return false;
    }

    // Keep the container/display resolution from extractor. For H264 this can differ
    // from decoder output size reported later (macroblock-aligned height, e.g. 180->192).
    expected_video_width_ = video_stream_info_.video_info.width;
    expected_video_height_ = video_stream_info_.video_info.height;

    {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        stats_.audio_rate = audio_stream_info_.audio_info.sample_rate;
        stats_.audio_channels = audio_stream_info_.audio_info.channel;
        stats_.audio_bits = audio_stream_info_.audio_info.bits_per_sample;
        stats_.video_width = video_stream_info_.video_info.width;
        stats_.video_height = video_stream_info_.video_info.height;
        stats_.video_fps = video_stream_info_.video_info.fps;
    }

    return true;
}

void Mp4Player::CleanupPlaybackResources() {
    if (extractor_) {
        esp_extractor_close(extractor_);
        extractor_ = nullptr;
    }
    CloseFile();
}

void Mp4Player::CloseFile() {
    if (file_ctx_.fp) {
        fclose(file_ctx_.fp);
        file_ctx_.fp = nullptr;
    }
}

bool Mp4Player::ProcessAudioFrame(const esp_extractor_frame_info_t& frame) {
    if (!av_render_ || frame.frame_buffer == nullptr || frame.frame_size == 0) {
        return false;
    }

    av_render_audio_data_t audio_data = {};
    audio_data.data = frame.frame_buffer;
    audio_data.size = frame.frame_size;
    audio_data.pts = frame.pts;
    audio_data.eos = EXTRACTOR_IS_EOS(frame.frame_flag);

    int retry = 0;
    while (!stop_requested_.load()) {
        int ret = av_render_add_audio_data(av_render_, &audio_data);
        if (ret == 0) {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            stats_.audio_frames++;
            return true;
        }
        if (++retry >= 40) {
            ESP_LOGW(TAG, "Drop audio frame, add_audio_data ret=%d", ret);
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return false;
}

bool Mp4Player::ProcessVideoFrame(const esp_extractor_frame_info_t& frame) {
    if (!av_render_ || frame.frame_buffer == nullptr || frame.frame_size == 0) {
        return false;
    }

    av_render_video_data_t video_data = {};
    video_data.data = frame.frame_buffer;
    video_data.size = frame.frame_size;
    video_data.pts = frame.pts;
    video_data.eos = EXTRACTOR_IS_EOS(frame.frame_flag);
    video_data.key_frame = true;

    int retry = 0;
    while (!stop_requested_.load()) {
        int ret = av_render_add_video_data(av_render_, &video_data);
        if (ret == 0) {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            stats_.video_frames++;
            return true;
        }
        if (++retry >= 40) {
            ESP_LOGW(TAG, "Drop video frame, add_video_data ret=%d", ret);
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return false;
}

bool Mp4Player::PushEos() {
    if (!av_render_) return false;

    av_render_audio_data_t audio_eos = {};
    audio_eos.eos = true;

    av_render_video_data_t video_eos = {};
    video_eos.eos = true;

    /* Retry briefly; av_render queues may be full if decode already finished. */
    int a_ret = -1, v_ret = -1;
    for (int i = 0; i < 200 && !stop_requested_.load(); ++i) {
        if (a_ret != 0) a_ret = av_render_add_audio_data(av_render_, &audio_eos);
        if (v_ret != 0) v_ret = av_render_add_video_data(av_render_, &video_eos);
        if (a_ret == 0 && v_ret == 0) break;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    ESP_LOGI(TAG, "PushEos: audio=%d video=%d", a_ret, v_ret);
    return (a_ret == 0 && v_ret == 0);
}

void Mp4Player::ResetRenderEosFlags() {
    audio_render_eos_.store(false);
    video_render_eos_.store(false);
}

bool Mp4Player::WaitForRenderDrain(uint32_t timeout_ms) {
    if (!av_render_) {
        return true;
    }

    const TickType_t sleep_ticks = pdMS_TO_TICKS(10);
    const uint32_t max_loops = std::max<uint32_t>(1, timeout_ms / 10);

    for (uint32_t i = 0; i < max_loops && !stop_requested_.load(); ++i) {
        if (audio_render_eos_.load() && video_render_eos_.load()) {
            return true;
        }

        av_render_fifo_stat_t audio_fifo = {};
        av_render_fifo_stat_t video_fifo = {};
        bool got_audio = (av_render_get_audio_fifo_level(av_render_, &audio_fifo) == 0);
        bool got_video = (av_render_get_video_fifo_level(av_render_, &video_fifo) == 0);

        bool audio_empty = (!got_audio) ||
                           (audio_fifo.q_num <= 0 && audio_fifo.render_q_num <= 0 &&
                            audio_fifo.data_size <= 0 && audio_fifo.render_data_size <= 0 &&
                            audio_fifo.duration == 0);
        bool video_empty = (!got_video) ||
                           (video_fifo.q_num <= 0 && video_fifo.render_q_num <= 0 &&
                            video_fifo.data_size <= 0 && video_fifo.render_data_size <= 0 &&
                            video_fifo.duration == 0);
        if (audio_empty && video_empty) {
            return true;
        }

        vTaskDelay(sleep_ticks);
    }

    av_render_fifo_stat_t audio_fifo = {};
    av_render_fifo_stat_t video_fifo = {};
    (void)av_render_get_audio_fifo_level(av_render_, &audio_fifo);
    (void)av_render_get_video_fifo_level(av_render_, &video_fifo);
    ESP_LOGW(TAG,
             "Timeout waiting render drain: audio_eos=%d video_eos=%d a(q=%d rq=%d dur=%lu) v(q=%d rq=%d dur=%lu)",
             static_cast<int>(audio_render_eos_.load()),
             static_cast<int>(video_render_eos_.load()),
             audio_fifo.q_num,
             audio_fifo.render_q_num,
             static_cast<unsigned long>(audio_fifo.duration),
             video_fifo.q_num,
             video_fifo.render_q_num,
             static_cast<unsigned long>(video_fifo.duration));
    return false;
}

bool Mp4Player::EnsureDrawBuffer(size_t bytes) {
    if (bytes == 0) return false;
    if (video_draw_buf_ && video_draw_buf_size_ >= bytes) {
        return true;
    }
    if (video_draw_buf_) {
        heap_caps_free(video_draw_buf_);
        video_draw_buf_ = nullptr;
        video_draw_buf_size_ = 0;
    }
    video_draw_buf_ = static_cast<uint8_t*>(
        heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (!video_draw_buf_) {
        ESP_LOGE(TAG, "Failed to alloc draw buffer (%u)", static_cast<unsigned>(bytes));
        return false;
    }
    video_draw_buf_size_ = bytes;
    return true;
}

bool Mp4Player::DrawFrameToLcd(const uint8_t* frame,
                               uint16_t width,
                               uint16_t height,
                               av_render_video_frame_type_t frame_type) {
    if (!lcd_panel_ || !frame || width == 0 || height == 0) return false;

    // `width/height` here come from decoder output frame info (coded size), not always
    // the visible display size. Clamp/crop to extractor resolution when available.
    uint16_t frame_w = width;
    uint16_t frame_h = height;
    if (expected_video_width_ > 0 && expected_video_width_ <= width) {
        frame_w = expected_video_width_;
    }
    if (expected_video_height_ > 0 && expected_video_height_ <= height) {
        frame_h = expected_video_height_;
    }

    const uint16_t draw_w = std::min(frame_w, lcd_width_);
    const uint16_t draw_h = std::min(frame_h, lcd_height_);

    const int x_offset = (lcd_width_ - draw_w) / 2;
    const int y_offset = (lcd_height_ - draw_h) / 2;
    const bool swap_bytes = (frame_type == AV_RENDER_VIDEO_RAW_TYPE_RGB565);

    const size_t chunk_bytes = static_cast<size_t>(draw_w) * MP4_BUFFER_LINES * sizeof(uint16_t);
    if (!EnsureDrawBuffer(chunk_bytes)) {
        return false;
    }

    const size_t src_stride = static_cast<size_t>(width) * sizeof(uint16_t);
    const size_t row_bytes = static_cast<size_t>(draw_w) * sizeof(uint16_t);

    for (uint16_t row = 0; row < draw_h; row += MP4_BUFFER_LINES) {
        const uint16_t row_end = std::min(static_cast<uint16_t>(row + MP4_BUFFER_LINES), draw_h);
        const uint16_t rows_to_copy = row_end - row;

        const uint8_t* src = frame + static_cast<size_t>(row) * src_stride;
        uint8_t* dst = video_draw_buf_;

        for (uint16_t r = 0; r < rows_to_copy; ++r) {
            memcpy(dst, src, row_bytes);
            if (swap_bytes) {
                for (size_t i = 0; i < row_bytes; i += 2) {
                    std::swap(dst[i], dst[i + 1]);
                }
            }
            src += src_stride;
            dst += row_bytes;
        }

        esp_err_t err = esp_lcd_panel_draw_bitmap(lcd_panel_,
            x_offset, y_offset + row,
            x_offset + draw_w, y_offset + row_end,
            video_draw_buf_);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "draw_bitmap failed: %s", esp_err_to_name(err));
            return false;
        }
    }
    return true;
}

bool Mp4Player::DrawFrameToCanvas(const uint8_t* frame,
                                  uint16_t width,
                                  uint16_t height,
                                  av_render_video_frame_type_t frame_type) {
#if defined(HAVE_LVGL) && HAVE_LVGL
    if (!display_ || !video_canvas_ || !canvas_buf_ || !frame || width == 0 || height == 0) {
        return false;
    }

    // Same rule as LCD path: canvas draw uses decoder stride, but visible area follows
    // extractor-provided display size to avoid showing padded H264 lines.
    uint16_t frame_w = width;
    uint16_t frame_h = height;
    if (expected_video_width_ > 0 && expected_video_width_ <= width) {
        frame_w = expected_video_width_;
    }
    if (expected_video_height_ > 0 && expected_video_height_ <= height) {
        frame_h = expected_video_height_;
    }

    const uint16_t draw_w = std::min(frame_w, lcd_width_);
    const uint16_t draw_h = std::min(frame_h, lcd_height_);
    const int x_offset = (lcd_width_ - draw_w) / 2;
    const int y_offset = (lcd_height_ - draw_h) / 2;

    const size_t frame_bytes = static_cast<size_t>(lcd_width_) * lcd_height_ * sizeof(uint16_t);
    memset(canvas_buf_, 0, frame_bytes);

    const size_t src_stride = static_cast<size_t>(width) * sizeof(uint16_t);
    const size_t row_bytes = static_cast<size_t>(draw_w) * sizeof(uint16_t);

    for (uint16_t row = 0; row < draw_h; ++row) {
        const uint8_t* src = frame + static_cast<size_t>(row) * src_stride;
        uint8_t* dst = reinterpret_cast<uint8_t*>(
            canvas_buf_ + static_cast<size_t>(y_offset + row) * lcd_width_ + x_offset);

        memcpy(dst, src, row_bytes);

        if (frame_type == AV_RENDER_VIDEO_RAW_TYPE_RGB565_BE) {
            for (size_t i = 0; i < row_bytes; i += 2) {
                std::swap(dst[i], dst[i + 1]);
            }
        }
    }

    {
        DisplayLockGuard lock(display_);
        lv_obj_invalidate(static_cast<lv_obj_t*>(video_canvas_));
    }
    return true;
#else
    (void)frame;
    (void)width;
    (void)height;
    (void)frame_type;
    return false;
#endif
}

void Mp4Player::CreateVideoCanvas() {
#if defined(HAVE_LVGL) && HAVE_LVGL
    if (!display_) {
        ESP_LOGW(TAG, "Cannot create MP4 canvas: no Display* provided");
        return;
    }

    DestroyVideoCanvas();

    const size_t buf_size = static_cast<size_t>(lcd_width_) * lcd_height_ * sizeof(uint16_t);
    canvas_buf_ = static_cast<uint16_t*>(
        heap_caps_malloc(buf_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (!canvas_buf_) {
        ESP_LOGE(TAG, "Failed to allocate MP4 canvas buffer (%u)", static_cast<unsigned>(buf_size));
        return;
    }
    memset(canvas_buf_, 0, buf_size);

    {
        DisplayLockGuard lock(display_);
        lv_obj_t* canvas = lv_canvas_create(lv_scr_act());
        lv_canvas_set_buffer(canvas, canvas_buf_, lcd_width_, lcd_height_, LV_COLOR_FORMAT_RGB565);
        lv_obj_set_pos(canvas, 0, 0);
        lv_obj_set_size(canvas, lcd_width_, lcd_height_);
        lv_canvas_fill_bg(canvas, lv_color_black(), LV_OPA_COVER);
        lv_obj_move_foreground(canvas);
        video_canvas_ = canvas;
    }
    ESP_LOGI(TAG, "MP4 canvas created: %ux%u", static_cast<unsigned>(lcd_width_), static_cast<unsigned>(lcd_height_));
#endif
}

void Mp4Player::DestroyVideoCanvas() {
#if defined(HAVE_LVGL) && HAVE_LVGL
    if (video_canvas_ && display_) {
        DisplayLockGuard lock(display_);
        lv_obj_del(static_cast<lv_obj_t*>(video_canvas_));
    }
#endif
    video_canvas_ = nullptr;
    if (canvas_buf_) {
        heap_caps_free(canvas_buf_);
        canvas_buf_ = nullptr;
    }
}

void Mp4Player::OutputPcmToCodec(const int16_t* pcm, size_t samples, int channels) {
    if (!audio_codec_ || !pcm || samples == 0) return;

    int dst_ch = audio_codec_->output_channels();
    if (dst_ch <= 0) dst_ch = channels;

    std::vector<int16_t> out;
    if (dst_ch == channels) {
        out.assign(pcm, pcm + samples);
    } else if (channels == 1 && dst_ch == 2) {
        out.reserve(samples * 2);
        for (size_t i = 0; i < samples; ++i) {
            int16_t v = ClampToInt16(static_cast<float>(pcm[i]) * volume_factor_);
            out.push_back(v);
            out.push_back(v);
        }
    } else if (channels == 2 && dst_ch == 1) {
        out.reserve(samples / 2);
        for (size_t i = 0; i + 1 < samples; i += 2) {
            float mixed = (static_cast<float>(pcm[i]) + pcm[i + 1]) * 0.5f * volume_factor_;
            out.push_back(ClampToInt16(mixed));
        }
    } else {
        out.assign(pcm, pcm + samples);
    }

    if (dst_ch == channels && volume_factor_ != 1.0f) {
        for (auto& s : out) {
            s = ClampToInt16(static_cast<float>(s) * volume_factor_);
        }
    }

    audio_codec_->OutputData(out);
}

void Mp4Player::SetState(Mp4PlayerState new_state) {
    Mp4PlayerState old_state = state_.exchange(new_state);
    if (old_state == new_state) return;

    if (render_mode_ == Mp4RenderMode::LvglCanvas) {
        if (new_state == Mp4PlayerState::Loading) {
            CreateVideoCanvas();
        }
        if (new_state == Mp4PlayerState::Idle || new_state == Mp4PlayerState::Error) {
            DestroyVideoCanvas();
        }
    }

    if (state_callback_) {
        state_callback_(old_state, new_state);
    }
}

void Mp4Player::SetRenderMode(Mp4RenderMode mode) {
    if (mode == render_mode_) {
        return;
    }

    Mp4RenderMode old = render_mode_;
    render_mode_ = mode;

    if (state_.load() == Mp4PlayerState::Loading ||
        state_.load() == Mp4PlayerState::Playing ||
        state_.load() == Mp4PlayerState::Paused) {
        if (mode == Mp4RenderMode::LvglCanvas) {
            CreateVideoCanvas();
        } else {
            DestroyVideoCanvas();
        }
    }

    ESP_LOGI(TAG, "Render mode: %s -> %s",
             old == Mp4RenderMode::DirectLcd ? "DirectLcd" : "LvglCanvas",
             mode == Mp4RenderMode::DirectLcd ? "DirectLcd" : "LvglCanvas");
}

std::string Mp4Player::BuildFullPath(const std::string& filename) const {
    if (filename.empty()) return {};
    if (filename.front() == '/' || filename.find(':') != std::string::npos) return filename;
    return mount_point_ + "/" + mp4_directory_ + "/" + filename;
}

bool Mp4Player::IsSupportedMp4Name(const std::string& name) const {
    if (name.size() < 5) return false;
    std::string ext = name.substr(name.size() - 4);
    std::transform(ext.begin(), ext.end(), ext.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return ext == ".mp4";
}

av_render_audio_codec_t Mp4Player::MapAudioCodec(esp_extractor_format_t format) const {
    switch (format) {
        case ESP_EXTRACTOR_AUDIO_FORMAT_AAC:
            return AV_RENDER_AUDIO_CODEC_AAC;
        case ESP_EXTRACTOR_AUDIO_FORMAT_MP3:
            return AV_RENDER_AUDIO_CODEC_MP3;
        case ESP_EXTRACTOR_AUDIO_FORMAT_FLAC:
            return AV_RENDER_AUDIO_CODEC_FLAC;
        default:
            return AV_RENDER_AUDIO_CODEC_NONE;
    }
}

av_render_video_codec_t Mp4Player::MapVideoCodec(esp_extractor_format_t format) const {
    switch (format) {
        case ESP_EXTRACTOR_VIDEO_FORMAT_H264:
            return AV_RENDER_VIDEO_CODEC_H264;
        case ESP_EXTRACTOR_VIDEO_FORMAT_MJPEG:
            return AV_RENDER_VIDEO_CODEC_MJPEG;
        default:
            return AV_RENDER_VIDEO_CODEC_NONE;
    }
}

void Mp4Player::PlaybackTaskLoop() {
    /* Max consecutive WAITING_OUTPUT ticks before giving up (~5 seconds). */
    constexpr int kWaitingOutputMax = 5000;
    int consecutive_waiting = 0;

    while (!stop_requested_.load()) {
        if (paused_.load()) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        esp_extractor_frame_info_t frame = {};
        esp_extractor_err_t err = esp_extractor_read_frame(extractor_, &frame);
        if (err == ESP_EXTRACTOR_ERR_EOS) {
            ESP_LOGI(TAG, "Extractor EOS");
            break;
        }
        if (err == ESP_EXTRACTOR_ERR_SKIPPED) {
            consecutive_waiting = 0;
            std::lock_guard<std::mutex> lock(stats_mutex_);
            stats_.dropped_frames++;
            continue;
        }
        if (err == ESP_EXTRACTOR_ERR_WAITING_OUTPUT) {
            if (++consecutive_waiting >= kWaitingOutputMax) {
                ESP_LOGW(TAG, "Extractor WAITING_OUTPUT timeout (%d ms), force exit",
                         kWaitingOutputMax);
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }
        if (err != ESP_EXTRACTOR_ERR_OK) {
            ESP_LOGW(TAG, "read_frame err: %d", err);
            break;
        }

        consecutive_waiting = 0;

        bool is_eos_frame = (frame.frame_flag & EXTRACTOR_FRAME_FLAG_EOS) != 0;
        bool ok = true;
        if (frame.frame_buffer) {
            if (frame.stream_type == ESP_EXTRACTOR_STREAM_TYPE_AUDIO) {
                ok = ProcessAudioFrame(frame);
            } else if (frame.stream_type == ESP_EXTRACTOR_STREAM_TYPE_VIDEO) {
                ok = ProcessVideoFrame(frame);
            }
        }
        esp_extractor_release_frame(extractor_, &frame);

        if (!ok) {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            stats_.dropped_frames++;
        }

        /* Always break on EOS regardless of whether the frame was delivered. */
        if (is_eos_frame) {
            if (ok) {
                ESP_LOGI(TAG, "EOS frame received, ending loop");
                break;
            }
            ESP_LOGW(TAG, "EOS frame was not queued, continue reading until extractor EOS");
        }
    }

    if (!stop_requested_.load() && PushEos()) {
        // Allow av_render to consume all queued packets and emit stream EOS.
        (void)WaitForRenderDrain(8000);
    }

    /* Stop av_render's internal decode/render threads BEFORE invoking any
     * callbacks or state transitions.  After extractor closes, av_render
     * continues rendering queued frames (observed in logs as "Video pts:…"
     * messages), holding the LCD during that time.  Calling av_render_reset()
     * here sends CLOSE to all four internal threads and waits for them to
     * exit, so no render callback (VideoRenderWrite → DrawFrameToLcd) is in
     * flight when state_callback_ runs below. */
    if (av_render_ && !stop_requested_.load()) {
        ESP_LOGI(TAG, "Waiting for av_render threads to exit…");
        av_render_reset(av_render_);
        ESP_LOGI(TAG, "av_render_reset done");
        /* Delay 500ms to ensure all av_render internal cleanup callbacks
         * (VideoRenderClose, AudioRenderClose, etc.) finish before calling
         * state_callback_ to avoid callback blocking. */
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    bool should_repeat = false;
    std::string ended_path = current_file_path_;
    if (!stop_requested_.load()) {
        should_repeat = end_callback_ ? end_callback_(ended_path) : false;
    }

    playback_task_running_.store(false);
    CleanupPlaybackResources();
    current_file_path_.clear();
    stop_requested_.store(false);
    paused_.store(false);
    if (!should_repeat) {
        SetState(Mp4PlayerState::Idle);
    }
    ESP_LOGI(TAG, "Playback finished: %s", ended_path.c_str());
    vTaskDelete(NULL);
}

int Mp4Player::AvRenderEventCb(av_render_event_t event, void* ctx) {
    auto* self = static_cast<Mp4Player*>(ctx);
    if (!self) {
        return -1;
    }

    if (event == AV_RENDER_EVENT_AUDIO_EOS) {
        self->audio_render_eos_.store(true);
    } else if (event == AV_RENDER_EVENT_VIDEO_EOS) {
        self->video_render_eos_.store(true);
    }

    if (event == AV_RENDER_EVENT_AUDIO_DECODE_ERR || event == AV_RENDER_EVENT_VIDEO_DECODE_ERR) {
        ESP_LOGE(TAG, "av_render decode error event=%d", static_cast<int>(event));
        self->SetState(Mp4PlayerState::Error);
    }
    return 0;
}

audio_render_handle_t Mp4Player::AudioRenderInit(void* cfg, int cfg_size) {
    if (!cfg || cfg_size != static_cast<int>(sizeof(RendererInitCfg))) {
        return nullptr;
    }
    auto* init_cfg = static_cast<RendererInitCfg*>(cfg);
    if (!init_cfg->owner) {
        return nullptr;
    }

    auto* ctx = new AudioRenderCtx();
    ctx->owner = init_cfg->owner;
    return ctx;
}

int Mp4Player::AudioRenderOpen(audio_render_handle_t render, av_render_audio_frame_info_t* info) {
    auto* ctx = static_cast<AudioRenderCtx*>(render);
    if (!ctx || !ctx->owner || !info) {
        return -1;
    }
    ctx->info = *info;

    if (ctx->owner->audio_codec_) {
        ctx->owner->audio_codec_->EnableOutput(true);
        ctx->owner->audio_codec_->SetOutputSampleRate(static_cast<int>(info->sample_rate));
    }
    return 0;
}

int Mp4Player::AudioRenderWrite(audio_render_handle_t render, av_render_audio_frame_t* audio_data) {
    auto* ctx = static_cast<AudioRenderCtx*>(render);
    if (!ctx || !ctx->owner || !audio_data || !audio_data->data || audio_data->size <= 0) {
        return -1;
    }
    if (ctx->info.bits_per_sample != 16 || ctx->info.channel == 0) {
        ESP_LOGW(TAG, "Unsupported PCM format: bits=%u ch=%u",
                 static_cast<unsigned>(ctx->info.bits_per_sample),
                 static_cast<unsigned>(ctx->info.channel));
        return -1;
    }

    auto* pcm = reinterpret_cast<int16_t*>(audio_data->data);
    const size_t samples = static_cast<size_t>(audio_data->size) / sizeof(int16_t);

    if (ctx->owner->audio_callback_) {
        ctx->owner->audio_callback_(pcm, samples, ctx->info.channel);
    }
    ctx->owner->OutputPcmToCodec(pcm, samples, ctx->info.channel);
    return 0;
}

int Mp4Player::AudioRenderGetLatency(audio_render_handle_t render, uint32_t* latency) {
    if (!render || !latency) {
        return -1;
    }
    *latency = 0;
    return 0;
}

int Mp4Player::AudioRenderGetFrameInfo(audio_render_handle_t render, av_render_audio_frame_info_t* info) {
    auto* ctx = static_cast<AudioRenderCtx*>(render);
    if (!ctx || !info) {
        return -1;
    }
    *info = ctx->info;
    return 0;
}

int Mp4Player::AudioRenderSetSpeed(audio_render_handle_t render, float speed) {
    if (!render) {
        return -1;
    }
    (void)speed;
    return 0;
}

int Mp4Player::AudioRenderClose(audio_render_handle_t render) {
    auto* ctx = static_cast<AudioRenderCtx*>(render);
    if (!ctx || !ctx->owner) {
        return -1;
    }

    if (ctx->owner->audio_codec_) {
        ctx->owner->audio_codec_->SetOutputSampleRate(-1);
    }
    ESP_LOGI(TAG, "AudioRenderClose called");
    return 0;
}

void Mp4Player::AudioRenderDeinit(audio_render_handle_t render) {
    auto* ctx = static_cast<AudioRenderCtx*>(render);
    delete ctx;
}

video_render_handle_t Mp4Player::VideoRenderOpen(void* cfg, int size) {
    if (!cfg || size != static_cast<int>(sizeof(RendererInitCfg))) {
        return nullptr;
    }
    auto* init_cfg = static_cast<RendererInitCfg*>(cfg);
    if (!init_cfg->owner) {
        return nullptr;
    }

    auto* ctx = new VideoRenderCtx();
    ctx->owner = init_cfg->owner;
    return ctx;
}

bool Mp4Player::VideoRenderFormatSupported(video_render_handle_t render, av_render_video_frame_type_t type) {
    if (!render) {
        return false;
    }
    return type == AV_RENDER_VIDEO_RAW_TYPE_RGB565 || type == AV_RENDER_VIDEO_RAW_TYPE_RGB565_BE;
}

int Mp4Player::VideoRenderSetFrameInfo(video_render_handle_t render, av_render_video_frame_info_t* info) {
    auto* ctx = static_cast<VideoRenderCtx*>(render);
    if (!ctx || !ctx->owner || !info) {
        return -1;
    }
    if (!VideoRenderFormatSupported(render, info->type)) {
        return -1;
    }

    ctx->info = *info;

    // av_render passes decoder frame size (often aligned). For stats/log we also expose
    // the intended display size from extractor to make runtime diagnostics less confusing.
    uint16_t out_w = info->width;
    uint16_t out_h = info->height;
    if (ctx->owner->expected_video_width_ > 0 && ctx->owner->expected_video_width_ <= info->width) {
        out_w = ctx->owner->expected_video_width_;
    }
    if (ctx->owner->expected_video_height_ > 0 && ctx->owner->expected_video_height_ <= info->height) {
        out_h = ctx->owner->expected_video_height_;
    }

    {
        std::lock_guard<std::mutex> lock(ctx->owner->stats_mutex_);
        ctx->owner->stats_.video_width = out_w;
        ctx->owner->stats_.video_height = out_h;
        ctx->owner->stats_.video_fps = info->fps;
        ESP_LOGI(TAG, "Video frame info decoded=%ux%u display=%ux%u fps=%u type=%d",
                 static_cast<unsigned>(info->width),
                 static_cast<unsigned>(info->height),
                 static_cast<unsigned>(out_w),
                 static_cast<unsigned>(out_h),
                 static_cast<unsigned>(info->fps),
                 static_cast<int>(info->type));
    }
    return 0;
}

int Mp4Player::VideoRenderGetFrameBuffer(video_render_handle_t render, av_render_frame_buffer_t* frame_buffer) {
    (void)render;
    (void)frame_buffer;
    return -1;
}

int Mp4Player::VideoRenderWrite(video_render_handle_t render, av_render_video_frame_t* video_data) {
    auto* ctx = static_cast<VideoRenderCtx*>(render);
    if (!ctx || !ctx->owner || !video_data || !video_data->data || video_data->size <= 0) {
        return -1;
    }

    if (ctx->owner->frame_callback_) {
        ctx->owner->frame_callback_(reinterpret_cast<uint16_t*>(video_data->data),
                                    ctx->info.width,
                                    ctx->info.height);
    }

    bool drawn = false;
    if (ctx->owner->render_mode_ == Mp4RenderMode::LvglCanvas) {
        drawn = ctx->owner->DrawFrameToCanvas(video_data->data,
                                              ctx->info.width,
                                              ctx->info.height,
                                              ctx->info.type);
        if (!drawn) {
            drawn = ctx->owner->DrawFrameToLcd(video_data->data,
                                               ctx->info.width,
                                               ctx->info.height,
                                               ctx->info.type);
        }
    } else {
        drawn = ctx->owner->DrawFrameToLcd(video_data->data,
                                           ctx->info.width,
                                           ctx->info.height,
                                           ctx->info.type);
    }

    return drawn
               ? 0
               : -1;
}

int Mp4Player::VideoRenderGetLatency(video_render_handle_t render, uint32_t* latency) {
    if (!render || !latency) {
        return -1;
    }
    *latency = 0;
    return 0;
}

int Mp4Player::VideoRenderGetFrameInfo(video_render_handle_t render, av_render_video_frame_info_t* info) {
    auto* ctx = static_cast<VideoRenderCtx*>(render);
    if (!ctx || !info) {
        return -1;
    }
    *info = ctx->info;
    return 0;
}

int Mp4Player::VideoRenderClear(video_render_handle_t render) {
    (void)render;
    ESP_LOGI(TAG, "VideoRenderClear called");
    return 0;
}

int Mp4Player::VideoRenderClose(video_render_handle_t render) {
    auto* ctx = static_cast<VideoRenderCtx*>(render);
    delete ctx;
    ESP_LOGI(TAG, "VideoRenderClose called");
    return 0;
}
