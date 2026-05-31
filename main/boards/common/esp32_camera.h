#pragma once
#include "sdkconfig.h"

#ifndef CONFIG_IDF_TARGET_ESP32
#include <lvgl.h>
#include <thread>
#include <memory>
#include <vector>
#include <esp_timer.h>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include "camera.h"
#include "jpg/image_to_jpeg.h"
#include "esp_video_init.h"
#include "esp_camera.h"

struct JpegChunk {
    uint8_t* data;
    size_t len;
};

class CameraActivityGuard;

class Esp32Camera : public Camera {
private:
    struct FrameBuffer {
        uint8_t *data = nullptr;
        size_t len = 0;
        uint16_t width = 0;
        uint16_t height = 0;
        v4l2_pix_fmt_t format = 0;
    } frame_;
    v4l2_pix_fmt_t sensor_format_ = 0;
#ifdef CONFIG_XIAOZHI_ENABLE_ROTATE_CAMERA_IMAGE
    uint16_t sensor_width_ = 0;
    uint16_t sensor_height_ = 0;
#endif  // CONFIG_XIAOZHI_ENABLE_ROTATE_CAMERA_IMAGE
    int video_fd_ = -1;
    bool streaming_on_ = false;
    struct MmapBuffer { void *start = nullptr; size_t length = 0; };
    std::vector<MmapBuffer> mmap_buffers_;
    std::string explain_url_;
    std::string explain_token_;
    std::thread encoder_thread_;

    // Legacy driver variables
    bool use_legacy_ = false;
    bool swap_bytes_enabled_ = true;
    camera_fb_t *current_fb_ = nullptr;
    uint8_t *encode_buf_ = nullptr;
    size_t encode_buf_size_ = 0;

    // Configuration & state for lazy initialization
    camera_config_t legacy_config_ = {};
    esp_video_init_config_t video_config_ = {};
    bool initialized_ = false;
    bool hmirror_ = false;
    bool vflip_ = false;
    esp_timer_handle_t deinit_timer_ = nullptr;
    friend class CameraActivityGuard;

    bool InitHardware();
    void DeinitHardware();
    void StartDeinitTimer();
    void StopDeinitTimer();
    static void InactivityTimerCallback(void* arg);

public:
    Esp32Camera(const esp_video_init_config_t& config);
    Esp32Camera(const camera_config_t& config);
    ~Esp32Camera();

    virtual void SetExplainUrl(const std::string& url, const std::string& token) override;
    virtual bool Capture() override;
    virtual bool IsReady() override;
    // 翻转控制函数
    virtual bool SetHMirror(bool enabled) override;
    virtual bool SetVFlip(bool enabled) override;
    virtual bool SetSwapBytes(bool enabled) override;
    virtual std::string Explain(const std::string& question) override;
};

#endif // ndef CONFIG_IDF_TARGET_ESP32