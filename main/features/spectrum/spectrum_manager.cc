/**
 * @file spectrum_manager.cc
 * @brief Implementation of the high-level spectrum visualization façade.
 *
 * Manages the FreeRTOS task that periodically reads PCM data, runs the
 * FFT analyzer, and refreshes the LVGL bar renderer.  All LVGL access
 * is properly guarded by lvgl_port_lock / lvgl_port_unlock.
 *
 * Contributors: Xiaozhi AI-IoT Vietnam Team
 */
#include "spectrum_manager.h"

#include <cstring>
#include <esp_heap_caps.h>
#include <esp_log.h>
#include <esp_lvgl_port.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char* TAG = "SpectrumManager";

namespace spectrum {

// ---------------------------------------------------------------------------
// Construction / Destruction
// ---------------------------------------------------------------------------

SpectrumManager::SpectrumManager(const SpectrumConfig& config)
    : config_(config),
      analyzer_(std::make_unique<SpectrumAnalyzer>(config)),
      renderer_(std::make_unique<SpectrumRenderer>(config)) {}

SpectrumManager::~SpectrumManager() {
    Stop();
    ReleaseAudioBuffer();
}

// ---------------------------------------------------------------------------
// Start — initialize sub-components, create canvas, spawn task
// ---------------------------------------------------------------------------

bool SpectrumManager::Start(lv_obj_t* parent) {
    if (task_handle_) {
        ESP_LOGW(TAG, "Already running");
        return true;
    }

    // Initialize the FFT analyzer
    if (!analyzer_->Initialize()) {
        ESP_LOGE(TAG, "Analyzer initialization failed");
        return false;
    }

    // Create LVGL canvas (must hold the LVGL lock)
    if (lvgl_port_lock(3000)) {
        bool ok = renderer_->CreateCanvas(parent);
        lvgl_port_unlock();
        if (!ok) {
            ESP_LOGE(TAG, "Renderer canvas creation failed");
            analyzer_->Deinitialize();
            return false;
        }
    } else {
        ESP_LOGE(TAG, "Failed to acquire LVGL lock for canvas creation");
        analyzer_->Deinitialize();
        return false;
    }

    // Bail out if Stop() was called concurrently before the task was created
    if (should_stop_) {
        if (lvgl_port_lock(1000)) {
            renderer_->DestroyCanvas();
            lvgl_port_unlock();
        }
        analyzer_->Deinitialize();
        return false;
    }

    // Spawn the background processing task
    should_stop_ = false;
    BaseType_t ret = xTaskCreatePinnedToCore(
        TaskWrapper,
        "spectrum",
        config_.task_stack_size,
        this,
        config_.task_priority,
        &task_handle_,
        config_.task_core);

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create spectrum task");
        if (lvgl_port_lock(1000)) {
            renderer_->DestroyCanvas();
            lvgl_port_unlock();
        }
        analyzer_->Deinitialize();
        task_handle_ = nullptr;
        return false;
    }

    ESP_LOGI(TAG, "Started (task on core %d)", config_.task_core);
    return true;
}

// ---------------------------------------------------------------------------
// Stop — signal task, wait, clean up
// ---------------------------------------------------------------------------

void SpectrumManager::Stop() {
    ESP_LOGI(TAG, "Stopping...");

    // Signal the task to exit
    if (task_handle_) {
        should_stop_ = true;

        // Wait for graceful exit (max 1 s)
        int wait = 0;
        while (task_handle_ && wait < 100) {
            vTaskDelay(pdMS_TO_TICKS(10));
            wait++;
        }
        if (task_handle_) {
            ESP_LOGW(TAG, "Task did not stop gracefully — force deleting");
            vTaskDelete(task_handle_);
            task_handle_ = nullptr;
        } else {
            ESP_LOGI(TAG, "Task stopped gracefully");
        }
    }

    // Destroy renderer canvas (LVGL lock required)
    if (lvgl_port_lock(3000)) {
        renderer_->DestroyCanvas();
        lvgl_port_unlock();
    }

    // Deinitialize analyzer
    analyzer_->Deinitialize();

    ESP_LOGI(TAG, "Stopped");
}

// ---------------------------------------------------------------------------
// Audio buffer management
// ---------------------------------------------------------------------------

int16_t* SpectrumManager::AllocateAudioBuffer(size_t bytes) {
    if (!pcm_input_buffer_) {
        pcm_input_buffer_ = static_cast<int16_t*>(
            heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM));
        if (pcm_input_buffer_) {
            pcm_buffer_size_ = bytes;
            memset(pcm_input_buffer_, 0, bytes);
        } else {
            ESP_LOGE(TAG, "Failed to allocate PCM buffer (%u bytes)",
                     static_cast<unsigned>(bytes));
        }
    }
    return pcm_input_buffer_;
}

void SpectrumManager::FeedAudioData(const int16_t* data, size_t bytes) {
    if (pcm_input_buffer_ && data) {
        size_t copy_bytes = (bytes < pcm_buffer_size_) ? bytes : pcm_buffer_size_;
        memcpy(pcm_input_buffer_, data, copy_bytes);
    }
}

void SpectrumManager::ReleaseAudioBuffer() {
    if (pcm_input_buffer_) {
        heap_caps_free(pcm_input_buffer_);
        pcm_input_buffer_ = nullptr;
        pcm_buffer_size_  = 0;
    }
}

// ---------------------------------------------------------------------------
// Periodic callback
// ---------------------------------------------------------------------------

void SpectrumManager::SetPeriodicCallback(PeriodicCallback cb, int interval_ms) {
    periodic_callback_    = std::move(cb);
    periodic_interval_ms_ = interval_ms;
}

// ---------------------------------------------------------------------------
// FreeRTOS task
// ---------------------------------------------------------------------------

void SpectrumManager::TaskWrapper(void* arg) {
    static_cast<SpectrumManager*>(arg)->TaskLoop();
}

void SpectrumManager::TaskLoop() {
    ESP_LOGI(TAG, "Task started");

    const TickType_t display_interval = pdMS_TO_TICKS(1000 / config_.refresh_rate_hz);
    const TickType_t audio_interval   = pdMS_TO_TICKS(config_.audio_process_interval_ms);
    const TickType_t periodic_interval = pdMS_TO_TICKS(periodic_interval_ms_);

    TickType_t last_display_time  = xTaskGetTickCount();
    TickType_t last_audio_time    = xTaskGetTickCount();
    TickType_t last_periodic_time = xTaskGetTickCount();

    bool spectrum_ready = false;

    while (!should_stop_) {
        TickType_t now = xTaskGetTickCount();

        // ---- Audio processing ----
        if (now - last_audio_time >= audio_interval) {
            if (pcm_input_buffer_) {
                int sample_count = config_.audio_frame_size;
                spectrum_ready = analyzer_->ProcessPcmFrame(
                    pcm_input_buffer_, sample_count);
            } else {
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            last_audio_time = now;
        }

        // ---- Display refresh ----
        if (spectrum_ready && (now - last_display_time >= display_interval)) {
            if (lvgl_port_lock(30)) {
                renderer_->Render(analyzer_->GetPowerSpectrum(),
                                  analyzer_->GetSpectrumSize());
                renderer_->Invalidate();
                lvgl_port_unlock();
            }
            spectrum_ready    = false;
            last_display_time = now;
        }

        // ---- Optional periodic callback ----
        if (periodic_callback_ && (now - last_periodic_time >= periodic_interval)) {
            if (lvgl_port_lock(30)) {
                periodic_callback_();
                lvgl_port_unlock();
            }
            last_periodic_time = now;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    ESP_LOGI(TAG, "Task exiting");
    task_handle_ = nullptr;
    vTaskDelete(NULL);
}

} // namespace spectrum
