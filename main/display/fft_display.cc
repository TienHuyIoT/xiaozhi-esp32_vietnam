#include "fft_display.h"
#include "display.h"
#include "board.h"

#include <esp_log.h>
#include <esp_heap_caps.h>
#include <esp_lvgl_port.h>
#include <algorithm>

#define TAG "FFTDisplay"

// DisplayAdapterLockGuard Implementation
DisplayAdapterLockGuard::DisplayAdapterLockGuard(DisplayAdapter* adapter) 
    : adapter_(adapter), locked_(false) {
    
    if (!adapter_) {
        ESP_LOGE(TAG, "DisplayAdapter is null in DisplayAdapterLockGuard");
        return;
    }
    
    // Check adapter type and apply appropriate locking
#if defined(HAVE_LVGL) || __has_include(<lvgl.h>)
    auto lcd_adapter = dynamic_cast<LCDDisplayAdapter*>(adapter_);
    if (lcd_adapter) {
        // LCD adapter uses LVGL locking
        locked_ = lvgl_port_lock(30000);
        if (!locked_) {
            ESP_LOGE(TAG, "Failed to lock LVGL port for LCD adapter");
        }
        return;
    }
#endif
    
    auto oled_adapter = dynamic_cast<OLEDDisplayAdapter*>(adapter_);
    if (oled_adapter) {
        // OLED adapter doesn't need locking (single-threaded framebuffer access)
        locked_ = true;
        return;
    }
    
    // Unknown adapter type, assume no locking needed
    locked_ = true;
    ESP_LOGW(TAG, "Unknown adapter type, proceeding without locking");
}

DisplayAdapterLockGuard::~DisplayAdapterLockGuard() {
    if (!locked_ || !adapter_) return;
    
#if defined(HAVE_LVGL) || __has_include(<lvgl.h>)
    auto lcd_adapter = dynamic_cast<LCDDisplayAdapter*>(adapter_);
    if (lcd_adapter) {
        lvgl_port_unlock();
        return;
    }
#endif
    
    // OLED and other adapters don't need unlock
}

FFTDisplay::FFTDisplay(std::unique_ptr<DisplayAdapter> adapter)
    : adapter_(std::move(adapter))
    , current_effect_(FFTEffect::SPECTRUM_BARS)
    , fft_real_(nullptr)
    , fft_imag_(nullptr)
    , hanning_window_(nullptr)
    , power_spectrum_(nullptr)
    , avg_power_spectrum_(nullptr)
    , bar_heights_(nullptr)
    , audio_data_(nullptr)
    , frame_audio_data_(nullptr)
    , audio_buffer_size_(0)
    , fft_task_handle_(nullptr)
    , fft_task_should_stop_(false)
    , is_running_(false)
    , last_fft_update_(0)
    , fft_data_ready_(false) {
    
    if (!adapter_) {
        ESP_LOGE(TAG, "DisplayAdapter is null");
        return;
    }
    
    display_width_ = adapter_->getWidth();
    display_height_ = adapter_->getHeight();
    is_color_display_ = adapter_->isColor();
    
    ESP_LOGI(TAG, "FFT Display initialized - %dx%d, %s", 
             display_width_, display_height_, 
             is_color_display_ ? "Color" : "Monochrome");
    
    initializeFFTBuffers();
}

FFTDisplay::~FFTDisplay() {
    stop();
    cleanupFFTBuffers();
    ESP_LOGI(TAG, "FFT Display destroyed");
}

void FFTDisplay::initializeFFTBuffers() {
    // Allocate FFT processing buffers
    fft_real_ = (float*)heap_caps_malloc(FFT_SIZE * sizeof(float), MALLOC_CAP_SPIRAM);
    fft_imag_ = (float*)heap_caps_malloc(FFT_SIZE * sizeof(float), MALLOC_CAP_SPIRAM);
    hanning_window_ = (float*)heap_caps_malloc(FFT_SIZE * sizeof(float), MALLOC_CAP_SPIRAM);
    power_spectrum_ = (float*)heap_caps_malloc((FFT_SIZE / 2) * sizeof(float), MALLOC_CAP_SPIRAM);
    avg_power_spectrum_ = (float*)heap_caps_malloc((FFT_SIZE / 2) * sizeof(float), MALLOC_CAP_SPIRAM);
    bar_heights_ = (int*)heap_caps_malloc(FFT_BARS_COUNT * sizeof(int), MALLOC_CAP_SPIRAM);
    
    if (!fft_real_ || !fft_imag_ || !hanning_window_ || !power_spectrum_ || !avg_power_spectrum_ || !bar_heights_) {
        ESP_LOGE(TAG, "Failed to allocate FFT buffers");
        cleanupFFTBuffers();
        return;
    }
    
    // Initialize Hanning window
    for (int i = 0; i < FFT_SIZE; i++) {
        hanning_window_[i] = 0.5f * (1.0f - cosf(2.0f * M_PI * i / (FFT_SIZE - 1)));
    }
    
    // Initialize average power spectrum with default values
    for (int i = 0; i < FFT_SIZE / 2; i++) {
        avg_power_spectrum_[i] = -25.0f;
    }
    
    // Initialize bar heights
    for (int i = 0; i < FFT_BARS_COUNT; i++) {
        bar_heights_[i] = 0;
    }
    
    ESP_LOGI(TAG, "FFT buffers initialized successfully");
}

void FFTDisplay::cleanupFFTBuffers() {
    if (fft_real_) {
        heap_caps_free(fft_real_);
        fft_real_ = nullptr;
    }
    if (fft_imag_) {
        heap_caps_free(fft_imag_);
        fft_imag_ = nullptr;
    }
    if (hanning_window_) {
        heap_caps_free(hanning_window_);
        hanning_window_ = nullptr;
    }
    if (power_spectrum_) {
        heap_caps_free(power_spectrum_);
        power_spectrum_ = nullptr;
    }
    if (avg_power_spectrum_) {
        heap_caps_free(avg_power_spectrum_);
        avg_power_spectrum_ = nullptr;
    }
    if (bar_heights_) {
        heap_caps_free(bar_heights_);
        bar_heights_ = nullptr;
    }
    if (audio_data_) {
        heap_caps_free(audio_data_);
        audio_data_ = nullptr;
    }
    if (frame_audio_data_) {
        heap_caps_free(frame_audio_data_);
        frame_audio_data_ = nullptr;
    }
}

bool FFTDisplay::start() {
    if (is_running_.load()) {
        ESP_LOGW(TAG, "FFT Display already running");
        return true;
    }
    
    if (!adapter_ || !fft_real_) {
        ESP_LOGE(TAG, "FFT Display not properly initialized");
        return false;
    }
    
    fft_task_should_stop_ = false;
    is_running_ = true;
    
    // Create FFT processing task
    BaseType_t result = xTaskCreate(
        fftTaskWrapper,
        "fft_display_task",
        8192,  // Stack size
        this,
        5,     // Priority
        &fft_task_handle_
    );
    
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create FFT task");
        is_running_ = false;
        return false;
    }
    
    ESP_LOGI(TAG, "FFT Display started");
    return true;
}

void FFTDisplay::stop() {
    if (!is_running_.load()) {
        return;
    }
    
    ESP_LOGI(TAG, "Stopping FFT Display");
    
    fft_task_should_stop_ = true;
    is_running_ = false;
    
    // Wait for task to finish
    if (fft_task_handle_) {
        vTaskDelete(fft_task_handle_);
        fft_task_handle_ = nullptr;
    }
    
    // Clear display
    if (adapter_) {
        adapter_->clearDisplay();
        adapter_->updateDisplay();
    }
    
    ESP_LOGI(TAG, "FFT Display stopped");
}

int16_t* FFTDisplay::createAudioDataBuffer(size_t sample_count) {
    audio_buffer_size_ = sample_count;
    
    if (audio_data_) {
        heap_caps_free(audio_data_);
    }
    if (frame_audio_data_) {
        heap_caps_free(frame_audio_data_);
    }
    
    audio_data_ = (int16_t*)heap_caps_malloc(sample_count * sizeof(int16_t), MALLOC_CAP_SPIRAM);
    frame_audio_data_ = (int16_t*)heap_caps_malloc(sample_count * sizeof(int16_t), MALLOC_CAP_SPIRAM);
    
    if (!audio_data_ || !frame_audio_data_) {
        ESP_LOGE(TAG, "Failed to allocate audio data buffers");
        return nullptr;
    }
    
    return audio_data_;
}

void FFTDisplay::updateAudioDataBuffer(int16_t* data, size_t sample_count) {
    if (!data || !frame_audio_data_ || sample_count > audio_buffer_size_) {
        return;
    }
    
    memcpy(frame_audio_data_, data, sample_count * sizeof(int16_t));
    fft_data_ready_ = true;
}

void FFTDisplay::releaseAudioDataBuffer(int16_t* buffer) {
    // Buffer cleanup is handled in destructor or when creating new buffer
}

void FFTDisplay::clearDisplay() {
    if (adapter_) {
        adapter_->clearDisplay();
    }
}

void FFTDisplay::updateDisplay() {
    if (adapter_) {
        adapter_->updateDisplay();
    }
}

void FFTDisplay::fftTaskWrapper(void* param) {
    FFTDisplay* fft_display = static_cast<FFTDisplay*>(param);
    fft_display->fftTask();
}

void FFTDisplay::fftTask() {
    const TickType_t update_interval = pdMS_TO_TICKS(50);  // 20 FPS
    TickType_t last_wake_time = xTaskGetTickCount();
    
    while (!fft_task_should_stop_.load()) {
        if (fft_data_ready_) {
            processAudioData();
            updateSpectrum();
            
            // Render based on current effect
            switch (current_effect_) {
                case FFTEffect::SPECTRUM_BARS:
                    renderSpectrumBars();
                    break;
                case FFTEffect::WAVEFORM:
                    renderWaveform();
                    break;
                case FFTEffect::CIRCULAR:
                    renderCircular();
                    break;
                case FFTEffect::WATERFALL:
                    renderWaterfall();
                    break;
            }
            // DisplayLockGuard lock(this);
            adapter_->updateDisplay();
        }
        
        vTaskDelayUntil(&last_wake_time, update_interval);
    }
}

void FFTDisplay::processAudioData() {
    if (!frame_audio_data_ || !fft_real_ || !fft_imag_) {
        return;
    }
    
    // Copy audio data and apply Hanning window
    size_t samples_to_process = std::min(audio_buffer_size_, static_cast<size_t>(FFT_SIZE));
    
    for (size_t i = 0; i < samples_to_process; i++) {
        fft_real_[i] = static_cast<float>(frame_audio_data_[i]) * hanning_window_[i];
        fft_imag_[i] = 0.0f;
    }
    
    // Zero-pad if necessary
    for (size_t i = samples_to_process; i < FFT_SIZE; i++) {
        fft_real_[i] = 0.0f;
        fft_imag_[i] = 0.0f;
    }
    
    // Perform FFT
    computeFFT(fft_real_, fft_imag_, FFT_SIZE, true);
    
    // Calculate power spectrum
    for (int i = 0; i < FFT_SIZE / 2; i++) {
        float magnitude = sqrtf(fft_real_[i] * fft_real_[i] + fft_imag_[i] * fft_imag_[i]);
        power_spectrum_[i] = 20.0f * log10f(magnitude + 1e-10f);  // Convert to dB
        
        // Apply smoothing
        avg_power_spectrum_[i] = 0.8f * avg_power_spectrum_[i] + 0.2f * power_spectrum_[i];
    }
}

void FFTDisplay::updateSpectrum() {
    if (!avg_power_spectrum_ || !bar_heights_) {
        return;
    }
    
    // Map frequency bins to display bars
    int bins_per_bar = (FFT_SIZE / 2) / FFT_BARS_COUNT;
    
    for (int bar = 0; bar < FFT_BARS_COUNT; bar++) {
        float sum = 0.0f;
        int start_bin = bar * bins_per_bar;
        int end_bin = start_bin + bins_per_bar;
        
        // Average the magnitude over the frequency range for this bar
        for (int bin = start_bin; bin < end_bin && bin < FFT_SIZE / 2; bin++) {
            sum += avg_power_spectrum_[bin];
        }
        
        float avg_magnitude = sum / bins_per_bar;
        
        // Convert to bar height (0 to display_height_)
        float normalized = (avg_magnitude + 60.0f) / 60.0f;  // Normalize dB range
        normalized = std::max(0.0f, std::min(1.0f, normalized));
        
        int target_height = static_cast<int>(normalized * (display_height_ * 0.8f));
        
        // Smooth bar height changes
        if (target_height > bar_heights_[bar]) {
            bar_heights_[bar] = target_height;  // Quick rise
        } else {
            bar_heights_[bar] = static_cast<int>(0.9f * bar_heights_[bar] + 0.1f * target_height);  // Slow fall
        }
    }
}

void FFTDisplay::renderSpectrumBars() {
    if (!adapter_) return;
    
    adapter_->clearDisplay();
    
    int bar_width = display_width_ / FFT_BARS_COUNT;
    int bar_spacing = bar_width > 2 ? 1 : 0;
    int actual_bar_width = bar_width - bar_spacing;
    
    // Try to use optimized batch drawing for LCDDisplayAdapter
#if defined(HAVE_LVGL) || __has_include(<lvgl.h>)
    auto lcd_adapter = dynamic_cast<LCDDisplayAdapter*>(adapter_.get());
    if (lcd_adapter) {
        // Prepare arrays for batch drawing
        int x_positions[FFT_BARS_COUNT];
        int heights[FFT_BARS_COUNT];
        uint16_t colors[FFT_BARS_COUNT];
        int valid_bars = 0;
        
        for (int i = 0; i < FFT_BARS_COUNT; i++) {
            if (bar_heights_[i] > 0) {
                x_positions[valid_bars] = i * bar_width;
                heights[valid_bars] = bar_heights_[i];
                colors[valid_bars] = getBarColor(i, static_cast<float>(bar_heights_[i]) / display_height_);
                valid_bars++;
            }
        }
        
        if (valid_bars > 0) {
            lcd_adapter->drawBars(x_positions, heights, colors, valid_bars, actual_bar_width, display_height_);
        }
        return;
    }
#endif
    
    // Fallback to individual drawing for other adapters
    for (int i = 0; i < FFT_BARS_COUNT; i++) {
        int x = i * bar_width;
        int height = bar_heights_[i];
        int y = display_height_ - height;
        
        if (height > 0) {
            uint16_t color = getBarColor(i, static_cast<float>(height) / display_height_);
            adapter_->drawRect(x, y, actual_bar_width, height, color, true);
        }
    }
}

void FFTDisplay::renderWaveform() {
    if (!adapter_ || !frame_audio_data_) return;
    
    adapter_->clearDisplay();
    
    int samples_to_draw = std::min(static_cast<int>(audio_buffer_size_), display_width_);
    int center_y = display_height_ / 2;
    
    uint16_t color = is_color_display_ ? COLOR_GREEN : COLOR_WHITE;
    
    for (int i = 1; i < samples_to_draw; i++) {
        int prev_y = center_y + (frame_audio_data_[i-1] * center_y) / 32768;
        int curr_y = center_y + (frame_audio_data_[i] * center_y) / 32768;
        
        prev_y = std::max(0, std::min(display_height_ - 1, prev_y));
        curr_y = std::max(0, std::min(display_height_ - 1, curr_y));
        
        adapter_->drawLine(i-1, prev_y, i, curr_y, color);
    }
}

void FFTDisplay::renderCircular() {
    if (!adapter_) return;
    
    adapter_->clearDisplay();
    
    int center_x = display_width_ / 2;
    int center_y = display_height_ / 2;
    int max_radius = std::min(center_x, center_y) - 5;
    
    for (int i = 0; i < FFT_BARS_COUNT; i++) {
        float angle = (2.0f * M_PI * i) / FFT_BARS_COUNT;
        int radius = (bar_heights_[i] * max_radius) / display_height_;
        
        int x1 = center_x + static_cast<int>((max_radius * 0.3f) * cosf(angle));
        int y1 = center_y + static_cast<int>((max_radius * 0.3f) * sinf(angle));
        int x2 = center_x + static_cast<int>((max_radius * 0.3f + radius) * cosf(angle));
        int y2 = center_y + static_cast<int>((max_radius * 0.3f + radius) * sinf(angle));
        
        uint16_t color = getBarColor(i, static_cast<float>(radius) / max_radius);
        adapter_->drawLine(x1, y1, x2, y2, color);
    }
}

void FFTDisplay::renderWaterfall() {
    // Waterfall effect - scrolling spectrum display
    // This would require additional buffers for historical data
    // For now, fall back to spectrum bars
    renderSpectrumBars();
}

uint16_t FFTDisplay::getBarColor(int position, float magnitude) {
    if (!is_color_display_) {
        return COLOR_WHITE;
    }
    
    // Create a color gradient based on position and magnitude
    if (magnitude < 0.3f) {
        return COLOR_BLUE;
    } else if (magnitude < 0.6f) {
        return interpolateColor(COLOR_BLUE, COLOR_GREEN, (magnitude - 0.3f) / 0.3f);
    } else if (magnitude < 0.8f) {
        return interpolateColor(COLOR_GREEN, COLOR_YELLOW, (magnitude - 0.6f) / 0.2f);
    } else {
        return interpolateColor(COLOR_YELLOW, COLOR_RED, (magnitude - 0.8f) / 0.2f);
    }
}

uint16_t FFTDisplay::interpolateColor(uint16_t color1, uint16_t color2, float ratio) {
    ratio = std::max(0.0f, std::min(1.0f, ratio));
    
    // Extract RGB components from 16-bit colors
    uint8_t r1 = (color1 >> 11) & 0x1F;
    uint8_t g1 = (color1 >> 5) & 0x3F;
    uint8_t b1 = color1 & 0x1F;
    
    uint8_t r2 = (color2 >> 11) & 0x1F;
    uint8_t g2 = (color2 >> 5) & 0x3F;
    uint8_t b2 = color2 & 0x1F;
    
    // Interpolate
    uint8_t r = static_cast<uint8_t>(r1 + ratio * (r2 - r1));
    uint8_t g = static_cast<uint8_t>(g1 + ratio * (g2 - g1));
    uint8_t b = static_cast<uint8_t>(b1 + ratio * (b2 - b1));
    
    return (r << 11) | (g << 5) | b;
}

void FFTDisplay::computeFFT(float* real, float* imag, int n, bool forward) {
    // Bit-reversal permutation
    int j = 0;
    for (int i = 1; i < n; i++) {
        int bit = n >> 1;
        while (j & bit) {
            j ^= bit;
            bit >>= 1;
        }
        j ^= bit;
        
        if (i < j) {
            std::swap(real[i], real[j]);
            std::swap(imag[i], imag[j]);
        }
    }
    
    // Cooley-Tukey FFT
    for (int len = 2; len <= n; len *= 2) {
        float angle = (forward ? -2.0f : 2.0f) * M_PI / len;
        float wlen_r = cosf(angle);
        float wlen_i = sinf(angle);
        
        for (int i = 0; i < n; i += len) {
            float w_r = 1.0f;
            float w_i = 0.0f;
            
            for (int j = 0; j < len / 2; j++) {
                int u = i + j;
                int v = i + j + len / 2;
                
                float u_r = real[u];
                float u_i = imag[u];
                float v_r = real[v] * w_r - imag[v] * w_i;
                float v_i = real[v] * w_i + imag[v] * w_r;
                
                real[u] = u_r + v_r;
                imag[u] = u_i + v_i;
                real[v] = u_r - v_r;
                imag[v] = u_i - v_i;
                
                float w_r_new = w_r * wlen_r - w_i * wlen_i;
                w_i = w_r * wlen_i + w_i * wlen_r;
                w_r = w_r_new;
            }
        }
    }
}

// LCD Display Adapter Implementation
#ifdef HAVE_LVGL
LCDDisplayAdapter::LCDDisplayAdapter(int width, int height)
    : canvas_(nullptr), canvas_buffer_(nullptr), width_(width), height_(height) {
}

LCDDisplayAdapter::~LCDDisplayAdapter() {
    if (canvas_buffer_) {
        heap_caps_free(canvas_buffer_);
    }
}

bool LCDDisplayAdapter::initialize() {
    size_t buffer_size = width_ * height_ * sizeof(uint16_t);
    canvas_buffer_ = (uint16_t*)heap_caps_malloc(buffer_size, MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
    
    if (!canvas_buffer_) {
        ESP_LOGE(TAG, "Failed to allocate canvas buffer for LCD");
        return false;
    }
    
    canvas_ = lv_canvas_create(lv_screen_active());
    lv_canvas_set_buffer(canvas_, canvas_buffer_, width_, height_, LV_COLOR_FORMAT_RGB565);
    lv_obj_set_size(canvas_, width_, height_);
    
    return true;
}

void LCDDisplayAdapter::drawPixel(int x, int y, uint16_t color) {
    if (!canvas_ || x < 0 || x >= width_ || y < 0 || y >= height_) return;
    
    DisplayAdapterLockGuard lock(this);
    if (!lock.isLocked()) return;
    
    lv_color_t lv_color = lv_color_make(
        (color >> 11) & 0x1F,
        (color >> 5) & 0x3F,  
        color & 0x1F
    );
    
    lv_canvas_set_px(canvas_, x, y, lv_color, LV_OPA_COVER);
}

void LCDDisplayAdapter::drawLine(int x0, int y0, int x1, int y1, uint16_t color) {
    if (!canvas_) return;
    
    DisplayAdapterLockGuard lock(this);
    if (!lock.isLocked()) return;
    
    lv_color_t lv_color = lv_color_make(
        (color >> 11) & 0x1F,
        (color >> 5) & 0x3F,
        color & 0x1F
    );
    
    lv_layer_t layer;
    lv_canvas_init_layer(canvas_, &layer);
    
    lv_draw_line_dsc_t line_dsc;
    lv_draw_line_dsc_init(&line_dsc);
    line_dsc.color = lv_color;
    line_dsc.width = 1;
    line_dsc.p1.x = x0;
    line_dsc.p1.y = y0;
    line_dsc.p2.x = x1;
    line_dsc.p2.y = y1;
    
    lv_draw_line(&layer, &line_dsc);
    lv_canvas_finish_layer(canvas_, &layer);
}

void LCDDisplayAdapter::drawRect(int x, int y, int width, int height, uint16_t color, bool filled) {
    if (!canvas_) return;
    
    DisplayAdapterLockGuard lock(this);
    if (!lock.isLocked()) return;
    
    lv_color_t lv_color = lv_color_make(
        (color >> 11) & 0x1F,
        (color >> 5) & 0x3F,
        color & 0x1F
    );
    
    lv_layer_t layer;
    lv_canvas_init_layer(canvas_, &layer);
    
    lv_draw_rect_dsc_t rect_dsc;
    lv_draw_rect_dsc_init(&rect_dsc);
    rect_dsc.bg_color = lv_color;
    rect_dsc.bg_opa = LV_OPA_COVER;
    
    if (!filled) {
        rect_dsc.bg_opa = LV_OPA_TRANSP;
        rect_dsc.border_color = lv_color;
        rect_dsc.border_width = 1;
        rect_dsc.border_opa = LV_OPA_COVER;
    }
    
    lv_area_t area = {
        .x1 = x,
        .y1 = y,
        .x2 = x + width - 1,
        .y2 = y + height - 1
    };
    
    lv_draw_rect(&layer, &rect_dsc, &area);
    lv_canvas_finish_layer(canvas_, &layer);
}

void LCDDisplayAdapter::drawCircle(int x, int y, int radius, uint16_t color, bool filled) {
    if (!canvas_) return;
    
    DisplayAdapterLockGuard lock(this);
    if (!lock.isLocked()) return;
    
    lv_color_t lv_color = lv_color_make(
        (color >> 11) & 0x1F,
        (color >> 5) & 0x3F,
        color & 0x1F
    );
    
    lv_layer_t layer;
    lv_canvas_init_layer(canvas_, &layer);
    
    lv_draw_arc_dsc_t arc_dsc;
    lv_draw_arc_dsc_init(&arc_dsc);
    arc_dsc.color = lv_color;
    arc_dsc.center.x = x;
    arc_dsc.center.y = y;
    arc_dsc.radius = radius;
    arc_dsc.start_angle = 0;
    arc_dsc.end_angle = 360;
    arc_dsc.opa = LV_OPA_COVER;
    
    if (filled) {
        // For filled circle, make the arc width thick enough to fill
        arc_dsc.width = radius;
    } else {
        // For circle outline
        arc_dsc.width = 1;
    }
    
    lv_draw_arc(&layer, &arc_dsc);
    lv_canvas_finish_layer(canvas_, &layer);
}

void LCDDisplayAdapter::clearDisplay() {
    if (!canvas_) return;
    
    DisplayAdapterLockGuard lock(this);
    if (!lock.isLocked()) return;
    
    lv_canvas_fill_bg(canvas_, lv_color_black(), LV_OPA_COVER);
}

void LCDDisplayAdapter::updateDisplay() {
    if (!canvas_) return;
    
    DisplayAdapterLockGuard lock(this);
    if (!lock.isLocked()) return;
    
    // LVGL handles the display update automatically
    lv_area_t refresh_area;
    refresh_area.x1 = 0;
    refresh_area.y1 = height_-100;
    refresh_area.x2 = width_ -1;
    refresh_area.y2 = height_ -1; // 只刷新频谱区域
    lv_obj_invalidate_area(canvas_, &refresh_area);
    // lv_obj_invalidate(canvas_);
}

void LCDDisplayAdapter::drawBars(const int* x_positions, const int* heights, const uint16_t* colors, int count, int bar_width, int base_y) {
    if (!canvas_ || !x_positions || !heights || !colors || count <= 0) return;
    
    DisplayAdapterLockGuard lock(this);
    if (!lock.isLocked()) return;
    
    // Use single layer for all bars for better performance
    lv_layer_t layer;
    lv_canvas_init_layer(canvas_, &layer);
    
    lv_draw_rect_dsc_t rect_dsc;
    lv_draw_rect_dsc_init(&rect_dsc);
    rect_dsc.bg_opa = LV_OPA_COVER;
    
    for (int i = 0; i < count; i++) {
        if (heights[i] <= 0) continue;
        
        // Convert RGB565 to lv_color_t
        lv_color_t lv_color = lv_color_make(
            (colors[i] >> 11) & 0x1F,
            (colors[i] >> 5) & 0x3F,
            colors[i] & 0x1F
        );
        
        rect_dsc.bg_color = lv_color;
        
        lv_area_t area = {
            .x1 = x_positions[i],
            .y1 = base_y - heights[i],
            .x2 = x_positions[i] + bar_width - 1,
            .y2 = base_y - 1
        };
        
        lv_draw_rect(&layer, &rect_dsc, &area);
    }
    
    lv_canvas_finish_layer(canvas_, &layer);
}
#endif

// OLED Display Adapter Implementation
OLEDDisplayAdapter::OLEDDisplayAdapter(int width, int height)
    : width_(width), height_(height), framebuffer_(nullptr) {
}

OLEDDisplayAdapter::~OLEDDisplayAdapter() {
    if (framebuffer_) {
        heap_caps_free(framebuffer_);
    }
}

bool OLEDDisplayAdapter::initialize() {
    size_t buffer_size = (width_ * height_) / 8;  // 1 bit per pixel
    framebuffer_ = (uint8_t*)heap_caps_malloc(buffer_size, MALLOC_CAP_SPIRAM);
    
    if (!framebuffer_) {
        ESP_LOGE(TAG, "Failed to allocate framebuffer for OLED");
        return false;
    }
    
    memset(framebuffer_, 0, buffer_size);
    return true;
}

void OLEDDisplayAdapter::setPixelInFramebuffer(int x, int y, bool on) {
    if (x < 0 || x >= width_ || y < 0 || y >= height_) return;
    
    int byte_index = (y / 8) * width_ + x;
    int bit_index = y % 8;
    
    if (on) {
        framebuffer_[byte_index] |= (1 << bit_index);
    } else {
        framebuffer_[byte_index] &= ~(1 << bit_index);
    }
}

bool OLEDDisplayAdapter::getPixelFromFramebuffer(int x, int y) const {
    if (x < 0 || x >= width_ || y < 0 || y >= height_) return false;
    
    int byte_index = (y / 8) * width_ + x;
    int bit_index = y % 8;
    
    return (framebuffer_[byte_index] & (1 << bit_index)) != 0;
}

void OLEDDisplayAdapter::drawPixel(int x, int y, uint16_t color) {
    DisplayAdapterLockGuard lock(this);
    if (!lock.isLocked()) return;
    
    setPixelInFramebuffer(x, y, color != 0);
}

void OLEDDisplayAdapter::drawLine(int x0, int y0, int x1, int y1, uint16_t color) {
    DisplayAdapterLockGuard lock(this);
    if (!lock.isLocked()) return;
    
    // Bresenham's line algorithm
    bool steep = abs(y1 - y0) > abs(x1 - x0);
    
    if (steep) {
        std::swap(x0, y0);
        std::swap(x1, y1);
    }
    
    if (x0 > x1) {
        std::swap(x0, x1);
        std::swap(y0, y1);
    }
    
    int dx = x1 - x0;
    int dy = abs(y1 - y0);
    int error = dx / 2;
    int ystep = y0 < y1 ? 1 : -1;
    int y = y0;
    
    for (int x = x0; x <= x1; x++) {
        if (steep) {
            setPixelInFramebuffer(y, x, color != 0);
        } else {
            setPixelInFramebuffer(x, y, color != 0);
        }
        
        error -= dy;
        if (error < 0) {
            y += ystep;
            error += dx;
        }
    }
}

void OLEDDisplayAdapter::drawRect(int x, int y, int width, int height, uint16_t color, bool filled) {
    DisplayAdapterLockGuard lock(this);
    if (!lock.isLocked()) return;
    
    if (filled) {
        for (int i = 0; i < height; i++) {
            for (int j = 0; j < width; j++) {
                setPixelInFramebuffer(x + j, y + i, color != 0);
            }
        }
    } else {
        drawLine(x, y, x + width - 1, y, color);
        drawLine(x + width - 1, y, x + width - 1, y + height - 1, color);
        drawLine(x + width - 1, y + height - 1, x, y + height - 1, color);
        drawLine(x, y + height - 1, x, y, color);
    }
}

void OLEDDisplayAdapter::drawCircle(int x, int y, int radius, uint16_t color, bool filled) {
    DisplayAdapterLockGuard lock(this);
    if (!lock.isLocked()) return;
    
    // Midpoint circle algorithm
    int f = 1 - radius;
    int ddf_x = 1;
    int ddf_y = -2 * radius;
    int dx = 0;
    int dy = radius;
    
    if (filled) {
        drawLine(x, y - radius, x, y + radius, color);
        drawLine(x - radius, y, x + radius, y, color);
    } else {
        setPixelInFramebuffer(x, y + radius, color != 0);
        setPixelInFramebuffer(x, y - radius, color != 0);
        setPixelInFramebuffer(x + radius, y, color != 0);
        setPixelInFramebuffer(x - radius, y, color != 0);
    }
    
    while (dx < dy) {
        if (f >= 0) {
            dy--;
            ddf_y += 2;
            f += ddf_y;
        }
        dx++;
        ddf_x += 2;
        f += ddf_x;
        
        if (filled) {
            drawLine(x - dx, y + dy, x + dx, y + dy, color);
            drawLine(x - dx, y - dy, x + dx, y - dy, color);
            drawLine(x - dy, y + dx, x + dy, y + dx, color);
            drawLine(x - dy, y - dx, x + dy, y - dx, color);
        } else {
            setPixelInFramebuffer(x + dx, y + dy, color != 0);
            setPixelInFramebuffer(x - dx, y + dy, color != 0);
            setPixelInFramebuffer(x + dx, y - dy, color != 0);
            setPixelInFramebuffer(x - dx, y - dy, color != 0);
            setPixelInFramebuffer(x + dy, y + dx, color != 0);
            setPixelInFramebuffer(x - dy, y + dx, color != 0);
            setPixelInFramebuffer(x + dy, y - dx, color != 0);
            setPixelInFramebuffer(x - dy, y - dx, color != 0);
        }
    }
}

void OLEDDisplayAdapter::clearDisplay() {
    DisplayAdapterLockGuard lock(this);
    if (!lock.isLocked()) return;
    
    if (framebuffer_) {
        memset(framebuffer_, 0, (width_ * height_) / 8);
    }
}

void OLEDDisplayAdapter::updateDisplay() {
    DisplayAdapterLockGuard lock(this);
    if (!lock.isLocked()) return;
    
    sendFramebufferToDisplay();
}

void OLEDDisplayAdapter::sendFramebufferToDisplay() {
    // This method should be implemented by the specific OLED driver
    // It should send the framebuffer_ to the actual OLED hardware
    // For now, it's a placeholder
}