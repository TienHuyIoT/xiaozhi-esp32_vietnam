#ifndef FFT_DISPLAY_H
#define FFT_DISPLAY_H

#include <esp_log.h>
#include <esp_heap_caps.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <atomic>
#include <memory>
#include <cmath>
#include <cstring>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define FFT_SIZE 512
#define FFT_BARS_COUNT 40

// FFT display effect types
enum class FFTEffect {
    SPECTRUM_BARS,      // Traditional spectrum bars
    WAVEFORM,          // Waveform display
    CIRCULAR,          // Circular spectrum
    WATERFALL          // Waterfall effect
};

// Color definitions for FFT display
#define COLOR_BLACK   0x0000
#define COLOR_RED     0xF800
#define COLOR_GREEN   0x07E0
#define COLOR_BLUE    0x001F
#define COLOR_YELLOW  0xFFE0
#define COLOR_CYAN    0x07FF
#define COLOR_MAGENTA 0xF81F
#define COLOR_WHITE   0xFFFF

// Abstract base class for different display types
class DisplayAdapter {
public:
    virtual ~DisplayAdapter() = default;
    
    // Drawing primitives that each display type must implement
    virtual void drawPixel(int x, int y, uint16_t color) = 0;
    virtual void drawLine(int x0, int y0, int x1, int y1, uint16_t color) = 0;
    virtual void drawRect(int x, int y, int width, int height, uint16_t color, bool filled = false) = 0;
    virtual void drawCircle(int x, int y, int radius, uint16_t color, bool filled = false) = 0;
    virtual void clearDisplay() = 0;
    virtual void updateDisplay() = 0;
    
    // Display properties
    virtual int getWidth() const = 0;
    virtual int getHeight() const = 0;
    virtual bool isColor() const = 0;  // true for color displays, false for monochrome

    friend class DisplayAdapterLockGuard;
    virtual bool Lock(int timeout_ms = 0) = 0;
    virtual void Unlock() = 0;
};

class FFTDisplay {
private:
    std::unique_ptr<DisplayAdapter> adapter_;
    FFTEffect current_effect_;
    
    // FFT processing variables
    float* fft_real_;
    float* fft_imag_;
    float* hanning_window_;
    float* power_spectrum_;
    float* avg_power_spectrum_;
    int* bar_heights_;
    
    // Audio data buffer
    int16_t* audio_data_;
    int16_t* frame_audio_data_;
    size_t audio_buffer_size_;
    
    // Task management
    TaskHandle_t fft_task_handle_;
    std::atomic<bool> fft_task_should_stop_;
    std::atomic<bool> is_running_;
    
    // Timing
    uint32_t last_fft_update_;
    bool fft_data_ready_;
    
    // Configuration
    int display_width_;
    int display_height_;
    bool is_color_display_;
    
    // Private methods
    void initializeFFTBuffers();
    void cleanupFFTBuffers();
    void computeFFT(float* real, float* imag, int n, bool forward);
    void processAudioData();
    void updateSpectrum();
    
    // Effect rendering methods
    void renderSpectrumBars();
    void renderWaveform();
    void renderCircular();
    void renderWaterfall();
    
    // Color generation
    uint16_t getBarColor(int position, float magnitude);
    uint16_t interpolateColor(uint16_t color1, uint16_t color2, float ratio);
    
    // FFT task
    static void fftTaskWrapper(void* param);
    void fftTask();

public:
    FFTDisplay(std::unique_ptr<DisplayAdapter> adapter);
    ~FFTDisplay();
    
    // Control methods
    bool start();
    void stop();
    bool isRunning() const { return is_running_.load(); }
    
    // Configuration
    void setEffect(FFTEffect effect) { current_effect_ = effect; }
    FFTEffect getEffect() const { return current_effect_; }
    
    // Audio data interface
    int16_t* createAudioDataBuffer(size_t sample_count);
    void updateAudioDataBuffer(int16_t* data, size_t sample_count);
    void releaseAudioDataBuffer(int16_t* buffer = nullptr);
    
    // Display management
    void clearDisplay();
    void updateDisplay();
};

// LCD Display Adapter for LVGL-based displays
#if defined(HAVE_LVGL) || __has_include(<lvgl.h>)
#include <lvgl.h>
#ifndef HAVE_LVGL
#define HAVE_LVGL 1
#endif

class LCDDisplayAdapter : public DisplayAdapter {
private:
    lv_obj_t* canvas_;
    uint16_t* canvas_buffer_;
    int width_;
    int height_;

    virtual bool Lock(int timeout_ms = 0) override;
    virtual void Unlock() override;
    
public:
    LCDDisplayAdapter(int width, int height);
    ~LCDDisplayAdapter();
    
    // Initialize canvas for LVGL
    bool initialize();
    lv_obj_t* getCanvas() const { return canvas_; }
    
    // DisplayAdapter implementation
    void drawPixel(int x, int y, uint16_t color) override;
    void drawLine(int x0, int y0, int x1, int y1, uint16_t color) override;
    void drawRect(int x, int y, int width, int height, uint16_t color, bool filled = false) override;
    void drawCircle(int x, int y, int radius, uint16_t color, bool filled = false) override;
    void clearDisplay() override;
    void updateDisplay() override;
    
    // Optimized batch drawing for FFT bars
    void drawBars(const int* x_positions, const int* heights, const uint16_t* colors, int count, int bar_width, int base_y);
    
    int getWidth() const override { return width_; }
    int getHeight() const override { return height_; }
    bool isColor() const override { return true; }
};
#endif

// OLED Display Adapter for SSD1306/SH1106-based displays
class OLEDDisplayAdapter : public DisplayAdapter {
private:
    int width_;
    int height_;
    uint8_t* framebuffer_;  // Monochrome framebuffer
    
    // OLED-specific drawing methods
    void setPixelInFramebuffer(int x, int y, bool on);
    bool getPixelFromFramebuffer(int x, int y) const;
    
public:
    OLEDDisplayAdapter(int width, int height);
    ~OLEDDisplayAdapter();
    
    // Initialize OLED framebuffer
    bool initialize();
    
    // DisplayAdapter implementation
    void drawPixel(int x, int y, uint16_t color) override;
    void drawLine(int x0, int y0, int x1, int y1, uint16_t color) override;
    void drawRect(int x, int y, int width, int height, uint16_t color, bool filled = false) override;
    void drawCircle(int x, int y, int radius, uint16_t color, bool filled = false) override;
    void clearDisplay() override;
    void updateDisplay() override;
    
    int getWidth() const override { return width_; }
    int getHeight() const override { return height_; }
    bool isColor() const override { return false; }
    
    // OLED-specific methods
    uint8_t* getFramebuffer() const { return framebuffer_; }
    void sendFramebufferToDisplay();  // To be implemented by specific OLED driver
};

// RAII-style lock guard for DisplayAdapter operations
class DisplayAdapterLockGuard {
public:
    DisplayAdapterLockGuard(DisplayAdapter *adapter) : adapter_(adapter) {
        if (!adapter_->Lock(30000)) {
            ESP_LOGE("Display", "Failed to lock display");
        }
    }
    ~DisplayAdapterLockGuard() {
        adapter_->Unlock();
    }

private:
    DisplayAdapter *adapter_;
};

#endif // FFT_DISPLAY_H