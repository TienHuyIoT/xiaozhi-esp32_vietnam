/**
 * @file fft_display_example.cc
 * @brief Example demonstrating how to use the refactored FFT Display system
 * 
 * This example shows how to integrate FFT visualization with different display types
 * (LCD and OLED) using the new modular FFTDisplay architecture.
 */

#include "fft_display.h"
#include "lcd_display.h"
#include "oled_display.h"
#include "board.h"

#include <esp_log.h>
#include <memory>

#define TAG "FFTDisplayExample"

class FFTDisplayExample {
private:
    std::unique_ptr<Display> display_;
    bool is_playing_;

public:
    FFTDisplayExample() : is_playing_(false) {}
    
    /**
     * @brief Initialize display based on board configuration
     */
    bool initializeDisplay() {
        auto& board = Board::GetInstance();
        display_ = std::unique_ptr<Display>(board.GetDisplay());
        
        if (!display_) {
            ESP_LOGE(TAG, "Failed to get display from board");
            return false;
        }
        
        ESP_LOGI(TAG, "Display initialized: %dx%d", display_->width(), display_->height());
        return true;
    }
    
    /**
     * @brief Start FFT visualization
     */
    void startFFTVisualization() {
        if (!display_) {
            ESP_LOGE(TAG, "Display not initialized");
            return;
        }
        
        ESP_LOGI(TAG, "Starting FFT visualization");
        display_->start();
        is_playing_ = true;
        
        // Set initial display message
        display_->SetMusicInfo("FFT Visualization Active");
    }
    
    /**
     * @brief Stop FFT visualization
     */
    void stopFFTVisualization() {
        if (!display_) {
            return;
        }
        
        ESP_LOGI(TAG, "Stopping FFT visualization");
        display_->stopFft();
        display_->clearScreen();
        is_playing_ = false;
        
        display_->SetMusicInfo("");
    }
    
    /**
     * @brief Simulate audio data for FFT processing
     * This would normally come from audio codec/radio/music player
     */
    void simulateAudioData() {
        if (!display_ || !is_playing_) {
            return;
        }
        
        const size_t sample_count = 1152; // Common audio frame size
        
        // Create audio buffer
        int16_t* audio_buffer = display_->createAudioDataBuffer(sample_count);
        if (!audio_buffer) {
            ESP_LOGE(TAG, "Failed to create audio buffer");
            return;
        }
        
        // Generate test tone (sine wave) - replace with real audio data
        static float phase = 0.0f;
        const float frequency = 440.0f; // A4 note
        const float sample_rate = 44100.0f;
        const float amplitude = 16000.0f;
        
        for (size_t i = 0; i < sample_count / sizeof(int16_t); i++) {
            float sample = amplitude * sinf(2.0f * M_PI * frequency * phase / sample_rate);
            audio_buffer[i] = static_cast<int16_t>(sample);
            phase += 1.0f;
            if (phase >= sample_rate) {
                phase = 0.0f;
            }
        }
        
        // Update display with audio data
        display_->updateAudioDataBuffer(audio_buffer, sample_count);
        
        ESP_LOGD(TAG, "Updated audio buffer with %zu samples", sample_count);
    }
    
    /**
     * @brief Example of changing FFT effects (if supported by display type)
     */
    void cycleThroughFFTEffects() {
        // This functionality would need to be exposed through Display interface
        // For now, it's just a placeholder showing the concept
        
        ESP_LOGI(TAG, "Cycling through FFT effects");
        
        // Different effects could be:
        // - Spectrum bars (default)
        // - Waveform display
        // - Circular visualization
        // - Waterfall effect
        
        static int current_effect = 0;
        const char* effect_names[] = {
            "Spectrum Bars",
            "Waveform",
            "Circular",
            "Waterfall"
        };
        
        display_->SetMusicInfo(effect_names[current_effect % 4]);
        current_effect++;
        
        ESP_LOGI(TAG, "Switched to effect: %s", effect_names[(current_effect - 1) % 4]);
    }
    
    /**
     * @brief Main demo loop
     */
    void runDemo() {
        ESP_LOGI(TAG, "Starting FFT Display Demo");
        
        if (!initializeDisplay()) {
            return;
        }
        
        // Start visualization
        startFFTVisualization();
        
        // Demo loop - normally this would be called from audio processing thread
        for (int i = 0; i < 100; i++) {
            simulateAudioData();
            
            // Change effect every 20 iterations
            if (i % 20 == 0) {
                cycleThroughFFTEffects();
            }
            
            vTaskDelay(pdMS_TO_TICKS(50)); // 20 FPS
        }
        
        // Stop visualization
        stopFFTVisualization();
        
        ESP_LOGI(TAG, "FFT Display Demo completed");
    }
};

/**
 * @brief Example function showing how to integrate FFT with radio playback
 */
void example_radio_with_fft() {
    ESP_LOGI(TAG, "Radio with FFT example");
    
    auto& board = Board::GetInstance();
    auto display = board.GetDisplay();
    auto radio = board.GetRadio();
    
    if (!display || !radio) {
        ESP_LOGE(TAG, "Display or radio not available");
        return;
    }
    
    // Start radio
    if (radio->PlayStation("VOV1")) {
        ESP_LOGI(TAG, "Radio started successfully");
        
        // Start FFT visualization
        display->start();
        ESP_LOGI(TAG, "FFT visualization started for radio");
        
        // Let radio play for demo period
        vTaskDelay(pdMS_TO_TICKS(30000)); // 30 seconds
        
        // Stop everything
        radio->Stop();
        display->stopFft();
        ESP_LOGI(TAG, "Radio and FFT stopped");
    } else {
        ESP_LOGE(TAG, "Failed to start radio");
    }
}

/**
 * @brief Example function showing how to integrate FFT with music playback
 */
void example_music_with_fft() {
    ESP_LOGI(TAG, "Music with FFT example");
    
    auto& board = Board::GetInstance();
    auto display = board.GetDisplay();
    auto music = board.GetMusic();
    
    if (!display || !music) {
        ESP_LOGE(TAG, "Display or music player not available");
        return;
    }
    
    // Example music file (would need to exist)
    const char* music_file = "/assets/music/test.mp3";
    
    // Start music
    if (music->Play(music_file)) {
        ESP_LOGI(TAG, "Music started successfully");
        
        // Start FFT visualization
        display->start();
        ESP_LOGI(TAG, "FFT visualization started for music");
        
        // Let music play (would normally wait for completion)
        vTaskDelay(pdMS_TO_TICKS(60000)); // 1 minute demo
        
        // Stop everything
        music->Stop();
        display->stopFft();
        ESP_LOGI(TAG, "Music and FFT stopped");
    } else {
        ESP_LOGE(TAG, "Failed to start music playback");
    }
}

/**
 * @brief Main example task
 */
extern "C" void fft_display_example_task(void* pvParameter) {
    ESP_LOGI(TAG, "FFT Display Example Task Started");
    
    // Wait for system initialization
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // Run basic demo
    FFTDisplayExample example;
    example.runDemo();
    
    // Run radio integration example
    example_radio_with_fft();
    
    // Run music integration example  
    example_music_with_fft();
    
    ESP_LOGI(TAG, "All FFT Display examples completed");
    vTaskDelete(NULL);
}

/**
 * @brief Start the FFT display example
 * Call this from your main application
 */
void start_fft_display_example() {
    xTaskCreate(
        fft_display_example_task,
        "fft_example",
        8192,
        NULL,
        5,
        NULL
    );
}