#ifndef DISPLAY_H
#define DISPLAY_H

#include "emoji_collection.h"

#ifndef CONFIG_USE_EMOTE_MESSAGE_STYLE
#define HAVE_LVGL 1
#include <lvgl.h>
#endif

#include <esp_timer.h>
#include <esp_log.h>
#include <esp_pm.h>

#include <string>
#include <chrono>
#include <cstdint>

// Forward declarations for weather structures
struct IdleCardInfo;
struct WeatherInfo;

class Theme {
public:
    Theme(const std::string& name) : name_(name) {}
    virtual ~Theme() = default;

    inline std::string name() const { return name_; }
private:
    std::string name_;
};

class Display {
public:
    Display();
    virtual ~Display();

    virtual void SetStatus(const char* status);
    virtual void ShowNotification(const char* notification, int duration_ms = 3000);
    virtual void ShowNotification(const std::string &notification, int duration_ms = 3000);
    virtual void SetEmotion(const char* emotion);
    virtual void SetChatMessage(const char* role, const char* content);
    virtual void SetTheme(Theme* theme);
    virtual Theme* GetTheme() { return current_theme_; }
    virtual void UpdateStatusBar(bool update_all = false);
    virtual void SetPowerSaveMode(bool on);

    /**
     * @brief Show or hide the media overlay on the main UI.
     *
     * When active, hides the main emotion/chat UI (emoji, chat message,
     * idle card) to make room for media content (FFT spectrum, video
     * canvas, etc.).  When deactivated, the original UI is restored.
     *
     * @param active  true = media overlay is active (hide main UI),
     *                false = restore main UI
     */
    virtual void SetMediaOverlayActive(bool active) {}

    // Preview image control: keep visible (no auto-hide) during AI analysis,
    // and clear when entering listening mode.
    virtual void SetCameraPreviewRgb565(const uint8_t* data, int src_w, int src_h, int src_stride, bool rotate_180) {}
    virtual void KeepPreviewVisible(bool keep) {}
    virtual void ClearPreviewImage() {}

    // For rotation display
    virtual bool SetRotation(int rotation_degree, bool save_setting) { return false; }
    
#ifdef CONFIG_WEATHER_IDLE_DISPLAY_ENABLE
    // For weather idle card
    virtual void ShowIdleCard(const IdleCardInfo& info) {}
    virtual void HideIdleCard() {}
#endif

    inline int width() const { return width_; }
    inline int height() const { return height_; }

protected:
    int width_ = 0;
    int height_ = 0;

    Theme* current_theme_ = nullptr;

    friend class DisplayLockGuard;
    virtual bool Lock(int timeout_ms = 0) = 0;
    virtual void Unlock() = 0;
};


class DisplayLockGuard {
public:
    DisplayLockGuard(Display *display) : display_(display) {
        locked_ = display_ != nullptr && display_->Lock(30000);
        if (!locked_) {
            ESP_LOGE("Display", "Failed to lock display");
        }
    }
    ~DisplayLockGuard() {
        if (locked_) {
            display_->Unlock();
        }
    }

private:
    Display *display_;
    bool locked_ = false;
};

class NoDisplay : public Display {
private:
    virtual bool Lock(int timeout_ms = 0) override {
        return true;
    }
    virtual void Unlock() override {}
};

#endif
