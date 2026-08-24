#include <esp_log.h>
#include <esp_err.h>
#include <algorithm>
#include <cstdio>
#include <string>
#include <cstdlib>
#include <cstring>
#include <font_awesome.h>

#include "lvgl_display.h"
#include "board.h"
#include "application.h"
#include "audio_codec.h"
#include "settings.h"
#include "assets/lang_config.h"
#include "jpg/image_to_jpeg.h"

#define TAG "Display"

LvglDisplay::LvglDisplay() {
    // Notification timer
    esp_timer_create_args_t notification_timer_args = {
        .callback = [](void *arg) {
            LvglDisplay *display = static_cast<LvglDisplay*>(arg);
            DisplayLockGuard lock(display);
            lv_obj_add_flag(display->notification_label_, LV_OBJ_FLAG_HIDDEN);
            lv_obj_remove_flag(display->status_label_, LV_OBJ_FLAG_HIDDEN);
        },
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "notification_timer",
        .skip_unhandled_events = false,
    };
    ESP_ERROR_CHECK(esp_timer_create(&notification_timer_args, &notification_timer_));

    // Create a power management lock
    auto ret = esp_pm_lock_create(ESP_PM_APB_FREQ_MAX, 0, "display_update", &pm_lock_);
    if (ret == ESP_ERR_NOT_SUPPORTED) {
        ESP_LOGI(TAG, "Power management not supported");
    } else {
        ESP_ERROR_CHECK(ret);
    }
}

LvglDisplay::~LvglDisplay() {
    payment_overlay_ = nullptr;
    payment_image_cached_.reset();
    if (notification_timer_ != nullptr) {
        esp_timer_stop(notification_timer_);
        esp_timer_delete(notification_timer_);
    }

    if (network_label_ != nullptr) {
        lv_obj_del(network_label_);
    }
    if (notification_label_ != nullptr) {
        lv_obj_del(notification_label_);
    }
    if (status_label_ != nullptr) {
        lv_obj_del(status_label_);
    }
    if (mute_label_ != nullptr) {
        lv_obj_del(mute_label_);
    }
    if (battery_label_ != nullptr) {
        lv_obj_del(battery_label_);
    }
    if( low_battery_popup_ != nullptr ) {
        lv_obj_del(low_battery_popup_);
    }
    if (pm_lock_ != nullptr) {
        esp_pm_lock_delete(pm_lock_);
    }
}

void LvglDisplay::SetStatus(const char* status) {
    DisplayLockGuard lock(this);
    if (status_label_ == nullptr) {
        return;
    }
    lv_label_set_text(status_label_, status);
    lv_obj_remove_flag(status_label_, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(notification_label_, LV_OBJ_FLAG_HIDDEN);

    last_status_update_time_ = std::chrono::system_clock::now();
}

void LvglDisplay::ShowNotification(const std::string &notification, int duration_ms) {
    ShowNotification(notification.c_str(), duration_ms);
}

void LvglDisplay::ShowNotification(const char* notification, int duration_ms) {
    DisplayLockGuard lock(this);
    if (notification_label_ == nullptr) {
        return;
    }
    lv_label_set_text(notification_label_, notification);
    lv_obj_remove_flag(notification_label_, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(status_label_, LV_OBJ_FLAG_HIDDEN);

    esp_timer_stop(notification_timer_);
    ESP_ERROR_CHECK(esp_timer_start_once(notification_timer_, duration_ms * 1000));
}

void LvglDisplay::ResetPaymentOverlay() {
    if (payment_overlay_ == nullptr) {
        payment_overlay_ = lv_obj_create(lv_screen_active());
        lv_obj_set_size(payment_overlay_, LV_HOR_RES, LV_VER_RES);
        lv_obj_align(payment_overlay_, LV_ALIGN_CENTER, 0, 0);
        lv_obj_set_style_radius(payment_overlay_, 0, 0);
        lv_obj_set_style_border_width(payment_overlay_, 0, 0);
        lv_obj_set_style_pad_all(payment_overlay_, 12, 0);
        lv_obj_set_scrollbar_mode(payment_overlay_, LV_SCROLLBAR_MODE_OFF);
    } else {
        lv_obj_clean(payment_overlay_);
    }
    payment_image_cached_.reset();
    lv_obj_remove_flag(payment_overlay_, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(payment_overlay_);
}

void LvglDisplay::ShowPaymentPreparing(const std::string& product_title,
                                       int seconds_remaining) {
    DisplayLockGuard lock(this);
    ResetPaymentOverlay();
    lv_obj_set_style_bg_color(payment_overlay_, lv_color_hex(0x101418), 0);
    lv_obj_set_style_bg_opa(payment_overlay_, LV_OPA_COVER, 0);

    auto* heading = lv_label_create(payment_overlay_);
    lv_label_set_text(heading, "Chu\xE1\xBA\xA9n b\xE1\xBB\x8B thanh to\xC3\xA1n");
    lv_obj_set_width(heading, LV_PCT(92));
    lv_obj_set_style_text_align(heading, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(heading, lv_color_white(), 0);
    lv_obj_align(heading, LV_ALIGN_CENTER, 0, -58);

    auto* product = lv_label_create(payment_overlay_);
    lv_label_set_text(product, product_title.c_str());
    lv_label_set_long_mode(product, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(product, LV_PCT(92));
    lv_obj_set_style_text_align(product, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(product, lv_color_white(), 0);
    lv_obj_align(product, LV_ALIGN_CENTER, 0, -8);

    char countdown[64];
    snprintf(countdown, sizeof(countdown),
             "M\xC3\xA3 QR hi\xE1\xBB\x83n th\xE1\xBB\x8B sau %d gi\xC3\xA2y",
             seconds_remaining);
    auto* countdown_label = lv_label_create(payment_overlay_);
    lv_label_set_text(countdown_label, countdown);
    lv_obj_set_width(countdown_label, LV_PCT(92));
    lv_obj_set_style_text_align(countdown_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(countdown_label, lv_color_hex(0xD0D7DE), 0);
    lv_obj_align(countdown_label, LV_ALIGN_CENTER, 0, 48);
}

void LvglDisplay::ShowPaymentQrImage(std::unique_ptr<LvglImage> image) {
    if (!image) return;
    DisplayLockGuard lock(this);
    ResetPaymentOverlay();
    lv_obj_set_style_bg_color(payment_overlay_, lv_color_white(), 0);
    lv_obj_set_style_bg_opa(payment_overlay_, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_all(payment_overlay_, 0, 0);

    payment_image_cached_ = std::move(image);
    auto* image_obj = lv_image_create(payment_overlay_);
    auto* image_dsc = payment_image_cached_->image_dsc();
    lv_image_set_src(image_obj, image_dsc);
    if (image_dsc->header.w > 0 && image_dsc->header.h > 0) {
        int available = std::min(width_, height_) - 8;
        int scale_w = 256 * available / image_dsc->header.w;
        int scale_h = 256 * available / image_dsc->header.h;
        lv_image_set_scale(image_obj, std::min(scale_w, scale_h));
    }
    lv_obj_center(image_obj);
}

void LvglDisplay::ShowPaymentSuccess(const std::string& product_title) {
    DisplayLockGuard lock(this);
    ResetPaymentOverlay();
    lv_obj_set_style_bg_color(payment_overlay_, lv_color_hex(0x075E3A), 0);
    lv_obj_set_style_bg_opa(payment_overlay_, LV_OPA_COVER, 0);

    auto* heading = lv_label_create(payment_overlay_);
    lv_label_set_text(heading, "Thanh to\xC3\xA1n th\xC3\xA0nh c\xC3\xB4ng");
    lv_obj_set_width(heading, LV_PCT(92));
    lv_obj_set_style_text_align(heading, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(heading, lv_color_white(), 0);
    lv_obj_align(heading, LV_ALIGN_CENTER, 0, -30);

    auto* product = lv_label_create(payment_overlay_);
    lv_label_set_text(product, product_title.c_str());
    lv_label_set_long_mode(product, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(product, LV_PCT(92));
    lv_obj_set_style_text_align(product, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(product, lv_color_white(), 0);
    lv_obj_align(product, LV_ALIGN_CENTER, 0, 25);
}

void LvglDisplay::ClearPaymentScreen() {
    DisplayLockGuard lock(this);
    if (payment_overlay_ != nullptr) {
        lv_obj_del(payment_overlay_);
        payment_overlay_ = nullptr;
    }
    payment_image_cached_.reset();
}

void LvglDisplay::UpdateStatusBar(bool update_all) {
    auto& app = Application::GetInstance();
    auto& board = Board::GetInstance();
    auto codec = board.GetAudioCodec();

    // Update mute icon
    {
        DisplayLockGuard lock(this);
        if (mute_label_ == nullptr) {
            return;
        }

        // 如果静音状态改变，则更新图标
        if (codec->output_volume() == 0 && !muted_) {
            muted_ = true;
            lv_label_set_text(mute_label_, FONT_AWESOME_VOLUME_XMARK);
        } else if (codec->output_volume() > 0 && muted_) {
            muted_ = false;
            lv_label_set_text(mute_label_, "");
        }
    }

    // Update time
    if (app.GetDeviceState() == kDeviceStateIdle) {
        if (last_status_update_time_ + std::chrono::seconds(10) < std::chrono::system_clock::now()) {
            // Set status to clock "HH:MM"
            time_t now = time(NULL);
            struct tm* tm = localtime(&now);
            // Check if the we have already set the time
            if (tm->tm_year >= 2025 - 1900) {
                char time_str[16];
                strftime(time_str, sizeof(time_str), "%H:%M  ", tm);
                SetStatus(time_str);
            } else {
                ESP_LOGW(TAG, "System time is not set, tm_year: %d", tm->tm_year);
            }
        }
    }

    esp_pm_lock_acquire(pm_lock_);
    // 更新电池图标
    int battery_level;
    bool charging, discharging;
    const char* icon = nullptr;
    if (board.GetBatteryLevel(battery_level, charging, discharging)) {
        if (charging) {
            icon = FONT_AWESOME_BATTERY_BOLT;
        } else {
            const char* levels[] = {
                FONT_AWESOME_BATTERY_EMPTY, // 0-19%
                FONT_AWESOME_BATTERY_QUARTER,    // 20-39%
                FONT_AWESOME_BATTERY_HALF,    // 40-59%
                FONT_AWESOME_BATTERY_THREE_QUARTERS,    // 60-79%
                FONT_AWESOME_BATTERY_FULL, // 80-99%
                FONT_AWESOME_BATTERY_FULL, // 100%
            };
            icon = levels[battery_level / 20];
        }
        DisplayLockGuard lock(this);
        if (battery_label_ != nullptr && battery_icon_ != icon) {
            battery_icon_ = icon;
            lv_label_set_text(battery_label_, battery_icon_);
        }

        if (low_battery_popup_ != nullptr) {
            if (strcmp(icon, FONT_AWESOME_BATTERY_EMPTY) == 0 && discharging) {
                if (lv_obj_has_flag(low_battery_popup_, LV_OBJ_FLAG_HIDDEN)) { // 如果低电量提示框隐藏，则显示
                    lv_obj_remove_flag(low_battery_popup_, LV_OBJ_FLAG_HIDDEN);
                    app.PlaySound(Lang::Sounds::OGG_LOW_BATTERY);
                }
            } else {
                // Hide the low battery popup when the battery is not empty
                if (!lv_obj_has_flag(low_battery_popup_, LV_OBJ_FLAG_HIDDEN)) { // 如果低电量提示框显示，则隐藏
                    lv_obj_add_flag(low_battery_popup_, LV_OBJ_FLAG_HIDDEN);
                }
            }
        }
    }

    // 每 10 秒更新一次网络图标
    static int seconds_counter = 0;
    if (update_all || seconds_counter++ % 10 == 0) {
        // 升级固件时，不读取 4G 网络状态，避免占用 UART 资源
        auto device_state = Application::GetInstance().GetDeviceState();
        static const std::vector<DeviceState> allowed_states = {
            kDeviceStateIdle,
            kDeviceStateStarting,
            kDeviceStateWifiConfiguring,
            kDeviceStateListening,
            kDeviceStateActivating,
        };
        if (std::find(allowed_states.begin(), allowed_states.end(), device_state) != allowed_states.end()) {
            icon = board.GetNetworkStateIcon();
            if (network_label_ != nullptr && icon != nullptr && network_icon_ != icon) {
                DisplayLockGuard lock(this);
                network_icon_ = icon;
                lv_label_set_text(network_label_, network_icon_);
            }
        }
    }

    esp_pm_lock_release(pm_lock_);
}

void LvglDisplay::SetPreviewImage(std::unique_ptr<LvglImage> image) {
}

void LvglDisplay::SetPowerSaveMode(bool on) {
    if (on) {
        SetChatMessage("system", "");
        SetEmotion("sleepy");
    } else {
        SetChatMessage("system", "");
        SetEmotion("neutral");
    }
}

bool LvglDisplay::SnapshotToJpeg(std::string& jpeg_data, int quality) {
#if CONFIG_LV_USE_SNAPSHOT
    DisplayLockGuard lock(this);

    lv_obj_t* screen = lv_screen_active();
    lv_draw_buf_t* draw_buffer = lv_snapshot_take(screen, LV_COLOR_FORMAT_RGB565);
    if (draw_buffer == nullptr) {
        ESP_LOGE(TAG, "Failed to take snapshot, draw_buffer is nullptr");
        return false;
    }

    // swap bytes
    uint16_t* data = (uint16_t*)draw_buffer->data;
    size_t pixel_count = draw_buffer->data_size / 2;
    for (size_t i = 0; i < pixel_count; i++) {
        data[i] = __builtin_bswap16(data[i]);
    }

    // 清空输出字符串并使用回调版本，避免预分配大内存块
    jpeg_data.clear();

    // 🚀 使用回调版本的JPEG编码器，进一步节省内存
    bool ret = image_to_jpeg_cb((uint8_t*)draw_buffer->data, draw_buffer->data_size, draw_buffer->header.w, draw_buffer->header.h, V4L2_PIX_FMT_RGB565, quality,
        [](void *arg, size_t index, const void *data, size_t len) -> size_t {
        std::string* output = static_cast<std::string*>(arg);
        if (data && len > 0) {
            output->append(static_cast<const char*>(data), len);
        }
        return len;
    }, &jpeg_data);
    if (!ret) {
        ESP_LOGE(TAG, "Failed to convert image to JPEG");
    }

    lv_draw_buf_destroy(draw_buffer);
    return ret;
#else
    ESP_LOGE(TAG, "LV_USE_SNAPSHOT is not enabled");
    return false;
#endif
}
