#include "features/UI/setting/brightness_volume_screen.h"

#include <algorithm>
#include <utility>

#include <esp_log.h>

#include "audio/audio_codec.h"
#include "backlight.h"
#include "board.h"

namespace {
constexpr const char* kTag = "BvSettings";
constexpr int kMinBrightness = 5;
constexpr int kMaxBrightness = 100;
constexpr int kMinVolume = 0;
constexpr int kMaxVolume = 100;
}  // namespace

BrightnessVolumeScreen::BrightnessVolumeScreen() {
}

BrightnessVolumeScreen::~BrightnessVolumeScreen() {
    if (root_ != nullptr) {
        lv_obj_del(root_);
        root_ = nullptr;
    }
}

void BrightnessVolumeScreen::Create(lv_obj_t* parent) {
    if (root_ != nullptr) {
        return;
    }

    BuildUi(parent);
    Hide();
}

void BrightnessVolumeScreen::BuildUi(lv_obj_t* parent) {
    root_ = lv_obj_create(parent);
    lv_obj_set_size(root_, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_pad_all(root_, 10, 0);
    lv_obj_set_flex_flow(root_, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(root_, 8, 0);
    lv_obj_set_scrollbar_mode(root_, LV_SCROLLBAR_MODE_OFF);
    lv_obj_add_event_cb(root_, OnRootGesture, LV_EVENT_GESTURE, this);
    lv_obj_add_event_cb(root_, OnRootLongPressed, LV_EVENT_LONG_PRESSED, this);

    title_label_ = lv_label_create(root_);
    lv_label_set_text(title_label_, "Settings");

    brightness_row_ = lv_obj_create(root_);
    lv_obj_set_width(brightness_row_, LV_PCT(100));
    lv_obj_set_height(brightness_row_, LV_SIZE_CONTENT);
    lv_obj_set_style_pad_all(brightness_row_, 8, 0);
    lv_obj_set_flex_flow(brightness_row_, LV_FLEX_FLOW_COLUMN);

    lv_obj_t* brightness_header = lv_obj_create(brightness_row_);
    lv_obj_set_style_bg_opa(brightness_header, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(brightness_header, 0, 0);
    lv_obj_set_width(brightness_header, LV_PCT(100));
    lv_obj_set_height(brightness_header, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(brightness_header, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(brightness_header, LV_FLEX_ALIGN_SPACE_BETWEEN,
                          LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_all(brightness_header, 0, 0);

    lv_obj_t* brightness_label = lv_label_create(brightness_header);
    lv_label_set_text(brightness_label, "Brightness");

    brightness_value_label_ = lv_label_create(brightness_header);

    brightness_slider_ = lv_slider_create(brightness_row_);
    lv_obj_set_width(brightness_slider_, LV_PCT(100));
    lv_slider_set_range(brightness_slider_, kMinBrightness, kMaxBrightness);
    lv_obj_add_flag(brightness_row_, LV_OBJ_FLAG_EVENT_BUBBLE);
    lv_obj_add_flag(brightness_slider_, LV_OBJ_FLAG_EVENT_BUBBLE);
    lv_obj_add_event_cb(brightness_slider_, OnSliderValueChanged, LV_EVENT_VALUE_CHANGED, this);

    volume_row_ = lv_obj_create(root_);
    lv_obj_set_width(volume_row_, LV_PCT(100));
    lv_obj_set_height(volume_row_, LV_SIZE_CONTENT);
    lv_obj_set_style_pad_all(volume_row_, 8, 0);
    lv_obj_set_flex_flow(volume_row_, LV_FLEX_FLOW_COLUMN);

    lv_obj_t* volume_header = lv_obj_create(volume_row_);
    lv_obj_set_style_bg_opa(volume_header, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(volume_header, 0, 0);
    lv_obj_set_width(volume_header, LV_PCT(100));
    lv_obj_set_height(volume_header, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(volume_header, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(volume_header, LV_FLEX_ALIGN_SPACE_BETWEEN,
                          LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_all(volume_header, 0, 0);

    lv_obj_t* volume_label = lv_label_create(volume_header);
    lv_label_set_text(volume_label, "Volume");

    volume_value_label_ = lv_label_create(volume_header);

    volume_slider_ = lv_slider_create(volume_row_);
    lv_obj_set_width(volume_slider_, LV_PCT(100));
    lv_slider_set_range(volume_slider_, kMinVolume, kMaxVolume);
    lv_obj_add_flag(volume_row_, LV_OBJ_FLAG_EVENT_BUBBLE);
    lv_obj_add_flag(volume_slider_, LV_OBJ_FLAG_EVENT_BUBBLE);
    lv_obj_add_event_cb(volume_slider_, OnSliderValueChanged, LV_EVENT_VALUE_CHANGED, this);

    footer_row_ = lv_obj_create(root_);
    lv_obj_set_style_bg_opa(footer_row_, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(footer_row_, 0, 0);
    lv_obj_set_width(footer_row_, LV_PCT(100));
    lv_obj_set_height(footer_row_, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(footer_row_, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_all(footer_row_, 0, 0);

    confirm_btn_ = lv_btn_create(footer_row_);
    lv_obj_set_size(confirm_btn_, LV_PCT(48), LV_SIZE_CONTENT);
    lv_obj_add_flag(confirm_btn_, LV_OBJ_FLAG_EVENT_BUBBLE);
    lv_obj_add_event_cb(confirm_btn_, OnButtonClicked, LV_EVENT_CLICKED, this);
    lv_obj_t* confirm_label = lv_label_create(confirm_btn_);
    lv_label_set_text(confirm_label, "Confirm");
    lv_obj_center(confirm_label);

    cancel_btn_ = lv_btn_create(footer_row_);
    lv_obj_set_size(cancel_btn_, LV_PCT(48), LV_SIZE_CONTENT);
    lv_obj_set_style_margin_left(cancel_btn_, LV_PCT(4), 0);
    lv_obj_add_flag(cancel_btn_, LV_OBJ_FLAG_EVENT_BUBBLE);
    lv_obj_add_event_cb(cancel_btn_, OnButtonClicked, LV_EVENT_CLICKED, this);
    lv_obj_t* cancel_label = lv_label_create(cancel_btn_);
    lv_label_set_text(cancel_label, "Cancel");
    lv_obj_center(cancel_label);

    backlight_ = Board::GetInstance().GetBacklight();
    codec_ = Board::GetInstance().GetAudioCodec();

    RefreshLabels();
    ApplyFocusStyle();
}

void BrightnessVolumeScreen::Show() {
    if (root_ == nullptr) {
        ESP_LOGW(kTag, "Show called before Create");
        return;
    }

    visible_ = true;
    edit_mode_ = false;
    focused_ = kFocusBrightness;
    SnapshotEntryValues();

    lv_obj_clear_flag(root_, LV_OBJ_FLAG_HIDDEN);
    ApplyFocusStyle();
}

void BrightnessVolumeScreen::Hide() {
    if (root_ == nullptr) {
        return;
    }

    visible_ = false;
    edit_mode_ = false;
    lv_obj_add_flag(root_, LV_OBJ_FLAG_HIDDEN);
}

void BrightnessVolumeScreen::SnapshotEntryValues() {
    if (backlight_ != nullptr) {
        entry_brightness_ = std::max<int>(kMinBrightness, backlight_->brightness());
        working_brightness_ = entry_brightness_;
        lv_slider_set_value(brightness_slider_, working_brightness_, LV_ANIM_OFF);
    } else {
        lv_obj_add_flag(brightness_row_, LV_OBJ_FLAG_HIDDEN);
        focused_ = kFocusVolume;
    }

    if (codec_ != nullptr) {
        entry_volume_ = std::clamp(codec_->output_volume(), kMinVolume, kMaxVolume);
        working_volume_ = entry_volume_;
        lv_slider_set_value(volume_slider_, working_volume_, LV_ANIM_OFF);
    } else {
        lv_obj_add_flag(volume_row_, LV_OBJ_FLAG_HIDDEN);
        if (focused_ == kFocusVolume) {
            focused_ = kFocusConfirm;
        }
    }

    RefreshLabels();
}

void BrightnessVolumeScreen::RestoreEntryValues() {
    if (backlight_ != nullptr) {
        backlight_->SetBrightness(static_cast<uint8_t>(entry_brightness_), false);
    }
    if (codec_ != nullptr) {
        codec_->SetOutputVolume(entry_volume_);
    }

    working_brightness_ = entry_brightness_;
    working_volume_ = entry_volume_;
    lv_slider_set_value(brightness_slider_, working_brightness_, LV_ANIM_OFF);
    lv_slider_set_value(volume_slider_, working_volume_, LV_ANIM_OFF);
    RefreshLabels();
}

void BrightnessVolumeScreen::RefreshLabels() {
    if (brightness_value_label_ != nullptr) {
        lv_label_set_text_fmt(brightness_value_label_, "%d%%", working_brightness_);
    }
    if (volume_value_label_ != nullptr) {
        lv_label_set_text_fmt(volume_value_label_, "%d%%", working_volume_);
    }
}

void BrightnessVolumeScreen::ApplyFocusStyle() {
    if (root_ == nullptr) {
        return;
    }

    const lv_color_t focus_color = edit_mode_ ? lv_color_hex(0x2D8CFF)
                                              : lv_color_hex(0x2AAE66);
    const lv_opa_t focus_opa = LV_OPA_20;

    lv_obj_set_style_bg_opa(brightness_row_, LV_OPA_TRANSP, 0);
    lv_obj_set_style_bg_opa(volume_row_, LV_OPA_TRANSP, 0);
    lv_obj_set_style_bg_opa(confirm_btn_, LV_OPA_TRANSP, 0);
    lv_obj_set_style_bg_opa(cancel_btn_, LV_OPA_TRANSP, 0);

    switch (focused_) {
        case kFocusBrightness:
            lv_obj_set_style_bg_color(brightness_row_, focus_color, 0);
            lv_obj_set_style_bg_opa(brightness_row_, focus_opa, 0);
            break;
        case kFocusVolume:
            lv_obj_set_style_bg_color(volume_row_, focus_color, 0);
            lv_obj_set_style_bg_opa(volume_row_, focus_opa, 0);
            break;
        case kFocusConfirm:
            lv_obj_set_style_bg_color(confirm_btn_, focus_color, 0);
            lv_obj_set_style_bg_opa(confirm_btn_, focus_opa, 0);
            break;
        case kFocusCancel:
            lv_obj_set_style_bg_color(cancel_btn_, focus_color, 0);
            lv_obj_set_style_bg_opa(cancel_btn_, focus_opa, 0);
            break;
        default:
            break;
    }
}

void BrightnessVolumeScreen::MoveFocus(int8_t delta) {
    if (delta == 0) {
        return;
    }

    int idx = static_cast<int>(focused_);
    for (int i = 0; i < kFocusCount; ++i) {
        idx = (idx + delta + kFocusCount) % kFocusCount;

        if (idx == kFocusBrightness && lv_obj_has_flag(brightness_row_, LV_OBJ_FLAG_HIDDEN)) {
            continue;
        }
        if (idx == kFocusVolume && lv_obj_has_flag(volume_row_, LV_OBJ_FLAG_HIDDEN)) {
            continue;
        }

        focused_ = static_cast<FocusIndex>(idx);
        break;
    }

    ApplyFocusStyle();
}

void BrightnessVolumeScreen::ToggleEditMode() {
    if (focused_ != kFocusBrightness && focused_ != kFocusVolume) {
        return;
    }

    edit_mode_ = !edit_mode_;
    ApplyFocusStyle();
}

void BrightnessVolumeScreen::AdjustFocusedValue(int8_t direction, uint8_t step) {
    if (direction == 0) {
        return;
    }

    const int delta = direction > 0 ? static_cast<int>(step) : -static_cast<int>(step);

    if (focused_ == kFocusBrightness && backlight_ != nullptr) {
        working_brightness_ = std::clamp(working_brightness_ + delta, kMinBrightness, kMaxBrightness);
        lv_slider_set_value(brightness_slider_, working_brightness_, LV_ANIM_OFF);
        backlight_->SetBrightness(static_cast<uint8_t>(working_brightness_), false);
    } else if (focused_ == kFocusVolume && codec_ != nullptr) {
        working_volume_ = std::clamp(working_volume_ + delta, kMinVolume, kMaxVolume);
        lv_slider_set_value(volume_slider_, working_volume_, LV_ANIM_OFF);
        codec_->SetOutputVolume(working_volume_);
    }

    RefreshLabels();
}

void BrightnessVolumeScreen::HandleGesture(lv_dir_t direction) {
    if (!visible_) {
        return;
    }

    switch (direction) {
        case LV_DIR_RIGHT:
        case LV_DIR_BOTTOM:
            if (edit_mode_) {
                AdjustFocusedValue(+1, 1);
            } else {
                MoveFocus(+1);
            }
            break;
        case LV_DIR_LEFT:
        case LV_DIR_TOP:
            if (edit_mode_) {
                AdjustFocusedValue(-1, 1);
            } else {
                MoveFocus(-1);
            }
            break;
        default:
            break;
    }
}

void BrightnessVolumeScreen::HandleLongPressCancel() {
    if (!visible_) {
        return;
    }
    CancelAndClose();
}

void BrightnessVolumeScreen::CommitAndClose() {
    if (backlight_ != nullptr) {
        backlight_->SetBrightness(static_cast<uint8_t>(working_brightness_), true);
    }
    if (codec_ != nullptr) {
        codec_->SetOutputVolume(working_volume_);
    }

    Hide();
    if (close_callback_) {
        close_callback_(true);
    }
}

void BrightnessVolumeScreen::CancelAndClose() {
    RestoreEntryValues();
    Hide();
    if (close_callback_) {
        close_callback_(false);
    }
}

void BrightnessVolumeScreen::HandleAction(SettingsUiAction action, uint8_t step) {
    if (!visible_) {
        return;
    }

    switch (action) {
        case SettingsUiAction::kFocusNext:
            if (edit_mode_) {
                AdjustFocusedValue(+1, step);
            } else {
                MoveFocus(+1);
            }
            break;
        case SettingsUiAction::kFocusPrev:
            if (edit_mode_) {
                AdjustFocusedValue(-1, step);
            } else {
                MoveFocus(-1);
            }
            break;
        case SettingsUiAction::kAdjustIncrease:
            AdjustFocusedValue(+1, step);
            break;
        case SettingsUiAction::kAdjustDecrease:
            AdjustFocusedValue(-1, step);
            break;
        case SettingsUiAction::kEnter:
            if (focused_ == kFocusConfirm) {
                CommitAndClose();
            } else if (focused_ == kFocusCancel) {
                CancelAndClose();
            } else {
                ToggleEditMode();
            }
            break;
        case SettingsUiAction::kCancel:
            CancelAndClose();
            break;
        case SettingsUiAction::kNone:
        default:
            break;
    }
}

void BrightnessVolumeScreen::SetCloseCallback(CloseCallback callback) {
    close_callback_ = std::move(callback);
}

void BrightnessVolumeScreen::OnSliderValueChanged(lv_event_t* event) {
    if (event == nullptr) {
        return;
    }

    auto* self = static_cast<BrightnessVolumeScreen*>(lv_event_get_user_data(event));
    if (self == nullptr) {
        return;
    }

    lv_obj_t* target = static_cast<lv_obj_t*>(lv_event_get_target(event));
    if (target == self->brightness_slider_) {
        self->working_brightness_ = lv_slider_get_value(self->brightness_slider_);
        if (self->backlight_ != nullptr) {
            self->backlight_->SetBrightness(static_cast<uint8_t>(self->working_brightness_), false);
        }
    } else if (target == self->volume_slider_) {
        self->working_volume_ = lv_slider_get_value(self->volume_slider_);
        if (self->codec_ != nullptr) {
            self->codec_->SetOutputVolume(self->working_volume_);
        }
    }

    self->RefreshLabels();
}

void BrightnessVolumeScreen::OnButtonClicked(lv_event_t* event) {
    if (event == nullptr) {
        return;
    }

    auto* self = static_cast<BrightnessVolumeScreen*>(lv_event_get_user_data(event));
    if (self == nullptr) {
        return;
    }

    if (lv_event_get_target(event) == self->confirm_btn_) {
        self->CommitAndClose();
    } else if (lv_event_get_target(event) == self->cancel_btn_) {
        self->CancelAndClose();
    }
}

void BrightnessVolumeScreen::OnRootGesture(lv_event_t* event) {
    if (event == nullptr) {
        return;
    }

    auto* self = static_cast<BrightnessVolumeScreen*>(lv_event_get_user_data(event));
    if (self == nullptr) {
        return;
    }

    lv_indev_t* indev = lv_event_get_indev(event);
    if (indev == nullptr) {
        indev = lv_indev_active();
    }
    if (indev == nullptr) {
        return;
    }

    self->HandleGesture(lv_indev_get_gesture_dir(indev));
}

void BrightnessVolumeScreen::OnRootLongPressed(lv_event_t* event) {
    if (event == nullptr) {
        return;
    }

    auto* self = static_cast<BrightnessVolumeScreen*>(lv_event_get_user_data(event));
    if (self == nullptr) {
        return;
    }

    self->HandleLongPressCancel();
}
