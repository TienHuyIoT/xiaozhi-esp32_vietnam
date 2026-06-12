#include "features/UI/setting/settings_encoder_map.h"

SettingsEncoderMap::SettingsEncoderMap() {
}

void SettingsEncoderMap::SetTiming(const SettingsEncoderTiming& timing) {
    timing_ = timing;
    current_step_ = timing_.normal_step;
}

void SettingsEncoderMap::SetEditMode(bool edit_mode) {
    edit_mode_ = edit_mode;
}

SettingsUiAction SettingsEncoderMap::MoveActionForDelta(int32_t delta) const {
    if (delta > 0) {
        return SettingsUiAction::kFocusNext;
    }
    if (delta < 0) {
        return SettingsUiAction::kFocusPrev;
    }
    return SettingsUiAction::kNone;
}

SettingsUiAction SettingsEncoderMap::AdjustActionForDelta(int32_t delta) const {
    if (delta > 0) {
        return SettingsUiAction::kAdjustIncrease;
    }
    if (delta < 0) {
        return SettingsUiAction::kAdjustDecrease;
    }
    return SettingsUiAction::kNone;
}

SettingsUiAction SettingsEncoderMap::OnRotate(int32_t delta) {
    if (delta == 0) {
        return SettingsUiAction::kNone;
    }
    return edit_mode_ ? AdjustActionForDelta(delta) : MoveActionForDelta(delta);
}

void SettingsEncoderMap::OnButtonState(bool pressed, uint32_t now_ms) {
    if (pressed == pressed_) {
        return;
    }

    pressed_ = pressed;
    if (pressed_) {
        press_start_ms_ = now_ms;
        last_repeat_ms_ = now_ms;
        long_press_fired_ = false;
        current_step_ = timing_.normal_step;
        return;
    }

    const uint32_t held_ms = (now_ms >= press_start_ms_) ? (now_ms - press_start_ms_) : 0;
    if (!long_press_fired_ && held_ms < timing_.long_press_ms) {
        pending_action_ = SettingsUiAction::kEnter;
    }

    press_start_ms_ = 0;
    last_repeat_ms_ = 0;
    long_press_fired_ = false;
    current_step_ = timing_.normal_step;
}

SettingsUiAction SettingsEncoderMap::Poll(uint32_t now_ms) {
    if (pending_action_ != SettingsUiAction::kNone) {
        const SettingsUiAction action = pending_action_;
        pending_action_ = SettingsUiAction::kNone;
        return action;
    }

    if (!pressed_) {
        return SettingsUiAction::kNone;
    }

    const uint32_t held_ms = (now_ms >= press_start_ms_) ? (now_ms - press_start_ms_) : 0;
    current_step_ = (held_ms >= timing_.accel_start_ms) ? timing_.accel_step : timing_.normal_step;

    if (!long_press_fired_ && held_ms >= timing_.long_press_ms) {
        long_press_fired_ = true;
        return SettingsUiAction::kCancel;
    }
    return SettingsUiAction::kNone;
}

uint8_t SettingsEncoderMap::CurrentStep() const {
    return current_step_;
}

void SettingsEncoderMap::Reset() {
    edit_mode_ = false;
    pressed_ = false;
    long_press_fired_ = false;
    press_start_ms_ = 0;
    last_repeat_ms_ = 0;
    pending_action_ = SettingsUiAction::kNone;
    current_step_ = timing_.normal_step;
}
