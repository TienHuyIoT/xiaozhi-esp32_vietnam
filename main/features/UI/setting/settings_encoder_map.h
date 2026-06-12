#ifndef SETTINGS_ENCODER_MAP_H_
#define SETTINGS_ENCODER_MAP_H_

#include <cstdint>

/**
 * @brief Logical UI actions emitted by encoder mapping.
 */
enum class SettingsUiAction {
    kNone = 0,
    kFocusNext,
    kFocusPrev,
    kAdjustIncrease,
    kAdjustDecrease,
    kEnter,
    kCancel,
};

/**
 * @brief Timing knobs for long-press and repeat behavior.
 */
struct SettingsEncoderTiming {
    uint32_t long_press_ms = 450;
    uint32_t repeat_start_delay_ms = 380;
    uint32_t repeat_interval_ms = 80;
    uint32_t accel_start_ms = 1500;
    uint8_t normal_step = 1;
    uint8_t accel_step = 3;
};

/**
 * @brief Maps raw encoder rotation/press into deterministic settings actions.
 */
class SettingsEncoderMap {
public:
    SettingsEncoderMap();

    /**
     * @brief Sets timing and step behavior used by the mapper.
     */
    void SetTiming(const SettingsEncoderTiming& timing);

    /**
     * @brief Informs mapper whether current UI state is slider edit mode.
     */
    void SetEditMode(bool edit_mode);

    /**
     * @brief Handles a raw encoder rotation delta and returns a single action.
     */
    SettingsUiAction OnRotate(int32_t delta);

    /**
     * @brief Handles encoder button press state transitions.
     */
    void OnButtonState(bool pressed, uint32_t now_ms);

    /**
     * @brief Polls timed events (long-press/repeat) and returns pending action.
     */
    SettingsUiAction Poll(uint32_t now_ms);

    /**
     * @brief Returns the current step size (normal/accelerated).
     */
    uint8_t CurrentStep() const;

    /**
     * @brief Clears button state and pending action.
     */
    void Reset();

private:
    SettingsUiAction MoveActionForDelta(int32_t delta) const;
    SettingsUiAction AdjustActionForDelta(int32_t delta) const;

    SettingsEncoderTiming timing_;
    bool edit_mode_ = false;
    bool pressed_ = false;
    bool long_press_fired_ = false;
    uint32_t press_start_ms_ = 0;
    uint32_t last_repeat_ms_ = 0;
    SettingsUiAction pending_action_ = SettingsUiAction::kNone;
    uint8_t current_step_ = 1;
};

#endif  // SETTINGS_ENCODER_MAP_H_
