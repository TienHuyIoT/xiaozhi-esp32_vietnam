#ifndef BRIGHTNESS_VOLUME_SCREEN_H_
#define BRIGHTNESS_VOLUME_SCREEN_H_

#include <cstdint>
#include <functional>

#include <lvgl.h>

#include "features/UI/setting/settings_encoder_map.h"

class AudioCodec;
class Backlight;

/**
 * @brief First-pass settings screen for brightness and volume with encoder-focused UX.
 */
class BrightnessVolumeScreen {
public:
    /**
     * @brief Callback fired when screen closes; true means confirm/commit, false means cancel.
     */
    using CloseCallback = std::function<void(bool)>;

    BrightnessVolumeScreen();
    ~BrightnessVolumeScreen();

    /**
     * @brief Builds the screen subtree under parent and keeps it hidden.
     */
    void Create(lv_obj_t* parent);

    /**
     * @brief Shows the settings screen and snapshots entry values.
     */
    void Show();

    /**
     * @brief Hides the settings screen without changing model state.
     */
    void Hide();

    /**
     * @brief Routes a mapped action into focus/edit/commit/cancel behavior.
     */
    void HandleAction(SettingsUiAction action, uint8_t step = 1);

    /**
     * @brief Sets close callback for integration with app navigation.
     */
    void SetCloseCallback(CloseCallback callback);

    /**
     * @brief Returns true when screen is visible and interactive.
     */
    bool visible() const { return visible_; }

    /**
     * @brief Returns true when focused slider is in edit mode.
     */
    bool edit_mode() const { return edit_mode_; }

private:
    enum FocusIndex : uint8_t {
        kFocusBrightness = 0,
        kFocusVolume = 1,
        kFocusConfirm = 2,
        kFocusCancel = 3,
        kFocusCount = 4,
    };

    /**
     * @brief Slider event callback updates preview value and labels.
     */
    static void OnSliderValueChanged(lv_event_t* event);

    /**
     * @brief Root gesture callback maps LVGL swipe directions to settings actions.
     */
    static void OnRootGesture(lv_event_t* event);

    /**
     * @brief Root long-press callback triggers cancel without saving.
     */
    static void OnRootLongPressed(lv_event_t* event);

    /**
     * @brief Button event callback handles touch fallback for confirm/cancel.
     */
    static void OnButtonClicked(lv_event_t* event);

    void BuildUi(lv_obj_t* parent);
    void SnapshotEntryValues();
    void RestoreEntryValues();
    void RefreshLabels();
    void ApplyFocusStyle();
    void MoveFocus(int8_t delta);
    void ToggleEditMode();
    void AdjustFocusedValue(int8_t direction, uint8_t step);
    void HandleGesture(lv_dir_t direction);
    void HandleLongPressCancel();
    void CommitAndClose();
    void CancelAndClose();

    lv_obj_t* root_ = nullptr;
    lv_obj_t* title_label_ = nullptr;
    lv_obj_t* brightness_row_ = nullptr;
    lv_obj_t* brightness_value_label_ = nullptr;
    lv_obj_t* brightness_slider_ = nullptr;
    lv_obj_t* volume_row_ = nullptr;
    lv_obj_t* volume_value_label_ = nullptr;
    lv_obj_t* volume_slider_ = nullptr;
    lv_obj_t* footer_row_ = nullptr;
    lv_obj_t* confirm_btn_ = nullptr;
    lv_obj_t* cancel_btn_ = nullptr;

    Backlight* backlight_ = nullptr;
    AudioCodec* codec_ = nullptr;

    int entry_brightness_ = 75;
    int entry_volume_ = 70;
    int working_brightness_ = 75;
    int working_volume_ = 70;

    FocusIndex focused_ = kFocusBrightness;
    bool edit_mode_ = false;
    bool visible_ = false;

    CloseCallback close_callback_;
};

#endif  // BRIGHTNESS_VOLUME_SCREEN_H_
