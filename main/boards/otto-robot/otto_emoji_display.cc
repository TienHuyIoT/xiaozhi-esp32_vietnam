#include "otto_emoji_display.h"

#include <esp_log.h>

#define TAG "OttoEmojiDisplay"

OttoEmojiDisplay::OttoEmojiDisplay(
    esp_lcd_panel_io_handle_t panel_io,
    esp_lcd_panel_handle_t panel,
    int width,
    int height,
    int offset_x,
    int offset_y,
    bool mirror_x,
    bool mirror_y,
    bool swap_xy
) : SpiLcdDisplay(
        panel_io,
        panel,
        width,
        height,
        offset_x,
        offset_y,
        mirror_x,
        mirror_y,
        swap_xy
    ) {
    SetupFullscreenOtto();
}

void OttoEmojiDisplay::InitializeOttoEmojis() {
    if (emoji_image_ == nullptr) {
        ESP_LOGW(TAG, "Otto emoji image is unavailable");
        return;
    }

    lv_obj_clear_flag(emoji_image_, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(emoji_label_, LV_OBJ_FLAG_HIDDEN);
}

void OttoEmojiDisplay::SetupChatLabel() {
    if (chat_message_label_ != nullptr) {
        lv_obj_add_flag(chat_message_label_, LV_OBJ_FLAG_HIDDEN);
    }
}

void OttoEmojiDisplay::SetupPreviewImage() {
    if (preview_image_ != nullptr) {
        lv_obj_add_flag(preview_image_, LV_OBJ_FLAG_HIDDEN);
    }
}

void OttoEmojiDisplay::SetupFullscreenOtto() {
    DisplayLockGuard lock(this);

    if (status_bar_ != nullptr) {
        lv_obj_add_flag(status_bar_, LV_OBJ_FLAG_HIDDEN);
    }

    if (content_ != nullptr) {
        lv_obj_clear_flag(content_, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_size(content_, width_, height_);
        lv_obj_set_pos(content_, 0, 0);
        lv_obj_set_style_pad_all(content_, 0, 0);
        lv_obj_set_style_border_width(content_, 0, 0);
        lv_obj_set_style_bg_opa(content_, LV_OPA_TRANSP, 0);
    }

    if (emoji_box_ != nullptr) {
        lv_obj_clear_flag(emoji_box_, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_size(emoji_box_, width_, height_);
        lv_obj_center(emoji_box_);
        lv_obj_set_style_pad_all(emoji_box_, 0, 0);
        lv_obj_set_style_border_width(emoji_box_, 0, 0);
        lv_obj_set_style_bg_opa(emoji_box_, LV_OPA_TRANSP, 0);
    }

    SetupChatLabel();
    SetupPreviewImage();
    InitializeOttoEmojis();

    if (emoji_image_ != nullptr) {
        lv_obj_center(emoji_image_);
    }
}

void OttoEmojiDisplay::SetStatus(const char* status) {
    // Ẩn giao diện trạng thái cũ, giữ nguyên xử lý trạng thái của ứng dụng.
    (void)status;
    SetupFullscreenOtto();
}

void OttoEmojiDisplay::SetEmotion(const char* emotion) {
    LcdDisplay::SetEmotion(emotion);
    SetupFullscreenOtto();
}

void OttoEmojiDisplay::SetPreviewImage(std::unique_ptr<LvglImage> image) {
    // Otto giữ giao diện khuôn mặt, không hiển thị ảnh xem trước.
    image.reset();
    SetupFullscreenOtto();
}
