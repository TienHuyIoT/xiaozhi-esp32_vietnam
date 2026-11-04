
void OttoEmojiDisplay::SetupFullscreenOtto() {
    DisplayLockGuard lock(this);

    // Ẩn thanh trạng thái, nhưng không ẩn content_
    if (status_bar_) {
        lv_obj_add_flag(status_bar_, LV_OBJ_FLAG_HIDDEN);
    }

    // content_ là đối tượng cha của emoji_box_ và emoji_image_.
    // Phải giữ content_ hiển thị.
    if (content_) {
        lv_obj_clear_flag(content_, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_size(content_, width_, height_);
        lv_obj_set_pos(content_, 0, 0);

        lv_obj_set_style_pad_all(content_, 0, 0);
        lv_obj_set_style_border_width(content_, 0, 0);
        lv_obj_set_style_bg_opa(content_, LV_OPA_TRANSP, 0);
    }

    // Cho vùng chứa Otto phủ toàn bộ màn hình
    if (emoji_box_) {
        lv_obj_clear_flag(emoji_box_, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_size(emoji_box_, width_, height_);
        lv_obj_center(emoji_box_);

        lv_obj_set_style_pad_all(emoji_box_, 0, 0);
        lv_obj_set_style_border_width(emoji_box_, 0, 0);
        lv_obj_set_style_bg_opa(emoji_box_, LV_OPA_TRANSP, 0);
    }

    // Ẩn chữ và thông báo cũ
    if (chat_message_label_) {
        lv_obj_add_flag(chat_message_label_, LV_OBJ_FLAG_HIDDEN);
    }

    if (emoji_label_) {
        lv_obj_add_flag(emoji_label_, LV_OBJ_FLAG_HIDDEN);
    }

    // Hiển thị khuôn mặt Otto
    if (emoji_image_) {
        lv_obj_clear_flag(emoji_image_, LV_OBJ_FLAG_HIDDEN);
        lv_obj_center(emoji_image_);
    }

    // Ảnh xem trước cũng phủ toàn màn hình
    if (preview_image_) {
        lv_obj_set_size(preview_image_, width_, height_);
        lv_obj_center(preview_image_);
        lv_obj_add_flag(preview_image_, LV_OBJ_FLAG_HIDDEN);
    }
}
