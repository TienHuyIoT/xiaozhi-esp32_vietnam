
#include "otto_emoji_display.h"

#include <esp_log.h>
#include <cstring>

#include "assets/lang_config.h"
#include "display/lvgl_display/emoji_collection.h"
#include "display/lvgl_display/lvgl_image.h"
#include "display/lvgl_display/lvgl_theme.h"
#include "otto_emoji_gif.h"

#define TAG "OttoEmojiDisplay"

OttoEmojiDisplay::OttoEmojiDisplay(
    esp_lcd_panel_io_handle_t panel_io,
    esp_lcd_panel_handle_t panel,
    int width, int height,
    int offset_x, int offset_y,
    bool mirror_x, bool mirror_y, bool swap_xy)
    : SpiLcdDisplay(panel_io, panel, width, height,
                    offset_x, offset_y, mirror_x, mirror_y, swap_xy) {
    InitializeOttoEmojis();
    SetupChatLabel();
    SetupPreviewImage();
    SetupFullscreenOtto();
}

void OttoEmojiDisplay::SetupPreviewImage() {
    DisplayLockGuard lock(this);

    if (preview_image_) {
        lv_obj_set_size(preview_image_, width_, height_);
        lv_obj_center(preview_image_);
    }
}

void OttoEmojiDisplay::SetupFullscreenOtto() {
    DisplayLockGuard lock(this);

    // Ẩn giao diện mặc định, giữ lại mặt Otto
    if (status_bar_) {
        lv_obj_add_flag(status_bar_, LV_OBJ_FLAG_HIDDEN);
    }

    if (content_) {
        lv_obj_add_flag(content_, LV_OBJ_FLAG_HIDDEN);
    }

    if (chat_message_label_) {
        lv_obj_add_flag(chat_message_label_, LV_OBJ_FLAG_HIDDEN);
    }

    if (emoji_label_) {
        lv_obj_add_flag(emoji_label_, LV_OBJ_FLAG_HIDDEN);
    }

    if (emoji_image_) {
        lv_obj_clear_flag(emoji_image_, LV_OBJ_FLAG_HIDDEN);
        lv_obj_center(emoji_image_);
    }
}

void OttoEmojiDisplay::InitializeOttoEmojis() {
    ESP_LOGI(TAG, "初始化Otto GIF表情");

    auto otto_emoji_collection = std::make_shared<EmojiCollection>();

    otto_emoji_collection->AddEmoji("staticstate", new LvglRawImage((void*)staticstate.data, staticstate.data_size));
    otto_emoji_collection->AddEmoji("neutral", new LvglRawImage((void*)staticstate.data, staticstate.data_size));
    otto_emoji_collection->AddEmoji("relaxed", new LvglRawImage((void*)staticstate.data, staticstate.data_size));
    otto_emoji_collection->AddEmoji("sleepy", new LvglRawImage((void*)staticstate.data, staticstate.data_size));
    otto_emoji_collection->AddEmoji("idle", new LvglRawImage((void*)staticstate.data, staticstate.data_size));

    otto_emoji_collection->AddEmoji("happy", new LvglRawImage((void*)happy.data, happy.data_size));
    otto_emoji_collection->AddEmoji("laughing", new LvglRawImage((void*)happy.data, happy.data_size));
    otto_emoji_collection->AddEmoji("funny", new LvglRawImage((void*)happy.data, happy.data_size));
    otto_emoji_collection->AddEmoji("loving", new LvglRawImage((void*)happy.data, happy.data_size));
    otto_emoji_collection->AddEmoji("confident", new LvglRawImage((void*)happy.data, happy.data_size));
    otto_emoji_collection->AddEmoji("winking", new LvglRawImage((void*)happy.data, happy.data_size));
    otto_emoji_collection->AddEmoji("cool", new LvglRawImage((void*)happy.data, happy.data_size));
    otto_emoji_collection->AddEmoji("delicious", new LvglRawImage((void*)happy.data, happy.data_size));
    otto_emoji_collection->AddEmoji("kissy", new LvglRawImage((void*)happy.data, happy.data_size));
    otto_emoji_collection->AddEmoji("silly", new LvglRawImage((void*)happy.data, happy.data_size));

    otto_emoji_collection->AddEmoji("sad", new LvglRawImage((void*)sad.data, sad.data_size));
    otto_emoji_collection->AddEmoji("crying", new LvglRawImage((void*)sad.data, sad.data_size));

    otto_emoji_collection->AddEmoji("anger", new LvglRawImage((void*)anger.data, anger.data_size));
    otto_emoji_collection->AddEmoji("angry", new LvglRawImage((void*)anger.data, anger.data_size));

    otto_emoji_collection->AddEmoji("scare", new LvglRawImage((void*)scare.data, scare.data_size));
    otto_emoji_collection->AddEmoji("surprised", new LvglRawImage((void*)scare.data, scare.data_size));
    otto_emoji_collection->AddEmoji("shocked", new LvglRawImage((void*)scare.data, scare.data_size));

    otto_emoji_collection->AddEmoji("buxue", new LvglRawImage((void*)buxue.data, buxue.data_size));
    otto_emoji_collection->AddEmoji("thinking", new LvglRawImage((void*)buxue.data, buxue.data_size));
    otto_emoji_collection->AddEmoji("confused", new LvglRawImage((void*)buxue.data, buxue.data_size));
    otto_emoji_collection->AddEmoji("embarrassed", new LvglRawImage((void*)buxue.data, buxue.data_size));

    auto& theme_manager = LvglThemeManager::GetInstance();
    auto light_theme = theme_manager.GetTheme("light");
    auto dark_theme = theme_manager.GetTheme("dark");

    if (light_theme) {
        light_theme->set_emoji_collection(otto_emoji_collection);
    }

    if (dark_theme) {
        dark_theme->set_emoji_collection(otto_emoji_collection);
    }

    SetEmotion("staticstate");

    ESP_LOGI(TAG, "Otto GIF表情初始化完成");
}

void OttoEmojiDisplay::SetupChatLabel() {
    DisplayLockGuard lock(this);

    if (chat_message_label_) {
        lv_obj_del(chat_message_label_);
    }

    chat_message_label_ = lv_label_create(container_);
    lv_label_set_text(chat_message_label_, "");
    lv_obj_set_width(chat_message_label_, width_ * 0.9);
    lv_label_set_long_mode(chat_message_label_, LV_LABEL_LONG_SCROLL_CIRCULAR);
    lv_obj_set_style_text_align(chat_message_label_, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(chat_message_label_, lv_color_white(), 0);

    SetTheme(LvglThemeManager::GetInstance().GetTheme("dark"));
}

LV_FONT_DECLARE(OTTO_ICON_FONT);

void OttoEmojiDisplay::SetStatus(const char* status) {
    if (!status) {
        ESP_LOGE(TAG, "SetStatus: status is nullptr");
        return;
    }

    auto lvgl_theme = static_cast<LvglTheme*>(current_theme_);
    auto text_font = lvgl_theme->text_font()->font();

    DisplayLockGuard lock(this);

    if (strcmp(status, Lang::Strings::LISTENING) == 0) {
        lv_obj_set_style_text_font(status_label_, &OTTO_ICON_FONT, 0);
        lv_label_set_text(status_label_, "\xEF\x84\xB0");
        lv_obj_add_flag(status_label_, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(network_label_, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(battery_label_, LV_OBJ_FLAG_HIDDEN);
        return;
    }

    if (strcmp(status, Lang::Strings::SPEAKING) == 0) {
        lv_obj_set_style_text_font(status_label_, &OTTO_ICON_FONT, 0);
        lv_label_set_text(status_label_, "\xEF\x80\xA8");
        lv_obj_add_flag(status_label_, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(network_label_, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(battery_label_, LV_OBJ_FLAG_HIDDEN);
        return;
    }

    if (strcmp(status, Lang::Strings::CONNECTING) == 0) {
        lv_obj_set_style_text_font(status_label_, &OTTO_ICON_FONT, 0);
        lv_label_set_text(status_label_, "\xEF\x83\x81");
        return;
    }

    if (strcmp(status, Lang::Strings::STANDBY) == 0) {
        lv_obj_set_style_text_font(status_label_, text_font, 0);
        lv_label_set_text(status_label_, "");
        return;
    }

    lv_obj_set_style_text_font(status_label_, text_font, 0);
    lv_label_set_text(status_label_, status);
}

void OttoEmojiDisplay::SetEmotion(const char* emotion) {
    SpiLcdDisplay::SetEmotion(emotion);

    if (!emotion || !emoji_image_ || !current_theme_) {
        return;
    }

    auto theme = static_cast<LvglTheme*>(current_theme_);
    auto collection = theme->emoji_collection();

    if (!collection) {
        return;
    }

    auto image = collection->GetEmojiImage(emotion);

    if (!image || !image->image_dsc()) {
        return;
    }

    const auto* desc = image->image_dsc();

    if (desc->header.w == 0 || desc->header.h == 0) {
        return;
    }

    uint32_t scale_x = (uint32_t)width_ * 256 / desc->header.w;
    uint32_t scale_y = (uint32_t)height_ * 256 / desc->header.h;
    uint32_t scale = scale_x < scale_y ? scale_x : scale_y;

    DisplayLockGuard lock(this);
    lv_image_set_scale(emoji_image_, scale);
    lv_obj_center(emoji_image_);
    lv_obj_clear_flag(emoji_image_, LV_OBJ_FLAG_HIDDEN);
}

void OttoEmojiDisplay::SetPreviewImage(std::unique_ptr<LvglImage> image) {
    DisplayLockGuard lock(this);

    if (!preview_image_) {
        ESP_LOGE(TAG, "Preview image is not initialized");
        return;
    }

    if (!image) {
        esp_timer_stop(preview_timer_);
        lv_obj_add_flag(preview_image_, LV_OBJ_FLAG_HIDDEN);
        preview_image_cached_.reset();

        if (gif_controller_) {
            gif_controller_->Start();
        }

        lv_obj_clear_flag(emoji_image_, LV_OBJ_FLAG_HIDDEN);
        return;
    }

    preview_image_cached_ = std::move(image);
    auto img_dsc = preview_image_cached_->image_dsc();

    lv_image_set_src(preview_image_, img_dsc);
    lv_image_set_rotation(preview_image_, -900);

    if (img_dsc->header.w > 0 && img_dsc->header.h > 0) {
        uint32_t scale_x = (uint32_t)width_ * 256 / img_dsc->header.w;
        uint32_t scale_y = (uint32_t)height_ * 256 / img_dsc->header.h;
        lv_image_set_scale(preview_image_,
                           scale_x < scale_y ? scale_x : scale_y);
    }

    if (gif_controller_) {
        gif_controller_->Stop();
    }

    lv_obj_add_flag(emoji_image_, LV_OBJ_FLAG_HIDDEN);
    lv_obj_remove_flag(preview_image_, LV_OBJ_FLAG_HIDDEN);
    lv_obj_center(preview_image_);

    esp_timer_stop(preview_timer_);
    ESP_ERROR_CHECK(
        esp_timer_start_once(preview_timer_,
                              PREVIEW_IMAGE_DURATION_MS * 1000));
}
