#pragma once

#include "display/lcd_display.h"

class OttoEmojiDisplay : public SpiLcdDisplay {
   public:
    OttoEmojiDisplay(esp_lcd_panel_io_handle_t panel_io,
                     esp_lcd_panel_handle_t panel,
                     int width, int height,
                     int offset_x, int offset_y,
                     bool mirror_x, bool mirror_y, bool swap_xy);

    virtual ~OttoEmojiDisplay() = default;

    virtual void SetStatus(const char* status) override;
    virtual void SetEmotion(const char* emotion) override;
    virtual void SetPreviewImage(std::unique_ptr<LvglImage> image) override;

   private:
    void InitializeOttoEmojis();
    void SetupChatLabel();
    void SetupPreviewImage();
    void SetupFullscreenOtto();
};
