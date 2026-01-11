#include "wifi_board.h"
#include "codecs/no_audio_codec.h"
#include "display/lcd_display.h"
#include "system_reset.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "mcp_server.h"
#include "lamp_controller.h"
#include "led/single_led.h"
#include "assets/lang_config.h"

#include <wifi_station.h>
#include <esp_log.h>
#include <driver/i2c_master.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <driver/spi_common.h>

#ifdef CONFIG_TOUCH_PANEL_ENABLE
#include <esp_lcd_touch.h>
#include "esp_lcd_touch_xpt2046.h"
#include "display/lcd_touch.h"
#endif

#ifdef CONFIG_SD_CARD_MMC_INTERFACE
#include "sdmmc.h"
#elif defined(CONFIG_SD_CARD_SPI_INTERFACE)
#include "sdspi.h"
#endif

#if defined(LCD_TYPE_ILI9341_SERIAL)
#include "esp_lcd_ili9341.h"
#elif defined(LCD_TYPE_ILI9488_SERIAL)
#include "esp_lcd_ili9488.h"
#endif

#if defined(LCD_TYPE_GC9A01_SERIAL)
#include "esp_lcd_gc9a01.h"
static const gc9a01_lcd_init_cmd_t gc9107_lcd_init_cmds[] = {
    //  {cmd, { data }, data_size, delay_ms}
    {0xfe, (uint8_t[]){0x00}, 0, 0},
    {0xef, (uint8_t[]){0x00}, 0, 0},
    {0xb0, (uint8_t[]){0xc0}, 1, 0},
    {0xb1, (uint8_t[]){0x80}, 1, 0},
    {0xb2, (uint8_t[]){0x27}, 1, 0},
    {0xb3, (uint8_t[]){0x13}, 1, 0},
    {0xb6, (uint8_t[]){0x19}, 1, 0},
    {0xb7, (uint8_t[]){0x05}, 1, 0},
    {0xac, (uint8_t[]){0xc8}, 1, 0},
    {0xab, (uint8_t[]){0x0f}, 1, 0},
    {0x3a, (uint8_t[]){0x05}, 1, 0},
    {0xb4, (uint8_t[]){0x04}, 1, 0},
    {0xa8, (uint8_t[]){0x08}, 1, 0},
    {0xb8, (uint8_t[]){0x08}, 1, 0},
    {0xea, (uint8_t[]){0x02}, 1, 0},
    {0xe8, (uint8_t[]){0x2A}, 1, 0},
    {0xe9, (uint8_t[]){0x47}, 1, 0},
    {0xe7, (uint8_t[]){0x5f}, 1, 0},
    {0xc6, (uint8_t[]){0x21}, 1, 0},
    {0xc7, (uint8_t[]){0x15}, 1, 0},
    {0xf0,
    (uint8_t[]){0x1D, 0x38, 0x09, 0x4D, 0x92, 0x2F, 0x35, 0x52, 0x1E, 0x0C,
                0x04, 0x12, 0x14, 0x1f},
    14, 0},
    {0xf1,
    (uint8_t[]){0x16, 0x40, 0x1C, 0x54, 0xA9, 0x2D, 0x2E, 0x56, 0x10, 0x0D,
                0x0C, 0x1A, 0x14, 0x1E},
    14, 0},
    {0xf4, (uint8_t[]){0x00, 0x00, 0xFF}, 3, 0},
    {0xba, (uint8_t[]){0xFF, 0xFF}, 2, 0},
};
#endif
 
#define TAG "XiaozhiAIIoTVietNamDTD"

class XiaozhiAIIoTVietNamDTD : public WifiBoard {
private:
 
    Button boot_button_;
    Button volume_up_button_;
    Button volume_down_button_;
    LcdDisplay* display_;
#ifdef CONFIG_TOUCH_PANEL_ENABLE
    LcdTouch *touch_;
    // Touch interrupt semaphore
    SemaphoreHandle_t touch_isr_mux_ = nullptr;
#endif

    void InitializeSpi() {
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = DISPLAY_MOSI_PIN;
        buscfg.miso_io_num = DISPLAY_MISO_PIN;
        buscfg.sclk_io_num = DISPLAY_CLK_PIN;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(DISPLAY_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));

#if defined(CONFIG_TOUCH_PANEL_ENABLE) && !defined(CONFIG_XPT2046_ENABLE_SAME_BUS_AS_LCD)
        spi_bus_config_t touch_buscfg = {};
        touch_buscfg.mosi_io_num = TOUCH_MOSI_PIN;
        touch_buscfg.miso_io_num = TOUCH_MISO_PIN;
        touch_buscfg.sclk_io_num = TOUCH_CLK_PIN;
        touch_buscfg.quadwp_io_num = GPIO_NUM_NC;
        touch_buscfg.quadhd_io_num = GPIO_NUM_NC;
        touch_buscfg.max_transfer_sz = DISPLAY_WIDTH * 80 * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(TOUCH_SPI_HOST, &touch_buscfg, SPI_DMA_CH_AUTO));
#endif
    }

    void InitializeLcdDisplay() {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;
        // LCD screen control IO initialization
        ESP_LOGD(TAG, "Install panel IO");
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = DISPLAY_CS_PIN;
        io_config.dc_gpio_num = DISPLAY_DC_PIN;
        io_config.spi_mode = DISPLAY_SPI_MODE;
        io_config.pclk_hz = 40 * 1000 * 1000;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(DISPLAY_SPI_HOST, &io_config, &panel_io));

        // Initialize LCD driver chip
        ESP_LOGD(TAG, "Install LCD driver");
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = DISPLAY_RST_PIN;
        panel_config.rgb_ele_order = DISPLAY_RGB_ORDER;
        panel_config.bits_per_pixel = 16;
#if defined(LCD_TYPE_ILI9341_SERIAL)
        ESP_ERROR_CHECK(esp_lcd_new_panel_ili9341(panel_io, &panel_config, &panel));
#elif defined(LCD_TYPE_ILI9488_SERIAL)
        // https://github.com/atanisoft/esp_lcd_ili9488/blob/main/examples/lvgl/main/main.c#L67
        constexpr size_t LV_BUFFER_SIZE = DISPLAY_WIDTH * 25;
        ESP_ERROR_CHECK(esp_lcd_new_panel_ili9488(panel_io, &panel_config, LV_BUFFER_SIZE, &panel));
#elif defined(LCD_TYPE_GC9A01_SERIAL)
        ESP_ERROR_CHECK(esp_lcd_new_panel_gc9a01(panel_io, &panel_config, &panel));
        gc9a01_vendor_config_t gc9107_vendor_config = {
            .init_cmds = gc9107_lcd_init_cmds,
            .init_cmds_size = sizeof(gc9107_lcd_init_cmds) / sizeof(gc9a01_lcd_init_cmd_t),
        };        
#else
        ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(panel_io, &panel_config, &panel));
#endif
        
        esp_lcd_panel_reset(panel);

        esp_lcd_panel_init(panel);
        esp_lcd_panel_invert_color(panel, DISPLAY_INVERT_COLOR);
        esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
#ifdef  LCD_TYPE_GC9A01_SERIAL
        panel_config.vendor_config = &gc9107_vendor_config;
#endif
        display_ = new SpiLcdDisplay(panel_io, panel,
                                    DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
    }

#ifdef CONFIG_TOUCH_PANEL_ENABLE
    static void IRAM_ATTR esp_touch_isr_callback(esp_lcd_touch_handle_t tp) {
        if (tp->config.user_data == NULL) {
            return;
        }
        XiaozhiAIIoTVietNamDTD *board = static_cast<XiaozhiAIIoTVietNamDTD *>(tp->config.user_data);
        board->NotifyTouchEvent();
    }

    bool WaitForTouchEvent(TickType_t timeout = portMAX_DELAY) {
        if (touch_isr_mux_ != NULL) {
            BaseType_t result = xSemaphoreTake(touch_isr_mux_, timeout);
            // ESP_LOGW(TAG, "Touch event received");
            vTaskDelay(pdMS_TO_TICKS(10)); // Debounce delay minimum 10ms
            return result == pdTRUE;
        }
        return false;
    }

    void NotifyTouchEvent() {
        if (touch_isr_mux_ != NULL) {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xSemaphoreGiveFromISR(touch_isr_mux_, &xHigherPriorityTaskWoken);
            if( xHigherPriorityTaskWoken != pdFALSE ) {
                portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            }
        }
    }
    void InitializeTouch() {
        ESP_LOGI(TAG, "Initialize touch controller XPT2046...");
        
        // Create touch interrupt semaphore
        touch_isr_mux_ = xSemaphoreCreateBinary();
        if (touch_isr_mux_ == NULL) {
            ESP_LOGE(TAG, "Failed to create touch semaphore");
        }
        
#ifdef TOUCH_RST_PIN
        // Manual reset of touch controller
        if (TOUCH_RST_PIN != GPIO_NUM_NC) {
            /* Prepare pin for touch controller reset */
            ESP_LOGI(TAG, "Resetting touch controller...");
            const gpio_config_t rst_gpio_config = {
                .pin_bit_mask = BIT64(TOUCH_RST_PIN),
                .mode = GPIO_MODE_OUTPUT,
                .pull_up_en = GPIO_PULLUP_DISABLE,
                .pull_down_en = GPIO_PULLDOWN_DISABLE,
                .intr_type = GPIO_INTR_DISABLE
            };
            gpio_config(&rst_gpio_config);
            
            gpio_set_level(TOUCH_RST_PIN, 0);  // Reset low
            vTaskDelay(pdMS_TO_TICKS(10));
            gpio_set_level(TOUCH_RST_PIN, 1);  // Reset high
            vTaskDelay(pdMS_TO_TICKS(200));    // 200ms is a minimum wait for touch controller to boot
            ESP_LOGI(TAG, "Touch controller reset complete");
        }
#endif

        // Touch configuration - enable interrupt, use polling mode
        esp_lcd_touch_config_t tp_cfg = {
            .x_max = DISPLAY_WIDTH - 1,
            .y_max = DISPLAY_HEIGHT - 1,
            // TOUCH_RST_PIN already handled above, should not handle reset here by driver 
            // due to timing delay 10ms is very short and causes issues inside driver
            .rst_gpio_num = GPIO_NUM_NC,   // TOUCH_RST_PIN
            .int_gpio_num = TOUCH_INT_PIN, // TOUCH_INT_PIN
            .levels = {
                .reset = 0,
                .interrupt = 0,
            },
            .flags = {
                .swap_xy = DISPLAY_SWAP_XY ? 1U : 0U,
                .mirror_x = DISPLAY_MIRROR_X ? 1U : 0U,
                .mirror_y = DISPLAY_MIRROR_Y ? 0U : 1U,
            },
            .interrupt_callback = esp_touch_isr_callback,
            .user_data = this
        };
        
        ESP_LOGI(TAG, "Touch config: x_max=%d, y_max=%d, swap_xy=%d, mirror_x=%d, mirror_y=%d",
                tp_cfg.x_max, tp_cfg.y_max, tp_cfg.flags.swap_xy, 
                tp_cfg.flags.mirror_x, tp_cfg.flags.mirror_y);

        esp_err_t ret;
        esp_lcd_panel_io_handle_t tp_io_handle = NULL;
        esp_lcd_panel_io_spi_config_t tp_io_config = ESP_LCD_TOUCH_IO_SPI_XPT2046_CONFIG(TOUCH_CS_PIN);
        ret = esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)TOUCH_SPI_HOST, &tp_io_config, &tp_io_handle);
            if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create touch I2C panel IO: %s", esp_err_to_name(ret));
            return;
        }
        ESP_LOGI(TAG, "Creating touch controller XPT2046");
        esp_lcd_touch_handle_t tp = NULL;
        ret = esp_lcd_touch_new_spi_xpt2046(tp_io_handle, &tp_cfg, &tp);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create FT6x36 touch controller: %s", esp_err_to_name(ret));
            return;
        }
        ESP_LOGI(TAG, "✅ Touch panel XPT2046 initialized successfully with custom driver!");

        touch_ = new SpiLcdTouch(tp, tp_io_handle, 
                            DISPLAY_WIDTH, DISPLAY_HEIGHT, 
                            DISPLAY_SWAP_XY, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
        
        touch_->SetRatioXY(2.0f);
        touch_->SetSwipeThreshold(60); // pixels
                            
        touch_->SetInterruptCallback([this]()->bool {
            return this->WaitForTouchEvent();
        });

        touch_->SetGestureCallback([this](TouchGesture gesture, int16_t x, int16_t y) {
        ESP_LOGI(TAG, "Touch gesture detected: %d at (%d, %d)", static_cast<int>(gesture), x, y);
        switch (gesture) {
            case TOUCH_GESTURE_SWIPE_RIGHT:
            {
                Display::DisplaySourceType source = static_cast<LcdDisplay*>(display_)->DetectSourceFromInfo();
                ESP_LOGI(TAG, "Current source detected: %d", static_cast<int>(source));
                if (source == Display::DisplaySourceType::SD_CARD) {
                ESP_LOGI(TAG, "Play Next track");
                auto& app = Application::GetInstance();
                auto sd_music = app.GetSdMusic();
                if (sd_music) {
                    sd_music->stop();
                    sd_music->next();
                    vTaskDelay(pdMS_TO_TICKS(500));
                }
                } else {
                auto& board = Board::GetInstance();
                auto backlight = board.GetBacklight();
                int new_brightness = backlight->brightness();
                
                // Swipe right - increase brightness
                new_brightness += 5;
                if (new_brightness > 100) new_brightness = 100;
                ESP_LOGI(TAG, "Brightness: %d → %d", backlight->brightness(), new_brightness);
                
                backlight->SetBrightness(new_brightness);
                auto display = board.GetDisplay();
                display->ShowNotification("Brightness: " + std::to_string(new_brightness));
                }
            }
            break;
            case TOUCH_GESTURE_SWIPE_LEFT:
            {
                Display::DisplaySourceType source = static_cast<LcdDisplay*>(display_)->DetectSourceFromInfo();
                ESP_LOGI(TAG, "Current source detected: %d", static_cast<int>(source));
                if (source == Display::DisplaySourceType::SD_CARD) {
                ESP_LOGI(TAG, "Play Previous track");
                auto& app = Application::GetInstance();
                auto sd_music = app.GetSdMusic();
                if (sd_music) {
                    sd_music->stop();
                    sd_music->prev();
                    vTaskDelay(pdMS_TO_TICKS(500));
                }
                break;
                }
                auto& board = Board::GetInstance();
                auto backlight = board.GetBacklight();
                int new_brightness = backlight->brightness();
                
                // Swipe left - decrease brightness
                new_brightness -= 5;
                if (new_brightness <= 0) new_brightness = 0;  // Min 5% to keep visible
                ESP_LOGI(TAG, "Brightness: %d → %d", backlight->brightness(), new_brightness);
                
                backlight->SetBrightness(new_brightness);
                auto display = board.GetDisplay();
                display->ShowNotification("Brightness: " + std::to_string(new_brightness));
            }
            break;
            case TOUCH_GESTURE_SWIPE_DOWN:
            {
                auto codec = GetAudioCodec();
                auto volume = codec->output_volume() - 5;
                if (volume < 0) {
                volume = 0;
                }
                codec->SetOutputVolume(volume);
                GetDisplay()->ShowNotification(Lang::Strings::VOLUME + std::to_string(volume));
            }
            break;
            case TOUCH_GESTURE_SWIPE_UP:
            {
                auto codec = GetAudioCodec();
                auto volume = codec->output_volume() + 5;
                if (volume > 100) {
                volume = 100;
                }
                codec->SetOutputVolume(volume);
                GetDisplay()->ShowNotification(Lang::Strings::VOLUME + std::to_string(volume));
            }
            break;
            case TOUCH_GESTURE_TAP:
            break;
            case TOUCH_GESTURE_DOUBLE_TAP:
            {
                auto& app = Application::GetInstance();
                if (app.GetDeviceState() == kDeviceStateStarting && !WifiStation::GetInstance().IsConnected()) {
                ResetWifiConfiguration();
                }
                app.ToggleChatState();
            }
            break;
            case TOUCH_GESTURE_LONG_PRESS:
            ESP_LOGW(TAG, "Long Press at (%d, %d)", x, y);
            {
                Display::DisplaySourceType source = static_cast<LcdDisplay*>(display_)->DetectSourceFromInfo();
                ESP_LOGI(TAG, "Current source detected: %d", static_cast<int>(source));
                if (source == Display::DisplaySourceType::NONE) {
                auto& app = Application::GetInstance();
                auto sd_music = app.GetSdMusic();
                if (sd_music) {
                    ESP_LOGI(TAG, "Toggle Play/Pause");
                    sd_music->play();
                }
                } else {
                GetAudioCodec()->SetOutputVolume(0);
                GetDisplay()->ShowNotification(Lang::Strings::MUTED);
                }
            }
            break;
            default:
                break;
        } });
        ESP_LOGI(TAG, "Touch screen is ready - try touching now...");
    }
#endif

    void InitializeButtons() {
        boot_button_.OnMultipleClick([this]() {
            ResetWifiConfiguration();
        }, 5);

        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting && !WifiStation::GetInstance().IsConnected()) {
                ResetWifiConfiguration();
            }
            app.ToggleChatState();
        });

        volume_up_button_.OnClick([this]() {
            auto codec = GetAudioCodec();
            auto volume = codec->output_volume() + 10;
            if (volume > 100) {
                volume = 100;
            }
            codec->SetOutputVolume(volume);
            GetDisplay()->ShowNotification(Lang::Strings::VOLUME + std::to_string(volume));
        });

        volume_up_button_.OnLongPress([this]() {
            GetAudioCodec()->SetOutputVolume(100);
            GetDisplay()->ShowNotification(Lang::Strings::MAX_VOLUME);
        });

        volume_down_button_.OnClick([this]() {
            auto codec = GetAudioCodec();
            auto volume = codec->output_volume() - 10;
            if (volume < 0) {
                volume = 0;
            }
            codec->SetOutputVolume(volume);
            GetDisplay()->ShowNotification(Lang::Strings::VOLUME + std::to_string(volume));
        });

        volume_down_button_.OnLongPress([this]() {
            GetAudioCodec()->SetOutputVolume(0);
            GetDisplay()->ShowNotification(Lang::Strings::MUTED);
        });
    }

    // Initialize IoT tools, adding support for AI visible devices
    void InitializeTools() {
        static LampController lamp(LAMP_GPIO);
    }

public:
    XiaozhiAIIoTVietNamDTD() :
        boot_button_(BOOT_BUTTON_GPIO),
        volume_up_button_(VOLUME_UP_BUTTON_GPIO),
        volume_down_button_(VOLUME_DOWN_BUTTON_GPIO) {
        InitializeSpi();
        InitializeLcdDisplay();
#ifdef CONFIG_TOUCH_PANEL_ENABLE
        InitializeTouch();
#endif
        InitializeButtons();
        InitializeTools();
        if (DISPLAY_BACKLIGHT_PIN != GPIO_NUM_NC) {
            GetBacklight()->RestoreBrightness();
        }
    }

    virtual Led* GetLed() override {
        static SingleLed led(BUILTIN_LED_GPIO);
        return &led;
    }

    virtual AudioCodec* GetAudioCodec() override {
#ifdef AUDIO_I2S_METHOD_SIMPLEX
        static NoAudioCodecSimplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_SPK_GPIO_BCLK, AUDIO_I2S_SPK_GPIO_LRCK, AUDIO_I2S_SPK_GPIO_DOUT, AUDIO_I2S_MIC_GPIO_SCK, AUDIO_I2S_MIC_GPIO_WS, AUDIO_I2S_MIC_GPIO_DIN);
#else
        static NoAudioCodecDuplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN);
#endif
        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }

#ifdef CONFIG_TOUCH_PANEL_ENABLE
    virtual LcdTouch *GetTouch() override { return touch_; }
#endif

    virtual Backlight* GetBacklight() override {
        if (DISPLAY_BACKLIGHT_PIN != GPIO_NUM_NC) {
            static PwmBacklight backlight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
            return &backlight;
        }
        return nullptr;
    }

#ifdef CONFIG_SD_CARD_MMC_INTERFACE
    virtual SdCard* GetSdCard() override {
#ifdef CARD_SDMMC_BUS_WIDTH_4BIT
        static SdMMC sdmmc(CARD_SDMMC_CLK_GPIO,
                           CARD_SDMMC_CMD_GPIO,
                           CARD_SDMMC_D0_GPIO,
                           CARD_SDMMC_D1_GPIO,
                           CARD_SDMMC_D2_GPIO,
                           CARD_SDMMC_D3_GPIO);
#else
        static SdMMC sdmmc(CARD_SDMMC_CLK_GPIO,
                           CARD_SDMMC_CMD_GPIO,
                           CARD_SDMMC_D0_GPIO);
#endif
        return &sdmmc;
    }
#endif
#ifdef CONFIG_SD_CARD_SPI_INTERFACE
    virtual SdCard* GetSdCard() override {
        static SdSPI sdspi(CARD_SPI_MISO_GPIO,
                           CARD_SPI_MOSI_GPIO,
                           CARD_SPI_SCLK_GPIO,
                           CARD_SPI_CS_GPIO);
        return &sdspi;
    }
#endif
};

DECLARE_BOARD(XiaozhiAIIoTVietNamDTD);
