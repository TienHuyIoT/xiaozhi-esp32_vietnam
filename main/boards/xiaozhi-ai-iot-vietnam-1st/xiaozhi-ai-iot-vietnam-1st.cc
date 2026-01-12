#include "wifi_board.h"
#include "codecs/box_audio_codec.h"
#include "display/lcd_display.h"
#ifdef CONFIG_SD_CARD_MMC_INTERFACE
#include "sdmmc.h"
#elif defined(CONFIG_SD_CARD_SPI_INTERFACE)
#include "sdspi.h"
#endif
#include "system_reset.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "power_save_timer.h"
#include "led/single_led.h"
#include "assets/lang_config.h"
#include "power_manager.h"

#include <esp_log.h>
#include <esp_lcd_panel_vendor.h>
#include <wifi_station.h>

#include <driver/rtc_io.h>
#include <esp_sleep.h>

#include <driver/i2c_master.h>
#include "codecs/box_audio_codec.h"
#include "ssid_manager.h"

#include <esp_lcd_nv3023.h>
#include "settings.h"
#include "pm.h"

#define TAG "XINGZHI_CUBE_1_83TFT_WIFI_1ST"
static const nv3023_lcd_init_cmd_t lcd_init_cmds[] = {
    {0xfd,(const uint8_t[]){0x06,0x08},2,0},
	{0x61,(const uint8_t[]){0x07,0x04},2,0},
	{0x62,(const uint8_t[]){0x00,0x44,0x45},3,0},
	{0x63,(const uint8_t[]){0x41,0x07,0x12,0x12},4,0},
	{0x64,(const uint8_t[]){0x37},1,0},
	{0x65,(const uint8_t[]){0x09,0x10,0x21},3,0},
	{0x66,(const uint8_t[]){0x09,0x10,0x21},3,0},
	{0x67,(const uint8_t[]){0x20,0x40},2,0},
	{0x68,(const uint8_t[]){0x90,0x4c,0x7C,0x66},4,0},
	{0xb1,(const uint8_t[]){0x0F,0x02,0x01},3,0},
	{0xB4,(const uint8_t[]){0x01},1,0},
	{0xB5,(const uint8_t[]){0x02,0x02,0x0a,0x14},4,0},
	{0xB6,(const uint8_t[]){0x04,0x01,0x9f,0x00,0x02},5,0},
	{0xdf,(const uint8_t[]){0x11},1,0},
	{0xE2,(const uint8_t[]){0x13,0x00,0x00,0x30,0x33,0x3f},6,0},
	{0xE5,(const uint8_t[]){0x3f,0x33,0x30,0x00,0x00,0x13},6,0},
	{0xE1,(const uint8_t[]){0x00,0x57},2,0},
	{0xE4,(const uint8_t[]){0x58,0x00},2,0},
	{0xE0,(const uint8_t[]){0x01,0x03,0x0d,0x0e,0x0e,0x0c,0x15,0x19},8,0},
	{0xE3,(const uint8_t[]){0x1a,0x16,0x0C,0x0f,0x0e,0x0d,0x02,0x01},8,0},
	{0xE6,(const uint8_t[]){0x00,0xff},2,0},
	{0xE7,(const uint8_t[]){0x01,0x04,0x03,0x03,0x00,0x12},6,0},
	{0xE8,(const uint8_t[]){0x00,0x70,0x00},3,0},
	{0xEc,(const uint8_t[]){0x52},1,0},
	{0xF1,(const uint8_t[]){0x01,0x01,0x02},3,0},
	{0xF6,(const uint8_t[]){0x09,0x10,0x00,0x00},4,0},
	{0xfd,(const uint8_t[]){0xfa,0xfc},2,0},
	{0x3a,(const uint8_t[]){0x05},1,0},
	{0x35,(const uint8_t[]){0x00},1,0},
	{0x36,(const uint8_t[]){0x08},1,0},
	{0x36,(const uint8_t[]){0xc8},1,0},
	{0x36,(const uint8_t[]){0x78},1,0},
	{0x36,(const uint8_t[]){0xa8},1,0},
	{0x21,(const uint8_t[]){0},0,0},
	{0x11,(const uint8_t[]){0},0,200},
	{0x29,(const uint8_t[]){0},0,10},
};

class LogoLcdDisplay : public SpiLcdDisplay {
public:
    LogoLcdDisplay(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_handle_t panel,
                  int width, int height, int offset_x, int offset_y,
                  bool mirror_x, bool mirror_y, bool swap_xy)
        : SpiLcdDisplay(panel_io, panel, width, height, offset_x, offset_y, mirror_x, mirror_y, swap_xy) {}

    void Logo() {
        DisplayLockGuard lock(this);
        lv_obj_t * img = lv_img_create(lv_layer_top());
        if (img == NULL) {
            ESP_LOGE(TAG, "Failed to create LVGL image object");
            return;
        }

        LV_IMG_DECLARE(xiaozhi_ai_iot_vietnam_logo);
        lv_img_set_src(img, &xiaozhi_ai_iot_vietnam_logo);
        if (lv_img_get_src(img) == NULL) {
            ESP_LOGE("LVGL", "Failed to load image from /spiffs/logo.png");
            lv_obj_del(img);
            return;
        }

        lv_obj_center(img);
        ESP_LOGI("LVGL", "load image from logo.png");

        const TickType_t end_time = xTaskGetTickCount() + pdMS_TO_TICKS(5000);
        while (xTaskGetTickCount() < end_time) {
            lv_task_handler();
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        if (img != NULL && lv_obj_is_valid(img)) {
            ESP_LOGI("LVGL", "Deleting image object");
            lv_obj_del(img);
        } else {
            ESP_LOGE("LVGL", "Image object is invalid or already deleted");
        }

    }
};

class XINGZHI_CUBE_1_83TFT_WIFI_1ST : public WifiBoard {
private:
    Button boot_button_;
    Button volume_up_button_;
    Button volume_down_button_;
    LogoLcdDisplay* display_;
    PowerSaveTimer* power_save_timer_;
    PowerManager* power_manager_;
    esp_lcd_panel_io_handle_t panel_io_ = nullptr;
    esp_lcd_panel_handle_t panel_ = nullptr;

    i2c_master_bus_handle_t i2c_bus_;
    void InitializeI2c() {
        // Initialize I2C peripheral
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = (i2c_port_t)1,
            .sda_io_num = AUDIO_CODEC_I2C_SDA_PIN,
            .scl_io_num = AUDIO_CODEC_I2C_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus_));
    }

    void InitializePowerManager() {
        power_manager_ = new PowerManager(GPIO_NUM_47);
        power_manager_->OnChargingStatusChanged([this](bool is_charging) {
            if (is_charging) {
                power_save_timer_->SetEnabled(false);
            } else {
                power_save_timer_->SetEnabled(true);
            }
        });
    }

    void InitializePowerSaveTimer() {      
        power_save_timer_ = new PowerSaveTimer(-1, SECONDS_TO_SLEEP_MODE, SECONDS_TO_SHUTDOWN);
        power_save_timer_->OnEnterSleepMode([this]() {
            ESP_LOGI(TAG, "Enabling sleep mode");
            display_->SetChatMessage("system", "");
            display_->SetEmotion("sleepy");
            GetBacklight()->SetBrightness(1);
        });
        power_save_timer_->OnExitSleepMode([this]() {
            display_->SetChatMessage("system", "");
            display_->SetEmotion("neutral");

            // Send the sleep exit command (0x11) and the display on command (0x29).
            esp_lcd_panel_io_tx_param(panel_io_, 0x11, NULL, 0);
            esp_lcd_panel_io_tx_param(panel_io_, 0x29, NULL, 0);
            GetBacklight()->RestoreBrightness();
        });
        power_save_timer_->OnShutdownRequest([this]() {
            ESP_LOGI(TAG, "Shutting down");
            // Send the display off command (0x28) and the sleep enter command (0x10).
            esp_lcd_panel_io_tx_param(panel_io_, 0x28, NULL, 0);
            esp_lcd_panel_io_tx_param(panel_io_, 0x10, NULL, 0);
            pm_low_power_shutdown();
        });
        power_save_timer_->SetEnabled(true);
    }

    void InitializeSpi() {
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = DISPLAY_SDA;
        buscfg.miso_io_num = GPIO_NUM_NC;
        buscfg.sclk_io_num = DISPLAY_SCL;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_HEIGHT * DISPLAY_WIDTH *sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }
    
    void InitializeButtons() {
        boot_button_.OnMultipleClick([this]() {
            ResetWifiConfiguration();
        }, 6);

        boot_button_.OnClick([this]() {
            power_save_timer_->WakeUp();
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting && !WifiStation::GetInstance().IsConnected()) {
                ResetWifiConfiguration();
            }
            app.ToggleChatState();
        });

        volume_up_button_.OnClick([this]() {
            power_save_timer_->WakeUp();
            auto codec = GetAudioCodec();
            auto volume = codec->output_volume() + 10;
            if (volume > 100) {
                volume = 100;
            }
            codec->SetOutputVolume(volume);
            GetDisplay()->ShowNotification(Lang::Strings::VOLUME + std::to_string(volume));
        });

        volume_up_button_.OnLongPress([this]() {
            power_save_timer_->WakeUp();
            GetAudioCodec()->SetOutputVolume(100);
            GetDisplay()->ShowNotification(Lang::Strings::MAX_VOLUME);
        });

        volume_down_button_.OnClick([this]() {
            power_save_timer_->WakeUp();
            auto codec = GetAudioCodec();
            auto volume = codec->output_volume() - 10;
            if (volume < 0) {
                volume = 0;
            }
            codec->SetOutputVolume(volume);
            GetDisplay()->ShowNotification(Lang::Strings::VOLUME + std::to_string(volume));
        });

        volume_down_button_.OnLongPress([this]() {
            power_save_timer_->WakeUp();
            GetAudioCodec()->SetOutputVolume(0);
            GetDisplay()->ShowNotification(Lang::Strings::MUTED);
        });
    } 

    void InitializeNv3023Display() {
        ESP_LOGD(TAG, "Install panel IO");
        esp_lcd_panel_io_spi_config_t io_config = NV3023_PANEL_IO_SPI_CONFIG(DISPLAY_CS, DISPLAY_DC, NULL, NULL);
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI3_HOST, &io_config, &panel_io_));

        ESP_LOGD(TAG, "Install LCD driver");
        esp_lcd_panel_dev_config_t panel_config = {};
        nv3023_vendor_config_t vendor_config = {  // Uncomment these lines if use custom initialization commands
            .init_cmds = lcd_init_cmds,
            .init_cmds_size = sizeof(lcd_init_cmds) / sizeof(nv3023_lcd_init_cmd_t),
        };
        panel_config.reset_gpio_num = DISPLAY_RES;
        panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR;
        panel_config.bits_per_pixel = 16;
        panel_config.vendor_config = &vendor_config;

        ESP_ERROR_CHECK(esp_lcd_new_panel_nv3023(panel_io_, &panel_config, &panel_));
        ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_));
        ESP_ERROR_CHECK(esp_lcd_panel_init(panel_));
        ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_, DISPLAY_SWAP_XY));
        ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y));
        ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_, true));
        ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_, true));
        
        display_ = new LogoLcdDisplay(panel_io_, panel_, DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, 
            DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
    }

public:
    XINGZHI_CUBE_1_83TFT_WIFI_1ST():
        boot_button_(BOOT_BUTTON_GPIO),
        volume_up_button_(VOLUME_UP_BUTTON_GPIO),
        volume_down_button_(VOLUME_DOWN_BUTTON_GPIO) {     
        InitializeI2c();
        InitializePowerManager();
        InitializePowerSaveTimer();
        InitializeSpi();
        InitializeButtons();
        InitializeNv3023Display();
        GetBacklight()->RestoreBrightness();

        display_->Logo();
    }

    virtual AudioCodec* GetAudioCodec() override { 
        static BoxAudioCodec audio_codec(i2c_bus_, AUDIO_INPUT_SAMPLE_RATE,  AUDIO_OUTPUT_SAMPLE_RATE,AUDIO_I2S_GPIO_MCLK, 
            AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN,AUDIO_CODEC_PA_PIN, AUDIO_CODEC_ES8311_ADDR, 
            AUDIO_CODEC_ES7210_ADDR, AUDIO_INPUT_REFERENCE);
        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }
    
    virtual Backlight* GetBacklight() override {
        static PwmBacklight backlight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
        return &backlight;
    }

    virtual bool GetBatteryLevel(int& level, bool& charging, bool& discharging) override {
        static bool last_discharging = false;
        charging = power_manager_->IsCharging();
        discharging = power_manager_->IsDischarging();
        if (discharging != last_discharging) {
            power_save_timer_->SetEnabled(discharging);
            last_discharging = discharging;
        }
        level = power_manager_->GetBatteryLevel();
        return true;
    }

    virtual void SetPowerSaveMode(bool enabled) override {
        if (!enabled) {
            power_save_timer_->WakeUp();
        }
        WifiBoard::SetPowerSaveMode(enabled);
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
#ifdef CARD_SDMMC_D3_GPIO
        if (CARD_SDMMC_D3_GPIO != GPIO_NUM_NC) {
            gpio_set_direction(CARD_SDMMC_D3_GPIO, GPIO_MODE_INPUT);
            gpio_pullup_en(CARD_SDMMC_D3_GPIO);
            vTaskDelay(pdMS_TO_TICKS(10)); // Wait for the pin to stabilize
        }
#endif
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

DECLARE_BOARD(XINGZHI_CUBE_1_83TFT_WIFI_1ST);
