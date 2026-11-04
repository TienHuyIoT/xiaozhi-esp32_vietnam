
#include <driver/spi_common.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_log.h>
#include <wifi_station.h>

#include "application.h"
#include "codecs/no_audio_codec.h"
#include "button.h"
#include "config.h"
#include "display/lcd_display.h"
#include "otto_emoji_display.h"
#include "otto_gpio_config.h"
#include "otto_head_controller.h"
#include "system_reset.h"
#include "wifi_board.h"
#include "websocket_control_server.h"

#define TAG "OttoRobot"

class OttoRobot : public WifiBoard {
private:
    LcdDisplay* display_ = nullptr;

    // GPIO configuration loaded from NVS.
    OttoGpioConfig gpio_config_;

    // Dedicated 2-servo head controller.
    OttoHeadController head_controller_;

    Button boot_button_;
    WebSocketControlServer* ws_control_server_ = nullptr;

    void InitializeSpi() {
        spi_bus_config_t buscfg = {};

        buscfg.mosi_io_num = gpio_config_.display_mosi;
        buscfg.miso_io_num = GPIO_NUM_NC;
        buscfg.sclk_io_num = gpio_config_.display_clk;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz =
            DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);

        ESP_ERROR_CHECK(
            spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO)
        );
    }

    void InitializeLcdDisplay() {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;

        ESP_LOGD(TAG, "Install panel IO");

        esp_lcd_panel_io_spi_config_t io_config = {};

        io_config.cs_gpio_num = gpio_config_.display_cs;
        io_config.dc_gpio_num = gpio_config_.display_dc;
        io_config.spi_mode = DISPLAY_SPI_MODE;
        io_config.pclk_hz = 40 * 1000 * 1000;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;

        ESP_ERROR_CHECK(
            esp_lcd_new_panel_io_spi(
                SPI3_HOST,
                &io_config,
                &panel_io
            )
        );

        ESP_LOGD(TAG, "Install LCD driver");

        esp_lcd_panel_dev_config_t panel_config = {};

        panel_config.reset_gpio_num = gpio_config_.display_rst;
        panel_config.rgb_ele_order = DISPLAY_RGB_ORDER;
        panel_config.bits_per_pixel = 16;

        ESP_ERROR_CHECK(
            esp_lcd_new_panel_st7789(
                panel_io,
                &panel_config,
                &panel
            )
        );

        ESP_ERROR_CHECK(esp_lcd_panel_reset(panel));
        ESP_ERROR_CHECK(esp_lcd_panel_init(panel));

        ESP_ERROR_CHECK(
            esp_lcd_panel_invert_color(panel, DISPLAY_INVERT_COLOR)
        );

        ESP_ERROR_CHECK(
            esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY)
        );

        ESP_ERROR_CHECK(
            esp_lcd_panel_mirror(
                panel,
                DISPLAY_MIRROR_X,
                DISPLAY_MIRROR_Y
            )
        );

        display_ = new OttoEmojiDisplay(
            panel_io,
            panel,
            DISPLAY_WIDTH,
            DISPLAY_HEIGHT,
            DISPLAY_OFFSET_X,
            DISPLAY_OFFSET_Y,
            DISPLAY_MIRROR_X,
            DISPLAY_MIRROR_Y,
            DISPLAY_SWAP_XY
        );
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();

            if (app.GetDeviceState() == kDeviceStateStarting &&
                !WifiStation::GetInstance().IsConnected()) {
                ResetWifiConfiguration();
            }

            app.ToggleChatState();
        });
    }

    void InitializeWebSocketControlServer() {
        ESP_LOGI(TAG, "Initialize WebSocket control server");

        ws_control_server_ = new WebSocketControlServer();

        if (!ws_control_server_->Start(8080)) {
            ESP_LOGE(TAG, "Failed to start WebSocket control server");

            delete ws_control_server_;
            ws_control_server_ = nullptr;
        } else {
            ESP_LOGI(
                TAG,
                "GPIO configuration page: http://<robot-ip>:8080/gpio"
            );
        }
    }

    void StartNetwork() override {
        WifiBoard::StartNetwork();

        vTaskDelay(pdMS_TO_TICKS(1000));

        InitializeWebSocketControlServer();
    }

public:
    OttoRobot()
        : gpio_config_(OttoGpioConfigManager::Load()),
          head_controller_(gpio_config_),
          boot_button_(BOOT_BUTTON_GPIO) {

        ESP_LOGI(TAG, "GPIO configuration loaded");

        InitializeSpi();
        InitializeLcdDisplay();
        InitializeButtons();

        GetBacklight()->RestoreBrightness();
    }

    AudioCodec* GetAudioCodec() override {
        static NoAudioCodecSimplex audio_codec(
            AUDIO_INPUT_SAMPLE_RATE,
            AUDIO_OUTPUT_SAMPLE_RATE,
            gpio_config_.spk_bclk,
            gpio_config_.spk_lrck,
            gpio_config_.spk_dout,
            gpio_config_.mic_sck,
            gpio_config_.mic_ws,
            gpio_config_.mic_din
        );

        return &audio_codec;
    }

    Display* GetDisplay() override {
        return display_;
    }

    Backlight* GetBacklight() override {
        static PwmBacklight backlight(
            gpio_config_.display_backlight,
            DISPLAY_BACKLIGHT_OUTPUT_INVERT
        );

        return &backlight;
    }
};

DECLARE_BOARD(OttoRobot);
