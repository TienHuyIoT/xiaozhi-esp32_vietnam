
#include "otto_gpio_config.h"
#include "config.h"
#include "settings.h"

#include <esp_log.h>
#include <driver/gpio.h>
#include <set>

static const char* TAG = "OttoGPIO";
static const char* NVS_NAMESPACE = "otto_gpio";

// ============================================================
// LOAD GPIO CONFIGURATION
// ============================================================

OttoGpioConfig OttoGpioConfigManager::Load() {
    OttoGpioConfig c = {
        // INMP441 microphone
        (int)AUDIO_I2S_MIC_GPIO_WS,
        (int)AUDIO_I2S_MIC_GPIO_SCK,
        (int)AUDIO_I2S_MIC_GPIO_DIN,

        // MAX98357A speaker
        (int)AUDIO_I2S_SPK_GPIO_DOUT,
        (int)AUDIO_I2S_SPK_GPIO_BCLK,
        (int)AUDIO_I2S_SPK_GPIO_LRCK,

        // ST7789 display
        (int)DISPLAY_MOSI_PIN,
        (int)DISPLAY_CLK_PIN,
        (int)DISPLAY_DC_PIN,
        (int)DISPLAY_RST_PIN,
        (int)DISPLAY_CS_PIN,
        (int)DISPLAY_BACKLIGHT_PIN,

        // Optional hardware: not connected yet
        -1, // servo_pan
        -1, // servo_tilt
        -1, // pir
        -1, // dht11
        -1, // lcd_sda
        -1  // lcd_scl
    };

    // Lưu bản mặc định để phục hồi nếu NVS không hợp lệ.
    const OttoGpioConfig defaults = c;

    Settings s(NVS_NAMESPACE);

    // Microphone
    c.mic_ws = s.GetInt("mic_ws", c.mic_ws);
    c.mic_sck = s.GetInt("mic_sck", c.mic_sck);
    c.mic_din = s.GetInt("mic_din", c.mic_din);

    // Speaker
    c.spk_dout = s.GetInt("spk_out", c.spk_dout);
    c.spk_bclk = s.GetInt("spk_bclk", c.spk_bclk);
    c.spk_lrck = s.GetInt("spk_lrck", c.spk_lrck);

    // Display
    c.display_mosi = s.GetInt("disp_mosi", c.display_mosi);
    c.display_clk = s.GetInt("disp_clk", c.display_clk);
    c.display_dc = s.GetInt("disp_dc", c.display_dc);
    c.display_rst = s.GetInt("disp_rst", c.display_rst);
    c.display_cs = s.GetInt("disp_cs", c.display_cs);
    c.display_backlight = s.GetInt("disp_bl", c.display_backlight);

    // Optional hardware
    c.servo_pan = s.GetInt("servo_pan", -1);
    c.servo_tilt = s.GetInt("servo_tilt", -1);
    c.pir = s.GetInt("pir", -1);
    c.dht11 = s.GetInt("dht11", -1);
    c.lcd_sda = s.GetInt("lcd_sda", -1);
    c.lcd_scl = s.GetInt("lcd_scl", -1);

    // Kiểm tra cấu hình sau khi đọc NVS.
    if (!Validate(c)) {
        ESP_LOGE(
            TAG,
            "NVS GPIO configuration invalid. Using default configuration."
        );

        return defaults;
    }

    ESP_LOGI(TAG, "GPIO configuration loaded successfully");

    return c;
}

// ============================================================
// VALIDATE GPIO CONFIGURATION
// ============================================================

bool OttoGpioConfigManager::Validate(const OttoGpioConfig& c) {
    const int pins[] = {
        c.mic_ws,
        c.mic_sck,
        c.mic_din,

        c.spk_dout,
        c.spk_bclk,
        c.spk_lrck,

        c.display_mosi,
        c.display_clk,
        c.display_dc,
        c.display_rst,
        c.display_cs,
        c.display_backlight,

        c.servo_pan,
        c.servo_tilt,

        c.pir,
        c.dht11,

        c.lcd_sda,
        c.lcd_scl
    };

    std::set<int> used;

    for (int pin : pins) {
        if (pin == -1) {
            continue;
        }

        if (pin < 0 || pin > 48) {
            ESP_LOGE(TAG, "GPIO outside allowed range: %d", pin);
            return false;
        }

        if (!used.insert(pin).second) {
            ESP_LOGE(TAG, "Duplicate GPIO detected: %d", pin);
            return false;
        }
    }

    // Các chân của thiết bị đang sử dụng không được để trống.
    const int required_pins[] = {
        c.mic_ws,
        c.mic_sck,
        c.mic_din,

        c.spk_dout,
        c.spk_bclk,
        c.spk_lrck,

        c.display_mosi,
        c.display_clk,
        c.display_dc,
        c.display_rst,
        c.display_cs,
        c.display_backlight
    };

    for (int pin : required_pins) {
        if (pin == -1) {
            ESP_LOGE(TAG, "A required GPIO is not configured");
            return false;
        }
    }

    // LCD1602 cần đủ hai chân I2C.
    if ((c.lcd_sda == -1) != (c.lcd_scl == -1)) {
        ESP_LOGE(TAG, "LCD1602 requires both SDA and SCL");
        return false;
    }

    // Hai servo cần được cấu hình cùng lúc.
    if ((c.servo_pan == -1) != (c.servo_tilt == -1)) {
        ESP_LOGE(TAG, "Servo requires both PAN and TILT");
        return false;
    }

    return true;
}

// ============================================================
// SAVE GPIO CONFIGURATION
// ============================================================

bool OttoGpioConfigManager::Save(const OttoGpioConfig& c) {
    if (!Validate(c)) {
        ESP_LOGE(TAG, "Cannot save invalid GPIO configuration");
        return false;
    }

    Settings s(NVS_NAMESPACE, true);

    // Microphone
    s.SetInt("mic_ws", c.mic_ws);
    s.SetInt("mic_sck", c.mic_sck);
    s.SetInt("mic_din", c.mic_din);

    // Speaker
    s.SetInt("spk_out", c.spk_dout);
    s.SetInt("spk_bclk", c.spk_bclk);
    s.SetInt("spk_lrck", c.spk_lrck);

    // Display
    s.SetInt("disp_mosi", c.display_mosi);
    s.SetInt("disp_clk", c.display_clk);
    s.SetInt("disp_dc", c.display_dc);
    s.SetInt("disp_rst", c.display_rst);
    s.SetInt("disp_cs", c.display_cs);
    s.SetInt("disp_bl", c.display_backlight);

    // Optional hardware
    s.SetInt("servo_pan", c.servo_pan);
    s.SetInt("servo_tilt", c.servo_tilt);
    s.SetInt("pir", c.pir);
    s.SetInt("dht11", c.dht11);
    s.SetInt("lcd_sda", c.lcd_sda);
    s.SetInt("lcd_scl", c.lcd_scl);

    ESP_LOGI(TAG, "GPIO configuration saved successfully");

    return true;
}
