
#include "otto_gpio_config.h"
#include "config.h"
#include "settings.h"

#include <esp_log.h>
#include <driver/gpio.h>
#include <nvs.h>
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

    if (!Validate(c)) {
        ESP_LOGE(TAG, "NVS GPIO configuration invalid. Using defaults.");
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

    // Kiểm tra GPIO hợp lệ và không trùng.
    for (int pin : pins) {
        if (pin == -1) {
            continue;
        }

        if (!GPIO_IS_VALID_GPIO(static_cast<gpio_num_t>(pin))) {
            ESP_LOGE(TAG, "Invalid GPIO: %d", pin);
            return false;
        }

        if (!used.insert(pin).second) {
            ESP_LOGE(TAG, "Duplicate GPIO detected: %d", pin);
            return false;
        }
    }

    // Các chân bắt buộc.
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

    // Những chân cần khả năng OUTPUT.
    const int output_pins[] = {
        c.mic_ws,
        c.mic_sck,
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
        c.lcd_sda,
        c.lcd_scl
    };

    for (int pin : output_pins) {
        if (pin == -1) {
            continue;
        }

        if (!GPIO_IS_VALID_OUTPUT_GPIO(static_cast<gpio_num_t>(pin))) {
            ESP_LOGE(TAG, "GPIO cannot be used as output: %d", pin);
            return false;
        }
    }

    // LCD1602 cần đủ SDA và SCL.
    if ((c.lcd_sda == -1) != (c.lcd_scl == -1)) {
        ESP_LOGE(TAG, "LCD1602 requires both SDA and SCL");
        return false;
    }

    // Hai servo cần cấu hình cùng lúc.
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

    nvs_handle_t handle = 0;

    esp_err_t err = nvs_open(
        NVS_NAMESPACE,
        NVS_READWRITE,
        &handle
    );

    if (err != ESP_OK) {
        ESP_LOGE(
            TAG,
            "Cannot open NVS namespace: %s",
            esp_err_to_name(err)
        );
        return false;
    }

    struct GpioEntry {
        const char* key;
        int value;
    };

    const GpioEntry entries[] = {
        {"mic_ws", c.mic_ws},
        {"mic_sck", c.mic_sck},
        {"mic_din", c.mic_din},

        {"spk_out", c.spk_dout},
        {"spk_bclk", c.spk_bclk},
        {"spk_lrck", c.spk_lrck},

        {"disp_mosi", c.display_mosi},
        {"disp_clk", c.display_clk},
        {"disp_dc", c.display_dc},
        {"disp_rst", c.display_rst},
        {"disp_cs", c.display_cs},
        {"disp_bl", c.display_backlight},

        {"servo_pan", c.servo_pan},
        {"servo_tilt", c.servo_tilt},
        {"pir", c.pir},
        {"dht11", c.dht11},
        {"lcd_sda", c.lcd_sda},
        {"lcd_scl", c.lcd_scl}
    };

    // Ghi toàn bộ GPIO vào NVS.
    for (const auto& entry : entries) {
        err = nvs_set_i32(
            handle,
            entry.key,
            static_cast<int32_t>(entry.value)
        );

        if (err != ESP_OK) {
            ESP_LOGE(
                TAG,
                "Failed to write %s: %s",
                entry.key,
                esp_err_to_name(err)
            );

            nvs_close(handle);
            return false;
        }
    }

    // Commit dữ liệu.
    err = nvs_commit(handle);
    nvs_close(handle);

    if (err != ESP_OK) {
        ESP_LOGE(
            TAG,
            "NVS commit failed: %s",
            esp_err_to_name(err)
        );
        return false;
    }

    ESP_LOGI(TAG, "GPIO configuration saved successfully");
    return true;
}
