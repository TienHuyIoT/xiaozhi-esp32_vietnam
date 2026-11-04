
#include "otto_gpio_config.h"
#include "config.h"
#include "settings.h"

#include <esp_log.h>
#include <driver/gpio.h>
#include <set>

static const char* TAG = "OttoGPIO";
static const char* NVS_NAMESPACE = "otto_gpio";

OttoGpioConfig OttoGpioConfigManager::Load() {
    OttoGpioConfig c = {
        (int)AUDIO_I2S_MIC_GPIO_WS,
        (int)AUDIO_I2S_MIC_GPIO_SCK,
        (int)AUDIO_I2S_MIC_GPIO_DIN,

        (int)AUDIO_I2S_SPK_GPIO_DOUT,
        (int)AUDIO_I2S_SPK_GPIO_BCLK,
        (int)AUDIO_I2S_SPK_GPIO_LRCK,

        (int)DISPLAY_MOSI_PIN,
        (int)DISPLAY_CLK_PIN,
        (int)DISPLAY_DC_PIN,
        (int)DISPLAY_RST_PIN,
        (int)DISPLAY_CS_PIN,
        (int)DISPLAY_BACKLIGHT_PIN,

        -1, -1, -1, -1, -1, -1
    };

    Settings s(NVS_NAMESPACE);

    c.mic_ws = s.GetInt("mic_ws", c.mic_ws);
    c.mic_sck = s.GetInt("mic_sck", c.mic_sck);
    c.mic_din = s.GetInt("mic_din", c.mic_din);

    c.spk_dout = s.GetInt("spk_out", c.spk_dout);
    c.spk_bclk = s.GetInt("spk_bclk", c.spk_bclk);
    c.spk_lrck = s.GetInt("spk_lrck", c.spk_lrck);

    c.display_mosi = s.GetInt("disp_mosi", c.display_mosi);
    c.display_clk = s.GetInt("disp_clk", c.display_clk);
    c.display_dc = s.GetInt("disp_dc", c.display_dc);
    c.display_rst = s.GetInt("disp_rst", c.display_rst);
    c.display_cs = s.GetInt("disp_cs", c.display_cs);
    c.display_backlight = s.GetInt("disp_bl", c.display_backlight);

    c.servo_pan = s.GetInt("servo_pan", -1);
    c.servo_tilt = s.GetInt("servo_tilt", -1);
    c.pir = s.GetInt("pir", -1);
    c.dht11 = s.GetInt("dht11", -1);
    c.lcd_sda = s.GetInt("lcd_sda", -1);
    c.lcd_scl = s.GetInt("lcd_scl", -1);

    return c;
}

bool OttoGpioConfigManager::Validate(const OttoGpioConfig& c) {
    const int pins[] = {
        c.mic_ws, c.mic_sck, c.mic_din,
        c.spk_dout, c.spk_bclk, c.spk_lrck,
        c.display_mosi, c.display_clk, c.display_dc,
        c.display_rst, c.display_cs, c.display_backlight,
        c.servo_pan, c.servo_tilt,
        c.pir, c.dht11, c.lcd_sda, c.lcd_scl
    };

    std::set<int> used;

    for (int pin : pins) {
        if (pin == -1) {
            continue;
        }

        // ESP32-S3: GPIO từ 0 đến 48
        if (pin < 0 || pin > 48) {
            ESP_LOGE(TAG, "GPIO ngoài phạm vi: %d", pin);
            return false;
        }

        if (!used.insert(pin).second) {
            ESP_LOGE(TAG, "GPIO bị trùng: %d", pin);
            return false;
        }
    }

    // Hai chân LCD1602 phải được cấu hình cùng nhau
    if ((c.lcd_sda == -1) != (c.lcd_scl == -1)) {
        ESP_LOGE(TAG, "LCD1602 cần cấu hình cả SDA và SCL");
        return false;
    }

    // Hai chân servo phải được cấu hình cùng nhau
    if ((c.servo_pan == -1) != (c.servo_tilt == -1)) {
        ESP_LOGE(TAG, "Servo cần cấu hình cả PAN và TILT");
        return false;
    }

    return true;
}

bool OttoGpioConfigManager::Save(const OttoGpioConfig& c) {
    if (!Validate(c)) {
        return false;
    }

    Settings s(NVS_NAMESPACE, true);

    s.SetInt("mic_ws", c.mic_ws);
    s.SetInt("mic_sck", c.mic_sck);
    s.SetInt("mic_din", c.mic_din);

    s.SetInt("spk_out", c.spk_dout);
    s.SetInt("spk_bclk", c.spk_bclk);
    s.SetInt("spk_lrck", c.spk_lrck);

    s.SetInt("disp_mosi", c.display_mosi);
    s.SetInt("disp_clk", c.display_clk);
    s.SetInt("disp_dc", c.display_dc);
    s.SetInt("disp_rst", c.display_rst);
    s.SetInt("disp_cs", c.display_cs);
    s.SetInt("disp_bl", c.display_backlight);

    s.SetInt("servo_pan", c.servo_pan);
    s.SetInt("servo_tilt", c.servo_tilt);
    s.SetInt("pir", c.pir);
    s.SetInt("dht11", c.dht11);
    s.SetInt("lcd_sda", c.lcd_sda);
    s.SetInt("lcd_scl", c.lcd_scl);

    ESP_LOGI(TAG, "Đã lưu cấu hình GPIO");
    return true;
}
