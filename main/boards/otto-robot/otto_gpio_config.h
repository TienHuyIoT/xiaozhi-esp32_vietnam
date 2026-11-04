
#pragma once

#include <stdint.h>

struct OttoGpioConfig {
    // Micro INMP441
    int mic_ws;
    int mic_sck;
    int mic_din;

    // Loa MAX98357A
    int spk_dout;
    int spk_bclk;
    int spk_lrck;

    // Màn hình ST7789
    int display_mosi;
    int display_clk;
    int display_dc;
    int display_rst;
    int display_cs;
    int display_backlight;

    // Thiết bị chưa kết nối: -1 nghĩa là chưa cấu hình
    int servo_pan;
    int servo_tilt;
    int pir;
    int dht11;
    int lcd_sda;
    int lcd_scl;
};

class OttoGpioConfigManager {
public:
    static OttoGpioConfig Load();
    static bool Save(const OttoGpioConfig& config);
    static bool Validate(const OttoGpioConfig& config);
};
