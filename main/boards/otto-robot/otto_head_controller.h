#pragma once

#include "otto_gpio_config.h"
#include "oscillator.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

class OttoHeadController {
public:
    explicit OttoHeadController(const OttoGpioConfig& config);
    ~OttoHeadController();

private:
    Oscillator pan_servo_;
    Oscillator tilt_servo_;

    SemaphoreHandle_t mutex_ = nullptr;
    bool initialized_ = false;

    void RegisterMcpTools();
    bool MoveTo(int pan, int tilt, int speed);
};
