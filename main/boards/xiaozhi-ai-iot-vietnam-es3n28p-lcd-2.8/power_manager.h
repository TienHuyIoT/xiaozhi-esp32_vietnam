#pragma once
#include <vector>
#include <functional>

#include <esp_timer.h>
#include <driver/gpio.h>
#include <esp_adc/adc_oneshot.h>
#include <driver/temperature_sensor.h> 

// GPIO09 (ADC1_CH8) is used for battery voltage measurement
#define ADC_CHANNEL_BATTERY ADC_CHANNEL_8
#define ADC_CHANNEL_UNIT    ADC_UNIT_1

class PowerManager {
private:
    esp_timer_handle_t timer_handle_;
    std::function<void(bool)> on_charging_status_changed_;
    std::function<void(bool)> on_low_battery_status_changed_;
    std::function<void(float)> on_temperature_changed_; 

    gpio_num_t charging_pin_ = GPIO_NUM_NC;
    std::vector<uint16_t> adc_values_;
    uint32_t battery_level_ = 0;
    bool is_charging_ = false;
    bool is_low_battery_ = false;
    float current_temperature_ = 0.0f;
    int ticks_ = 0;
    const int kBatteryAdcInterval = 60;
    const int kBatteryAdcDataCount = 3;
    const int kLowBatteryLevel = 20;
    const int kTemperatureReadInterval = 60; // Temperature is read every 10 seconds.

    adc_oneshot_unit_handle_t adc_handle_;
    temperature_sensor_handle_t temp_sensor_ = NULL;

    void CheckBatteryStatus() {
        if (charging_pin_ != GPIO_NUM_NC) {
            // Get charging status
            bool new_charging_status = gpio_get_level(charging_pin_) == 1;
            if (new_charging_status != is_charging_) {
                is_charging_ = new_charging_status;
                if (on_charging_status_changed_) {
                    on_charging_status_changed_(is_charging_);
                }
                ReadBatteryAdcData();
                return;
            }
        }

        // If battery ADC data is insufficient, read battery ADC data
        if (adc_values_.size() < kBatteryAdcDataCount) {
            ReadBatteryAdcData();
            return;
        }

        // If battery ADC data is sufficient, read battery ADC data every kBatteryAdcInterval ticks
        ticks_++;
        if (ticks_ % kBatteryAdcInterval == 0) {
            ReadBatteryAdcData();
        }

        if (ticks_ % kTemperatureReadInterval == 0) {
            ReadTemperature();
        }
    }

    void ReadBatteryAdcData() {
        int adc_value;
        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle_, ADC_CHANNEL_BATTERY, &adc_value));
        
        // Add ADC value to the queue
        adc_values_.push_back(adc_value);
        if (adc_values_.size() > kBatteryAdcDataCount) {
            adc_values_.erase(adc_values_.begin());
        }
        uint32_t average_adc = 0;
        for (auto value : adc_values_) {
            average_adc += value;
        }
        average_adc /= adc_values_.size();

        // Define battery level intervals
        const struct {
            uint16_t adc;
            uint8_t level;
        } levels[] = {
            {1970, 0},
            {2062, 20},
            {2154, 40},
            {2246, 60},
            {2338, 80},
            {2430, 100}
        };

        // Below the lowest value
        if (average_adc < levels[0].adc) {
            battery_level_ = 0;
        }
        // Above the highest value
        else if (average_adc >= levels[5].adc) {
            battery_level_ = 100;
        } else {
            // Linear interpolation to calculate intermediate values
            for (int i = 0; i < 5; i++) {
                if (average_adc >= levels[i].adc && average_adc < levels[i+1].adc) {
                    float ratio = static_cast<float>(average_adc - levels[i].adc) / (levels[i+1].adc - levels[i].adc);
                    battery_level_ = levels[i].level + ratio * (levels[i+1].level - levels[i].level);
                    break;
                }
            }
        }

        // Check low battery status
        if (adc_values_.size() >= kBatteryAdcDataCount) {
            bool new_low_battery_status = battery_level_ <= kLowBatteryLevel;
            if (new_low_battery_status != is_low_battery_) {
                is_low_battery_ = new_low_battery_status;
                if (on_low_battery_status_changed_) {
                    on_low_battery_status_changed_(is_low_battery_);
                }
            }
        }

        ESP_LOGI("PowerManager", "ADC value: %d average: %ld level: %ld", adc_value, average_adc, battery_level_);
    }

    void ReadTemperature() {
        float temperature = 0.0f;
        ESP_ERROR_CHECK(temperature_sensor_get_celsius(temp_sensor_, &temperature));
        
        if (abs(temperature - current_temperature_) >= 3.5f) {  // The callback is only triggered when the temperature change exceeds 3.5°C.
            current_temperature_ = temperature;
            if (on_temperature_changed_) {
                on_temperature_changed_(current_temperature_);
            }
        }      
        ESP_LOGI("PowerManager", "Temperature updated: %.1f°C", current_temperature_);
    }

public:
    PowerManager(gpio_num_t pin) : charging_pin_(pin) {
        // Initialize charging pin
        if (charging_pin_ != GPIO_NUM_NC) {
            gpio_config_t io_conf = {};
            io_conf.intr_type = GPIO_INTR_DISABLE;
            io_conf.mode = GPIO_MODE_INPUT;
            io_conf.pin_bit_mask = (1ULL << charging_pin_);
            io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE; 
            io_conf.pull_up_en = GPIO_PULLUP_DISABLE;     
            gpio_config(&io_conf);
        }

        // Create battery check timer
        esp_timer_create_args_t timer_args = {
            .callback = [](void* arg) {
                PowerManager* self = static_cast<PowerManager*>(arg);
                self->CheckBatteryStatus();
            },
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "battery_check_timer",
            .skip_unhandled_events = true,
        };
        ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer_handle_));
        ESP_ERROR_CHECK(esp_timer_start_periodic(timer_handle_, 1000000));

        // Initialize ADC
        adc_oneshot_unit_init_cfg_t init_config = {
            .unit_id = ADC_CHANNEL_UNIT,
            .ulp_mode = ADC_ULP_MODE_DISABLE,
        };
        ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle_));
        
        adc_oneshot_chan_cfg_t chan_config = {
            .atten = ADC_ATTEN_DB_12,
            .bitwidth = ADC_BITWIDTH_12,
        };
        ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle_, ADC_CHANNEL_BATTERY, &chan_config));

        // Initialize the temperature sensor
        temperature_sensor_config_t temp_config = {
            .range_min = 10,
            .range_max = 80,
            .clk_src = TEMPERATURE_SENSOR_CLK_SRC_DEFAULT
        };
        ESP_ERROR_CHECK(temperature_sensor_install(&temp_config, &temp_sensor_));
        ESP_ERROR_CHECK(temperature_sensor_enable(temp_sensor_));
        ESP_LOGI("PowerManager", "Temperature sensor initialized (new driver)");
    }

    ~PowerManager() {
        if (timer_handle_) {
            esp_timer_stop(timer_handle_);
            esp_timer_delete(timer_handle_);
        }
        if (adc_handle_) {
            adc_oneshot_del_unit(adc_handle_);
        }
    }

    bool IsCharging() {
        // If the battery is already full, do not show charging
        if (battery_level_ == 100) {
            return false;
        }
        return is_charging_;
    }

    bool IsDischarging() {
        // No distinction between charging and discharging, so return the opposite state
        return !is_charging_;
    }

    uint8_t GetBatteryLevel() {
        return battery_level_;
    }

    float GetTemperature() const { return current_temperature_; }  // Get the current temperature

    void OnTemperatureChanged(std::function<void(float)> callback) { 
        on_temperature_changed_ = callback; 
    }

    void OnLowBatteryStatusChanged(std::function<void(bool)> callback) {
        on_low_battery_status_changed_ = callback;
    }

    void OnChargingStatusChanged(std::function<void(bool)> callback) {
        on_charging_status_changed_ = callback;
    }
};
