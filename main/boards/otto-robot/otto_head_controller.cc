
#include "otto_head_controller.h"

#include "mcp_server.h"

#include <driver/gpio.h>
#include <esp_log.h>
#include <freertos/task.h>

#include <algorithm>
#include <cstdlib>

#define TAG "OttoHead"

OttoHeadController::OttoHeadController(
    const OttoGpioConfig& config
) {
    mutex_ = xSemaphoreCreateMutex();

    const bool servo_pins_valid =
        config.servo_pan >= 0 &&
        config.servo_tilt >= 0 &&
        GPIO_IS_VALID_GPIO(static_cast<gpio_num_t>(config.servo_pan)) &&
        GPIO_IS_VALID_GPIO(static_cast<gpio_num_t>(config.servo_tilt)) &&
        GPIO_IS_VALID_OUTPUT_GPIO(static_cast<gpio_num_t>(config.servo_pan)) &&
        GPIO_IS_VALID_OUTPUT_GPIO(static_cast<gpio_num_t>(config.servo_tilt)) &&
        config.servo_pan != config.servo_tilt;

    if (servo_pins_valid && mutex_ != nullptr) {
        pan_servo_.Attach(config.servo_pan);
        tilt_servo_.Attach(config.servo_tilt);

        pan_servo_.SetPosition(90);
        tilt_servo_.SetPosition(90);

        initialized_ = true;

        ESP_LOGI(
            TAG,
            "Pan/Tilt initialized: PAN=%d, TILT=%d",
            config.servo_pan,
            config.servo_tilt
        );
    } else {
        ESP_LOGI(
            TAG,
            "Pan/Tilt disabled: pins not configured or invalid"
        );
    }

    RegisterMcpTools();
}

OttoHeadController::~OttoHeadController() {
    if (initialized_) {
        pan_servo_.Detach();
        tilt_servo_.Detach();
    }

    if (mutex_ != nullptr) {
        vSemaphoreDelete(mutex_);
        mutex_ = nullptr;
    }
}

bool OttoHeadController::MoveTo(
    int pan,
    int tilt,
    int speed
) {
    if (!initialized_ || mutex_ == nullptr) {
        return false;
    }

    pan = std::clamp(pan, 0, 180);
    tilt = std::clamp(tilt, 0, 180);
    speed = std::clamp(speed, 30, 180);

    if (xSemaphoreTake(
            mutex_,
            pdMS_TO_TICKS(100)
        ) != pdTRUE) {
        return false;
    }

    pan_servo_.SetLimiter(speed);
    tilt_servo_.SetLimiter(speed);

    const int timeout_ms = 6500;
    int elapsed = 0;
    bool reached = false;

    while (elapsed <= timeout_ms) {
        pan_servo_.SetPosition(pan);
        tilt_servo_.SetPosition(tilt);

        const int pan_diff =
            std::abs(pan_servo_.GetPosition() - pan);

        const int tilt_diff =
            std::abs(tilt_servo_.GetPosition() - tilt);

        if (pan_diff <= 1 && tilt_diff <= 1) {
            reached = true;
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(20));
        elapsed += 20;
    }

    pan_servo_.DisableLimiter();
    tilt_servo_.DisableLimiter();

    xSemaphoreGive(mutex_);

    return reached;
}

void OttoHeadController::RegisterMcpTools() {
    auto& mcp = McpServer::GetInstance();

    mcp.AddTool(
        "self.otto.head",
        "Điều khiển đầu robot Otto. "
        "pan: góc quay trái phải 0-180. "
        "tilt: góc cúi ngẩng 0-180. "
        "speed: tốc độ 30-180 độ mỗi giây. "
        "90 là vị trí trung tâm.",

        PropertyList({
            Property("pan", kPropertyTypeInteger, 90, 0, 180),
            Property("tilt", kPropertyTypeInteger, 90, 0, 180),
            Property("speed", kPropertyTypeInteger, 90, 30, 180)
        }),

        [this](const PropertyList& properties) -> ReturnValue {
            if (!initialized_) {
                return "Servo chưa được cấu hình hoặc chưa kích hoạt.";
            }

            const int pan =
                properties["pan"].value<int>();

            const int tilt =
                properties["tilt"].value<int>();

            const int speed =
                properties["speed"].value<int>();

            if (!MoveTo(pan, tilt, speed)) {
                return "Không thể hoàn thành chuyển động đầu robot.";
            }

            return true;
        }
    );

    ESP_LOGI(TAG, "Head MCP tool registered");
}
