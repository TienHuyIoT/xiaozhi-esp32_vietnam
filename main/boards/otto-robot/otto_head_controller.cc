#include "otto_head_controller.h"

#include "mcp_server.h"

#include <esp_log.h>
#include <freertos/task.h>
#include <cstdlib>

#define TAG "OttoHead"

OttoHeadController::OttoHeadController(const OttoGpioConfig& config) {
    mutex_ = xSemaphoreCreateMutex();

    if (config.servo_pan >= 0 && config.servo_tilt >= 0 && mutex_ != nullptr) {
        pan_servo_.Attach(config.servo_pan);
        tilt_servo_.Attach(config.servo_tilt);

        pan_servo_.SetPosition(90);
        tilt_servo_.SetPosition(90);

        initialized_ = true;

        ESP_LOGI(TAG, "Pan/Tilt initialized");
    } else {
        ESP_LOGI(TAG, "Pan/Tilt disabled: GPIO not configured");
    }

    RegisterMcpTools();
}

OttoHeadController::~OttoHeadController() {
    if (mutex_ != nullptr) {
        vSemaphoreDelete(mutex_);
        mutex_ = nullptr;
    }
}

bool OttoHeadController::MoveTo(int pan, int tilt, int speed) {
    if (!initialized_) {
        return false;
    }

    pan = std::max(0, std::min(180, pan));
    tilt = std::max(0, std::min(180, tilt));
    speed = std::max(30, std::min(180, speed));

    if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(100)) != pdTRUE) {
        return false;
    }

    pan_servo_.SetLimiter(speed);
    tilt_servo_.SetLimiter(speed);

    const int timeout_ms = 3000;
    int elapsed = 0;

    while (elapsed < timeout_ms) {
        pan_servo_.SetPosition(pan);
        tilt_servo_.SetPosition(tilt);

        int pan_diff = abs(pan_servo_.GetPosition() - pan);
        int tilt_diff = abs(tilt_servo_.GetPosition() - tilt);

        if (pan_diff <= 1 && tilt_diff <= 1) {
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(20));
        elapsed += 20;
    }

    pan_servo_.DisableLimiter();
    tilt_servo_.DisableLimiter();

    xSemaphoreGive(mutex_);

    return true;
}

void OttoHeadController::RegisterMcpTools() {
    auto& mcp = McpServer::GetInstance();

    mcp.AddTool(
        "self.otto.head",
        "Điều khiển đầu robot Otto. pan: góc quay trái phải 0-180, "
        "tilt: góc cúi ngẩng 0-180, speed: tốc độ 30-180 độ mỗi giây. "
        "90 là vị trí trung tâm.",
        PropertyList({
            Property("pan", kPropertyTypeInteger, 90, 0, 180),
            Property("tilt", kPropertyTypeInteger, 90, 0, 180),
            Property("speed", kPropertyTypeInteger, 90, 30, 180)
        }),
        [this](const PropertyList& properties) -> ReturnValue {
            if (!initialized_) {
                return "Servo chưa được cấu hình GPIO hoặc chưa kích hoạt.";
            }

            int pan = properties["pan"].value<int>();
            int tilt = properties["tilt"].value<int>();
            int speed = properties["speed"].value<int>();

            if (!MoveTo(pan, tilt, speed)) {
                return "Không thể điều khiển đầu robot.";
            }

            return true;
        }
    );

    ESP_LOGI(TAG, "Head MCP tool registered");
}
