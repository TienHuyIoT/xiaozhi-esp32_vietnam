
#include "otto_head_motion.h"

#include <algorithm>

#include "driver/gpio.h"
#include "esp_log.h"

static const char* TAG = "OttoHeadMotion";

OttoHeadMotion::OttoHeadMotion()
    : motion_task_(nullptr),
      pan_gpio_(-1),
      tilt_gpio_(-1),
      target_pan_(0),
      target_tilt_(0),
      initialized_(false) {
}

OttoHeadMotion::~OttoHeadMotion() {
    Deinitialize();
}

bool OttoHeadMotion::Initialize(int pan_gpio, int tilt_gpio) {
    if (initialized_) {
        return true;
    }

    // Không cho phép khởi tạo khi chưa cấu hình GPIO.
    if (pan_gpio < 0 || tilt_gpio < 0 || pan_gpio == tilt_gpio) {
        ESP_LOGE(TAG, "Invalid Pan/Tilt GPIO");
        return false;
    }

    if (!GPIO_IS_VALID_OUTPUT_GPIO((gpio_num_t)pan_gpio) ||
        !GPIO_IS_VALID_OUTPUT_GPIO((gpio_num_t)tilt_gpio)) {
        ESP_LOGE(TAG, "GPIO is not valid for servo output");
        return false;
    }

    pan_gpio_ = pan_gpio;
    tilt_gpio_ = tilt_gpio;

    // Khởi tạo hai servo.
    pan_servo_.Attach(pan_gpio_);
    tilt_servo_.Attach(tilt_gpio_);

    pan_servo_.SetLimiter(DEFAULT_SPEED);
    tilt_servo_.SetLimiter(DEFAULT_SPEED);

    target_pan_ = 0;
    target_tilt_ = 0;

    pan_servo_.SetPosition(CENTER);
    tilt_servo_.SetPosition(CENTER);

    initialized_ = true;

    BaseType_t result = xTaskCreate(
        MotionTask,
        "otto_head_motion",
        3072,
        this,
        5,
        &motion_task_
    );

    if (result != pdPASS) {
        ESP_LOGE(TAG, "Cannot create motion task");

        pan_servo_.Detach();
        tilt_servo_.Detach();

        initialized_ = false;
        motion_task_ = nullptr;
        pan_gpio_ = -1;
        tilt_gpio_ = -1;

        return false;
    }

    ESP_LOGI(TAG, "Pan/Tilt initialized: GPIO %d / %d",
             pan_gpio_, tilt_gpio_);

    return true;
}

void OttoHeadMotion::Deinitialize() {
    if (!initialized_) {
        return;
    }

    if (motion_task_ != nullptr) {
        vTaskDelete(motion_task_);
        motion_task_ = nullptr;
    }

    pan_servo_.Detach();
    tilt_servo_.Detach();

    initialized_ = false;
    pan_gpio_ = -1;
    tilt_gpio_ = -1;

    ESP_LOGI(TAG, "Pan/Tilt deinitialized");
}

void OttoHeadMotion::MoveTo(int pan_offset, int tilt_offset) {
    if (!initialized_) {
        return;
    }

    target_pan_ = std::clamp(pan_offset, -PAN_LIMIT, PAN_LIMIT);
    target_tilt_ = std::clamp(tilt_offset, -TILT_LIMIT, TILT_LIMIT);
}

void OttoHeadMotion::Center() {
    MoveTo(0, 0);
}

void OttoHeadMotion::LookLeft() {
    MoveTo(-PAN_LIMIT, 0);
}

void OttoHeadMotion::LookRight() {
    MoveTo(PAN_LIMIT, 0);
}

void OttoHeadMotion::LookUp() {
    MoveTo(0, -TILT_LIMIT);
}

void OttoHeadMotion::LookDown() {
    MoveTo(0, TILT_LIMIT);
}

void OttoHeadMotion::SetSpeed(int degrees_per_second) {
    if (!initialized_) {
        return;
    }

    int speed = std::clamp(degrees_per_second, 30, 240);

    pan_servo_.SetLimiter(speed);
    tilt_servo_.SetLimiter(speed);
}

bool OttoHeadMotion::IsInitialized() const {
    return initialized_;
}

void OttoHeadMotion::MotionTask(void* arg) {
    OttoHeadMotion* self = static_cast<OttoHeadMotion*>(arg);

    while (true) {
        self->UpdateMotion();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void OttoHeadMotion::UpdateMotion() {
    if (!initialized_) {
        return;
    }

    int pan_position = CENTER + target_pan_;
    int tilt_position = CENTER + target_tilt_;

    pan_servo_.SetPosition(pan_position);
    tilt_servo_.SetPosition(tilt_position);
}
