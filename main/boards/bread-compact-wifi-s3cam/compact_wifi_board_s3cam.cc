#include "wifi_board.h"
#include "codecs/no_audio_codec.h"
#include "display/lcd_display.h"
#include "system_reset.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "device_state_event.h"
#include "mcp_server.h"
#include "lamp_controller.h"
#include "led/single_led.h"
#include "esp32_camera.h"
#include "esp_camera.h"

#include <wifi_station.h>
#include <esp_log.h>
#include <driver/i2c_master.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_timer.h>
#include <driver/ledc.h>
#include <driver/spi_common.h>
#include <cJSON.h>

#include <cstddef>
#include <mutex>
#include <string>

#include "flash_fatfs.h"

#if defined(LCD_TYPE_ILI9341_SERIAL)
#include "esp_lcd_ili9341.h"
#endif

#if defined(LCD_TYPE_GC9A01_SERIAL)
#include "esp_lcd_gc9a01.h"
static const gc9a01_lcd_init_cmd_t gc9107_lcd_init_cmds[] = {
    //  {cmd, { data }, data_size, delay_ms}
    {0xfe, (uint8_t[]){0x00}, 0, 0},
    {0xef, (uint8_t[]){0x00}, 0, 0},
    {0xb0, (uint8_t[]){0xc0}, 1, 0},
    {0xb1, (uint8_t[]){0x80}, 1, 0},
    {0xb2, (uint8_t[]){0x27}, 1, 0},
    {0xb3, (uint8_t[]){0x13}, 1, 0},
    {0xb6, (uint8_t[]){0x19}, 1, 0},
    {0xb7, (uint8_t[]){0x05}, 1, 0},
    {0xac, (uint8_t[]){0xc8}, 1, 0},
    {0xab, (uint8_t[]){0x0f}, 1, 0},
    {0x3a, (uint8_t[]){0x05}, 1, 0},
    {0xb4, (uint8_t[]){0x04}, 1, 0},
    {0xa8, (uint8_t[]){0x08}, 1, 0},
    {0xb8, (uint8_t[]){0x08}, 1, 0},
    {0xea, (uint8_t[]){0x02}, 1, 0},
    {0xe8, (uint8_t[]){0x2A}, 1, 0},
    {0xe9, (uint8_t[]){0x47}, 1, 0},
    {0xe7, (uint8_t[]){0x5f}, 1, 0},
    {0xc6, (uint8_t[]){0x21}, 1, 0},
    {0xc7, (uint8_t[]){0x15}, 1, 0},
    {0xf0,
    (uint8_t[]){0x1D, 0x38, 0x09, 0x4D, 0x92, 0x2F, 0x35, 0x52, 0x1E, 0x0C,
                0x04, 0x12, 0x14, 0x1f},
    14, 0},
    {0xf1,
    (uint8_t[]){0x16, 0x40, 0x1C, 0x54, 0xA9, 0x2D, 0x2E, 0x56, 0x10, 0x0D,
                0x0C, 0x1A, 0x14, 0x1E},
    14, 0},
    {0xf4, (uint8_t[]){0x00, 0x00, 0xFF}, 3, 0},
    {0xba, (uint8_t[]){0xFF, 0xFF}, 2, 0},
};
#endif
 
#define TAG "CompactWifiBoardS3Cam"

namespace {
constexpr int kLcdDrawBufferLines = 20;
constexpr ledc_mode_t kArmServoSpeedMode = LEDC_LOW_SPEED_MODE;

struct ArmServoFrame {
    int left_degree;
    int right_degree;
    int hold_ticks;
};

constexpr ArmServoFrame kParkFrame = {ARM_SERVO_PARK_DEGREE, ARM_SERVO_PARK_DEGREE, 0};
constexpr ArmServoFrame kNeutralMotion[] = {kParkFrame};
constexpr ArmServoFrame kHappyMotion[] = {{160, 20, 3}, {40, 140, 3}, {160, 20, 3}, {90, 90, 0}};
constexpr ArmServoFrame kLaughingMotion[] = {{170, 10, 2}, {120, 60, 2}, {170, 10, 2}, {120, 60, 2}, {170, 10, 0}};
constexpr ArmServoFrame kWiggleMotion[] = {{160, 20, 2}, {30, 150, 2}, {160, 20, 2}, {30, 150, 2}, {90, 90, 0}};
constexpr ArmServoFrame kSadMotion[] = {{120, 60, 5}, {10, 170, 5}, {30, 150, 4}, {0, 180, 0}};
constexpr ArmServoFrame kCryingMotion[] = {{180, 0, 2}, {140, 40, 2}, {180, 0, 2}, {140, 40, 2}, {180, 0, 4}};
constexpr ArmServoFrame kAngryMotion[] = {{180, 0, 2}, {10, 170, 2}, {180, 0, 2}, {10, 170, 2}, {90, 90, 0}};
constexpr ArmServoFrame kEmbarrassedMotion[] = {{140, 40, 10}, {90, 90, 5}};
constexpr ArmServoFrame kSurprisedMotion[] = {{180, 0, 6}};
constexpr ArmServoFrame kShockedMotion[] = {{180, 0, 10}};
constexpr ArmServoFrame kThinkingMotion[] = {{130, 50, 5}, {110, 70, 3}, {130, 50, 3}, {110, 70, 0}};
constexpr ArmServoFrame kWaveMotion[] = {{180, 0, 3}, {60, 120, 3}, {180, 0, 3}, {60, 120, 3}, {90, 90, 0}};
constexpr ArmServoFrame kRelaxedMotion[] = {{90, 90, 6}, {30, 150, 6}};
constexpr ArmServoFrame kSleepyMotion[] = {{60, 120, 8}, {10, 170, 8}};
constexpr ArmServoFrame kSpeakingMotion[] = {
    {20, 160, 4},
    {165, 15, 4},
    {70, 110, 3},
    {180, 0, 4},
    {45, 135, 3},
};
constexpr ArmServoFrame kManualParkPose[] = {kParkFrame};
constexpr ArmServoFrame kManualLeftUpPose[] = {{180, 0, 0}};
constexpr ArmServoFrame kManualLeftDownPose[] = {{0, 0, 0}};
constexpr ArmServoFrame kManualBothUpPose[] = {{180, 180, 0}};
constexpr ArmServoFrame kManualBothDownPose[] = {{0, 0, 0}};
constexpr ArmServoFrame kManualWavePose[] = {{ARM_SERVO_PARK_DEGREE, ARM_SERVO_PARK_DEGREE, 2}, {180, 180, 4}, kParkFrame};

template <size_t N>
constexpr size_t FrameCount(const ArmServoFrame (&)[N]) {
    return N;
}

struct EmotionMotion {
    const char* emotion;
    const ArmServoFrame* frames;
    size_t frame_count;
};

constexpr EmotionMotion kEmotionMotions[] = {
    {"neutral", kNeutralMotion, FrameCount(kNeutralMotion)},
    {"happy", kHappyMotion, FrameCount(kHappyMotion)},
    {"loving", kHappyMotion, FrameCount(kHappyMotion)},
    {"laughing", kLaughingMotion, FrameCount(kLaughingMotion)},
    {"funny", kWiggleMotion, FrameCount(kWiggleMotion)},
    {"silly", kWiggleMotion, FrameCount(kWiggleMotion)},
    {"sad", kSadMotion, FrameCount(kSadMotion)},
    {"crying", kCryingMotion, FrameCount(kCryingMotion)},
    {"angry", kAngryMotion, FrameCount(kAngryMotion)},
    {"embarrassed", kEmbarrassedMotion, FrameCount(kEmbarrassedMotion)},
    {"surprised", kSurprisedMotion, FrameCount(kSurprisedMotion)},
    {"shocked", kShockedMotion, FrameCount(kShockedMotion)},
    {"thinking", kThinkingMotion, FrameCount(kThinkingMotion)},
    {"confused", kThinkingMotion, FrameCount(kThinkingMotion)},
    {"winking", kWaveMotion, FrameCount(kWaveMotion)},
    {"kissy", kWaveMotion, FrameCount(kWaveMotion)},
    {"delicious", kWaveMotion, FrameCount(kWaveMotion)},
    {"confident", kWaveMotion, FrameCount(kWaveMotion)},
    {"cool", kWaveMotion, FrameCount(kWaveMotion)},
    {"relaxed", kRelaxedMotion, FrameCount(kRelaxedMotion)},
    {"sleepy", kSleepyMotion, FrameCount(kSleepyMotion)},
};

constexpr EmotionMotion kManualArmPoses[] = {
    {"neutral", kNeutralMotion, FrameCount(kNeutralMotion)},
    {"park", kManualParkPose, FrameCount(kManualParkPose)},
    {"left_up", kManualLeftUpPose, FrameCount(kManualLeftUpPose)},
    {"left_down", kManualLeftDownPose, FrameCount(kManualLeftDownPose)},
    {"both_up", kManualBothUpPose, FrameCount(kManualBothUpPose)},
    {"both_down", kManualBothDownPose, FrameCount(kManualBothDownPose)},
    {"wave", kManualWavePose, FrameCount(kManualWavePose)},
};

enum class TimerState {
    kIdle,
    kRunning,
    kIdleRelease,
};

enum class ArmTarget {
    kLeft,
    kRight,
    kBoth,
};

enum class MotionMode {
    kNone,
    kEmotion,
    kManual,
    kSpeakingLoop,
};

class ArmServoController {
public:
    ArmServoController() {
        InitializeLedc();
        CreateTimer();
    }

    ~ArmServoController() {
        if (timer_ != nullptr) {
            esp_timer_stop(timer_);
            esp_timer_delete(timer_);
        }
        StopServo(ARM_SERVO_LEFT_GPIO, ARM_SERVO_LEFT_LEDC_CHANNEL);
        StopServo(ARM_SERVO_RIGHT_GPIO, ARM_SERVO_RIGHT_LEDC_CHANNEL);
    }

    void SetEmotion(const char* emotion) {
        const char* requested_emotion = (emotion != nullptr && emotion[0] != '\0') ? emotion : "neutral";

        std::lock_guard<std::mutex> lock(mutex_);
        if (last_emotion_ == requested_emotion && PositionKnown() &&
                timer_state_ == TimerState::kRunning && motion_mode_ == MotionMode::kEmotion) {
            return;
        }
        last_emotion_ = requested_emotion;

        const EmotionMotion& motion = FindMotion(requested_emotion);
        StartMotion(motion.frames, motion.frame_count, MotionMode::kEmotion);

        const ArmServoFrame& final_frame = motion.frames[motion.frame_count - 1];
        ESP_LOGI(TAG, "Arm servo emotion '%s' -> left=%d right=%d",
                 requested_emotion, final_frame.left_degree, final_frame.right_degree);
    }

    void SetSpeakingActive(bool active) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (speaking_active_ == active) {
            return;
        }

        speaking_active_ = active;
        last_emotion_.clear();
        if (speaking_active_) {
            StartSpeakingLoop();
            ESP_LOGI(TAG, "Arm servo speaking loop started");
            return;
        }

        if (timer_state_ == TimerState::kRunning && motion_mode_ == MotionMode::kSpeakingLoop) {
            StartMotion(kNeutralMotion, FrameCount(kNeutralMotion), MotionMode::kEmotion);
        }
        ESP_LOGI(TAG, "Arm servo speaking loop stopped");
    }

    void SetManualAngle(const std::string& arm, int degree) {
        const ArmTarget target = ParseArmTarget(arm);
        const int clamped_degree = ClampDegree(degree);

        std::lock_guard<std::mutex> lock(mutex_);
        StopTimer();
        ClearMotionState();
        last_emotion_.clear();

        bool wrote_servo = false;
        if (target == ArmTarget::kLeft || target == ArmTarget::kBoth) {
            EnsureArmConfigured("left", ARM_SERVO_LEFT_GPIO);
            current_left_degree_ = clamped_degree;
            target_left_degree_ = clamped_degree;
            left_position_known_ = true;
            WriteServo(ARM_SERVO_LEFT_LEDC_CHANNEL, current_left_degree_);
            wrote_servo = true;
        }
        if (target == ArmTarget::kRight || target == ArmTarget::kBoth) {
            if (ARM_SERVO_RIGHT_GPIO == GPIO_NUM_NC) {
                if (target == ArmTarget::kRight) {
                    throw std::runtime_error("Right arm servo is not configured");
                }
            } else {
                current_right_degree_ = clamped_degree;
                target_right_degree_ = clamped_degree;
                right_position_known_ = true;
                WriteServo(ARM_SERVO_RIGHT_LEDC_CHANNEL, current_right_degree_);
                wrote_servo = true;
            }
        }
        if (!wrote_servo) {
            throw std::runtime_error("No arm servo is configured");
        }

        if (speaking_active_) {
            StartSpeakingLoop();
        } else {
            StartIdleRelease(false);
        }
        ESP_LOGI(TAG, "Manual arm angle: arm=%s degree=%d", arm.c_str(), clamped_degree);
    }

    void SetManualPose(const std::string& pose) {
        std::lock_guard<std::mutex> lock(mutex_);
        last_emotion_.clear();

        const EmotionMotion& motion = FindManualPose(pose);
        StartMotion(motion.frames, motion.frame_count, MotionMode::kManual);
        ESP_LOGI(TAG, "Manual arm pose: %s", pose.c_str());
    }

    void Release() {
        std::lock_guard<std::mutex> lock(mutex_);
        last_emotion_.clear();
        StopTimer();
        ClearMotionState();
        FinishIdleRelease();
        ESP_LOGI(TAG, "Manual arm release");
    }

    std::string GetStatusJson() {
        std::lock_guard<std::mutex> lock(mutex_);
        cJSON* root = cJSON_CreateObject();
        cJSON_AddNumberToObject(root, "left_gpio", static_cast<int>(ARM_SERVO_LEFT_GPIO));
        cJSON_AddNumberToObject(root, "right_gpio", static_cast<int>(ARM_SERVO_RIGHT_GPIO));
        cJSON_AddBoolToObject(root, "left_configured", ARM_SERVO_LEFT_GPIO != GPIO_NUM_NC);
        cJSON_AddBoolToObject(root, "right_configured", ARM_SERVO_RIGHT_GPIO != GPIO_NUM_NC);
        cJSON_AddBoolToObject(root, "position_known", PositionKnown());
        cJSON_AddBoolToObject(root, "left_position_known", left_position_known_);
        cJSON_AddBoolToObject(root, "right_position_known", right_position_known_);
        cJSON_AddStringToObject(root, "timer_state", TimerStateName(timer_state_));
        cJSON_AddStringToObject(root, "motion_mode", MotionModeName(motion_mode_));
        cJSON_AddBoolToObject(root, "speaking_active", speaking_active_);
        cJSON_AddNumberToObject(root, "left_degree", current_left_degree_);
        cJSON_AddNumberToObject(root, "right_degree", current_right_degree_);
        cJSON_AddStringToObject(root, "last_emotion", last_emotion_.c_str());

        char* json = cJSON_PrintUnformatted(root);
        std::string result = json ? json : "{}";
        if (json) {
            cJSON_free(json);
        }
        cJSON_Delete(root);
        return result;
    }

private:
    void InitializeLedc() {
        if (ARM_SERVO_LEFT_GPIO == GPIO_NUM_NC && ARM_SERVO_RIGHT_GPIO == GPIO_NUM_NC) {
            ESP_LOGW(TAG, "Arm servo disabled: no GPIO configured");
            return;
        }
        ESP_LOGI(TAG, "Arm servo PWM: left_gpio=%d left_channel=%d right_gpio=%d right_channel=%d timer=%d freq=%d",
                 static_cast<int>(ARM_SERVO_LEFT_GPIO), static_cast<int>(ARM_SERVO_LEFT_LEDC_CHANNEL),
                 static_cast<int>(ARM_SERVO_RIGHT_GPIO), static_cast<int>(ARM_SERVO_RIGHT_LEDC_CHANNEL),
                 static_cast<int>(ARM_SERVO_LEDC_TIMER), ARM_SERVO_PWM_FREQ_HZ);
        const ledc_timer_config_t servo_timer = {
            .speed_mode = kArmServoSpeedMode,
            .duty_resolution = ARM_SERVO_LEDC_DUTY_RESOLUTION,
            .timer_num = ARM_SERVO_LEDC_TIMER,
            .freq_hz = ARM_SERVO_PWM_FREQ_HZ,
            .clk_cfg = LEDC_AUTO_CLK,
            .deconfigure = false
        };
        ESP_ERROR_CHECK(ledc_timer_config(&servo_timer));

        ConfigureChannel(ARM_SERVO_LEFT_GPIO, ARM_SERVO_LEFT_LEDC_CHANNEL);
        ConfigureChannel(ARM_SERVO_RIGHT_GPIO, ARM_SERVO_RIGHT_LEDC_CHANNEL);
    }

    void ConfigureChannel(gpio_num_t gpio, ledc_channel_t channel) {
        if (gpio == GPIO_NUM_NC) {
            return;
        }
        const ledc_channel_config_t servo_channel = {
            .gpio_num = gpio,
            .speed_mode = kArmServoSpeedMode,
            .channel = channel,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = ARM_SERVO_LEDC_TIMER,
            .duty = 0,
            .hpoint = ServoHpoint(channel),
            .flags = {
                .output_invert = 0,
            }
        };
        ESP_ERROR_CHECK(ledc_channel_config(&servo_channel));
    }

    void CreateTimer() {
        const esp_timer_create_args_t timer_args = {
            .callback = [](void* arg) {
                static_cast<ArmServoController*>(arg)->OnTimer();
            },
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "arm_servo_timer",
            .skip_unhandled_events = true,
        };
        ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer_));
    }

    int ServoHpoint(ledc_channel_t channel) const {
        if (ARM_SERVO_LEFT_GPIO != GPIO_NUM_NC &&
                ARM_SERVO_RIGHT_GPIO != GPIO_NUM_NC &&
                channel == ARM_SERVO_RIGHT_LEDC_CHANNEL) {
            return 1 << (ARM_SERVO_LEDC_DUTY_RESOLUTION - 1);
        }
        return 0;
    }

    ArmTarget ParseArmTarget(const std::string& arm) const {
        if (arm == "left") {
            return ArmTarget::kLeft;
        }
        if (arm == "right") {
            return ArmTarget::kRight;
        }
        if (arm == "both") {
            return ArmTarget::kBoth;
        }
        throw std::runtime_error("Invalid arm. Use left, right, or both");
    }

    void EnsureArmConfigured(const char* arm_name, gpio_num_t gpio) const {
        if (gpio == GPIO_NUM_NC) {
            throw std::runtime_error(std::string(arm_name) + " arm servo is not configured");
        }
    }

    const char* MotionModeName(MotionMode mode) const {
        switch (mode) {
        case MotionMode::kEmotion:
            return "emotion";
        case MotionMode::kManual:
            return "manual";
        case MotionMode::kSpeakingLoop:
            return "speaking_loop";
        case MotionMode::kNone:
        default:
            return "none";
        }
    }

    const EmotionMotion& FindManualPose(const std::string& pose) const {
        for (const auto& motion : kManualArmPoses) {
            if (pose == motion.emotion) {
                return motion;
            }
        }
        throw std::runtime_error("Invalid pose. Use neutral, park, left_up, left_down, both_up, both_down, or wave");
    }

    const char* TimerStateName(TimerState state) const {
        switch (state) {
        case TimerState::kRunning:
            return "running";
        case TimerState::kIdleRelease:
            return "idle_release";
        case TimerState::kIdle:
        default:
            return "idle";
        }
    }

    bool PositionKnown() const {
        const bool left_known = (ARM_SERVO_LEFT_GPIO == GPIO_NUM_NC) || left_position_known_;
        const bool right_known = (ARM_SERVO_RIGHT_GPIO == GPIO_NUM_NC) || right_position_known_;
        return left_known && right_known;
    }

    void StopTimer() {
        if (timer_ != nullptr) {
            esp_timer_stop(timer_);
        }
    }

    void ClearMotionState() {
        frames_ = nullptr;
        frame_count_ = 0;
        frame_index_ = 0;
        parking_ = false;
        hold_ticks_ = 0;
        idle_ticks_remaining_ = 0;
        timer_state_ = TimerState::kIdle;
        motion_mode_ = MotionMode::kNone;
    }

    void StartMotion(const ArmServoFrame* frames, size_t frame_count, MotionMode motion_mode,
                     bool timer_running = false) {
        if (!timer_running) {
            StopTimer();
        }
        frames_ = frames;
        frame_count_ = frame_count;
        frame_index_ = 0;
        parking_ = false;
        idle_ticks_remaining_ = 0;
        timer_state_ = TimerState::kRunning;
        motion_mode_ = motion_mode;

        if (frames_ == nullptr || frame_count_ == 0) {
            timer_state_ = TimerState::kIdle;
            motion_mode_ = MotionMode::kNone;
            return;
        }

        SetTargetFrame(frames_[frame_index_]);
        if (ARM_SERVO_LEFT_GPIO != GPIO_NUM_NC && !left_position_known_) {
            current_left_degree_ = target_left_degree_;
            WriteServo(ARM_SERVO_LEFT_LEDC_CHANNEL, current_left_degree_);
            left_position_known_ = true;
        }
        if (ARM_SERVO_RIGHT_GPIO != GPIO_NUM_NC && !right_position_known_) {
            current_right_degree_ = target_right_degree_;
            WriteServo(ARM_SERVO_RIGHT_LEDC_CHANNEL, current_right_degree_);
            right_position_known_ = true;
        }

        if (!timer_running && timer_ != nullptr) {
            ESP_ERROR_CHECK(esp_timer_start_periodic(timer_, ARM_SERVO_UPDATE_INTERVAL_MS * 1000));
        }
    }

    void StartSpeakingLoop(bool timer_running = false) {
        StartMotion(kSpeakingMotion, FrameCount(kSpeakingMotion), MotionMode::kSpeakingLoop, timer_running);
    }

    const EmotionMotion& FindMotion(const char* emotion) const {
        for (const auto& motion : kEmotionMotions) {
            if (strcmp(motion.emotion, emotion) == 0) {
                return motion;
            }
        }
        return kEmotionMotions[0];
    }

    void SetTargetFrame(const ArmServoFrame& frame) {
        target_left_degree_ = ClampDegree(frame.left_degree);
        target_right_degree_ = ClampDegree(frame.right_degree);
        hold_ticks_ = frame.hold_ticks;
    }

    void OnTimer() {
        std::lock_guard<std::mutex> lock(mutex_);

        if (timer_state_ == TimerState::kIdleRelease) {
            HandleIdleReleaseTick();
            return;
        }

        if (timer_state_ != TimerState::kRunning || frames_ == nullptr || frame_count_ == 0) {
            return;
        }

        const bool moved_left = (ARM_SERVO_LEFT_GPIO != GPIO_NUM_NC) &&
            StepToward(current_left_degree_, target_left_degree_);
        const bool moved_right = (ARM_SERVO_RIGHT_GPIO != GPIO_NUM_NC) &&
            StepToward(current_right_degree_, target_right_degree_);
        if (moved_left || moved_right) {
            WriteMovedServos(moved_left, moved_right);
            return;
        }

        if (hold_ticks_ > 0) {
            --hold_ticks_;
            return;
        }

        if (frame_index_ + 1 < frame_count_) {
            ++frame_index_;
            SetTargetFrame(frames_[frame_index_]);
            return;
        }

        if (motion_mode_ == MotionMode::kSpeakingLoop) {
            frame_index_ = 0;
            SetTargetFrame(frames_[frame_index_]);
            return;
        }

        if (speaking_active_) {
            StartSpeakingLoop(true);
            return;
        }

        if (!parking_ && NeedsPark()) {
            parking_ = true;
            SetTargetFrame(kParkFrame);
            return;
        }

        StartIdleRelease();
    }

    void StartIdleRelease(bool timer_running = true) {
        frames_ = nullptr;
        frame_count_ = 0;
        frame_index_ = 0;
        parking_ = false;
        hold_ticks_ = 0;
        idle_ticks_remaining_ =
            (ARM_SERVO_IDLE_RELEASE_MS + ARM_SERVO_UPDATE_INTERVAL_MS - 1) / ARM_SERVO_UPDATE_INTERVAL_MS;
        timer_state_ = TimerState::kIdleRelease;
        motion_mode_ = MotionMode::kNone;

        if (idle_ticks_remaining_ <= 0) {
            FinishIdleRelease();
            return;
        }

        if (!timer_running && timer_ != nullptr) {
            ESP_ERROR_CHECK(esp_timer_start_periodic(timer_, ARM_SERVO_UPDATE_INTERVAL_MS * 1000));
        }
    }

    void HandleIdleReleaseTick() {
        if (idle_ticks_remaining_ > 0) {
            --idle_ticks_remaining_;
        }

        if (idle_ticks_remaining_ <= 0) {
            FinishIdleRelease();
        }
    }

    void FinishIdleRelease() {
        StopServo(ARM_SERVO_LEFT_GPIO, ARM_SERVO_LEFT_LEDC_CHANNEL);
        StopServo(ARM_SERVO_RIGHT_GPIO, ARM_SERVO_RIGHT_LEDC_CHANNEL);
        left_position_known_ = false;
        right_position_known_ = false;
        timer_state_ = TimerState::kIdle;
        parking_ = false;
        idle_ticks_remaining_ = 0;
        StopTimer();
    }

    bool StepToward(int& current_degree, int target_degree) const {
        if (current_degree == target_degree) {
            return false;
        }

        const int delta = target_degree - current_degree;
        const int step_limit = ARM_SERVO_MAX_STEP_DEGREE;
        if (delta > 0) {
            current_degree += (delta > step_limit) ? step_limit : delta;
        } else {
            current_degree += (delta < -step_limit) ? -step_limit : delta;
        }
        return true;
    }

    int ClampDegree(int degree) const {
        if (degree < ARM_SERVO_MIN_DEGREE) {
            return ARM_SERVO_MIN_DEGREE;
        }
        if (degree > ARM_SERVO_MAX_DEGREE) {
            return ARM_SERVO_MAX_DEGREE;
        }
        return degree;
    }

    uint32_t DegreeToDuty(int degree) const {
        const uint32_t period_us = 1000000 / ARM_SERVO_PWM_FREQ_HZ;
        const uint32_t max_duty = (1UL << ARM_SERVO_LEDC_DUTY_RESOLUTION) - 1;
        return (DegreeToPulseUs(degree) * max_duty) / period_us;
    }

    uint32_t DegreeToPulseUs(int degree) const {
        const int clamped_degree = ClampDegree(degree);
        return ARM_SERVO_MIN_PULSE_US +
            ((ARM_SERVO_MAX_PULSE_US - ARM_SERVO_MIN_PULSE_US) * clamped_degree) /
            (ARM_SERVO_MAX_DEGREE - ARM_SERVO_MIN_DEGREE);
    }

    bool NeedsPark() const {
        const bool left_needs_park = (ARM_SERVO_LEFT_GPIO != GPIO_NUM_NC) &&
            current_left_degree_ != ARM_SERVO_PARK_DEGREE;
        const bool right_needs_park = (ARM_SERVO_RIGHT_GPIO != GPIO_NUM_NC) &&
            current_right_degree_ != ARM_SERVO_PARK_DEGREE;
        return left_needs_park || right_needs_park;
    }

    void WriteMovedServos(bool moved_left, bool moved_right) {
        if (moved_left) {
            WriteServo(ARM_SERVO_LEFT_LEDC_CHANNEL, current_left_degree_);
        }
        if (moved_right) {
            WriteServo(ARM_SERVO_RIGHT_LEDC_CHANNEL, current_right_degree_);
        }
    }

    void WriteServo(ledc_channel_t channel, int degree) {
        if (channel == ARM_SERVO_LEFT_LEDC_CHANNEL && ARM_SERVO_LEFT_GPIO == GPIO_NUM_NC) {
            return;
        }
        if (channel == ARM_SERVO_RIGHT_LEDC_CHANNEL && ARM_SERVO_RIGHT_GPIO == GPIO_NUM_NC) {
            return;
        }
        const uint32_t duty = DegreeToDuty(degree);
        ESP_ERROR_CHECK(ledc_set_duty(kArmServoSpeedMode, channel, duty));
        ESP_ERROR_CHECK(ledc_update_duty(kArmServoSpeedMode, channel));
    }

    const char* ServoName(ledc_channel_t channel) const {
        if (channel == ARM_SERVO_LEFT_LEDC_CHANNEL) {
            return "left";
        }
        if (channel == ARM_SERVO_RIGHT_LEDC_CHANNEL) {
            return "right";
        }
        return "unknown";
    }

    int ServoGpio(ledc_channel_t channel) const {
        if (channel == ARM_SERVO_LEFT_LEDC_CHANNEL) {
            return static_cast<int>(ARM_SERVO_LEFT_GPIO);
        }
        if (channel == ARM_SERVO_RIGHT_LEDC_CHANNEL) {
            return static_cast<int>(ARM_SERVO_RIGHT_GPIO);
        }
        return static_cast<int>(GPIO_NUM_NC);
    }

    void StopServo(gpio_num_t gpio, ledc_channel_t channel) {
        if (gpio == GPIO_NUM_NC) {
            return;
        }
        ESP_ERROR_CHECK(ledc_stop(kArmServoSpeedMode, channel, 0));
    }

    esp_timer_handle_t timer_ = nullptr;
    std::mutex mutex_;
    std::string last_emotion_;
    const ArmServoFrame* frames_ = nullptr;
    size_t frame_count_ = 0;
    size_t frame_index_ = 0;
    TimerState timer_state_ = TimerState::kIdle;
    MotionMode motion_mode_ = MotionMode::kNone;
    bool parking_ = false;
    bool speaking_active_ = false;
    bool left_position_known_ = false;
    bool right_position_known_ = false;
    int hold_ticks_ = 0;
    int idle_ticks_remaining_ = 0;
    int current_left_degree_ = ARM_SERVO_CENTER_DEGREE;
    int current_right_degree_ = ARM_SERVO_CENTER_DEGREE;
    int target_left_degree_ = ARM_SERVO_CENTER_DEGREE;
    int target_right_degree_ = ARM_SERVO_CENTER_DEGREE;
};

class EmojiArmLcdDisplay : public SpiLcdDisplay {
public:
    EmojiArmLcdDisplay(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_handle_t panel,
                       int width, int height, int offset_x, int offset_y,
                       bool mirror_x, bool mirror_y, bool swap_xy,
                       ArmServoController* arm_servo)
        : SpiLcdDisplay(panel_io, panel, width, height, offset_x, offset_y, mirror_x, mirror_y, swap_xy),
          arm_servo_(arm_servo) {
    }

    void SetEmotion(const char* emotion) override {
        LcdDisplay::SetEmotion(emotion);
        if (arm_servo_ != nullptr) {
            arm_servo_->SetEmotion(emotion);
        }
    }

private:
    ArmServoController* arm_servo_ = nullptr;
};
}

class CompactWifiBoardS3Cam : public WifiBoard {
private:

    Button boot_button_;
    ArmServoController arm_servos_;
    LcdDisplay* display_;
    Esp32Camera* camera_;

    void InitializeSpi() {
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = DISPLAY_MOSI_PIN;
        buscfg.miso_io_num = GPIO_NUM_NC;
        buscfg.sclk_io_num = DISPLAY_CLK_PIN;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * kLcdDrawBufferLines * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    void InitializeLcdDisplay() {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;
        // 液晶屏控制IO初始化
        ESP_LOGD(TAG, "Install panel IO");
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = DISPLAY_CS_PIN;
        io_config.dc_gpio_num = DISPLAY_DC_PIN;
        io_config.spi_mode = DISPLAY_SPI_MODE;
        io_config.pclk_hz = 40 * 1000 * 1000;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &panel_io));

        // 初始化液晶屏驱动芯片
        ESP_LOGD(TAG, "Install LCD driver");
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = DISPLAY_RST_PIN;
        panel_config.rgb_ele_order = DISPLAY_RGB_ORDER;
        panel_config.bits_per_pixel = 16;
#if defined(LCD_TYPE_ILI9341_SERIAL)
        ESP_ERROR_CHECK(esp_lcd_new_panel_ili9341(panel_io, &panel_config, &panel));
#elif defined(LCD_TYPE_GC9A01_SERIAL)
        ESP_ERROR_CHECK(esp_lcd_new_panel_gc9a01(panel_io, &panel_config, &panel));
        gc9a01_vendor_config_t gc9107_vendor_config = {
            .init_cmds = gc9107_lcd_init_cmds,
            .init_cmds_size = sizeof(gc9107_lcd_init_cmds) / sizeof(gc9a01_lcd_init_cmd_t),
        };        
#else
        ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(panel_io, &panel_config, &panel));
#endif
        
        esp_lcd_panel_reset(panel);

        esp_lcd_panel_init(panel);
        esp_lcd_panel_invert_color(panel, DISPLAY_INVERT_COLOR);
        esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
#ifdef  LCD_TYPE_GC9A01_SERIAL
        panel_config.vendor_config = &gc9107_vendor_config;
#endif
        display_ = new EmojiArmLcdDisplay(panel_io, panel,
                                    DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY,
                                    &arm_servos_);
    }

    void InitializeCamera() {
        camera_config_t config = {};
        config.pin_d0 = CAMERA_PIN_D0;
        config.pin_d1 = CAMERA_PIN_D1;
        config.pin_d2 = CAMERA_PIN_D2;
        config.pin_d3 = CAMERA_PIN_D3;
        config.pin_d4 = CAMERA_PIN_D4;
        config.pin_d5 = CAMERA_PIN_D5;
        config.pin_d6 = CAMERA_PIN_D6;
        config.pin_d7 = CAMERA_PIN_D7;
        config.pin_xclk = CAMERA_PIN_XCLK;
        config.pin_pclk = CAMERA_PIN_PCLK;
        config.pin_vsync = CAMERA_PIN_VSYNC;
        config.pin_href = CAMERA_PIN_HREF;
        config.pin_sccb_sda = CAMERA_PIN_SIOD;
        config.pin_sccb_scl = CAMERA_PIN_SIOC;
        config.sccb_i2c_port = 0;
        config.pin_pwdn = CAMERA_PIN_PWDN;
        config.pin_reset = CAMERA_PIN_RESET;
        config.xclk_freq_hz = XCLK_FREQ_HZ;
        config.ledc_timer = CAMERA_XCLK_LEDC_TIMER;
        config.ledc_channel = CAMERA_XCLK_LEDC_CHANNEL;
        config.pixel_format = PIXFORMAT_RGB565;
        config.frame_size = FRAMESIZE_VGA;
        config.jpeg_quality = 12;
        config.fb_count = 1;
        config.fb_location = CAMERA_FB_IN_PSRAM;
        config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
        camera_ = new Esp32Camera(config);
        camera_->SetHMirror(false);
    }

    void InitializeButtons() {
        boot_button_.OnMultipleClick([this]() {
            ResetWifiConfiguration();
        }, 5);

        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting && !WifiStation::GetInstance().IsConnected()) {
                ResetWifiConfiguration();
            }
            app.ToggleChatState();
        });
    }

    void InitializeTools() {
    }

    void InitializeStateCallbacks() {
        DeviceStateEventManager::GetInstance().RegisterStateChangeCallback(
            [this](DeviceState previous_state, DeviceState current_state) {
                if (current_state == kDeviceStateSpeaking) {
                    arm_servos_.SetSpeakingActive(true);
                } else if (previous_state == kDeviceStateSpeaking) {
                    arm_servos_.SetSpeakingActive(false);
                }
            });
    }

public:
    CompactWifiBoardS3Cam() :
        boot_button_(BOOT_BUTTON_GPIO) {
        InitializeSpi();
        InitializeLcdDisplay();
        InitializeButtons();
        InitializeCamera();
        InitializeTools();
        InitializeStateCallbacks();
        if (DISPLAY_BACKLIGHT_PIN != GPIO_NUM_NC) {
            GetBacklight()->RestoreBrightness();
        }

    }

    virtual Led* GetLed() override {
        if (BUILTIN_LED_GPIO == GPIO_NUM_NC) {
            static NoLed no_led;
            return &no_led;
        }
        static SingleLed led(BUILTIN_LED_GPIO);
        return &led;
    }

    virtual AudioCodec* GetAudioCodec() override {
#ifdef AUDIO_I2S_METHOD_SIMPLEX
        static NoAudioCodecSimplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_SPK_GPIO_BCLK, AUDIO_I2S_SPK_GPIO_LRCK, AUDIO_I2S_SPK_GPIO_DOUT, AUDIO_I2S_MIC_GPIO_SCK, AUDIO_I2S_MIC_GPIO_WS, AUDIO_I2S_MIC_GPIO_DIN);
#else
        static NoAudioCodecDuplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN);
#endif
        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }

    virtual Backlight* GetBacklight() override {
        if (DISPLAY_BACKLIGHT_PIN != GPIO_NUM_NC) {
            static PwmBacklight backlight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
            return &backlight;
        }
        return nullptr;
    }

    virtual Camera* GetCamera() override {
        return camera_;
    }

    virtual SdCard* GetSdCard() override {
        static FlashFatFs flash_fatfs;
        return &flash_fatfs;
    }
};

DECLARE_BOARD(CompactWifiBoardS3Cam);
