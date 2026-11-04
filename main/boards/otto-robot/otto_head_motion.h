
#ifndef OTTO_HEAD_MOTION_H
#define OTTO_HEAD_MOTION_H

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "oscillator.h"

class OttoHeadMotion {
public:
    OttoHeadMotion();
    ~OttoHeadMotion();

    bool Initialize(int pan_gpio, int tilt_gpio);
    void Deinitialize();

    // Góc tương đối so với vị trí trung tâm
    void MoveTo(int pan_offset, int tilt_offset);

    void Center();
    void LookLeft();
    void LookRight();
    void LookUp();
    void LookDown();

    void SetSpeed(int degrees_per_second);

    bool IsInitialized() const;

private:
    static void MotionTask(void* arg);
    void UpdateMotion();

    Oscillator pan_servo_;
    Oscillator tilt_servo_;

    TaskHandle_t motion_task_;

    int pan_gpio_;
    int tilt_gpio_;

    volatile int target_pan_;
    volatile int target_tilt_;

    bool initialized_;

    static constexpr int PAN_LIMIT = 45;
    static constexpr int TILT_LIMIT = 35;
    static constexpr int CENTER = 90;
    static constexpr int DEFAULT_SPEED = 180;
};

#endif
