#ifndef DEVICE_STATE_EVENT_MANAGER_H
#define DEVICE_STATE_EVENT_MANAGER_H

#include <functional>
#include "device_state.h"

class DeviceStateEventManager {
public:
    static DeviceStateEventManager& GetInstance();

    using StateChangeCallback = std::function<void(DeviceState old_state, DeviceState new_state)>;

    void SubscribeStateChange(StateChangeCallback callback);
    void PostStateChangeEvent(DeviceState old_state, DeviceState new_state);

    DeviceStateEventManager(const DeviceStateEventManager&) = delete;
    DeviceStateEventManager& operator=(const DeviceStateEventManager&) = delete;

private:
    DeviceStateEventManager() = default;

    StateChangeCallback state_change_callback_;
};

#endif  // DEVICE_STATE_EVENT_MANAGER_H
