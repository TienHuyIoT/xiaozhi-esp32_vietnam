#include "device_state_event_manager.h"

DeviceStateEventManager& DeviceStateEventManager::GetInstance() {
    static DeviceStateEventManager instance;
    return instance;
}

void DeviceStateEventManager::SubscribeStateChange(StateChangeCallback callback) {
    state_change_callback_ = std::move(callback);
}

void DeviceStateEventManager::PostStateChangeEvent(DeviceState old_state, DeviceState new_state) {
    if (state_change_callback_) {
        state_change_callback_(old_state, new_state);
    }
}
