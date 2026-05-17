#pragma once
#include <string_view>
#include "assets/lang_config.h"

/**
 * Registry of available alarm sounds.
 * Each entry maps a sound_id (string) to its Lang::Sounds OGG constant.
 */
struct AlarmSound {
    const char* id;          // e.g. "school", "wakeup"
    const char* name;        // Display name in Vietnamese
    const std::string_view* ogg; // pointer to the string_view constant
};

// Macro to get a pointer to each sound's OGG string_view
#define OGG_PTR(name) (&Lang::Sounds::OGG_##name)

static constexpr AlarmSound kAlarmSounds[] = {
    { "alarm",   "Chuông báo thức mặc định", OGG_PTR(ALARM) },
    { "school",  "Đã tới giờ đi học rồi đấy", OGG_PTR(SCHOOL) },
    { "wakeup",  "Dậy thôi, sáng rồi!",        OGG_PTR(WAKEUP) },
    { "medicine","Đã tới giờ uống thuốc",       OGG_PTR(MEDICINE) },
};

static constexpr size_t kAlarmSoundCount = sizeof(kAlarmSounds) / sizeof(kAlarmSounds[0]);

/**
 * Find alarm sound by ID.
 * Returns nullptr if not found.
 */
inline const AlarmSound* FindAlarmSound(const std::string& id) {
    for (size_t i = 0; i < kAlarmSoundCount; i++) {
        if (id == kAlarmSounds[i].id) {
            return &kAlarmSounds[i];
        }
    }
    return nullptr;
}

/**
 * Get the OGG string_view for a given sound_id.
 * Falls back to "alarm" if not found.
 */
inline std::string_view GetAlarmSoundOgg(const std::string& sound_id) {
    const AlarmSound* sound = FindAlarmSound(sound_id);
    if (sound) {
        return *(sound->ogg);
    }
    // Fallback to default alarm
    return Lang::Sounds::OGG_ALARM;
}
