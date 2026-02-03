#ifndef _OTA_H
#define _OTA_H

#include <functional>
#include <string>

#include <esp_err.h>
#include "board.h"

/*
{
    "firmware": {
        "name": "xingzhi-cube-1.54tft-wifi",
        "version": "2.0.5.06",
        "url": "",
        "size": 0,
        "checksum": "",
        "force": 0,
        "enable": 1,
        "features": {
            "music": true,
            "radio": true,
            "weather": false,
            "sdCardMusic": true,
            "alarm": true,
            "reminder": true,
            "voiceRecording": true,
            "bluetooth": false,
            "homeAssistant": false
        }
    }
}
*/
typedef struct {
    bool music;
    bool radio;
    bool weather;
    bool sdCardMusic;
    bool alarm;
    bool reminder;
    bool voiceRecording;
    bool bluetooth;
    bool homeAssistant;
} ota_features_t;

class Ota {
public:
    Ota();
    ~Ota();

    bool CheckVersion(std::string& url);
    esp_err_t Activate();
    bool HasActivationChallenge() { return has_activation_challenge_; }
    bool HasNewVersion() { return has_new_version_; }
    bool HasMqttConfig() { return has_mqtt_config_; }
    bool HasWebsocketConfig() { return has_websocket_config_; }
    bool HasActivationCode() { return has_activation_code_; }
    bool HasServerTime() { return has_server_time_; }
    bool StartUpgrade(std::function<void(int progress, size_t speed)> callback);
    bool StartUpgradeFromUrl(const std::string& url, std::function<void(int progress, size_t speed)> callback);
    void MarkCurrentVersionValid();

    const std::string& GetFirmwareVersion() const { return firmware_version_; }
    const std::string& GetCurrentVersion() const { return current_version_; }
    const std::string& GetFirmwareUrl() const { return firmware_url_; }
    const std::string& GetActivationMessage() const { return activation_message_; }
    const std::string& GetActivationCode() const { return activation_code_; }
    std::string GetCheckVersionUrl();
    std::string GetFirmwareChecksum();

    uint8_t IsForced() const { return has_forced_; }
    uint8_t IsEnabled() const { return has_enabled_; }
    // Getter methods for features
    bool IsMusicEnabled() const { return features_.music; }
    bool IsRadioEnabled() const { return features_.radio; }
    bool IsWeatherEnabled() const { return features_.weather; }
    bool IsSdCardMusicEnabled() const { return features_.sdCardMusic; }
    bool IsAlarmEnabled() const { return features_.alarm; }
    bool IsReminderEnabled() const { return features_.reminder; }
    bool IsVoiceRecordingEnabled() const { return features_.voiceRecording; }
    bool IsBluetoothEnabled() const { return features_.bluetooth; }
    bool IsHomeAssistantEnabled() const { return features_.homeAssistant; }
    const ota_features_t& GetFeatures() const { return features_; }

private:
    std::string activation_message_;
    std::string activation_code_;
    bool has_new_version_ = false;
    bool has_mqtt_config_ = false;
    bool has_websocket_config_ = false;
    bool has_server_time_ = false;
    bool has_activation_code_ = false;
    bool has_serial_number_ = false;
    bool has_activation_challenge_ = false;
    bool has_enabled_ = true;
    bool has_forced_ = false;
    std::string checksum_;
    std::string current_version_;
    std::string firmware_version_;
    std::string firmware_url_;
    std::string activation_challenge_;
    std::string serial_number_;
    int firmware_size_ = 0;
    int activation_timeout_ms_ = 30000;
    ota_features_t features_ = {};

    bool Upgrade(const std::string& firmware_url);
    std::function<void(int progress, size_t speed)> upgrade_callback_;
    std::vector<int> ParseVersion(const std::string& version);
    bool IsNewVersionAvailable(const std::string& currentVersion, const std::string& newVersion);
    std::string GetActivationPayload();
    std::unique_ptr<Http> SetupHttp();
};

#endif // _OTA_H
