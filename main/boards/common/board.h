#ifndef BOARD_H
#define BOARD_H

#include <http.h>
#include <web_socket.h>
#include <mqtt.h>
#include <udp.h>
#include <functional>
#include <string>
#include <network_interface.h>

#include "led/led.h"
#include "backlight.h"
#include "camera.h"
#include "assets.h"

void* create_board();
class AudioCodec;
class Display;
class LcdTouch;
class SdCard;

// Network event types (used by all board implementations)
using NetworkEventCallback = std::function<void(int event, const std::string& data)>;

enum class NetworkEvent {
    Disconnected = 0,
    Connecting,
    Connected,
    Error,
    // Modem-specific events (used by Ml307Board)
    ModemDetecting,
    ModemErrorNoSim,
    ModemErrorRegDenied,
    ModemErrorInitFailed,
    ModemErrorTimeout,
};

enum class PowerSaveLevel {
    LOW_POWER,
    BALANCED,
    PERFORMANCE,
};

class Board {
private:
    Board(const Board&) = delete; // Disable copy constructor
    Board& operator=(const Board&) = delete; // Disable assignment operator

protected:
    Board();
    std::string GenerateUuid();

    // Software-generated unique device identifier
    std::string uuid_;
public:
    static Board& GetInstance() {
        static Board* instance = static_cast<Board*>(create_board());
        return *instance;
    }

    virtual ~Board();// = default;
    virtual std::string GetBoardType() = 0;
    virtual std::string GetUuid() { return uuid_; }
    virtual Backlight* GetBacklight() { return nullptr; }
    virtual Led* GetLed();
    virtual AudioCodec* GetAudioCodec() = 0;
    virtual bool GetTemperature(float& esp32temp);
    virtual Display* GetDisplay();
    virtual LcdTouch* GetTouch() { return nullptr; }
    virtual Camera* GetCamera();
    virtual SdCard* GetSdCard();
    virtual NetworkInterface* GetNetwork() = 0;
    virtual void StartNetwork() = 0;
    virtual const char* GetNetworkStateIcon() = 0;
    virtual bool GetBatteryLevel(int &level, bool& charging, bool& discharging);
    virtual std::string GetSystemInfoJson();
    virtual void SetPowerSaveMode(bool enabled) = 0;
    virtual void SetNetworkEventCallback(NetworkEventCallback callback) {}
    virtual void SetPowerSaveLevel(PowerSaveLevel level) {}
    virtual std::string GetBoardJson() = 0;
    virtual std::string GetDeviceStatusJson() = 0;
};

#define DECLARE_BOARD(BOARD_CLASS_NAME) \
void* create_board() { \
    return new BOARD_CLASS_NAME(); \
}

#endif // BOARD_H
