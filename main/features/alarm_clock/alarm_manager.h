#pragma once

#include <string>
#include <vector>
#include <time.h>
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"

// Forward declarations
class Application;
class Board;
class Display;

class AlarmManager {
public:
    // Struct công khai để MCP có thể dùng
    struct Alarm {
        uint8_t hour;
        uint8_t minute;
        std::string message;
        std::string sound_id;  // ID của file âm thanh (mặc định "alarm")
        bool enabled;
        bool repeated;
    };

    static AlarmManager& getInstance();
    
    // Khởi tạo
    void init();
    
    // Parse text từ LLM
    void parseResponse(const std::string& text);
    
    // Kiểm tra alarm (gọi mỗi giây)
    void checkAlarms();
    
    // Lấy thông tin alarm tiếp theo
    std::string getNextAlarmInfo();
    
    // MỚI: Lấy thông tin tất cả alarms
    std::string getAllAlarmsInfo();
    
    // Xóa tất cả alarms
    void clearAll();
    
    // Thêm alarm thủ công (cho MCP)
    void addAlarm(const Alarm& alarm);
    
    // Lấy danh sách alarms
    const std::vector<Alarm>& getAlarms() const { return alarms_; }
    
    // Xóa các alarm đã hết hạn
    void cleanupExpiredAlarms();

    // Kiểm tra trùng lặp
    bool isDuplicateAlarm(uint8_t hour, uint8_t minute);

    // Lưu alarms vào NVS (dùng bởi MCP để cập nhật sound_id)
    void saveAlarmsToNVS() { saveToNVS(); }

    // Cập nhật sound_id cho alarm cụ thể
    bool setAlarmSound(uint8_t hour, uint8_t minute, const std::string& sound_id);

private:
    AlarmManager() = default;
    ~AlarmManager() = default;
    AlarmManager(const AlarmManager&) = delete;
    AlarmManager& operator=(const AlarmManager&) = delete;

    std::vector<Alarm> alarms_;
    nvs_handle_t nvs_handle_;
    time_t last_check_time_;
    
    void loadFromNVS();
    void saveToNVS();
    bool parseTime(const std::string& text, uint8_t& hour, uint8_t& minute);
    void triggerAlarm(const Alarm& alarm);
};