#include "alarm_manager.h"
#include "application.h"
#include "board.h"
#include "display/display.h"
#include "assets/lang_config.h"

#include <regex>
#include <sstream>
#include <cstring>
#include <sys/time.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <algorithm>

static const char* TAG = "AlarmManager";

// Âm thanh báo thức (file OGG)
extern const uint8_t alarm_beep_ogg_start[] asm("_binary_alarm_beep_ogg_start");
extern const uint8_t alarm_beep_ogg_end[] asm("_binary_alarm_beep_ogg_end");

AlarmManager& AlarmManager::getInstance() {
    static AlarmManager instance;
    return instance;
}

void AlarmManager::init() {
    ESP_LOGI(TAG, "Initializing Alarm Manager");
    
    esp_err_t err = nvs_open("alarms", NVS_READWRITE, &nvs_handle_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(err));
        return;
    }
    
    loadFromNVS();
    last_check_time_ = 0;
    
    // XÓA CÁC ALARM ĐÃ HẾT HẠN (repeated=false và enabled=false)
    cleanupExpiredAlarms();
    
    ESP_LOGI(TAG, "Alarm Manager initialized with %d alarms", alarms_.size());
}

void AlarmManager::loadFromNVS() {
    uint8_t count = 0;
    nvs_get_u8(nvs_handle_, "count", &count);
    
    ESP_LOGI(TAG, "Loading %d alarms from NVS", count);
    
    for (uint8_t i = 0; i < count && i < 10; i++) {
        char key[16];
        snprintf(key, sizeof(key), "alarm_%d", i);
        
        size_t required_size = 0;
        esp_err_t err = nvs_get_blob(nvs_handle_, key, NULL, &required_size);
        
        if (err == ESP_OK && required_size > 0) {
            char* buffer = (char*)malloc(required_size);
            nvs_get_blob(nvs_handle_, key, buffer, &required_size);
            
            Alarm alarm;
            int parsed = sscanf(buffer, "%hhu:%hhu|", &alarm.hour, &alarm.minute);
            
            if (parsed == 2) {
                char* msg_start = strchr(buffer, '|');
                if (msg_start) {
                    msg_start++;
                    char* flag_start = strchr(msg_start, '|');
                    if (flag_start) {
                        *flag_start = '\0';
                        alarm.message = msg_start;
                        alarm.enabled = (flag_start[1] == '1');
                        alarm.repeated = (flag_start[3] == '1');
                        alarms_.push_back(alarm);
                        ESP_LOGI(TAG, "Loaded alarm: %02d:%02d - %s (enabled: %d, repeated: %d)", 
                                alarm.hour, alarm.minute, alarm.message.c_str(), 
                                alarm.enabled, alarm.repeated);
                    }
                }
            }
            
            free(buffer);
        }
    }
}

void AlarmManager::saveToNVS() {
    nvs_set_u8(nvs_handle_, "count", alarms_.size());
    
    for (size_t i = 0; i < alarms_.size(); i++) {
        char key[16];
        snprintf(key, sizeof(key), "alarm_%d", i);
        
        char buffer[256];
        snprintf(buffer, sizeof(buffer), "%02d:%02d|%s|%d|%d",
                 alarms_[i].hour, alarms_[i].minute,
                 alarms_[i].message.c_str(),
                 alarms_[i].enabled ? 1 : 0,
                 alarms_[i].repeated ? 1 : 0);
        
        nvs_set_blob(nvs_handle_, key, buffer, strlen(buffer) + 1);
    }
    
    nvs_commit(nvs_handle_);
    ESP_LOGI(TAG, "Saved %d alarms to NVS", alarms_.size());
}

void AlarmManager::cleanupExpiredAlarms() {
    auto it = std::remove_if(alarms_.begin(), alarms_.end(), 
        [](const Alarm& alarm) {
            return !alarm.enabled && !alarm.repeated;
        });
    
    if (it != alarms_.end()) {
        int removed_count = std::distance(it, alarms_.end());
        alarms_.erase(it, alarms_.end());
        saveToNVS();
        ESP_LOGI(TAG, "Cleaned up %d expired alarms", removed_count);
    }
}

bool AlarmManager::isDuplicateAlarm(uint8_t hour, uint8_t minute) {
    for (const auto& alarm : alarms_) {
        if (alarm.enabled && alarm.hour == hour && alarm.minute == minute) {
            return true;
        }
    }
    return false;
}

bool AlarmManager::parseTime(const std::string& text, uint8_t& hour, uint8_t& minute) {
    // Pattern 1: "9h45", "9h"
    std::regex pattern_compact("(\\d{1,2})h(\\d{1,2})?");
    std::smatch match_compact;
    
    if (std::regex_search(text, match_compact, pattern_compact)) {
        hour = std::stoi(match_compact[1]);
        minute = match_compact[2].matched ? std::stoi(match_compact[2]) : 0;
        ESP_LOGI(TAG, "Parsed compact: %02d:%02d", hour, minute);
        return true;
    }
    
    // Pattern 2: "9 gio 45"
    std::regex pattern_full("(\\d{1,2})\\s*(gio|giờ|h)\\s*(\\d{1,2})?");
    std::smatch match_full;
    
    if (std::regex_search(text, match_full, pattern_full)) {
        hour = std::stoi(match_full[1]);
        minute = match_full[3].matched ? std::stoi(match_full[3]) : 0;
        
        if (text.find("chieu") != std::string::npos || 
            text.find("chiều") != std::string::npos ||
            text.find("toi") != std::string::npos ||
            text.find("tối") != std::string::npos) {
            if (hour < 12) hour += 12;
        } else if (text.find("sang") != std::string::npos ||
                   text.find("sáng") != std::string::npos) {
            if (hour == 12) hour = 0;
        }
        
        ESP_LOGI(TAG, "Parsed full: %02d:%02d", hour, minute);
        return true;
    }
    
    // Pattern 3: "09:45"
    std::regex pattern_time("(\\d{1,2}):(\\d{2})");
    std::smatch match_time;
    
    if (std::regex_search(text, match_time, pattern_time)) {
        hour = std::stoi(match_time[1]);
        minute = std::stoi(match_time[2]);
        ESP_LOGI(TAG, "Parsed time: %02d:%02d", hour, minute);
        return true;
    }
    
    ESP_LOGW(TAG, "Could not parse time from: %s", text.c_str());
    return false;
}

void AlarmManager::parseResponse(const std::string& text) {
    // 1. KIỂM TRA XÓA TRƯỚC
    if (text.find("xoa bao thuc") != std::string::npos ||
        text.find("xóa báo thức") != std::string::npos ||
        text.find("xoa tat ca") != std::string::npos ||
        text.find("huy bao") != std::string::npos) {
        clearAll();
        ESP_LOGI(TAG, "All alarms cleared");
        return;
    }
    
    // 2. KIỂM TRA XEM DANH SÁCH (KHÔNG XỬ LÝ Ở ĐÂY, ĐỂ MCP XỬ LÝ)
    if (text.find("kiem tra") != std::string::npos ||
        text.find("kiểm tra") != std::string::npos ||
        text.find("xem") != std::string::npos ||
        text.find("danh sach") != std::string::npos ||
        text.find("danh sách") != std::string::npos ||
        text.find("co bao nhieu") != std::string::npos ||
        text.find("có bao nhiêu") != std::string::npos) {
        ESP_LOGI(TAG, "List alarm request detected - let MCP handle it");
        return;
    }
    
    // 3. KIỂM TRA ĐẶT BÁO THỨC MỚI
    bool is_alarm = (text.find("bao thuc") != std::string::npos ||
                     text.find("báo thức") != std::string::npos ||
                     text.find("alarm") != std::string::npos ||
                     text.find("dat bao") != std::string::npos ||
                     text.find("đặt báo") != std::string::npos);
    
    bool is_reminder = (text.find("nhac") != std::string::npos ||
                        text.find("nhắc") != std::string::npos ||
                        text.find("reminder") != std::string::npos);
    
    if (!is_alarm && !is_reminder) {
        return;
    }
    
    ESP_LOGI(TAG, "Detected alarm request: %s", text.c_str());
    
    uint8_t hour, minute;
    if (!parseTime(text, hour, minute)) {
        ESP_LOGW(TAG, "Could not parse time");
        return;
    }
    
    if (hour > 23 || minute > 59) {
        ESP_LOGW(TAG, "Invalid time: %02d:%02d", hour, minute);
        return;
    }
    
    if (isDuplicateAlarm(hour, minute)) {
        ESP_LOGW(TAG, "Duplicate alarm %02d:%02d - skipping", hour, minute);
        return;
    }
    
    Alarm new_alarm;
    new_alarm.hour = hour;
    new_alarm.minute = minute;
    new_alarm.enabled = true;
    new_alarm.repeated = false;
    
    if (text.length() > 50) {
        new_alarm.message = text.substr(0, 50) + "...";
    } else {
        new_alarm.message = text;
    }
    
    if (text.find("hang ngay") != std::string::npos ||
        text.find("hàng ngày") != std::string::npos ||
        text.find("moi ngay") != std::string::npos ||
        text.find("mỗi ngày") != std::string::npos) {
        new_alarm.repeated = true;
    }
    
    alarms_.push_back(new_alarm);
    saveToNVS();
    
    ESP_LOGI(TAG, "Added alarm: %02d:%02d - %s (repeated: %d)",
             hour, minute, new_alarm.message.c_str(), new_alarm.repeated);
}

void AlarmManager::addAlarm(const Alarm& alarm) {
    if (alarm.hour > 23 || alarm.minute > 59) {
        ESP_LOGW(TAG, "Invalid alarm time");
        return;
    }
    
    // KIEM TRA TRUNG
    if (isDuplicateAlarm(alarm.hour, alarm.minute)) {
        ESP_LOGW(TAG, "Duplicate alarm %02d:%02d - skipping", alarm.hour, alarm.minute);
        return;
    }
    
    alarms_.push_back(alarm);
    saveToNVS();
    ESP_LOGI(TAG, "Added alarm via MCP: %02d:%02d", alarm.hour, alarm.minute);
}

void AlarmManager::checkAlarms() {
    time_t now;
    time(&now);
    
    if (now == last_check_time_) {
        return;
    }
    last_check_time_ = now;
    
    struct tm timeinfo;
    localtime_r(&now, &timeinfo);
    
    bool triggered_any = false;
    
    for (auto& alarm : alarms_) {
        if (!alarm.enabled) continue;
        
        if (alarm.hour == timeinfo.tm_hour && 
            alarm.minute == timeinfo.tm_min &&
            timeinfo.tm_sec == 0) {
            
            triggerAlarm(alarm);
            triggered_any = true;
            
            // DISABLE NGAY LAP TUC
            if (!alarm.repeated) {
                alarm.enabled = false;
                ESP_LOGI(TAG, "Disabled one-time alarm %02d:%02d", alarm.hour, alarm.minute);
            }
        }
    }
    
    // LUU VA XOA SAU KHI TRIGGER
    if (triggered_any) {
        saveToNVS();
        cleanupExpiredAlarms();
    }
}

void AlarmManager::triggerAlarm(const Alarm& alarm) {
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "ALARM TRIGGERED: %02d:%02d", alarm.hour, alarm.minute);
    ESP_LOGI(TAG, "========================================");
    
    // 1. Hiển thị
    auto& board = Board::GetInstance();
    auto display = board.GetDisplay();
    
    if (display) {
        char buffer[256];
        snprintf(buffer, sizeof(buffer), 
                 "BÁO THỨC\n%02d:%02d", alarm.hour, alarm.minute);
        display->SetChatMessage("system", buffer);
    }
    
    // 2. Phát âm thanh báo thức 5 lần
    auto& app = Application::GetInstance();
    
    app.Schedule([&app]() {
        // Tạo view cho file OGG
        std::string_view beep_sound(
            reinterpret_cast<const char*>(alarm_beep_ogg_start),
            alarm_beep_ogg_end - alarm_beep_ogg_start
        );
        
        // Phát 5 lần
        for (int i = 0; i < 15; i++) {
            ESP_LOGI(TAG, "Playing alarm beep %d/5", i + 1);
            app.PlaySound(beep_sound);
            vTaskDelay(pdMS_TO_TICKS(1000)); // Đợi 1 giây giữa các lần phát
        }
    });
    
    ESP_LOGI(TAG, "Alarm completed");
}

std::string AlarmManager::getNextAlarmInfo() {
    if (alarms_.empty()) {
        return "Khong co bao thuc";
    }
    
    time_t now;
    time(&now);
    struct tm timeinfo;
    localtime_r(&now, &timeinfo);
    
    int current_minutes = timeinfo.tm_hour * 60 + timeinfo.tm_min;
    int min_diff = 24 * 60;
    const Alarm* next_alarm = nullptr;
    
    for (const auto& alarm : alarms_) {
        if (!alarm.enabled) continue;
        
        int alarm_minutes = alarm.hour * 60 + alarm.minute;
        int diff = alarm_minutes - current_minutes;
        
        if (diff < 0) diff += 24 * 60;
        
        if (diff < min_diff) {
            min_diff = diff;
            next_alarm = &alarm;
        }
    }
    
    if (next_alarm) {
        char buffer[128];
        snprintf(buffer, sizeof(buffer), "%02d:%02d - %s",
                 next_alarm->hour, next_alarm->minute,
                 next_alarm->message.c_str());
        return std::string(buffer);
    }
    
    return "Khong co bao thuc";
}

std::string AlarmManager::getAllAlarmsInfo() {
    if (alarms_.empty()) {
        return "Không có báo thức nào";
    }
    
    time_t now;
    time(&now);
    struct tm timeinfo;
    localtime_r(&now, &timeinfo);
    
    int current_minutes = timeinfo.tm_hour * 60 + timeinfo.tm_min;
    
    // Tạo danh sách các alarm với thông tin thời gian còn lại
    std::vector<std::pair<int, const Alarm*>> sorted_alarms;
    
    for (const auto& alarm : alarms_) {
        if (!alarm.enabled) continue;
        
        int alarm_minutes = alarm.hour * 60 + alarm.minute;
        int diff = alarm_minutes - current_minutes;
        
        if (diff < 0) diff += 24 * 60;  // Nếu âm thì là ngày mai
        
        sorted_alarms.push_back({diff, &alarm});
    }
    
    if (sorted_alarms.empty()) {
        return "Không có báo thức nào đang hoạt động";
    }
    
    // Sắp xếp theo thời gian gần nhất
    std::sort(sorted_alarms.begin(), sorted_alarms.end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });
    
    // Tạo chuỗi kết quả
    std::string result = "Danh sách báo thức:\n";
    int count = 1;
    
    for (const auto& [diff, alarm] : sorted_alarms) {
        char buffer[256];
        int hours_left = diff / 60;
        int mins_left = diff % 60;
        
        snprintf(buffer, sizeof(buffer), "%d. %02d:%02d - %s%s (còn %dh%02dp)\n",
                 count++,
                 alarm->hour, 
                 alarm->minute,
                 alarm->message.c_str(),
                 alarm->repeated ? " [Hàng ngày]" : "",
                 hours_left,
                 mins_left);
        
        result += buffer;
    }
    
    return result;
}

void AlarmManager::clearAll() {
    alarms_.clear();
    saveToNVS();
    ESP_LOGI(TAG, "Cleared all alarms");
}