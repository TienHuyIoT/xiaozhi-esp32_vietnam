#include "ota.h"
#include "system_info.h"
#include "http_client.h"
#include "settings.h"
#include "assets/lang_config.h"

#include <cJSON.h>
#include <esp_log.h>
#include <esp_partition.h>
#include <esp_ota_ops.h>
#include <esp_app_format.h>
#include <esp_efuse.h>
#include <esp_efuse_table.h>
#ifdef SOC_HMAC_SUPPORTED
#include <esp_hmac.h>
#endif

#include <cstring>
#include <vector>
#include <sstream>
#include <algorithm>

#define TAG "Ota"

Ota::Ota() {
#ifdef ESP_EFUSE_BLOCK_USR_DATA
    // Read Serial Number from efuse user_data
    uint8_t serial_number[33] = {0};
    if (esp_efuse_read_field_blob(ESP_EFUSE_USER_DATA, serial_number, 32 * 8) == ESP_OK) {
        if (serial_number[0] == 0) {
            has_serial_number_ = false;
        } else {
            serial_number_ = std::string(reinterpret_cast<char*>(serial_number), 32);
            has_serial_number_ = true;
        }
    }
#endif
}

Ota::~Ota() {
}

std::string Ota::GetCheckVersionUrl() {
    Settings settings("wifi", true);
    std::string url = settings.GetString("ota_url");
    if (url.empty()) {
        url = CONFIG_OTA_URL;
        settings.SetString("ota_url", url);
    }
    return url;
}

std::string Ota::GetFirmwareChecksum() {
    Settings settings("firmware", true);
    std::string url = settings.GetString("checksum");
    if (url.empty()) {
        url = "ffffffffffffffffffffffffffffffff"; // Default invalid checksum
        settings.SetString("checksum", url);
    }
    return url;
}

std::unique_ptr<Http> Ota::SetupHttp() {
    auto& board = Board::GetInstance();
    auto network = board.GetNetwork();
    auto http = network->CreateHttp(0);
    auto user_agent = SystemInfo::GetUserAgent();
    http->SetHeader("Activation-Version", has_serial_number_ ? "2" : "1");
    http->SetHeader("Device-Id", SystemInfo::GetMacAddress().c_str());
    http->SetHeader("Client-Id", board.GetUuid());
    if (has_serial_number_) {
        http->SetHeader("Serial-Number", serial_number_.c_str());
        ESP_LOGI(TAG, "Setup HTTP, User-Agent: %s, Serial-Number: %s", user_agent.c_str(), serial_number_.c_str());
    }
    http->SetHeader("User-Agent", user_agent);
    http->SetHeader("Accept-Language", Lang::CODE);
    http->SetHeader("Content-Type", "application/json");

    return http;
}

/*
 * Specification: https://ccnphfhqs21z.feishu.cn/wiki/FjW6wZmisimNBBkov6OcmfvknVd

Header for check version request:
    Activation-Version  : 1
    Device-Id           : 90:70:69:19:9d:00
    Client-Id           : 15040d5c-6f08-4244-9062-b467dab3f290
    User-Agent          : xingzhi-cube-1.54tft-wifi/2.0.4
    Accept-Language     : vi-VN
    Content-Type        : application/json

Check version URL: https://api.tenclass.net/xiaozhi/ota/
Method: POST
Body for check version request data:

{
  "version": 2,
  "language": "vi-VN",
  "flash_size": 16777216,
  "minimum_free_heap_size": "8361304",
  "mac_address": "90:70:69:15:ef:e8",
  "uuid": "389263c8-5b18-4d9d-9769-57b679a0c4d5",
  "chip_model_name": "esp32s3",
  "chip_info": {
    "model": 9,
    "cores": 2,
    "revision": 2,
    "features": 18
  },
  "application": {
    "name": "xiaozhi_vn",
    "version": "2.0.5.06",
    "compile_time": "Jan 22 2026T15:19:24Z",
    "idf_version": "v5.5.1",
    "elf_sha256": "9648fda547881b88961455a756e4b63e5b988933d36628d9cb1d0562e1267024"
  },
  "partition_table": [
    {
      "label": "nvs",
      "type": 1,
      "subtype": 2,
      "address": 36864,
      "size": 16384
    },
    {
      "label": "otadata",
      "type": 1,
      "subtype": 0,
      "address": 53248,
      "size": 8192
    },
    {
      "label": "phy_init",
      "type": 1,
      "subtype": 1,
      "address": 61440,
      "size": 4096
    },
    {
      "label": "ota_0",
      "type": 0,
      "subtype": 16,
      "address": 131072,
      "size": 5242880
    },
    {
      "label": "ota_1",
      "type": 0,
      "subtype": 17,
      "address": 5373952,
      "size": 5242880
    },
    {
      "label": "assets",
      "type": 1,
      "subtype": 130,
      "address": 10616832,
      "size": 6160384
    }
  ],
  "ota": {
    "label": "ota_0"
  },
  "display": {
    "monochrome": false,
    "width": 284,
    "height": 240
  },
  "board": {
    "type": "xiaozhi-ai-iot-vietnam-1st",
    "name": "xiaozhi-ai-iot-vietnam-1st",
    "ssid": "Quyen_2.4G_T2",
    "rssi": -50,
    "channel": 12,
    "ip": "192.168.3.239",
    "mac": "90:70:69:15:ef:e8"
  }
}

https://api.tenclass.net/xiaozhi/ota/
Server response xiaozhi-ota response:
{
  "mqtt": {
    "endpoint": "mqtt.xiaozhi.me",
    "client_id": "GID_test@@@90_70_69_19_9d_00@@@ae4b2f2d-cab6-4887-9ad1-ea6e31226b63",
    "username": "eyJpcCI6IjExNi4xMDIuMTMxLjMwIn0=",
    "password": "T7oKmTPKxYcjd/L4BwxEYgTqw2KSP5eg+99SdMQZDvg=",
    "publish_topic": "device-server",
    "subscribe_topic": "null"
  },
  "websocket": {
    "url": "wss://api.tenclass.net/xiaozhi/v1/",
    "token": "test-token"
  },
  "server_time": {
    "timestamp": 1769784717976,
    "timezone_offset": 420
  },
  "firmware": {
    "version": "2.0.5.06",
    "url": ""
  }
}

https://xiaozhi-ai-iot.vn/api/v1/ota
xiaozhi-ai-iot-vietnam server response:
{
  "server_time": {
    "timestamp": 1769784718537,
    "timezone_offset": 420
  },
  "websocket": {
    "url": "wss://xiaozhi-ai-iot.vn/api/v1/ws",
    "token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJkYXRhIjoiNEpDMDJyVnRkUTFmUFdEWWJvZlpHR0N5cjZ0Y3BjLTNMLTFaYkdxX09QTjYzT3FBZXlEMkVtb0p5SE83b3dWSFdRaTRGNmNyU1cyQ2pEWGV6UUxFbUZ1dE9lM0xQZDZnU05XZS1OTElaOWRjZkRWdF9Fblp4dz09In0.nXMxdk4cpJK1iHZ-LWX0t4HxFZV9OPPDhkaJJQWeZ7A"
  },
  "mqtt_common": {
    "endpoint": "mqtt://xiaozhi-ai-iot.vn:1883",
    "username": "xiaozhi_device",
    "password": "XiaozhiDevice2026!",
    "subscribe_topic": "device/90:70:69:19:9d:00/#"
  },
  "audio_params": {
    "format": "opus",
    "sample_rate": 16000,
    "channels": 1,
    "frame_duration": 60,
    "buffer_size_ms": 500,
    "min_buffer_ms": 200,
    "max_buffer_ms": 2000
  }
}
 */
bool Ota::CheckVersion(std::string& url) {
    auto& board = Board::GetInstance();
    auto app_desc = esp_app_get_description();

    // Check if there is a new firmware version available
    current_version_ = app_desc->version;
    ESP_LOGI(TAG, "Current version: %s", current_version_.c_str());

    if (url.length() == 0) {
        url = GetCheckVersionUrl();
        if (url == CONFIG_OTA_URL) {
            ESP_LOGI(TAG, "Check version URL is using default: %s - canceling check", url.c_str());
            return true;
        }
    }

    if (url.length() < 10) {
        ESP_LOGE(TAG, "Check version URL is not properly set");
        return false;
    }

    auto http = SetupHttp();
    http->SetHeader("Device-Mac", SystemInfo::GetMacAddress().c_str());
    http->SetHeader("OTA-ForceUpdate", "0"); // 1 to force update, 0 otherwise

    std::string data = board.GetSystemInfoJson();
    ESP_LOGI(TAG, "Check version URL: %s", url.c_str());
    ESP_LOGI(TAG, "Check version request data: %s", data.c_str());
    std::string method = data.length() > 0 ? "POST" : "GET";
    http->SetContent(std::move(data));

    if (!http->Open(method, url)) {
        ESP_LOGE(TAG, "Failed to open HTTP connection");
        return false;
    }

    auto status_code = http->GetStatusCode();
    if (status_code != 200) {
        ESP_LOGE(TAG, "Failed to check version, status code: %d", status_code);
        return false;
    }

    data = http->ReadAll();
    http->Close();

    // Response: { "firmware": { "version": "1.0.0", "url": "http://", "force": 0 } }
    // Parse the JSON response and check if the version is newer
    // If it is, set has_new_version_ to true and store the new version and URL
    ESP_LOGI(TAG, "JSON response  %s", data.c_str());
    cJSON *root = cJSON_Parse(data.c_str());
    if (root == NULL) {
        ESP_LOGE(TAG, "Failed to parse JSON response");
        return false;
    }

    has_activation_code_ = false;
    has_activation_challenge_ = false;
    cJSON *activation = cJSON_GetObjectItem(root, "activation");
    if (cJSON_IsObject(activation)) {
        cJSON* message = cJSON_GetObjectItem(activation, "message");
        if (cJSON_IsString(message)) {
            activation_message_ = message->valuestring;
        }
        cJSON* code = cJSON_GetObjectItem(activation, "code");
        if (cJSON_IsString(code)) {
            activation_code_ = code->valuestring;
            has_activation_code_ = true;
        }
        cJSON* challenge = cJSON_GetObjectItem(activation, "challenge");
        if (cJSON_IsString(challenge)) {
            activation_challenge_ = challenge->valuestring;
            has_activation_challenge_ = true;
        }
        cJSON* timeout_ms = cJSON_GetObjectItem(activation, "timeout_ms");
        if (cJSON_IsNumber(timeout_ms)) {
            activation_timeout_ms_ = timeout_ms->valueint;
        }
        ESP_LOGI(TAG, "Activation message: %s", activation_message_.c_str());
    }

    has_mqtt_config_ = false;
    cJSON *mqtt = cJSON_GetObjectItem(root, "mqtt");
    if (cJSON_IsObject(mqtt)) {
        Settings settings("mqtt", true);
        cJSON *item = NULL;
        cJSON_ArrayForEach(item, mqtt) {
            if (cJSON_IsString(item)) {
                if (settings.GetString(item->string) != item->valuestring) {
                    settings.SetString(item->string, item->valuestring);
                }
            } else if (cJSON_IsNumber(item)) {
                if (settings.GetInt(item->string) != item->valueint) {
                    settings.SetInt(item->string, item->valueint);
                }
            }
        }
        has_mqtt_config_ = true;
    } else {
        ESP_LOGI(TAG, "No mqtt section found !");
    }

    has_websocket_config_ = false;
    cJSON *websocket = cJSON_GetObjectItem(root, "websocket");
    if (cJSON_IsObject(websocket)) {
        Settings settings("websocket", true);
        cJSON *item = NULL;
        cJSON_ArrayForEach(item, websocket) {
            if (cJSON_IsString(item)) {
                if (settings.GetString(item->string) != item->valuestring) {
                    settings.SetString(item->string, item->valuestring);
                }
            } else if (cJSON_IsNumber(item)) {
                if (settings.GetInt(item->string) != item->valueint) {
                    settings.SetInt(item->string, item->valueint);
                }
            }
        }
        has_websocket_config_ = true;
    } else {
        ESP_LOGI(TAG, "No websocket section found!");
    }

    has_server_time_ = false;
    cJSON *server_time = cJSON_GetObjectItem(root, "server_time");
    if (cJSON_IsObject(server_time)) {
        cJSON *timestamp = cJSON_GetObjectItem(server_time, "timestamp");
        cJSON *timezone_offset = cJSON_GetObjectItem(server_time, "timezone_offset");
        
        if (cJSON_IsNumber(timestamp)) {
            // Set system time
            struct timeval tv;
            double ts = timestamp->valuedouble;
            
            // If there is a timezone offset, calculate local time
            if (cJSON_IsNumber(timezone_offset)) {
                ts += (timezone_offset->valueint * 60 * 1000); // Convert minutes to milliseconds
            }
            
            tv.tv_sec = (time_t)(ts / 1000);  // Convert milliseconds to seconds
            tv.tv_usec = (suseconds_t)((long long)ts % 1000) * 1000;  // Convert remaining milliseconds to microseconds
            settimeofday(&tv, NULL);
            has_server_time_ = true;
        }
    } else {
        ESP_LOGW(TAG, "No server_time section found!");
    }

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
    has_new_version_ = false;
    cJSON *firmware = cJSON_GetObjectItem(root, "firmware");
    if (url != CONFIG_OTA_URL && cJSON_IsObject(firmware)) {
        do {
            cJSON *enable = cJSON_GetObjectItem(firmware, "enable");
            if (cJSON_IsNumber(enable) && enable->valueint == 0) {
                ESP_LOGI(TAG, "Firmware update is disabled by server");
                has_enabled_ = false;
                break;
            }

            cJSON *version = cJSON_GetObjectItem(firmware, "version");
            if (cJSON_IsString(version)) {
                firmware_version_ = version->valuestring;
            }
            cJSON *url = cJSON_GetObjectItem(firmware, "url");
            if (cJSON_IsString(url)) {
                firmware_url_ = url->valuestring;
            }
            cJSON *size = cJSON_GetObjectItem(firmware, "size");
            firmware_size_ = 0;
            if (cJSON_IsNumber(size)) {
                firmware_size_ = size->valueint;
                ESP_LOGI(TAG, "Firmware size from server: %d bytes", firmware_size_);
            }

            if (cJSON_IsString(version) && cJSON_IsString(url)) {
                // Check if the version is newer, for example, 0.1.0 is newer than 0.0.1
                has_new_version_ = IsNewVersionAvailable(current_version_, firmware_version_);
                if (has_new_version_) {
                    ESP_LOGI(TAG, "New version available: %s", firmware_version_.c_str());
                } else {
                    ESP_LOGI(TAG, "Current is the latest version");
                }
                // If the force flag is set to 1, the given version is forced to be installed
                cJSON *force = cJSON_GetObjectItem(firmware, "force");
                if (cJSON_IsNumber(force) && force->valueint == 1) {
                    has_new_version_ = true;
                    has_forced_ = true;
                    ESP_LOGI(TAG, "Firmware update is forced by server");
                }

                cJSON *checksum = cJSON_GetObjectItem(firmware, "checksum");
                if (cJSON_IsString(checksum)) {
                    checksum_ = checksum->valuestring;
                    ESP_LOGI(TAG, "Firmware checksum from server: %s", checksum_.c_str());

                    Settings settings("firmware", true);
                    std::string checksum = settings.GetString("checksum");
                    if (checksum != checksum_) {
                        settings.SetString("checksum", checksum_);
                        ESP_LOGI(TAG, "Updated firmware checksum in settings");
                    } else {
                        ESP_LOGI(TAG, "Firmware checksum in settings is up-to-date");
                        has_new_version_ = false;
                    }
                }

                if (firmware_url_.length() == 0) {
                    ESP_LOGE(TAG, "Firmware URL is empty");
                    has_new_version_ = false;
                }
            }

            cJSON *features = cJSON_GetObjectItem(firmware, "features");
            if (cJSON_IsObject(features)) {
                cJSON *music = cJSON_GetObjectItem(features, "music");
                if (cJSON_IsBool(music)) {
                    features_.music = cJSON_IsTrue(music);
                }
                cJSON *radio = cJSON_GetObjectItem(features, "radio");
                if (cJSON_IsBool(radio)) {
                    features_.radio = cJSON_IsTrue(radio);
                }
                cJSON *weather = cJSON_GetObjectItem(features, "weather");
                if (cJSON_IsBool(weather)) {
                    features_.weather = cJSON_IsTrue(weather);
                }
                cJSON *sdCardMusic = cJSON_GetObjectItem(features, "sdCardMusic");
                if (cJSON_IsBool(sdCardMusic)) {
                    features_.sdCardMusic = cJSON_IsTrue(sdCardMusic);
                }
                cJSON *alarm = cJSON_GetObjectItem(features, "alarm");
                if (cJSON_IsBool(alarm)) {
                    features_.alarm = cJSON_IsTrue(alarm);
                }
                cJSON *reminder = cJSON_GetObjectItem(features, "reminder");
                if (cJSON_IsBool(reminder)) {
                    features_.reminder = cJSON_IsTrue(reminder);
                }
                cJSON *voiceRecording = cJSON_GetObjectItem(features, "voiceRecording");
                if (cJSON_IsBool(voiceRecording)) {
                    features_.voiceRecording = cJSON_IsTrue(voiceRecording);
                }
                cJSON *bluetooth = cJSON_GetObjectItem(features, "bluetooth");
                if (cJSON_IsBool(bluetooth)) {
                    features_.bluetooth = cJSON_IsTrue(bluetooth);
                }
                cJSON *homeAssistant = cJSON_GetObjectItem(features, "homeAssistant");
                if (cJSON_IsBool(homeAssistant)) {
                    features_.homeAssistant = cJSON_IsTrue(homeAssistant);
                }

                ESP_LOGI(TAG, "Feature music %u, radio %u, weather %u, sdCardMusic %u, alarm %u, reminder %u, voiceRecording %u, bluetooth %u, homeAssistant %u",
                        features_.music, features_.radio, features_.weather, features_.sdCardMusic, features_.alarm, features_.reminder,
                        features_.voiceRecording, features_.bluetooth, features_.homeAssistant);
            }
        } while (0);

        // For testing purposes
        // firmware_version_ = "2.0.4";
        // firmware_url_ = "https://cdn.jsdelivr.net/gh/TienHuyIoT/esp_web_flasher@master/ota_bin/xingzhi-cube-1.54tft-wifi.bin";
        // has_new_version_ = true;

    } else {
        ESP_LOGW(TAG, "No firmware section found!");
    }

    cJSON_Delete(root);
    return true;
}

void Ota::MarkCurrentVersionValid() {
    auto partition = esp_ota_get_running_partition();
    if (strcmp(partition->label, "factory") == 0) {
        ESP_LOGI(TAG, "Running from factory partition, skipping");
        return;
    }

    ESP_LOGI(TAG, "Running partition: %s", partition->label);
    esp_ota_img_states_t state;
    if (esp_ota_get_state_partition(partition, &state) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get state of partition");
        return;
    }

    if (state == ESP_OTA_IMG_PENDING_VERIFY) {
        ESP_LOGI(TAG, "Marking firmware as valid");
        esp_ota_mark_app_valid_cancel_rollback();
    }
}

bool Ota::Upgrade(const std::string& firmware_url) {
    ESP_LOGI(TAG, "Upgrading firmware from %s", firmware_url.c_str());
    esp_ota_handle_t update_handle = 0;
    auto update_partition = esp_ota_get_next_update_partition(NULL);
    if (update_partition == NULL) {
        ESP_LOGE(TAG, "Failed to get update partition");
        return false;
    }

    ESP_LOGI(TAG, "Writing to partition %s at offset 0x%lx", update_partition->label, update_partition->address);
    bool image_header_checked = false;
    std::string image_header;

    auto network = Board::GetInstance().GetNetwork();
    auto http = network->CreateHttp(0);
    auto user_agent = SystemInfo::GetUserAgent();
    http->SetHeader("User-Agent", user_agent);
    http->SetHeader("Content-Type", "*/*");
    if (!http->Open("GET", firmware_url)) {
        ESP_LOGE(TAG, "Failed to open HTTP connection");
        return false;
    }

    if (http->GetStatusCode() != 200) {
        ESP_LOGE(TAG, "Failed to get firmware, status code: %d", http->GetStatusCode());
        return false;
    }

    size_t content_length = http->GetBodyLength();
    if (content_length == 0) {
        ESP_LOGE(TAG, "Failed to get content length");
        if (firmware_size_ > 0) {
            content_length = firmware_size_;
        } else {
            esp_partition_iterator_t it = esp_partition_find(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_ANY, NULL);
            const esp_partition_t *partition = esp_partition_get(it);
            content_length = partition->size;
        }
        // return false;
    }
    ESP_LOGI(TAG, "Firmware size: %u bytes", content_length);

    char buffer[512];
    size_t total_read = 0, recent_read = 0;
    auto last_calc_time = esp_timer_get_time();
    while (true) {
        int ret = http->Read(buffer, sizeof(buffer));
        if (ret < 0) {
            ESP_LOGE(TAG, "Failed to read HTTP data: %s", esp_err_to_name(ret));
            return false;
        }

        // Calculate speed and progress every second
        recent_read += ret;
        total_read += ret;
        if (esp_timer_get_time() - last_calc_time >= 1000000 || ret == 0) {
            size_t progress = total_read * 100 / content_length;
            ESP_LOGI(TAG, "Progress: %u%% (%u/%u), Speed: %uB/s", progress, total_read, content_length, recent_read);
            if (upgrade_callback_) {
                upgrade_callback_(progress, recent_read);
            }
            last_calc_time = esp_timer_get_time();
            recent_read = 0;
        }

        if (ret == 0) {
            break;
        }

        if (!image_header_checked) {
            image_header.append(buffer, ret);
            if (image_header.size() >= sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t) + sizeof(esp_app_desc_t)) {
                esp_app_desc_t new_app_info;
                memcpy(&new_app_info, image_header.data() + sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t), sizeof(esp_app_desc_t));
                
                auto current_version = esp_app_get_description()->version;
                ESP_LOGI(TAG, "Current version: %s, New version: %s", current_version, new_app_info.version);

                if (esp_ota_begin(update_partition, OTA_WITH_SEQUENTIAL_WRITES, &update_handle)) {
                    esp_ota_abort(update_handle);
                    ESP_LOGE(TAG, "Failed to begin OTA");
                    return false;
                }

                image_header_checked = true;
                std::string().swap(image_header);
            }
        }
        auto err = esp_ota_write(update_handle, buffer, ret);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to write OTA data: %s", esp_err_to_name(err));
            esp_ota_abort(update_handle);
            return false;
        }
    }
    http->Close();

    esp_err_t err = esp_ota_end(update_handle);
    if (err != ESP_OK) {
        if (err == ESP_ERR_OTA_VALIDATE_FAILED) {
            ESP_LOGE(TAG, "Image validation failed, image is corrupted");
        } else {
            ESP_LOGE(TAG, "Failed to end OTA: %s", esp_err_to_name(err));
        }
        return false;
    }

    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set boot partition: %s", esp_err_to_name(err));
        return false;
    }

    ESP_LOGI(TAG, "Firmware upgrade successful");
    return true;
}

bool Ota::StartUpgrade(std::function<void(int progress, size_t speed)> callback) {
    upgrade_callback_ = callback;
    return Upgrade(firmware_url_);
}

bool Ota::StartUpgradeFromUrl(const std::string& url, std::function<void(int progress, size_t speed)> callback) {
    upgrade_callback_ = callback;
    return Upgrade(url);
}

std::vector<int> Ota::ParseVersion(const std::string& version) {
    std::vector<int> versionNumbers;
    std::stringstream ss(version);
    std::string segment;
    
    while (std::getline(ss, segment, '.')) {
        versionNumbers.push_back(std::stoi(segment));
    }
    
    return versionNumbers;
}

bool Ota::IsNewVersionAvailable(const std::string& currentVersion, const std::string& newVersion) {
    std::vector<int> current = ParseVersion(currentVersion);
    std::vector<int> newer = ParseVersion(newVersion);
    
    for (size_t i = 0; i < std::min(current.size(), newer.size()); ++i) {
        if (newer[i] > current[i]) {
            return true;
        } else if (newer[i] < current[i]) {
            return false;
        }
    }
    
    return newer.size() > current.size();
}

std::string Ota::GetActivationPayload() {
    if (!has_serial_number_) {
        return "{}";
    }

    std::string hmac_hex;
#ifdef SOC_HMAC_SUPPORTED
    uint8_t hmac_result[32]; // SHA-256 The output is 32 bytes.
    
    // Calculate HMAC using Key0
    esp_err_t ret = esp_hmac_calculate(HMAC_KEY0, (uint8_t*)activation_challenge_.data(), activation_challenge_.size(), hmac_result);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "HMAC calculation failed: %s", esp_err_to_name(ret));
        return "{}";
    }

    for (size_t i = 0; i < sizeof(hmac_result); i++) {
        char buffer[3];
        sprintf(buffer, "%02x", hmac_result[i]);
        hmac_hex += buffer;
    }
#endif

    cJSON *payload = cJSON_CreateObject();
    cJSON_AddStringToObject(payload, "algorithm", "hmac-sha256");
    cJSON_AddStringToObject(payload, "serial_number", serial_number_.c_str());
    cJSON_AddStringToObject(payload, "challenge", activation_challenge_.c_str());
    cJSON_AddStringToObject(payload, "hmac", hmac_hex.c_str());
    auto json_str = cJSON_PrintUnformatted(payload);
    std::string json(json_str);
    cJSON_free(json_str);
    cJSON_Delete(payload);

    ESP_LOGI(TAG, "Activation payload: %s", json.c_str());
    return json;
}

esp_err_t Ota::Activate() {
    if (!has_activation_challenge_) {
        ESP_LOGW(TAG, "No activation challenge found");
        return ESP_FAIL;
    }

    std::string url = GetCheckVersionUrl();
    if (url.back() != '/') {
        url += "/activate";
    } else {
        url += "activate";
    }

    auto http = SetupHttp();

    std::string data = GetActivationPayload();
    http->SetContent(std::move(data));

    if (!http->Open("POST", url)) {
        ESP_LOGE(TAG, "Failed to open HTTP connection");
        return ESP_FAIL;
    }
    
    auto status_code = http->GetStatusCode();
    if (status_code == 202) {
        return ESP_ERR_TIMEOUT;
    }
    if (status_code != 200) {
        ESP_LOGE(TAG, "Failed to activate, code: %d, body: %s", status_code, http->ReadAll().c_str());
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Activation successful");
    return ESP_OK;
}
