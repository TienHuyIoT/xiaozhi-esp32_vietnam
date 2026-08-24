#include "server_alarm_client.h"

#include "alarm_sounds.h"
#include "application.h"
#include "board.h"
#include "display/lvgl_display/lvgl_display.h"
#include "features/why_questions/image_display.h"
#include "settings.h"
#include "system_info.h"

#include <cJSON.h>
#include <esp_heap_caps.h>
#include <esp_log.h>
#include <mbedtls/base64.h>
#include <mbedtls/sha256.h>

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <ctime>

namespace {

constexpr char kTag[] = "ServerAlarm";

// MQTT điều khiển robot độc lập với MQTT/OTA chính của XiaoZhi.
constexpr char kDefaultMqttEndpoint[] =
    "e001c1a2917941428e414cd3d41f0ef3.s1.eu.hivemq.cloud:8883";
constexpr char kDefaultMqttUsername[] = "halo_username";
constexpr char kDefaultMqttPassword[] = "halo1234";
constexpr char kDefaultPaymentTitle[] =
    "Ch\xE1\xBB\xA9" "c n\xC4\x83ng \xC4\x91\xC3\xA3 \xC4\x91\xC4\x83ng k\xC3\xBD";

bool IsJsonString(cJSON* item, size_t max_length) {
    return cJSON_IsString(item) && item->valuestring != nullptr &&
           std::strlen(item->valuestring) <= max_length;
}

std::string Sha256Hex(const uint8_t* data, size_t size) {
    uint8_t digest[32] = {};
    mbedtls_sha256_context context;
    mbedtls_sha256_init(&context);
    bool success = mbedtls_sha256_starts(&context, 0) == 0 &&
                   mbedtls_sha256_update(&context, data, size) == 0 &&
                   mbedtls_sha256_finish(&context, digest) == 0;
    mbedtls_sha256_free(&context);
    if (!success) {
        return {};
    }
    static constexpr char kHex[] = "0123456789abcdef";
    std::string result(64, '0');
    for (size_t i = 0; i < sizeof(digest); ++i) {
        result[i * 2] = kHex[digest[i] >> 4];
        result[i * 2 + 1] = kHex[digest[i] & 0x0f];
    }
    return result;
}

}  // namespace

ServerAlarmClient::ServerAlarmClient() = default;

ServerAlarmClient::~ServerAlarmClient() {
    Stop();
}

bool ServerAlarmClient::LoadConfig() {
    Settings settings("alarm_mqtt", false);
    config_.enabled = settings.GetBool("enabled", true);
    config_.endpoint = settings.GetString("endpoint", kDefaultMqttEndpoint);
    config_.client_id = settings.GetString("client_id");
    config_.device_id = settings.GetString("device_id", SystemInfo::GetMacAddress());
    config_.username = settings.GetString("username", kDefaultMqttUsername);
    config_.password = settings.GetString("password", kDefaultMqttPassword);
    config_.keepalive = settings.GetInt("keepalive", 60);
    last_event_id_ = settings.GetString("last_event_id");

    if (!config_.enabled) {
        ESP_LOGI(kTag, "Server alarm MQTT is disabled; local alarms remain active");
        return false;
    }
    if (config_.endpoint.empty() || config_.device_id.empty()) {
        ESP_LOGE(kTag, "Server alarm MQTT requires endpoint and device_id");
        return false;
    }
    if (config_.client_id.empty()) {
        config_.client_id = "alarm-" + Board::GetInstance().GetUuid();
    }
    if (config_.keepalive < 15 || config_.keepalive > 600) {
        config_.keepalive = 60;
    }
    if (config_.device_id.find_first_of("/+#") != std::string::npos) {
        ESP_LOGE(kTag, "Server alarm device_id contains MQTT topic wildcards");
        return false;
    }
    return true;
}

bool ServerAlarmClient::Start() {
    if (worker_task_ != nullptr || connection_task_ != nullptr) {
        return configured_;
    }
    configured_ = LoadConfig();
    if (!configured_) {
        return false;
    }

    command_topic_ = "robots/" + config_.device_id + "/alarm/v1/command";
    ack_topic_ = "robots/" + config_.device_id + "/alarm/v1/ack";
    status_topic_ = "robots/" + config_.device_id + "/alarm/v1/status";
    payment_command_topic_ = "robots/" + config_.device_id + "/payment/v1/command";
    stopping_.store(false);
    message_queue_ = xQueueCreate(2, sizeof(QueuedMessage));
    if (message_queue_ == nullptr) {
        ESP_LOGE(kTag, "Failed to create alarm message queue");
        configured_ = false;
        return false;
    }

    if (xTaskCreate(WorkerTaskEntry, "alarm_audio", 8 * 1024, this, 3,
                    &worker_task_) != pdPASS) {
        ESP_LOGE(kTag, "Failed to create alarm audio task");
        vQueueDelete(message_queue_);
        message_queue_ = nullptr;
        configured_ = false;
        return false;
    }
    if (xTaskCreate(ConnectionTaskEntry, "alarm_mqtt", 5 * 1024, this, 2,
                    &connection_task_) != pdPASS) {
        ESP_LOGE(kTag, "Failed to create alarm MQTT task");
        stopping_.store(true);
        return false;
    }
    return true;
}

void ServerAlarmClient::Stop() {
    if (!configured_ && message_queue_ == nullptr) {
        return;
    }
    stopping_.store(true);
    if (mqtt_) {
        mqtt_->Disconnect();
    }
    if (message_queue_) {
        QueuedMessage stop_message{};
        xQueueSend(message_queue_, &stop_message, 0);
    }

    for (int i = 0; i < 20 && (worker_task_ || connection_task_); ++i) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (worker_task_ == nullptr && message_queue_) {
        QueuedMessage pending{};
        while (xQueueReceive(message_queue_, &pending, 0) == pdTRUE) {
            heap_caps_free(pending.data);
        }
        vQueueDelete(message_queue_);
        message_queue_ = nullptr;
    }
    mqtt_.reset();
    configured_ = false;
}

void ServerAlarmClient::ConnectionTaskEntry(void* arg) {
    auto* client = static_cast<ServerAlarmClient*>(arg);
    client->ConnectionLoop();
    client->connection_task_ = nullptr;
    vTaskDelete(nullptr);
}

void ServerAlarmClient::WorkerTaskEntry(void* arg) {
    auto* client = static_cast<ServerAlarmClient*>(arg);
    client->WorkerLoop();
    client->worker_task_ = nullptr;
    vTaskDelete(nullptr);
}

void ServerAlarmClient::ConnectionLoop() {
    auto network = Board::GetInstance().GetNetwork();
    mqtt_ = network->CreateMqtt(4);
    if (!mqtt_) {
        ESP_LOGE(kTag, "Network does not provide a separate MQTT client");
        return;
    }
    mqtt_->SetKeepAlive(config_.keepalive);
    mqtt_->OnConnected([this]() {
        ESP_LOGI(kTag, "Connected; subscribing to alarm and payment commands");
        mqtt_->Subscribe(command_topic_, 1);
        mqtt_->Subscribe(payment_command_topic_, 1);
        PublishStatus("online");
    });
    mqtt_->OnDisconnected([]() {
        ESP_LOGW(kTag, "Alarm MQTT disconnected; esp-mqtt will reconnect");
    });
    mqtt_->OnError([](const std::string& error) {
        ESP_LOGW(kTag, "Alarm MQTT error: %s", error.c_str());
    });
    mqtt_->OnMessage([this](const std::string& topic, const std::string& payload) {
        OnMessage(topic, payload);
    });

    std::string host = config_.endpoint;
    int port = 8883;
    size_t separator = host.rfind(':');
    if (separator != std::string::npos && host.find(':') == separator) {
        const std::string port_text = host.substr(separator + 1);
        char* end = nullptr;
        long parsed_port = std::strtol(port_text.c_str(), &end, 10);
        if (end == port_text.c_str() || *end != '\0' ||
            parsed_port <= 0 || parsed_port > 65535) {
            ESP_LOGE(kTag, "Invalid alarm MQTT endpoint port");
            return;
        }
        port = static_cast<int>(parsed_port);
        host.resize(separator);
    }
    ESP_LOGI(kTag, "Connecting alarm MQTT to %s:%d", host.c_str(), port);
    if (!mqtt_->Connect(host, port, config_.client_id,
                        config_.username, config_.password)) {
        ESP_LOGW(kTag, "Initial alarm MQTT connect timed out; waiting for automatic reconnect");
    }
}

void ServerAlarmClient::OnMessage(const std::string& topic,
                                  const std::string& payload) {
    if (stopping_.load() || payload.empty() || payload.size() > kMaxPayloadBytes) {
        ESP_LOGW(kTag, "Dropping alarm payload with size %u",
                 static_cast<unsigned>(payload.size()));
        return;
    }
    // The shared esp-mqtt wrapper reports an empty topic on the final fragment.
    bool is_payment = topic == payment_command_topic_;
    if (!topic.empty() && topic != command_topic_ && !is_payment) {
        return;
    }
    auto* copy = static_cast<char*>(heap_caps_malloc(
        payload.size() + 1, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (!copy) {
        ESP_LOGE(kTag, "Not enough PSRAM for alarm command");
        return;
    }
    std::memcpy(copy, payload.data(), payload.size());
    copy[payload.size()] = '\0';
    QueuedMessage message{copy, payload.size(), is_payment};
    if (xQueueSend(message_queue_, &message, 0) != pdTRUE) {
        ESP_LOGW(kTag, "Alarm queue is full; broker retry will redeliver");
        heap_caps_free(copy);
    }
}

void ServerAlarmClient::WorkerLoop() {
    while (!stopping_.load()) {
        QueuedMessage message{};
        if (xQueueReceive(message_queue_, &message, pdMS_TO_TICKS(1000)) != pdTRUE) {
            continue;
        }
        if (message.data == nullptr) {
            break;
        }
        if (message.is_payment) {
            ProcessPaymentMessage(message.data, message.size);
        } else {
            ProcessMessage(message.data, message.size);
        }
        heap_caps_free(message.data);
    }
}

void ServerAlarmClient::ProcessPaymentMessage(char* payload, size_t size) {
    cJSON* root = cJSON_ParseWithLength(payload, size);
    if (!root) {
        ESP_LOGW(kTag, "Invalid payment JSON");
        return;
    }
    cJSON* version = cJSON_GetObjectItem(root, "v");
    cJSON* order = cJSON_GetObjectItem(root, "order_id");
    cJSON* status = cJSON_GetObjectItem(root, "status");
    if (!cJSON_IsNumber(version) || version->valueint != 1 ||
        !IsJsonString(order, 64) || !IsJsonString(status, 16)) {
        ESP_LOGW(kTag, "Payment command metadata is invalid");
        cJSON_Delete(root);
        return;
    }

    if (std::strcmp(status->valuestring, "pending") == 0) {
        cJSON* image_url = cJSON_GetObjectItem(root, "image_url");
        cJSON* expires = cJSON_GetObjectItem(root, "expires_at_epoch");
        cJSON* product_title = cJSON_GetObjectItem(root, "product_title");
        bool valid_url = IsJsonString(image_url, 1024) &&
            (std::strncmp(image_url->valuestring, "https://", 8) == 0 ||
             std::strncmp(image_url->valuestring, "http://", 7) == 0);
        if (!valid_url) {
            ESP_LOGW(kTag, "Payment QR URL is invalid (length=%u)",
                     image_url && image_url->valuestring
                         ? static_cast<unsigned>(std::strlen(image_url->valuestring)) : 0U);
            cJSON_Delete(root);
            return;
        }
        if (!cJSON_IsNumber(expires)) {
            ESP_LOGW(kTag, "Payment QR command has no expiration timestamp");
            cJSON_Delete(root);
            return;
        }
        // The server enforces both the unguessable token and expiry when the JPEG
        // is fetched. Do not reject here: the device wall clock can be unsynchronised
        // even while the displayed hour looks correct.
        time_t now = std::time(nullptr);
        if (now > 1700000000 && now > static_cast<time_t>(expires->valuedouble)) {
            ESP_LOGW(kTag, "Payment QR appears expired by local clock; server will verify it");
        }
        current_payment_order_id_ = order->valuestring;
        current_payment_title_ = IsJsonString(product_title, 128)
            ? product_title->valuestring : kDefaultPaymentTitle;
        auto* display = dynamic_cast<LvglDisplay*>(Board::GetInstance().GetDisplay());
        if (display) {
            StartPaymentQrDisplayTask(display, image_url->valuestring,
                                      current_payment_title_, 5);
        } else {
            ESP_LOGW(kTag, "Payment QR requires an LVGL display");
        }
    } else if (std::strcmp(status->valuestring, "paid") == 0) {
        if (!current_payment_order_id_.empty() &&
            current_payment_order_id_ != order->valuestring) {
            ESP_LOGW(kTag, "Ignoring paid status for inactive order %s",
                     order->valuestring);
            cJSON_Delete(root);
            return;
        }
        std::string product_title = current_payment_title_.empty()
            ? kDefaultPaymentTitle : current_payment_title_;
        current_payment_order_id_.clear();
        current_payment_title_.clear();
        Application::GetInstance().Schedule([product_title]() {
            auto* display = dynamic_cast<LvglDisplay*>(Board::GetInstance().GetDisplay());
            if (display) {
                StartPaymentSuccessDisplayTask(display, product_title, 5000);
            }
        });
    } else {
        ESP_LOGW(kTag, "Ignoring unknown payment status");
    }
    cJSON_Delete(root);
}

void ServerAlarmClient::ProcessMessage(char* payload, size_t size) {
    cJSON* root = cJSON_ParseWithLength(payload, size);
    if (!root) {
        ESP_LOGW(kTag, "Invalid alarm JSON");
        return;
    }

    cJSON* version = cJSON_GetObjectItem(root, "v");
    cJSON* event = cJSON_GetObjectItem(root, "event_id");
    cJSON* message = cJSON_GetObjectItem(root, "message");
    cJSON* expires = cJSON_GetObjectItem(root, "expires_at_epoch");
    cJSON* audio = cJSON_GetObjectItem(root, "audio");
    if (!cJSON_IsNumber(version) || version->valueint != 1 ||
        !IsJsonString(event, 64) || !IsJsonString(message, 768) ||
        !cJSON_IsNumber(expires) || !cJSON_IsObject(audio)) {
        ESP_LOGW(kTag, "Alarm command metadata is invalid");
        cJSON_Delete(root);
        return;
    }
    std::string event_id = event->valuestring;
    if (event_id == last_event_id_ || event_id == current_event_id_) {
        PublishAck(event_id, "received", "duplicate");
        cJSON_Delete(root);
        return;
    }
    time_t now = std::time(nullptr);
    if (now > 1700000000 && static_cast<int64_t>(now) >
            static_cast<int64_t>(expires->valuedouble)) {
        PublishAck(event_id, "failed", "expired");
        cJSON_Delete(root);
        return;
    }

    cJSON* transport = cJSON_GetObjectItem(audio, "transport");
    cJSON* codec = cJSON_GetObjectItem(audio, "codec");
    cJSON* sample_rate = cJSON_GetObjectItem(audio, "sample_rate");
    cJSON* byte_size = cJSON_GetObjectItem(audio, "byte_size");
    cJSON* sha256 = cJSON_GetObjectItem(audio, "sha256");
    cJSON* data = cJSON_GetObjectItem(audio, "data");
    if (!IsJsonString(transport, 32) || std::strcmp(transport->valuestring, "inline_base64") != 0 ||
        !IsJsonString(codec, 32) || std::strcmp(codec->valuestring, "ogg_opus") != 0 ||
        !cJSON_IsNumber(sample_rate) || sample_rate->valueint != 24000 ||
        !cJSON_IsNumber(byte_size) || byte_size->valuedouble <= 0 ||
        byte_size->valuedouble > kMaxAudioBytes ||
        !IsJsonString(sha256, 64) || std::strlen(sha256->valuestring) != 64 ||
        !IsJsonString(data, 4 * ((kMaxAudioBytes + 2) / 3) + 4)) {
        PublishAck(event_id, "failed", "invalid_audio_metadata");
        cJSON_Delete(root);
        return;
    }

    current_event_id_ = event_id;
    size_t expected_size = static_cast<size_t>(byte_size->valuedouble);
    auto* decoded = static_cast<uint8_t*>(heap_caps_malloc(
        expected_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    size_t decoded_size = 0;
    int decode_result = decoded ? mbedtls_base64_decode(
        decoded, expected_size, &decoded_size,
        reinterpret_cast<const unsigned char*>(data->valuestring),
        std::strlen(data->valuestring)) : MBEDTLS_ERR_BASE64_BUFFER_TOO_SMALL;

    bool valid = decode_result == 0 && decoded_size == expected_size &&
                 decoded_size >= 4 && std::memcmp(decoded, "OggS", 4) == 0 &&
                 Sha256Hex(decoded, decoded_size) == sha256->valuestring;
    if (!valid) {
        ESP_LOGE(kTag, "Alarm audio validation failed");
        heap_caps_free(decoded);
        current_event_id_.clear();
        PublishAck(event_id, "failed", "audio_validation_failed");
        PlayFallback();
        cJSON_Delete(root);
        return;
    }

    std::string display_message = message->valuestring;
    auto& app = Application::GetInstance();
    app.Schedule([display_message]() {
        Application::GetInstance().PrepareForServerAlarm(display_message);
    });
    vTaskDelay(pdMS_TO_TICKS(100));
    app.PlaySound(std::string_view(reinterpret_cast<const char*>(decoded), decoded_size));
    heap_caps_free(decoded);

    last_event_id_ = event_id;
    current_event_id_.clear();
    {
        Settings settings("alarm_mqtt", true);
        settings.SetString("last_event_id", last_event_id_);
    }
    PublishAck(event_id, "started");
    cJSON_Delete(root);
}

void ServerAlarmClient::PublishAck(const std::string& event_id, const char* status,
                                   const std::string& detail) {
    if (!mqtt_ || !mqtt_->IsConnected()) {
        return;
    }
    cJSON* root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "v", 1);
    cJSON_AddStringToObject(root, "event_id", event_id.c_str());
    cJSON_AddStringToObject(root, "status", status);
    if (!detail.empty()) {
        cJSON_AddStringToObject(root, "detail", detail.c_str());
    }
    char* json = cJSON_PrintUnformatted(root);
    if (json) {
        mqtt_->Publish(ack_topic_, json, 1);
        cJSON_free(json);
    }
    cJSON_Delete(root);
}

void ServerAlarmClient::PublishStatus(const char* status) {
    if (!mqtt_ || !mqtt_->IsConnected()) {
        return;
    }
    cJSON* root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "v", 1);
    cJSON_AddStringToObject(root, "device_id", config_.device_id.c_str());
    cJSON_AddStringToObject(root, "status", status);
    char* json = cJSON_PrintUnformatted(root);
    if (json) {
        mqtt_->Publish(status_topic_, json, 1);
        cJSON_free(json);
    }
    cJSON_Delete(root);
}

void ServerAlarmClient::PlayFallback() {
    Application::GetInstance().Schedule([]() {
        auto& app = Application::GetInstance();
        app.PrepareForServerAlarm("Báo thức");
        app.PlaySound(GetAlarmSoundOgg("alarm"));
    });
}
