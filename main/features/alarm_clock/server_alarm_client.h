#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include <atomic>
#include <memory>
#include <string>

class Mqtt;

class ServerAlarmClient {
public:
    ServerAlarmClient();
    ~ServerAlarmClient();

    ServerAlarmClient(const ServerAlarmClient&) = delete;
    ServerAlarmClient& operator=(const ServerAlarmClient&) = delete;

    bool Start();
    void Stop();
    bool IsConfigured() const { return configured_; }

private:
    struct Config {
        bool enabled = false;
        std::string endpoint;
        std::string client_id;
        std::string device_id;
        std::string username;
        std::string password;
        int keepalive = 60;
    };

    struct QueuedMessage {
        char* data = nullptr;
        size_t size = 0;
        bool is_payment = false;
    };

    static constexpr size_t kMaxPayloadBytes = 160 * 1024;
    static constexpr size_t kMaxAudioBytes = 96 * 1024;

    Config config_;
    bool configured_ = false;
    std::atomic_bool stopping_{false};
    std::unique_ptr<Mqtt> mqtt_;
    QueueHandle_t message_queue_ = nullptr;
    TaskHandle_t connection_task_ = nullptr;
    TaskHandle_t worker_task_ = nullptr;
    std::string command_topic_;
    std::string ack_topic_;
    std::string status_topic_;
    std::string payment_command_topic_;
    std::string last_event_id_;
    std::string current_event_id_;
    std::string current_payment_order_id_;
    std::string current_payment_title_;

    bool LoadConfig();
    void ConnectionLoop();
    void WorkerLoop();
    void OnMessage(const std::string& topic, const std::string& payload);
    void ProcessMessage(char* payload, size_t size);
    void ProcessPaymentMessage(char* payload, size_t size);
    void PublishAck(const std::string& event_id, const char* status,
                    const std::string& detail = "");
    void PublishStatus(const char* status);
    void PlayFallback();

    static void ConnectionTaskEntry(void* arg);
    static void WorkerTaskEntry(void* arg);
};
