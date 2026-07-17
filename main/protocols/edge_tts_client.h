#ifndef EDGE_TTS_CLIENT_H
#define EDGE_TTS_CLIENT_H

#include <string>
#include <memory>
#include <functional>
#include <vector>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>

#include "web_socket.h"

class EdgeTtsClient {
public:
    EdgeTtsClient();
    ~EdgeTtsClient();

    void Start(const std::string& text, const std::string& voice = "vi-VN-HoaiMyNeural");
    void Stop();

    void OnConnected(std::function<void()> callback) { on_connected_ = callback; }
    void OnError(std::function<void(const std::string&)> callback) { on_error_ = callback; }
    void OnAudioData(std::function<void(const std::vector<int16_t>&, int)> callback) { on_audio_data_ = callback; }
    void OnCompleted(std::function<void()> callback) { on_completed_ = callback; }

private:
    std::unique_ptr<WebSocket> websocket_;
    std::function<void()> on_connected_;
    std::function<void(const std::string&)> on_error_;
    std::function<void(const std::vector<int16_t>&, int)> on_audio_data_;
    std::function<void()> on_completed_;
    
    std::string generate_request_id();
    std::string get_date();
    void send_speech_config();
    void send_ssml(const std::string& request_id, const std::string& text, const std::string& voice);
    
    bool is_speaking_ = false;
    bool running_ = true;
    TaskHandle_t task_handle_ = nullptr;
    EventGroupHandle_t event_group_;
    std::string pending_text_;
    std::string pending_voice_;

    void TaskRoutine();
};

#endif // EDGE_TTS_CLIENT_H
