#include "edge_tts_client.h"
#include <esp_log.h>
#include <esp_random.h>
#include <cstring>
#include <sstream>
#include <iomanip>
#include "board.h"

#define TAG "EdgeTts"

EdgeTtsClient::EdgeTtsClient() {
    event_group_ = xEventGroupCreate();
    xTaskCreate([](void* arg) {
        auto client = static_cast<EdgeTtsClient*>(arg);
        client->TaskRoutine();
    }, "edge_tts_task", 1024 * 6, this, 5, &task_handle_);
}

EdgeTtsClient::~EdgeTtsClient() {
    running_ = false;
    Stop();
    xEventGroupSetBits(event_group_, 1); // Wake up to exit
    if (task_handle_) {
        vTaskDelay(pdMS_TO_TICKS(50)); // Give it a moment to exit
    }
    if (event_group_) {
        vEventGroupDelete(event_group_);
    }
}

void EdgeTtsClient::Start(const std::string& text, const std::string& voice) {
    Stop();
    pending_text_ = text;
    pending_voice_ = voice;
    is_speaking_ = true;
    xEventGroupSetBits(event_group_, 1); // WAKE_BIT
}

void EdgeTtsClient::Stop() {
    is_speaking_ = false;
    if (websocket_) {
        websocket_->Close();
    }
}

std::string EdgeTtsClient::generate_request_id() {
    char buf[33];
    for (int i = 0; i < 32; ++i) {
        sprintf(&buf[i], "%01lx", (unsigned long)(esp_random() % 16));
    }
    buf[32] = 0;
    return std::string(buf);
}

std::string EdgeTtsClient::get_date() {
    // Just return a dummy date, Edge TTS usually doesn't strictly check the exact time for this
    return "Thu Oct 24 2024 13:42:04 GMT+0700 (Indochina Time)";
}

void EdgeTtsClient::send_speech_config() {
    std::string config = 
        "X-Timestamp:" + get_date() + "\r\n"
        "Content-Type:application/json; charset=utf-8\r\n"
        "Path:speech.config\r\n\r\n"
        "{\"context\":{\"synthesis\":{\"audio\":{\"metadataOptions\":{\"sentenceBoundaryEnabled\":false,\"wordBoundaryEnabled\":false},\"outputFormat\":\"raw-16khz-16bit-mono-pcm\"}}}}";
    websocket_->Send(config);
}

void EdgeTtsClient::send_ssml(const std::string& request_id, const std::string& text, const std::string& voice) {
    std::string escaped_text = text;
    // VERY basic escaping for XML
    size_t pos = 0;
    while ((pos = escaped_text.find("&", pos)) != std::string::npos) { escaped_text.replace(pos, 1, "&amp;"); pos += 5; }
    pos = 0;
    while ((pos = escaped_text.find("<", pos)) != std::string::npos) { escaped_text.replace(pos, 1, "&lt;"); pos += 4; }
    pos = 0;
    while ((pos = escaped_text.find(">", pos)) != std::string::npos) { escaped_text.replace(pos, 1, "&gt;"); pos += 4; }

    std::string ssml = 
        "<ssml version='1.0' xmlns='http://www.w3.org/2001/10/synthesis' xmlns:mstts='https://www.w3.org/2001/mstts' xml:lang='vi-VN'>"
        "<voice name='" + voice + "'>"
        "<prosody rate='+0%'>" + escaped_text + "</prosody>"
        "</voice></ssml>";

    std::string req = 
        "X-RequestId:" + request_id + "\r\n"
        "Content-Type:application/ssml+xml\r\n"
        "X-Timestamp:" + get_date() + "\r\n"
        "Path:ssml\r\n\r\n" + ssml;
    websocket_->Send(req);
}

void EdgeTtsClient::TaskRoutine() {
    while (running_) {
        // Wait for start signal
        xEventGroupWaitBits(event_group_, 1, pdTRUE, pdFALSE, portMAX_DELAY);
        if (!running_) break;
        if (!is_speaking_) continue;

        auto network = Board::GetInstance().GetNetwork();
        websocket_ = network->CreateWebSocket(2); // ID 2 for Edge TTS
        
        websocket_->SetReceiveBufferSize(8192);
        websocket_->SetHeader("Origin", "chrome-extension://jdiccldimpdaibmpdkjnbmckianbfold");
        websocket_->SetHeader("User-Agent", "Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/114.0.0.0 Safari/537.36 Edg/114.0.1823.51");

        bool connection_ok = false;
        
        websocket_->OnConnected([this, &connection_ok]() {
            ESP_LOGI(TAG, "Edge TTS Connected");
            connection_ok = true;
            if (on_connected_) on_connected_();
        });

        websocket_->OnDisconnected([this]() {
            ESP_LOGI(TAG, "Edge TTS Disconnected");
        });

        websocket_->OnError([this](int err) {
            ESP_LOGE(TAG, "Edge TTS Error: %d", err);
            if (on_error_) on_error_("Connection Error");
        });

        websocket_->OnData([this](const char* data, size_t len, bool binary) {
            if (!is_speaking_) return;
            if (!binary) {
                std::string msg(data, len);
                if (msg.find("Path:turn.end") != std::string::npos) {
                    if (on_completed_) on_completed_();
                    is_speaking_ = false;
                }
            } else {
                if (len > 2) {
                    uint16_t header_len = (data[0] << 8) | data[1];
                    if (len > 2 + header_len) {
                        size_t audio_len = len - 2 - header_len;
                        const char* audio_data = data + 2 + header_len;
                        
                        if (audio_len % 2 == 0) {
                            std::vector<int16_t> pcm(audio_len / 2);
                            memcpy(pcm.data(), audio_data, audio_len);
                            if (on_audio_data_) {
                                on_audio_data_(pcm, 16000);
                            }
                        }
                    }
                }
            }
        });

        std::string url = "wss://speech.platform.bing.com/consumer/speech/synthesize/readaloud/edge/v1?TrustedClientToken=6A5AA1D4EAFF4E9FB37E23D68491D6F4";
        if (websocket_->Connect(url.c_str())) {
            if (connection_ok || websocket_->IsConnected()) {
                std::string req_id = generate_request_id();
                send_speech_config();
                send_ssml(req_id, pending_text_, pending_voice_);
                
                while (is_speaking_ && running_ && websocket_->IsConnected()) {
                    vTaskDelay(pdMS_TO_TICKS(100));
                }
            }
        } else {
            ESP_LOGE(TAG, "Failed to connect to Edge TTS");
            if (on_error_) on_error_("Connect Failed");
        }
        
        if (websocket_) {
            websocket_->Close();
            websocket_.reset();
        }
        is_speaking_ = false;
    }
    
    task_handle_ = nullptr;
    vTaskDelete(NULL);
}
