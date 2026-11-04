
#ifndef WEBSOCKET_CONTROL_SERVER_H
#define WEBSOCKET_CONTROL_SERVER_H

#include <esp_http_server.h>
#include <cJSON.h>
#include <string>
#include <map>

class WebSocketControlServer {
public:
    WebSocketControlServer();
    ~WebSocketControlServer();

    bool Start(int port = 8080);

    void Stop();

    size_t GetClientCount() const;

private:
    httpd_handle_t server_handle_;

    std::map<int, httpd_req_t*> clients_;

    // WebSocket handler
    static esp_err_t ws_handler(httpd_req_t* req);

    // GPIO configuration web handlers
    static esp_err_t gpio_page_handler(httpd_req_t* req);
    static esp_err_t gpio_get_handler(httpd_req_t* req);
    static esp_err_t gpio_post_handler(httpd_req_t* req);

    // WebSocket message handling
    void HandleMessage(
        httpd_req_t* req,
        const char* data,
        size_t len
    );

    void AddClient(httpd_req_t* req);
    void RemoveClient(httpd_req_t* req);

    static WebSocketControlServer* instance_;
};

#endif // WEBSOCKET_CONTROL_SERVER_H
