
#include "websocket_control_server.h"
#include "mcp_server.h"
#include "otto_gpio_config.h"

#include <esp_log.h>
#include <esp_http_server.h>
#include <esp_system.h>
#include <sys/param.h>
#include <cstring>
#include <cstdlib>
#include <map>
#include <cJSON.h>

static const char* TAG = "WSControl";

WebSocketControlServer* WebSocketControlServer::instance_ = nullptr;

// ============================================================
// GPIO CONFIG HELPERS
// ============================================================

static void AddJsonInt(cJSON* root, const char* key, int value) {
    cJSON_AddNumberToObject(root, key, value);
}

static bool ReadJsonInt(cJSON* root, const char* key, int& value) {
    cJSON* item = cJSON_GetObjectItemCaseSensitive(root, key);

    if (!cJSON_IsNumber(item)) {
        return false;
    }

    value = item->valueint;
    return true;
}

static cJSON* GpioConfigToJson(const OttoGpioConfig& c) {
    cJSON* root = cJSON_CreateObject();

    if (!root) {
        return nullptr;
    }

    AddJsonInt(root, "mic_ws", c.mic_ws);
    AddJsonInt(root, "mic_sck", c.mic_sck);
    AddJsonInt(root, "mic_din", c.mic_din);

    AddJsonInt(root, "spk_dout", c.spk_dout);
    AddJsonInt(root, "spk_bclk", c.spk_bclk);
    AddJsonInt(root, "spk_lrck", c.spk_lrck);

    AddJsonInt(root, "display_mosi", c.display_mosi);
    AddJsonInt(root, "display_clk", c.display_clk);
    AddJsonInt(root, "display_dc", c.display_dc);
    AddJsonInt(root, "display_rst", c.display_rst);
    AddJsonInt(root, "display_cs", c.display_cs);
    AddJsonInt(root, "display_backlight", c.display_backlight);

    AddJsonInt(root, "servo_pan", c.servo_pan);
    AddJsonInt(root, "servo_tilt", c.servo_tilt);
    AddJsonInt(root, "pir", c.pir);
    AddJsonInt(root, "dht11", c.dht11);
    AddJsonInt(root, "lcd_sda", c.lcd_sda);
    AddJsonInt(root, "lcd_scl", c.lcd_scl);

    return root;
}

// ============================================================
// GPIO CONFIG API: GET /api/gpio
// ============================================================

static esp_err_t gpio_get_handler(httpd_req_t* req) {
    OttoGpioConfig config = OttoGpioConfigManager::Load();

    cJSON* root = GpioConfigToJson(config);

    if (!root) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                            "JSON allocation failed");
        return ESP_FAIL;
    }

    char* json = cJSON_PrintUnformatted(root);

    cJSON_Delete(root);

    if (!json) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                            "JSON serialization failed");
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "application/json");
    esp_err_t ret = httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);

    free(json);
    return ret;
}

// ============================================================
// GPIO CONFIG API: POST /api/gpio
// ============================================================

static esp_err_t gpio_post_handler(httpd_req_t* req) {
    const size_t MAX_BODY_SIZE = 2048;

    if (req->content_len <= 0 ||
        req->content_len > MAX_BODY_SIZE) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                            "Invalid request size");
        return ESP_FAIL;
    }

    size_t body_len = req->content_len;

    char* body = (char*)calloc(1, body_len + 1);

    if (!body) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                            "Out of memory");
        return ESP_ERR_NO_MEM;
    }

    size_t received = 0;

    while (received < body_len) {
        int ret = httpd_req_recv(
            req,
            body + received,
            body_len - received
        );

        if (ret <= 0) {
            free(body);
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                "Failed to receive request");
            return ESP_FAIL;
        }

        received += ret;
    }

    cJSON* root = cJSON_Parse(body);
    free(body);

    if (!root || !cJSON_IsObject(root)) {
        cJSON_Delete(root);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                            "Invalid JSON");
        return ESP_FAIL;
    }

    OttoGpioConfig c = {};

    bool valid =
        ReadJsonInt(root, "mic_ws", c.mic_ws) &&
        ReadJsonInt(root, "mic_sck", c.mic_sck) &&
        ReadJsonInt(root, "mic_din", c.mic_din) &&

        ReadJsonInt(root, "spk_dout", c.spk_dout) &&
        ReadJsonInt(root, "spk_bclk", c.spk_bclk) &&
        ReadJsonInt(root, "spk_lrck", c.spk_lrck) &&

        ReadJsonInt(root, "display_mosi", c.display_mosi) &&
        ReadJsonInt(root, "display_clk", c.display_clk) &&
        ReadJsonInt(root, "display_dc", c.display_dc) &&
        ReadJsonInt(root, "display_rst", c.display_rst) &&
        ReadJsonInt(root, "display_cs", c.display_cs) &&
        ReadJsonInt(root, "display_backlight", c.display_backlight) &&

        ReadJsonInt(root, "servo_pan", c.servo_pan) &&
        ReadJsonInt(root, "servo_tilt", c.servo_tilt) &&
        ReadJsonInt(root, "pir", c.pir) &&
        ReadJsonInt(root, "dht11", c.dht11) &&
        ReadJsonInt(root, "lcd_sda", c.lcd_sda) &&
        ReadJsonInt(root, "lcd_scl", c.lcd_scl);

    cJSON_Delete(root);

    if (!valid) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                            "Missing or invalid GPIO fields");
        return ESP_FAIL;
    }

    if (!OttoGpioConfigManager::Validate(c)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                            "GPIO validation failed. Check range and duplicates.");
        return ESP_FAIL;
    }

    if (!OttoGpioConfigManager::Save(c)) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                            "Failed to save GPIO configuration");
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "application/json");

    return httpd_resp_sendstr(
        req,
        "{\"success\":true,\"message\":\"GPIO saved. Please restart the robot.\"}"
    );
}

// ============================================================
// GPIO CONFIG WEB PAGE: GET /gpio
// ============================================================

static esp_err_t gpio_page_handler(httpd_req_t* req) {
    static const char html[] = R"HTML(
<!DOCTYPE html>
<html lang="vi">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Otto Robot GPIO</title>
<style>
*{box-sizing:border-box}
body{
    margin:0;
    padding:16px;
    background:#101827;
    color:#edf2f7;
    font-family:Arial,sans-serif
}
.container{max-width:620px;margin:auto}
h1{font-size:24px;margin-bottom:8px}
.description{color:#aab6c5;font-size:14px;line-height:1.5}
.group{
    background:#1c293b;
    padding:16px;
    border-radius:12px;
    margin:16px 0
}
h2{font-size:17px;margin:0 0 14px}
.field{
    display:flex;
    align-items:center;
    justify-content:space-between;
    gap:12px;
    margin:12px 0
}
.field label{font-size:14px;flex:1}
.field input{
    width:105px;
    padding:10px;
    border:1px solid #45566e;
    border-radius:7px;
    background:#101827;
    color:white;
    font-size:16px
}
.note{font-size:12px;color:#aab6c5}
button{
    width:100%;
    padding:15px;
    border:0;
    border-radius:9px;
    background:#2878e5;
    color:white;
    font-size:16px;
    font-weight:bold;
    margin-top:12px
}
button:disabled{opacity:.5}
#status{
    margin:14px 0;
    padding:12px;
    border-radius:8px;
    background:#26364c;
    white-space:pre-wrap
}
</style>
</head>
<body>
<div class="container">
<h1>Otto Robot - GPIO</h1>
<p class="description">
Cấu hình chân GPIO cho các thiết bị.
Nhập -1 nếu thiết bị chưa được kết nối.
Sau khi lưu, cần khởi động lại robot để áp dụng.
</p>

<div class="group">
<h2>Mic INMP441</h2>
<div class="field"><label>MIC WS</label><input id="mic_ws" type="number"></div>
<div class="field"><label>MIC SCK</label><input id="mic_sck" type="number"></div>
<div class="field"><label>MIC DIN</label><input id="mic_din" type="number"></div>
</div>

<div class="group">
<h2>Loa MAX98357A</h2>
<div class="field"><label>Speaker DOUT</label><input id="spk_dout" type="number"></div>
<div class="field"><label>Speaker BCLK</label><input id="spk_bclk" type="number"></div>
<div class="field"><label>Speaker LRCK</label><input id="spk_lrck" type="number"></div>
</div>

<div class="group">
<h2>Màn hình ST7789</h2>
<div class="field"><label>Display MOSI</label><input id="display_mosi" type="number"></div>
<div class="field"><label>Display CLK</label><input id="display_clk" type="number"></div>
<div class="field"><label>Display DC</label><input id="display_dc" type="number"></div>
<div class="field"><label>Display RST</label><input id="display_rst" type="number"></div>
<div class="field"><label>Display CS</label><input id="display_cs" type="number"></div>
<div class="field"><label>Backlight</label><input id="display_backlight" type="number"></div>
</div>

<div class="group">
<h2>Servo Pan / Tilt</h2>
<div class="field"><label>Servo PAN</label><input id="servo_pan" type="number"></div>
<div class="field"><label>Servo TILT</label><input id="servo_tilt" type="number"></div>
</div>

<div class="group">
<h2>Cảm biến</h2>
<div class="field"><label>PIR HC-SR501</label><input id="pir" type="number"></div>
<div class="field"><label>DHT11</label><input id="dht11" type="number"></div>
</div>

<div class="group">
<h2>LCD1602 I2C</h2>
<div class="field"><label>LCD SDA</label><input id="lcd_sda" type="number"></div>
<div class="field"><label>LCD SCL</label><input id="lcd_scl" type="number"></div>
</div>

<p class="note">
GPIO hợp lệ theo kiểm tra hiện tại: 0 đến 48 hoặc -1
cho thiết bị chưa dùng. Không nhập trùng GPIO.
Hãy kiểm tra khả năng sử dụng của từng chân trên bo mạch
trước khi lưu.
</p>

<button id="load" onclick="loadConfig()">Đọc cấu hình hiện tại</button>
<button id="save" onclick="saveConfig()">Lưu cấu hình GPIO</button>

<div id="status">Đang tải cấu hình...</div>
</div>

<script>
const fields = [
    "mic_ws","mic_sck","mic_din",
    "spk_dout","spk_bclk","spk_lrck",
    "display_mosi","display_clk","display_dc",
    "display_rst","display_cs","display_backlight",
    "servo_pan","servo_tilt","pir","dht11",
    "lcd_sda","lcd_scl"
];

const statusBox = document.getElementById("status");

async function loadConfig(){
    statusBox.textContent = "Đang đọc cấu hình...";

    try{
        const response = await fetch("/api/gpio");
        if(!response.ok) throw new Error("HTTP " + response.status);

        const data = await response.json();

        fields.forEach(key=>{
            document.getElementById(key).value = data[key];
        });

        statusBox.textContent = "Đã đọc cấu hình thành công.";
    }catch(error){
        statusBox.textContent = "Lỗi đọc cấu hình: " + error.message;
    }
}

async function saveConfig(){
    const data = {};

    for(const key of fields){
        const value = document.getElementById(key).value;

        if(value === ""){
            statusBox.textContent = "Không được để trống GPIO: " + key;
            return;
        }

        data[key] = Number(value);

        if(!Number.isInteger(data[key])){
            statusBox.textContent = "GPIO phải là số nguyên: " + key;
            return;
        }
    }

    if(!confirm("Bạn chắc chắn muốn lưu cấu hình GPIO?")){
        return;
    }

    statusBox.textContent = "Đang lưu...";
    document.getElementById("save").disabled = true;

    try{
        const response = await fetch("/api/gpio",{
            method:"POST",
            headers:{"Content-Type":"application/json"},
            body:JSON.stringify(data)
        });

        const result = await response.json().catch(()=>({}));

        if(!response.ok){
            throw new Error(result.message || "HTTP " + response.status);
        }

        statusBox.textContent =
            "Đã lưu cấu hình thành công.\n" +
            "Hãy khởi động lại robot để áp dụng.";
    }catch(error){
        statusBox.textContent = "Lỗi lưu cấu hình: " + error.message;
    }finally{
        document.getElementById("save").disabled = false;
    }
}

loadConfig();
</script>
</body>
</html>
)HTML";

    httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
}

// ============================================================
// WEBSOCKET SERVER
// ============================================================

WebSocketControlServer::WebSocketControlServer()
    : server_handle_(nullptr) {
    instance_ = this;
}

WebSocketControlServer::~WebSocketControlServer() {
    Stop();
    instance_ = nullptr;
}

esp_err_t WebSocketControlServer::ws_handler(httpd_req_t *req) {
    if (instance_ == nullptr) {
        return ESP_FAIL;
    }

    if (req->method == HTTP_GET) {
        ESP_LOGI(TAG, "Handshake done, the new connection was opened");
        instance_->AddClient(req);
        return ESP_OK;
    }

    httpd_ws_frame_t ws_pkt;
    uint8_t *buf = NULL;

    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;

    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "httpd_ws_recv_frame failed to get frame len with %d", ret);
        return ret;
    }

    ESP_LOGI(TAG, "frame len is %d", ws_pkt.len);

    if (ws_pkt.len) {
        buf = (uint8_t*)calloc(1, ws_pkt.len + 1);

        if (buf == NULL) {
            ESP_LOGE(TAG, "Failed to calloc memory for buf");
            return ESP_ERR_NO_MEM;
        }

        ws_pkt.payload = buf;

        ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);

        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "httpd_ws_recv_frame failed with %d", ret);
            free(buf);
            return ret;
        }

        ESP_LOGI(TAG, "Got packet with message: %s", ws_pkt.payload);
    }

    ESP_LOGI(TAG, "Packet type: %d", ws_pkt.type);

    if (ws_pkt.type == HTTPD_WS_TYPE_CLOSE) {
        ESP_LOGI(TAG, "WebSocket close frame received");
        instance_->RemoveClient(req);
        free(buf);
        return ESP_OK;
    }

    if (ws_pkt.type == HTTPD_WS_TYPE_TEXT) {
        if (ws_pkt.len > 0 && buf != nullptr) {
            buf[ws_pkt.len] = '\0';
            instance_->HandleMessage(req, (const char*)buf, ws_pkt.len);
        }
    } else {
        ESP_LOGW(TAG, "Unsupported frame type: %d", ws_pkt.type);
    }

    free(buf);
    return ESP_OK;
}

// ============================================================
// START SERVER AND REGISTER ROUTES
// ============================================================

bool WebSocketControlServer::Start(int port) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    config.server_port = port;
    config.max_open_sockets = 7;
    config.max_uri_handlers = 8;

    httpd_uri_t ws_uri = {
        .uri = "/ws",
        .method = HTTP_GET,
        .handler = ws_handler,
        .user_ctx = nullptr,
        .is_websocket = true
    };

    httpd_uri_t gpio_page_uri = {
        .uri = "/gpio",
        .method = HTTP_GET,
        .handler = gpio_page_handler,
        .user_ctx = nullptr
    };

    httpd_uri_t gpio_get_uri = {
        .uri = "/api/gpio",
        .method = HTTP_GET,
        .handler = gpio_get_handler,
        .user_ctx = nullptr
    };

    httpd_uri_t gpio_post_uri = {
        .uri = "/api/gpio",
        .method = HTTP_POST,
        .handler = gpio_post_handler,
        .user_ctx = nullptr
    };

    if (httpd_start(&server_handle_, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start WebSocket server");
        return false;
    }

    if (httpd_register_uri_handler(server_handle_, &ws_uri) != ESP_OK ||
        httpd_register_uri_handler(server_handle_, &gpio_page_uri) != ESP_OK ||
        httpd_register_uri_handler(server_handle_, &gpio_get_uri) != ESP_OK ||
        httpd_register_uri_handler(server_handle_, &gpio_post_uri) != ESP_OK) {

        ESP_LOGE(TAG, "Failed to register HTTP routes");
        httpd_stop(server_handle_);
        server_handle_ = nullptr;
        return false;
    }

    ESP_LOGI(TAG, "WebSocket server started on port %d", port);
    ESP_LOGI(TAG, "GPIO page: http://<robot-ip>:%d/gpio", port);

    return true;
}

void WebSocketControlServer::Stop() {
    if (server_handle_) {
        httpd_stop(server_handle_);
        server_handle_ = nullptr;
        clients_.clear();

        ESP_LOGI(TAG, "WebSocket server stopped");
    }
}

void WebSocketControlServer::HandleMessage(
    httpd_req_t *req,
    const char* data,
    size_t len
) {
    if (data == nullptr || len == 0) {
        ESP_LOGE(TAG, "Invalid message: data is null or len is 0");
        return;
    }

    if (len > 4096) {
        ESP_LOGE(TAG, "Message too long: %zu bytes", len);
        return;
    }

    char* temp_buf = (char*)malloc(len + 1);

    if (temp_buf == nullptr) {
        ESP_LOGE(TAG, "Failed to allocate memory");
        return;
    }

    memcpy(temp_buf, data, len);
    temp_buf[len] = '\0';

    cJSON* root = cJSON_Parse(temp_buf);
    free(temp_buf);

    if (root == nullptr) {
        ESP_LOGE(TAG, "Failed to parse JSON");
        return;
    }

    cJSON* payload = nullptr;
    cJSON* type = cJSON_GetObjectItem(root, "type");

    if (type && cJSON_IsString(type) &&
        strcmp(type->valuestring, "mcp") == 0) {

        payload = cJSON_GetObjectItem(root, "payload");

        if (payload != nullptr) {
            cJSON_DetachItemViaPointer(root, payload);
            McpServer::GetInstance().ParseMessage(payload);
            cJSON_Delete(payload);
        }
    } else {
        payload = cJSON_Duplicate(root, 1);

        if (payload != nullptr) {
            McpServer::GetInstance().ParseMessage(payload);
            cJSON_Delete(payload);
        }
    }

    if (payload == nullptr) {
        ESP_LOGE(TAG, "Invalid message format or failed to parse");
    }

    cJSON_Delete(root);
}

void WebSocketControlServer::AddClient(httpd_req_t *req) {
    int sock_fd = httpd_req_to_sockfd(req);

    if (clients_.find(sock_fd) == clients_.end()) {
        clients_[sock_fd] = req;
        ESP_LOGI(TAG, "Client connected: %d (total: %zu)",
                 sock_fd, clients_.size());
    }
}

void WebSocketControlServer::RemoveClient(httpd_req_t *req) {
    int sock_fd = httpd_req_to_sockfd(req);

    clients_.erase(sock_fd);

    ESP_LOGI(TAG, "Client disconnected: %d (total: %zu)",
             sock_fd, clients_.size());
}

size_t WebSocketControlServer::GetClientCount() const {
    return clients_.size();
}
