#include "image_display.h"

#include "board.h"
#include "display/display.h"
#include "display/lvgl_display/lvgl_display.h"
#include "display/lvgl_display/lvgl_image.h"

extern "C" {
#include "esp_jpeg_dec.h"
}

#include <esp_log.h>
#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <cstring>
#include <cJSON.h>

static const char* TAG = "ImageDisplay";
static constexpr size_t kMaxJpegBytes = 300 * 1024;

struct ImageDisplayCtx {
    LvglDisplay* display;
    std::vector<std::string> urls;
    int duration_ms;
};

static void FetchAndShow(LvglDisplay* display, const std::string& url) {
    // --- Download JPEG ---
    auto http = Board::GetInstance().GetNetwork()->CreateHttp(5000);
    http->SetHeader("User-Agent", "ESP32-Xiaozhi/1.0 (educational AI robot)");
    http->SetHeader("Accept", "image/jpeg,image/*");

    if (!http->Open("GET", url)) {
        ESP_LOGE(TAG, "Open failed: %s", url.c_str());
        return;
    }

    int status = http->GetStatusCode();
    if (status != 200 && status != 206) {
        ESP_LOGW(TAG, "HTTP %d for %s", status, url.c_str());
        http->Close();
        return;
    }

    size_t content_length = http->GetBodyLength();
    if (content_length == 0 || content_length > kMaxJpegBytes) {
        content_length = kMaxJpegBytes;
    }

    uint8_t* jpeg_buf = (uint8_t*)heap_caps_malloc(content_length,
                                                    MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
    if (!jpeg_buf) {
        jpeg_buf = (uint8_t*)heap_caps_malloc(content_length, MALLOC_CAP_8BIT);
    }
    if (!jpeg_buf) {
        ESP_LOGE(TAG, "OOM: cannot alloc %zu B for JPEG", content_length);
        http->Close();
        return;
    }

    size_t total = 0;
    bool read_error = false;
    while (total < content_length) {
        int n = http->Read((char*)(jpeg_buf + total), content_length - total);
        if (n < 0) {
            ESP_LOGE(TAG, "Read error after %zu B", total);
            read_error = true;
            break;
        }
        if (n == 0) break;
        total += n;
    }
    http->Close();

    if (total < 4 || read_error ||
        jpeg_buf[0] != 0xFF || jpeg_buf[1] != 0xD8 || jpeg_buf[2] != 0xFF) {
        heap_caps_free(jpeg_buf);
        ESP_LOGW(TAG, "Not a valid JPEG (%zu B): %s", total, url.c_str());
        return;
    }

    ESP_LOGI(TAG, "Fetched %zu B JPEG", total);

    // --- Decode JPEG → RGB565 (esp_new_jpeg software decoder) ---
    // LVGL has no JPEG decoder registered; we decode manually and use
    // LvglAllocatedImage's raw-pixel constructor to bypass the decoder chain.
    jpeg_dec_config_t cfg = {
        .output_type  = JPEG_PIXEL_FORMAT_RGB565_LE,
        .scale        = {.width = 0, .height = 0},
        .clipper      = {.width = 0, .height = 0},
        .rotate       = JPEG_ROTATE_0D,
        .block_enable = false,
    };

    jpeg_dec_handle_t dec = nullptr;
    if (jpeg_dec_open(&cfg, &dec) != JPEG_ERR_OK) {
        ESP_LOGE(TAG, "jpeg_dec_open failed");
        heap_caps_free(jpeg_buf);
        return;
    }

    jpeg_dec_io_t io;
    jpeg_dec_header_info_t hdr;
    memset(&io, 0, sizeof(io));
    memset(&hdr, 0, sizeof(hdr));
    io.inbuf     = jpeg_buf;
    io.inbuf_len = (int)total;

    if (jpeg_dec_parse_header(dec, &io, &hdr) != JPEG_ERR_OK) {
        ESP_LOGW(TAG, "JPEG header parse failed");
        jpeg_dec_close(dec);
        heap_caps_free(jpeg_buf);
        return;
    }

    int w      = hdr.width;
    int h      = hdr.height;
    int stride = w * 2;
    size_t rgb_size = (size_t)stride * h;

    // 16-byte aligned output buffer in PSRAM (required by esp_jpeg_dec DMA path)
    uint8_t* rgb_buf = (uint8_t*)heap_caps_aligned_alloc(
        16, rgb_size, MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
    if (!rgb_buf) {
        rgb_buf = (uint8_t*)heap_caps_aligned_alloc(16, rgb_size, MALLOC_CAP_8BIT);
    }
    if (!rgb_buf) {
        ESP_LOGE(TAG, "OOM: cannot alloc %zu B for RGB565", rgb_size);
        jpeg_dec_close(dec);
        heap_caps_free(jpeg_buf);
        return;
    }

    // Advance inbuf past bytes consumed by parse_header
    int consumed = io.inbuf_len - io.inbuf_remain;
    io.inbuf     = jpeg_buf + consumed;
    io.inbuf_len = io.inbuf_remain;
    io.outbuf    = rgb_buf;

    if (jpeg_dec_process(dec, &io) != JPEG_ERR_OK) {
        ESP_LOGW(TAG, "JPEG decode failed: %s", url.c_str());
        jpeg_dec_close(dec);
        heap_caps_free(jpeg_buf);
        heap_caps_free(rgb_buf);
        return;
    }

    jpeg_dec_close(dec);
    heap_caps_free(jpeg_buf);  // compressed data no longer needed

    ESP_LOGI(TAG, "Decoded %dx%d → %zu B RGB565", w, h, rgb_size);

    // --- Display via LVGL ---
    // The raw-pixel constructor sets header directly (magic, cf, w, h, stride).
    // LVGL renders it without invoking any decoder.
    // KeepPreviewVisible(true) disables the 10-second auto-hide timer so the
    // image stays on screen until the next question's image replaces it.
    {
        DisplayLockGuard lock(display);
        auto image = std::make_unique<LvglAllocatedImage>(
            rgb_buf, rgb_size, w, h, stride, LV_COLOR_FORMAT_RGB565);
        display->KeepPreviewVisible(true);
        display->SetPreviewImage(std::move(image));
    }
    // rgb_buf ownership is now held by LvglAllocatedImage; freed by its destructor
}

static void ImageDisplayTaskFn(void* arg) {
    auto* ctx = static_cast<ImageDisplayCtx*>(arg);
    for (const auto& url : ctx->urls) {
        if (url.empty()) {
            // Empty URL = clear / hide the preview image
            DisplayLockGuard lock(ctx->display);
            ctx->display->ClearPreviewImage();
            continue;
        }
        FetchAndShow(ctx->display, url);
        vTaskDelay(pdMS_TO_TICKS(ctx->duration_ms));
    }
    delete ctx;
    vTaskDelete(nullptr);
}

void StartImageDisplayTask(LvglDisplay* display,
                           std::vector<std::string> urls,
                           int duration_ms) {
    if (!display || urls.empty()) return;

    auto* ctx = new ImageDisplayCtx{display, std::move(urls), duration_ms};

    BaseType_t ret = xTaskCreate(
        ImageDisplayTaskFn, "img_disp",
        8 * 1024, ctx,
        3, nullptr);

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create image display task");
        delete ctx;
    }
}

// ── Image-poll task ────────────────────────────────────────────
// Called when the firmware detects `% show_why_image` in a sentence_start
// message.  It calls GET /api/image_queue on the MCP tools server, which
// long-polls up to 3 s for the wsrv.nl proxy URL that show_why_image stored
// right after being invoked.

struct ImagePollCtx {
    LvglDisplay* display;
    std::string  server_url;
};

static void ImagePollTaskFn(void* arg) {
    auto* ctx = static_cast<ImagePollCtx*>(arg);
    const std::string poll_url = ctx->server_url + "/api/image_queue";

    ESP_LOGI(TAG, "Polling image queue: %s", poll_url.c_str());

    // Timeout matches the server-side long-poll (3 s) + buffer
    auto http = Board::GetInstance().GetNetwork()->CreateHttp(5000);
    if (!http || !http->Open("GET", poll_url)) {
        ESP_LOGW(TAG, "Poll open failed: %s", poll_url.c_str());
        delete ctx;
        vTaskDelete(nullptr);
        return;
    }

    int status = http->GetStatusCode();
    if (status == 200) {
        std::string body;
        char buf[256];
        int  n;
        while ((n = http->Read(buf, sizeof(buf) - 1)) > 0) {
            buf[n] = '\0';
            body += buf;
        }
        http->Close();

        cJSON* json = cJSON_Parse(body.c_str());
        if (json) {
            const cJSON* url_item = cJSON_GetObjectItemCaseSensitive(json, "url");
            if (cJSON_IsString(url_item) && url_item->valuestring[0] != '\0') {
                ESP_LOGI(TAG, "Got image URL from queue: %.80s", url_item->valuestring);
                FetchAndShow(ctx->display, url_item->valuestring);
            } else {
                ESP_LOGW(TAG, "Poll returned empty URL");
            }
            cJSON_Delete(json);
        } else {
            ESP_LOGW(TAG, "Poll response parse error: %s", body.c_str());
        }
    } else {
        http->Close();
        ESP_LOGW(TAG, "Poll HTTP %d", status);
    }

    delete ctx;
    vTaskDelete(nullptr);
}

void StartImagePollingTask(LvglDisplay* display, const std::string& server_url) {
    if (!display || server_url.empty()) return;

    auto* ctx = new ImagePollCtx{display, server_url};
    BaseType_t ret = xTaskCreate(
        ImagePollTaskFn, "img_poll",
        6 * 1024, ctx,
        3, nullptr);

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create image poll task");
        delete ctx;
    }
}
