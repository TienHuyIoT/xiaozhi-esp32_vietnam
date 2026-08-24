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
#include <esp_random.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <algorithm>
#include <string>
#include <cstring>
#include <cJSON.h>
#include <mutex>
#include <new>

static const char* TAG = "ImageDisplay";
static constexpr size_t kMaxJpegBytes = 300 * 1024;
static constexpr size_t kMaxRgbBytes = 512 * 1024;
static constexpr int kMaxDecodedDimension = 1024;
static constexpr int kMaxImageDurationMs = 30000;
// The wsrv.nl proxy occasionally reports HTTP 404 (or 429/5xx) when Wikimedia
// rate-limits it upstream (Wikimedia 429 → wsrv.nl 404). These are transient,
// so retry a couple of times with a short backoff before giving up.
static constexpr int kFetchMaxAttempts = 3;
static constexpr int kFetchRetryDelayMs = 800;

enum class ImageDisplayMode {
    kImages,
    kPaymentQr,
    kPaymentSuccess,
};

struct ImageDisplayRequest {
    LvglDisplay* display;
    std::vector<std::string> urls;
    int duration_ms;
    ImageDisplayMode mode;
    std::string product_title;
    int preparation_seconds;
};

static std::mutex s_image_worker_mutex;
static TaskHandle_t s_image_worker_task = nullptr;
static ImageDisplayRequest* s_pending_request = nullptr;

static void ImageDisplayWorkerTaskFn(void* arg);

static bool EnsureImageDisplayWorkerStarted() {
    std::lock_guard<std::mutex> lock(s_image_worker_mutex);
    if (s_image_worker_task != nullptr) {
        return true;
    }

    TaskHandle_t worker = nullptr;
    BaseType_t ret = xTaskCreate(
        ImageDisplayWorkerTaskFn, "img_disp",
        8 * 1024, nullptr,
        3, &worker);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create image display worker");
        return false;
    }

    s_image_worker_task = worker;
    return true;
}

static ImageDisplayRequest* TakePendingImageRequest() {
    std::lock_guard<std::mutex> lock(s_image_worker_mutex);
    auto* request = s_pending_request;
    s_pending_request = nullptr;
    return request;
}

static bool WaitForDurationOrNewRequest(int duration_ms) {
    if (ulTaskNotifyTake(pdTRUE, 0) > 0) {
        return true;
    }

    if (duration_ms <= 0) {
        return false;
    }

    int remaining_ms = duration_ms;
    while (remaining_ms > 0) {
        int step_ms = remaining_ms > 100 ? 100 : remaining_ms;
        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(step_ms)) > 0) {
            return true;
        }
        remaining_ms -= step_ms;
    }
    return false;
}

// Downloads a JPEG into a freshly allocated buffer (caller frees with
// heap_caps_free). Returns nullptr on failure. Transient HTTP errors from the
// wsrv.nl proxy (404/429/5xx) and short reads are retried; hard errors (too
// large, OOM) fail immediately.
static uint8_t* DownloadJpeg(const std::string& url, size_t* out_len) {
    for (int attempt = 1; attempt <= kFetchMaxAttempts; ++attempt) {
        // wsrv.nl caches upstream failures per-URL: when Wikimedia rate-limits it
        // (429 → wsrv reports 404), the identical URL keeps serving that cached
        // error. On retries, append a unique cache-buster so wsrv.nl re-fetches
        // from origin under a fresh cache key instead of the poisoned entry.
        std::string fetch_url = url;
        if (attempt > 1) {
            fetch_url += (url.find('?') == std::string::npos ? '?' : '&');
            fetch_url += "cb=";
            fetch_url += std::to_string(esp_random());
        }

        auto http = Board::GetInstance().GetNetwork()->CreateHttp(5000);
        http->SetHeader("User-Agent", "ESP32-Xiaozhi/1.0 (educational AI robot)");
        http->SetHeader("Accept", "image/jpeg,image/*");

        if (!http->Open("GET", fetch_url)) {
            ESP_LOGW(TAG, "Open failed (attempt %d/%d): %s",
                     attempt, kFetchMaxAttempts, url.c_str());
            if (attempt < kFetchMaxAttempts) {
                vTaskDelay(pdMS_TO_TICKS(kFetchRetryDelayMs));
                continue;
            }
            return nullptr;
        }

        int status = http->GetStatusCode();
        if (status != 200 && status != 206) {
            http->Close();
            bool retryable = (status == 404 || status == 429 || status >= 500);
            ESP_LOGW(TAG, "HTTP %d (attempt %d/%d) for %s",
                     status, attempt, kFetchMaxAttempts, url.c_str());
            if (retryable && attempt < kFetchMaxAttempts) {
                vTaskDelay(pdMS_TO_TICKS(kFetchRetryDelayMs));
                continue;
            }
            return nullptr;
        }

        size_t content_length = http->GetBodyLength();
        if (content_length > kMaxJpegBytes) {
            ESP_LOGW(TAG, "JPEG too large: %zu B > %zu B", content_length, kMaxJpegBytes);
            http->Close();
            return nullptr;  // not retryable
        }
        if (content_length == 0) {
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
            return nullptr;  // not retryable
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
            ESP_LOGW(TAG, "Not a valid JPEG (%zu B, attempt %d/%d): %s",
                     total, attempt, kFetchMaxAttempts, url.c_str());
            if (attempt < kFetchMaxAttempts) {
                vTaskDelay(pdMS_TO_TICKS(kFetchRetryDelayMs));
                continue;
            }
            return nullptr;
        }

        *out_len = total;
        return jpeg_buf;
    }
    return nullptr;
}

static void FetchAndShow(LvglDisplay* display, const std::string& url,
                         bool payment_qr = false) {
    // --- Download JPEG (with retry for transient proxy rate-limits) ---
    size_t total = 0;
    uint8_t* jpeg_buf = DownloadJpeg(url, &total);
    if (!jpeg_buf) {
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
    if (w <= 0 || h <= 0 || w > kMaxDecodedDimension || h > kMaxDecodedDimension) {
        ESP_LOGW(TAG, "Invalid JPEG dimensions: %dx%d", w, h);
        jpeg_dec_close(dec);
        heap_caps_free(jpeg_buf);
        return;
    }

    size_t width = static_cast<size_t>(w);
    size_t height = static_cast<size_t>(h);
    size_t max_pixels = kMaxRgbBytes / sizeof(uint16_t);
    if (width > max_pixels / height) {
        ESP_LOGW(TAG, "Decoded image too large: %dx%d", w, h);
        jpeg_dec_close(dec);
        heap_caps_free(jpeg_buf);
        return;
    }

    size_t pixel_count = width * height;
    size_t rgb_size = pixel_count * sizeof(uint16_t);
    if (rgb_size > kMaxRgbBytes) {
        ESP_LOGW(TAG, "Decoded image too large: %dx%d -> %zu B", w, h, rgb_size);
        jpeg_dec_close(dec);
        heap_caps_free(jpeg_buf);
        return;
    }

    int stride = w * 2;

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
        if (payment_qr) {
            display->ShowPaymentQrImage(std::move(image));
        } else {
            display->KeepPreviewVisible(true);
            display->SetPreviewImage(std::move(image));
        }
    }
    // rgb_buf ownership is now held by LvglAllocatedImage; freed by its destructor
}

static void ProcessImageDisplayRequest(const ImageDisplayRequest& request) {
    ulTaskNotifyTake(pdTRUE, 0);

    if (request.mode == ImageDisplayMode::kPaymentQr) {
        for (int remaining = request.preparation_seconds; remaining > 0; --remaining) {
            request.display->ShowPaymentPreparing(request.product_title, remaining);
            if (WaitForDurationOrNewRequest(1000)) {
                return;
            }
        }
        if (!request.urls.empty()) {
            FetchAndShow(request.display, request.urls.front(), true);
        }
        return;
    }

    if (request.mode == ImageDisplayMode::kPaymentSuccess) {
        request.display->ShowPaymentSuccess(request.product_title);
        if (!WaitForDurationOrNewRequest(request.duration_ms)) {
            request.display->ClearPaymentScreen();
        }
        return;
    }

    for (const auto& url : request.urls) {
        if (url.empty()) {
            request.display->ClearPreviewImage();
        } else {
            FetchAndShow(request.display, url);
        }

        if (WaitForDurationOrNewRequest(request.duration_ms)) {
            break;
        }
    }
}

static void ImageDisplayWorkerTaskFn(void* arg) {
    (void)arg;

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        for (;;) {
            auto* request = TakePendingImageRequest();
            if (request == nullptr) {
                break;
            }

            ProcessImageDisplayRequest(*request);
            delete request;
        }
    }
}

void StartImageDisplayTask(LvglDisplay* display,
                           std::vector<std::string> urls,
                           int duration_ms) {
    if (!display || urls.empty()) return;

    if (duration_ms < 0) {
        duration_ms = 0;
    } else if (duration_ms > kMaxImageDurationMs) {
        duration_ms = kMaxImageDurationMs;
    }

    auto* request = new (std::nothrow) ImageDisplayRequest{
        display, std::move(urls), duration_ms, ImageDisplayMode::kImages, "", 0};
    if (request == nullptr) {
        ESP_LOGE(TAG, "OOM: cannot allocate image display request");
        return;
    }

    if (!EnsureImageDisplayWorkerStarted()) {
        delete request;
        return;
    }

    {
        std::lock_guard<std::mutex> lock(s_image_worker_mutex);
        delete s_pending_request;
        s_pending_request = request;
        xTaskNotifyGive(s_image_worker_task);
    }
}

void StartPaymentQrDisplayTask(LvglDisplay* display,
                               std::string image_url,
                               std::string product_title,
                               int preparation_seconds) {
    if (!display || image_url.empty()) return;
    preparation_seconds = std::max(0, std::min(preparation_seconds, 10));

    auto* request = new (std::nothrow) ImageDisplayRequest{
        display, {std::move(image_url)}, 0, ImageDisplayMode::kPaymentQr,
        std::move(product_title), preparation_seconds};
    if (request == nullptr) {
        ESP_LOGE(TAG, "OOM: cannot allocate payment QR request");
        return;
    }
    if (!EnsureImageDisplayWorkerStarted()) {
        delete request;
        return;
    }
    {
        std::lock_guard<std::mutex> lock(s_image_worker_mutex);
        delete s_pending_request;
        s_pending_request = request;
        xTaskNotifyGive(s_image_worker_task);
    }
}

void StartPaymentSuccessDisplayTask(LvglDisplay* display,
                                    std::string product_title,
                                    int duration_ms) {
    if (!display) return;
    duration_ms = std::max(1000, std::min(duration_ms, kMaxImageDurationMs));

    auto* request = new (std::nothrow) ImageDisplayRequest{
        display, {}, duration_ms, ImageDisplayMode::kPaymentSuccess,
        std::move(product_title), 0};
    if (request == nullptr) {
        ESP_LOGE(TAG, "OOM: cannot allocate payment success request");
        return;
    }
    if (!EnsureImageDisplayWorkerStarted()) {
        delete request;
        return;
    }
    {
        std::lock_guard<std::mutex> lock(s_image_worker_mutex);
        delete s_pending_request;
        s_pending_request = request;
        xTaskNotifyGive(s_image_worker_task);
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
