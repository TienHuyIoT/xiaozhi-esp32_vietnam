#include "mcp_tool_menu.h"

#include "board.h"
#include "display/display.h"
#include "display/lvgl_display/lvgl_theme.h"
#include "font_awesome.h"
#include "sd_card.h"

extern "C" {
#include "esp_jpeg_dec.h"
}

#include <esp_heap_caps.h>
#include <esp_log.h>
#include <esp_random.h>
#include <mbedtls/sha256.h>

#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <utility>

namespace {

constexpr char kTag[] = "McpToolMenu";
constexpr size_t kCacheCapacity = 3;
constexpr size_t kMaxJpegBytes = 32 * 1024;
constexpr int kImageWidth = 96;
constexpr int kImageHeight = 96;
constexpr int kTransitionMs = 160;

bool Sha256Matches(const uint8_t* data, size_t size, const char* expected) {
    unsigned char digest[32];
    mbedtls_sha256_context context;
    mbedtls_sha256_init(&context);
    bool okay = mbedtls_sha256_starts(&context, 0) == 0 &&
                mbedtls_sha256_update(&context, data, size) == 0 &&
                mbedtls_sha256_finish(&context, digest) == 0;
    mbedtls_sha256_free(&context);
    if (!okay) {
        return false;
    }
    char actual[65];
    for (size_t i = 0; i < sizeof(digest); ++i) {
        std::snprintf(actual + i * 2, 3, "%02x", digest[i]);
    }
    actual[64] = '\0';
    return std::strcmp(actual, expected) == 0;
}

bool IsBaselineJpeg(const uint8_t* data, size_t size) {
    if (!data || size < 4 || data[0] != 0xff || data[1] != 0xd8) {
        return false;
    }
    size_t offset = 2;
    while (offset + 4 <= size) {
        if (data[offset] != 0xff) {
            return false;
        }
        while (offset < size && data[offset] == 0xff) {
            ++offset;
        }
        if (offset >= size) {
            return false;
        }
        uint8_t marker = data[offset++];
        if (marker == 0xd8 || marker == 0xd9) {
            continue;
        }
        if (marker == 0xda) {
            return false;
        }
        if (offset + 2 > size) {
            return false;
        }
        size_t segment = (static_cast<size_t>(data[offset]) << 8) | data[offset + 1];
        if (segment < 2 || offset + segment > size) {
            return false;
        }
        if (marker == 0xc0) {
            return segment >= 8 && data[offset + 2] == 8;
        }
        if (marker >= 0xc1 && marker <= 0xcf && marker != 0xc4 && marker != 0xc8 &&
            marker != 0xcc) {
            return false;
        }
        offset += segment;
    }
    return false;
}

bool EnsureDirectory(const std::string& path) {
    struct stat st;
    if (stat(path.c_str(), &st) == 0) {
        return S_ISDIR(st.st_mode);
    }
    return mkdir(path.c_str(), 0775) == 0;
}

uint8_t* ReadJpegFromSdCard(const std::string& file_path, size_t expected_bytes,
                            const char* expected_sha256, size_t* out_len) {
    if (!out_len) {
        return nullptr;
    }
    *out_len = 0;
    struct stat st;
    if (stat(file_path.c_str(), &st) != 0 || st.st_size <= 0 ||
        st.st_size > static_cast<off_t>(kMaxJpegBytes)) {
        return nullptr;
    }
    if (expected_bytes > 0 && static_cast<size_t>(st.st_size) != expected_bytes) {
        ESP_LOGW(kTag, "SD icon size mismatch (%ld vs %zu), deleting %s",
                 static_cast<long>(st.st_size), expected_bytes, file_path.c_str());
        unlink(file_path.c_str());
        return nullptr;
    }
    FILE* f = fopen(file_path.c_str(), "rb");
    if (!f) {
        return nullptr;
    }
    size_t file_size = static_cast<size_t>(st.st_size);
    auto* jpeg = static_cast<uint8_t*>(heap_caps_malloc(
        file_size, MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM));
    if (!jpeg) {
        ESP_LOGW(kTag, "No PSRAM to read SD icon (%zu bytes)", file_size);
        fclose(f);
        return nullptr;
    }
    size_t read_bytes = fread(jpeg, 1, file_size, f);
    fclose(f);
    if (read_bytes != file_size || !IsBaselineJpeg(jpeg, file_size) ||
        !Sha256Matches(jpeg, file_size, expected_sha256)) {
        ESP_LOGW(kTag, "Corrupt SD icon (%s), deleting", file_path.c_str());
        heap_caps_free(jpeg);
        unlink(file_path.c_str());
        return nullptr;
    }
    *out_len = file_size;
    return jpeg;
}

void WriteJpegToSdCard(const std::string& file_path, const uint8_t* data, size_t size) {
    if (!data || size == 0) {
        return;
    }
    FILE* f = fopen(file_path.c_str(), "wb");
    if (!f) {
        ESP_LOGW(kTag, "Failed to open %s for caching SD icon", file_path.c_str());
        return;
    }
    size_t written = fwrite(data, 1, size, f);
    fclose(f);
    if (written != size) {
        ESP_LOGW(kTag, "Incomplete SD icon write (%zu/%zu), deleting %s",
                 written, size, file_path.c_str());
        unlink(file_path.c_str());
    } else {
        ESP_LOGI(kTag, "Cached MCP icon to SD: %s", file_path.c_str());
    }
}

uint8_t* DownloadJpegFromHttp(const McpToolMenu::ImageRequest& request, size_t* out_len) {
    if (!out_len) {
        return nullptr;
    }
    *out_len = 0;
    auto http = Board::GetInstance().GetNetwork()->CreateHttp(5000);
    if (!http) {
        return nullptr;
    }
    http->SetHeader("User-Agent", "ESP32-Xiaozhi/1.0");
    http->SetHeader("Accept", "image/jpeg");
    if (!http->Open("GET", request.url)) {
        ESP_LOGW(kTag, "Icon HTTP open failed");
        return nullptr;
    }
    int status = http->GetStatusCode();
    size_t content_length = http->GetBodyLength();
    if (status != 200 || request.expected_bytes == 0 ||
        request.expected_bytes > kMaxJpegBytes ||
        content_length != request.expected_bytes) {
        ESP_LOGW(kTag, "Icon HTTP metadata mismatch: status=%d len=%u expected=%u",
                 status, static_cast<unsigned>(content_length),
                 static_cast<unsigned>(request.expected_bytes));
        http->Close();
        return nullptr;
    }

    auto* jpeg = static_cast<uint8_t*>(heap_caps_malloc(
        content_length, MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM));
    if (!jpeg) {
        ESP_LOGW(kTag, "No PSRAM for %u-byte icon staging buffer",
                 static_cast<unsigned>(content_length));
        http->Close();
        return nullptr;
    }
    size_t total = 0;
    while (total < content_length) {
        int read = http->Read(reinterpret_cast<char*>(jpeg + total), content_length - total);
        if (read <= 0) {
            break;
        }
        total += static_cast<size_t>(read);
    }
    http->Close();
    if (total != content_length || !IsBaselineJpeg(jpeg, total) ||
        !Sha256Matches(jpeg, total, request.sha256)) {
        ESP_LOGW(kTag, "Rejected invalid MCP icon payload");
        heap_caps_free(jpeg);
        return nullptr;
    }
    *out_len = total;
    return jpeg;
}

std::unique_ptr<LvglAllocatedImage> DownloadAndDecode(
        const McpToolMenu::ImageRequest& request) {
    uint8_t* jpeg = nullptr;
    size_t jpeg_len = 0;

    auto* sd_card = Board::GetInstance().GetSdCard();
    bool sd_available = sd_card && sd_card->IsMounted() && sd_card->GetMountPoint();
    std::string icon_file;
    std::string icon_dir;

    if (sd_available) {
        const char* mp = sd_card->GetMountPoint();
        icon_dir = std::string(mp) + "/mcp_icons";
        icon_file = icon_dir + "/" + request.sha256 + ".jpg";

        jpeg = ReadJpegFromSdCard(icon_file, request.expected_bytes, request.sha256, &jpeg_len);
        if (jpeg) {
            ESP_LOGD(kTag, "Loaded icon from SD card: %s", icon_file.c_str());
        }
    }

    if (!jpeg) {
        jpeg = DownloadJpegFromHttp(request, &jpeg_len);
        if (jpeg && sd_available) {
            if (EnsureDirectory(icon_dir)) {
                WriteJpegToSdCard(icon_file, jpeg, jpeg_len);
            }
        }
    }

    if (!jpeg) {
        return nullptr;
    }

    jpeg_dec_config_t config = {
        .output_type = JPEG_PIXEL_FORMAT_RGB565_LE,
        .scale = {.width = 0, .height = 0},
        .clipper = {.width = 0, .height = 0},
        .rotate = JPEG_ROTATE_0D,
        .block_enable = false,
    };
    jpeg_dec_handle_t decoder = nullptr;
    if (jpeg_dec_open(&config, &decoder) != JPEG_ERR_OK) {
        heap_caps_free(jpeg);
        return nullptr;
    }
    jpeg_dec_io_t io = {};
    jpeg_dec_header_info_t header = {};
    io.inbuf = jpeg;
    io.inbuf_len = static_cast<int>(jpeg_len);
    if (jpeg_dec_parse_header(decoder, &io, &header) != JPEG_ERR_OK ||
        header.width != kImageWidth || header.height != kImageHeight) {
        ESP_LOGW(kTag, "Rejected MCP icon dimensions: %dx%d", header.width, header.height);
        jpeg_dec_close(decoder);
        heap_caps_free(jpeg);
        return nullptr;
    }

    constexpr size_t rgb_bytes = kImageWidth * kImageHeight * sizeof(uint16_t);
    auto* rgb = static_cast<uint8_t*>(heap_caps_aligned_alloc(
        16, rgb_bytes, MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM));
    if (!rgb) {
        ESP_LOGW(kTag, "No PSRAM for decoded MCP icon");
        jpeg_dec_close(decoder);
        heap_caps_free(jpeg);
        return nullptr;
    }
    int consumed = io.inbuf_len - io.inbuf_remain;
    io.inbuf = jpeg + consumed;
    io.inbuf_len = io.inbuf_remain;
    io.outbuf = rgb;
    bool decoded = jpeg_dec_process(decoder, &io) == JPEG_ERR_OK;
    jpeg_dec_close(decoder);
    heap_caps_free(jpeg);
    if (!decoded) {
        heap_caps_free(rgb);
        return nullptr;
    }
    return std::make_unique<LvglAllocatedImage>(
        rgb, rgb_bytes, kImageWidth, kImageHeight, kImageWidth * 2,
        LV_COLOR_FORMAT_RGB565);
}

#ifndef CONFIG_USE_EMOTE_MESSAGE_STYLE
void StyleLabel(lv_obj_t* label, lv_color_t color, lv_text_align_t align) {
    lv_obj_set_style_text_color(label, color, 0);
    lv_obj_set_style_text_align(label, align, 0);
    lv_obj_set_style_bg_opa(label, LV_OPA_TRANSP, 0);
}
#endif

}  // namespace

McpToolMenu::McpToolMenu() {
    image_queue_ = xQueueCreate(3, sizeof(ImageRequest*));
    worker_running_.store(image_queue_ != nullptr);
    if (!image_queue_ || xTaskCreate(ImageWorkerEntry, "mcp_icons", 6144, this, 3,
                                     &image_worker_task_) != pdPASS) {
        ESP_LOGE(kTag, "Failed to create MCP icon worker");
        if (image_queue_) {
            vQueueDelete(image_queue_);
            image_queue_ = nullptr;
        }
        image_worker_task_ = nullptr;
        worker_running_.store(false);
    }
}

McpToolMenu::~McpToolMenu() {
    stop_worker_.store(true);
    FreePendingRequests();
    if (image_queue_ && image_worker_task_) {
        ImageRequest* stop = nullptr;
        xQueueSend(image_queue_, &stop, 0);
        xTaskNotifyGive(image_worker_task_);
        for (int i = 0; i < 1200 && worker_running_.load(); ++i) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
    if (image_queue_) {
        vQueueDelete(image_queue_);
        image_queue_ = nullptr;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    HideLocked();
#ifndef CONFIG_USE_EMOTE_MESSAGE_STYLE
    auto* display = Board::GetInstance().GetDisplay();
    if (display && overlay_) {
        DisplayLockGuard display_lock(display);
        lv_obj_del(overlay_);
        overlay_ = nullptr;
    }
#endif
}

void McpToolMenu::SetInvokeHandler(InvokeHandler handler) {
    std::lock_guard<std::mutex> lock(mutex_);
    invoke_handler_ = std::move(handler);
}

void McpToolMenu::SetConnectionState(bool online) {
    std::lock_guard<std::mutex> lock(mutex_);
    online_ = online;
    if (open_.load() && !ready_) {
        RenderLocked();
    }
}

void McpToolMenu::SetCatalog(std::string revision, std::vector<McpToolEntry> tools,
                             bool ready, bool truncated) {
    std::lock_guard<std::mutex> lock(mutex_);
    bool revision_changed = revision != revision_;
    if (revision_changed) {
        ++image_generation_;
        FreePendingRequests();
        ClearCacheLocked();
    }
    ready_ = ready;
    truncated_ = truncated;
    revision_ = std::move(revision);
    tools_ = std::move(tools);
    if (tools_.empty() || selected_index_ >= tools_.size()) {
        selected_index_ = 0;
    }
    if (view_state_ != ViewState::kRunning) {
        view_state_ = ViewState::kBrowse;
    }
    if (open_.load()) {
        RenderLocked();
        ScheduleImagesLocked();
    }
}

void McpToolMenu::SetResult(const std::string& request_id, bool success,
                            const std::string& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (request_id.empty() || request_id != active_request_id_) {
        return;
    }
    result_success_ = success;
    result_message_ = message;
    view_state_ = ViewState::kResult;
    if (open_.load()) {
        RenderLocked();
    }
}

void McpToolMenu::Toggle() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (open_.load()) {
        HideLocked();
        return;
    }
    open_.store(true);
    view_state_ = ViewState::kBrowse;
    active_request_id_.clear();
    result_message_.clear();
    auto* display = Board::GetInstance().GetDisplay();
    if (display) {
        display->SetMediaOverlayActive(true);
    }
    RenderLocked();
    ScheduleImagesLocked();
}

void McpToolMenu::Close() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (open_.load()) {
        HideLocked();
    }
}

void McpToolMenu::Next() { Move(1); }
void McpToolMenu::Previous() { Move(-1); }

void McpToolMenu::Move(int direction) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!open_.load() || tools_.empty() || view_state_ == ViewState::kRunning) {
        return;
    }
    view_state_ = ViewState::kBrowse;
    if (direction > 0) {
        selected_index_ = (selected_index_ + 1) % tools_.size();
    } else {
        selected_index_ = (selected_index_ + tools_.size() - 1) % tools_.size();
    }
    ++image_generation_;
    FreePendingRequests();
    RenderLocked(direction);
    ScheduleImagesLocked();
}

void McpToolMenu::TripleClick() {
    InvokeHandler handler;
    std::string tool_name;
    std::string revision;
    std::string request_id;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!open_.load() || !ready_ || tools_.empty()) {
            return;
        }
        if (view_state_ == ViewState::kBrowse) {
            view_state_ = ViewState::kConfirm;
            RenderLocked();
            return;
        }
        if (view_state_ == ViewState::kConfirm) {
            const auto& tool = tools_[selected_index_];
            if (!tool.selectable || !invoke_handler_) {
                view_state_ = ViewState::kBrowse;
                RenderLocked();
                return;
            }
            active_request_id_ = NewRequestId();
            view_state_ = ViewState::kRunning;
            result_message_.clear();
            handler = invoke_handler_;
            tool_name = tool.name;
            revision = revision_;
            request_id = active_request_id_;
            RenderLocked();
        } else if (view_state_ == ViewState::kResult) {
            view_state_ = ViewState::kBrowse;
            RenderLocked();
            return;
        } else {
            return;
        }
    }
    handler(tool_name, revision, request_id);
}

std::string McpToolMenu::NewRequestId() const {
    char value[17];
    std::snprintf(value, sizeof(value), "%08lx%08lx",
                  static_cast<unsigned long>(esp_random()),
                  static_cast<unsigned long>(esp_random()));
    return value;
}

void McpToolMenu::BuildUiLocked() {
#ifdef CONFIG_USE_EMOTE_MESSAGE_STYLE
    return;
#else
    if (overlay_) {
        return;
    }
    auto* display = Board::GetInstance().GetDisplay();
    if (!display) {
        return;
    }
    DisplayLockGuard display_lock(display);
    const lv_font_t* icon_font = nullptr;
    auto* theme = static_cast<LvglTheme*>(display->GetTheme());
    if (theme && theme->icon_font()) {
        icon_font = theme->icon_font()->font();
    }
    overlay_ = lv_obj_create(lv_screen_active());
    lv_obj_remove_style_all(overlay_);
    lv_obj_set_size(overlay_, 240, 240);
    lv_obj_align(overlay_, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_color(overlay_, lv_color_hex(0xF4F6F9), 0);
    lv_obj_set_style_bg_opa(overlay_, LV_OPA_COVER, 0);
    lv_obj_set_scrollbar_mode(overlay_, LV_SCROLLBAR_MODE_OFF);

    card_ = lv_obj_create(overlay_);
    lv_obj_set_size(card_, 194, 220);
    lv_obj_align(card_, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_radius(card_, 24, 0);
    lv_obj_set_style_bg_color(card_, lv_color_white(), 0);
    lv_obj_set_style_bg_opa(card_, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(card_, 1, 0);
    lv_obj_set_style_border_color(card_, lv_color_hex(0xE5ECF5), 0);
    lv_obj_set_style_shadow_width(card_, 10, 0);
    lv_obj_set_style_shadow_opa(card_, LV_OPA_20, 0);
    lv_obj_set_style_shadow_color(card_, lv_color_hex(0xAAB7C8), 0);
    lv_obj_set_style_pad_all(card_, 0, 0);
    lv_obj_set_scrollbar_mode(card_, LV_SCROLLBAR_MODE_OFF);

    placeholder_ = lv_obj_create(card_);
    lv_obj_set_size(placeholder_, 96, 96);
    lv_obj_align(placeholder_, LV_ALIGN_TOP_MID, 0, 8);
    lv_obj_set_style_radius(placeholder_, 24, 0);
    lv_obj_set_style_border_width(placeholder_, 0, 0);
    lv_obj_set_style_bg_color(placeholder_, lv_color_hex(0xDCEBFF), 0);
    lv_obj_set_style_bg_opa(placeholder_, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_all(placeholder_, 0, 0);
    placeholder_icon_ = lv_label_create(placeholder_);
    lv_label_set_text(placeholder_icon_, FONT_AWESOME_IMAGE);
    StyleLabel(placeholder_icon_, lv_color_hex(0x3478D4), LV_TEXT_ALIGN_CENTER);
    if (icon_font) {
        lv_obj_set_style_text_font(placeholder_icon_, icon_font, 0);
    }
    lv_obj_center(placeholder_icon_);

    image_ = lv_image_create(card_);
    lv_obj_set_size(image_, 96, 96);
    lv_obj_align(image_, LV_ALIGN_TOP_MID, 0, 8);
    lv_obj_add_flag(image_, LV_OBJ_FLAG_HIDDEN);

    title_label_ = lv_label_create(card_);
    lv_obj_set_width(title_label_, 174);
    lv_label_set_long_mode(title_label_, LV_LABEL_LONG_DOT);
    StyleLabel(title_label_, lv_color_hex(0x3478D4), LV_TEXT_ALIGN_CENTER);
    lv_obj_align(title_label_, LV_ALIGN_TOP_MID, 0, 126);

    name_pill_ = lv_obj_create(card_);
    lv_obj_set_size(name_pill_, 166, 22);
    lv_obj_align(name_pill_, LV_ALIGN_TOP_MID, 0, 150);
    lv_obj_set_style_radius(name_pill_, 11, 0);
    lv_obj_set_style_border_width(name_pill_, 0, 0);
    lv_obj_set_style_bg_color(name_pill_, lv_color_hex(0xEEF4FC), 0);
    lv_obj_set_style_bg_opa(name_pill_, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_all(name_pill_, 0, 0);
    lv_obj_add_flag(name_pill_, LV_OBJ_FLAG_HIDDEN);
    name_label_ = lv_label_create(name_pill_);
    lv_obj_set_width(name_label_, 154);
    lv_label_set_long_mode(name_label_, LV_LABEL_LONG_DOT);
    StyleLabel(name_label_, lv_color_hex(0x53657D), LV_TEXT_ALIGN_CENTER);
    lv_obj_center(name_label_);



    status_label_ = lv_label_create(card_);
    lv_obj_set_width(status_label_, 176);
    lv_label_set_long_mode(status_label_, LV_LABEL_LONG_DOT);
    StyleLabel(status_label_, lv_color_hex(0x53657D), LV_TEXT_ALIGN_CENTER);
    lv_obj_align(status_label_, LV_ALIGN_BOTTOM_MID, 0, -4);

    left_chevron_ = lv_label_create(overlay_);
    lv_label_set_text(left_chevron_, FONT_AWESOME_ANGLE_LEFT);
    StyleLabel(left_chevron_, lv_color_hex(0x3478D4), LV_TEXT_ALIGN_CENTER);
    if (icon_font) {
        lv_obj_set_style_text_font(left_chevron_, icon_font, 0);
    }
    lv_obj_align(left_chevron_, LV_ALIGN_LEFT_MID, 4, -7);
    right_chevron_ = lv_label_create(overlay_);
    lv_label_set_text(right_chevron_, FONT_AWESOME_ANGLE_RIGHT);
    StyleLabel(right_chevron_, lv_color_hex(0x3478D4), LV_TEXT_ALIGN_CENTER);
    if (icon_font) {
        lv_obj_set_style_text_font(right_chevron_, icon_font, 0);
    }
    lv_obj_align(right_chevron_, LV_ALIGN_RIGHT_MID, -4, -7);

    counter_label_ = lv_label_create(overlay_);
    StyleLabel(counter_label_, lv_color_hex(0x3478D4), LV_TEXT_ALIGN_CENTER);
    lv_obj_align(counter_label_, LV_ALIGN_BOTTOM_MID, 0, -2);
#endif
}

void McpToolMenu::RenderLocked(int direction) {
#ifdef CONFIG_USE_EMOTE_MESSAGE_STYLE
    return;
#else
    BuildUiLocked();
    if (!overlay_) {
        return;
    }
    auto* display = Board::GetInstance().GetDisplay();
    DisplayLockGuard display_lock(display);
    lv_obj_remove_flag(overlay_, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(overlay_);

    if (!ready_ || tools_.empty()) {
        DetachImageLocked();
        lv_label_set_text(title_label_, ready_ ? "Chưa có chức năng" : "Đang đồng bộ");
        lv_label_set_text(name_label_, "Chức năng");
        lv_label_set_text(status_label_, "Nhấn đúp để đóng");
        lv_label_set_text(counter_label_, "—");
        return;
    }

    const auto& tool = tools_[selected_index_];
    lv_label_set_text(title_label_, tool.title.c_str());
    lv_label_set_text(name_label_, tool.name.c_str());
    lv_obj_set_style_text_color(status_label_, lv_color_hex(0x53657D), 0);
    if (view_state_ == ViewState::kBrowse) {
        lv_label_set_text(status_label_, "");
    } else if (view_state_ == ViewState::kConfirm) {
        lv_label_set_text(status_label_, tool.selectable ? "Sẵn sàng xác nhận" : "3 lần: quay lại");
        lv_obj_set_style_text_color(status_label_, tool.selectable
            ? lv_color_hex(0x2E8B57) : lv_color_hex(0x66758A), 0);
    } else if (view_state_ == ViewState::kRunning) {
        lv_label_set_text(status_label_, "Đang chạy");
        lv_obj_set_style_text_color(status_label_, lv_color_hex(0xB36A24), 0);
    } else {
        lv_label_set_text(status_label_, result_success_ ? "Hoàn tất · 3 lần để về" : "Không thành công · 3 lần để về");
        lv_obj_set_style_text_color(status_label_, result_success_
            ? lv_color_hex(0x2E8B57) : lv_color_hex(0xB74455), 0);
    }
    char counter[20];
    std::snprintf(counter, sizeof(counter), "%u/%u%s",
                  static_cast<unsigned>(selected_index_ + 1),
                  static_cast<unsigned>(tools_.size()), truncated_ ? "+" : "");
    lv_label_set_text(counter_label_, counter);
    ApplyCachedImageLocked(tool.illustration.valid ? tool.illustration.sha256 : "");
    AnimateCardLocked(direction);
#endif
}

void McpToolMenu::HideLocked() {
    ++image_generation_;
    FreePendingRequests();
    open_.store(false);
#ifndef CONFIG_USE_EMOTE_MESSAGE_STYLE
    auto* display = Board::GetInstance().GetDisplay();
    if (display && overlay_) {
        DisplayLockGuard display_lock(display);
        DetachImageLocked();
        lv_obj_add_flag(overlay_, LV_OBJ_FLAG_HIDDEN);
    }
    image_cache_.clear();
    if (display) {
        display->SetMediaOverlayActive(false);
    }
#else
    image_cache_.clear();
#endif
}

void McpToolMenu::AnimateTranslate(void* object, int32_t value) {
    lv_obj_set_style_translate_x(static_cast<lv_obj_t*>(object), value, 0);
}

void McpToolMenu::AnimateOpacity(void* object, int32_t value) {
    lv_obj_set_style_opa(static_cast<lv_obj_t*>(object), value, 0);
}

void McpToolMenu::AnimateCardLocked(int direction) {
#ifndef CONFIG_USE_EMOTE_MESSAGE_STYLE
    if (!card_ || direction == 0) {
        return;
    }
    lv_anim_delete(card_, AnimateTranslate);
    lv_anim_delete(card_, AnimateOpacity);
    lv_anim_t animation;
    lv_anim_init(&animation);
    lv_anim_set_var(&animation, card_);
    lv_anim_set_duration(&animation, kTransitionMs);
    lv_anim_set_path_cb(&animation, lv_anim_path_ease_out);
    lv_anim_set_values(&animation, direction > 0 ? 10 : -10, 0);
    lv_anim_set_exec_cb(&animation, AnimateTranslate);
    lv_anim_start(&animation);
    lv_anim_set_values(&animation, LV_OPA_50, LV_OPA_COVER);
    lv_anim_set_exec_cb(&animation, AnimateOpacity);
    lv_anim_start(&animation);
#endif
}

void McpToolMenu::DetachImageLocked() {
#ifndef CONFIG_USE_EMOTE_MESSAGE_STYLE
    if (image_) {
        lv_image_set_src(image_, nullptr);
        lv_obj_add_flag(image_, LV_OBJ_FLAG_HIDDEN);
    }
    if (placeholder_) {
        lv_obj_remove_flag(placeholder_, LV_OBJ_FLAG_HIDDEN);
    }
#endif
}

void McpToolMenu::ClearCacheLocked() {
#ifndef CONFIG_USE_EMOTE_MESSAGE_STYLE
    auto* display = Board::GetInstance().GetDisplay();
    if (display && image_) {
        DisplayLockGuard display_lock(display);
        DetachImageLocked();
    }
#endif
    image_cache_.clear();
}

void McpToolMenu::ApplyCachedImageLocked(const std::string& sha256) {
#ifndef CONFIG_USE_EMOTE_MESSAGE_STYLE
    auto found = std::find_if(image_cache_.begin(), image_cache_.end(),
        [&sha256](const CacheEntry& entry) { return entry.sha256 == sha256; });
    if (sha256.empty() || found == image_cache_.end()) {
        DetachImageLocked();
        return;
    }
    found->last_used = ++cache_clock_;
    lv_image_set_src(image_, found->image->image_dsc());
    lv_obj_remove_flag(image_, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(placeholder_, LV_OBJ_FLAG_HIDDEN);
#endif
}

void McpToolMenu::FreePendingRequests() {
    if (!image_queue_) {
        return;
    }
    ImageRequest* request = nullptr;
    while (xQueueReceive(image_queue_, &request, 0) == pdTRUE) {
        heap_caps_free(request);
    }
}

void McpToolMenu::QueueImageLocked(const McpToolEntry& tool) {
    if (!image_queue_ || !tool.illustration.valid) {
        return;
    }
    auto found = std::find_if(image_cache_.begin(), image_cache_.end(),
        [&tool](const CacheEntry& entry) {
            return entry.sha256 == tool.illustration.sha256;
        });
    if (found != image_cache_.end()) {
        found->last_used = ++cache_clock_;
        return;
    }
    auto* request = static_cast<ImageRequest*>(heap_caps_calloc(
        1, sizeof(ImageRequest), MALLOC_CAP_8BIT));
    if (!request) {
        return;
    }
    request->generation = image_generation_;
    request->expected_bytes = tool.illustration.bytes;
    std::snprintf(request->revision, sizeof(request->revision), "%s", revision_.c_str());
    std::snprintf(request->sha256, sizeof(request->sha256), "%s", tool.illustration.sha256.c_str());
    std::snprintf(request->url, sizeof(request->url), "%s", tool.illustration.url.c_str());
    if (xQueueSend(image_queue_, &request, 0) != pdTRUE) {
        heap_caps_free(request);
    }
}

void McpToolMenu::ScheduleImagesLocked() {
    if (!open_.load() || tools_.empty()) {
        return;
    }
    FreePendingRequests();
    QueueImageLocked(tools_[selected_index_]);
    if (tools_.size() > 1) {
        QueueImageLocked(tools_[(selected_index_ + tools_.size() - 1) % tools_.size()]);
        QueueImageLocked(tools_[(selected_index_ + 1) % tools_.size()]);
    }
}

void McpToolMenu::ImageWorkerEntry(void* arg) {
    static_cast<McpToolMenu*>(arg)->ImageWorkerLoop();
}

void McpToolMenu::ImageWorkerLoop() {
    while (!stop_worker_.load()) {
        ImageRequest* raw = nullptr;
        if (xQueueReceive(image_queue_, &raw, pdMS_TO_TICKS(500)) != pdTRUE) {
            continue;
        }
        if (!raw) {
            break;
        }
        std::unique_ptr<ImageRequest, decltype(&heap_caps_free)> request(raw, heap_caps_free);
        auto decoded = DownloadAndDecode(*request);
        if (!decoded) {
            continue;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        if (!open_.load() || request->generation != image_generation_ ||
            revision_ != request->revision) {
            continue;
        }
        auto duplicate = std::find_if(image_cache_.begin(), image_cache_.end(),
            [&request](const CacheEntry& entry) { return entry.sha256 == request->sha256; });
        if (duplicate == image_cache_.end()) {
            if (image_cache_.size() >= kCacheCapacity) {
                std::string selected_sha = tools_.empty()
                    ? std::string() : tools_[selected_index_].illustration.sha256;
                auto oldest = std::min_element(image_cache_.begin(), image_cache_.end(),
                    [&selected_sha](const CacheEntry& a, const CacheEntry& b) {
                        uint32_t a_age = a.sha256 == selected_sha ? UINT32_MAX : a.last_used;
                        uint32_t b_age = b.sha256 == selected_sha ? UINT32_MAX : b.last_used;
                        return a_age < b_age;
                    });
                image_cache_.erase(oldest);
            }
            image_cache_.push_back({request->sha256, std::move(decoded), ++cache_clock_});
        }
        if (!tools_.empty() && tools_[selected_index_].illustration.valid &&
            tools_[selected_index_].illustration.sha256 == request->sha256) {
            auto* display = Board::GetInstance().GetDisplay();
            if (display) {
                DisplayLockGuard display_lock(display);
                ApplyCachedImageLocked(request->sha256);
            }
        }
    }
    image_worker_task_ = nullptr;
    worker_running_.store(false);
    vTaskDelete(nullptr);
}
