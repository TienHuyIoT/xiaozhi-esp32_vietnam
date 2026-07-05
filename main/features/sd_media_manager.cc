#include "sd_media_manager.h"

#include "application.h"
#include "board.h"
#include "display/lcd_display.h"
#include "display/lvgl_display/lvgl_image.h"
#include "features/music/esp32_sd_music.h"
#include "features/video/video_player.h"
#include "sd_card.h"

#include <cJSON.h>
#include <esp_heap_caps.h>
#include <esp_log.h>

#ifndef MEDIA_SERVER_BASE_URL
#define MEDIA_SERVER_BASE_URL "http://192.168.100.97:7860"
#endif
#include <esp_vfs_fat.h>
#include <mbedtls/sha256.h>

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstring>
#include <dirent.h>
#include <memory>
#include <sys/stat.h>
#include <unistd.h>
#include <vector>

namespace {
constexpr char kTag[] = "SdMediaManager";
constexpr size_t kIoBufferSize = 8 * 1024;
constexpr uint32_t kWorkerStackSize = 12 * 1024;
constexpr int kHttpTimeoutMs = 30 * 1000;
constexpr int kDownloadAttempts = 3;
constexpr int kAckAttempts = 3;
constexpr uint64_t kImageMaxBytes = 1ULL * 1024 * 1024;
constexpr uint64_t kAudioMaxBytes = 30ULL * 1024 * 1024;
constexpr uint64_t kVideoMaxBytes = 100ULL * 1024 * 1024;

std::string JsonString(cJSON* root) {
    char* text = cJSON_PrintUnformatted(root);
    std::string result = text ? text : "{}";
    if (text) {
        cJSON_free(text);
    }
    cJSON_Delete(root);
    return result;
}

std::string SimpleResult(bool success, const std::string& error = "") {
    cJSON* root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "success", success);
    if (!error.empty()) {
        cJSON_AddStringToObject(root, "error", error.c_str());
    }
    return JsonString(root);
}

std::string LowerAscii(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
        return ch < 128 ? static_cast<char>(std::tolower(ch)) : static_cast<char>(ch);
    });
    return value;
}

bool EndsWith(const std::string& value, const char* suffix) {
    const size_t suffix_length = strlen(suffix);
    return value.size() >= suffix_length &&
           value.compare(value.size() - suffix_length, suffix_length, suffix) == 0;
}

bool ValidatePath(const std::string& path, uint64_t& max_size, const char*& directory) {
    struct Rule {
        const char* prefix;
        const char* extension;
        uint64_t max_size;
        const char* directory;
    };
    constexpr Rule rules[] = {
        {"/sdcard/images/", ".png", kImageMaxBytes, "/sdcard/images"},
        {"/sdcard/music/", ".mp3", kAudioMaxBytes, "/sdcard/music"},
        {"/sdcard/videos/", ".avi", kVideoMaxBytes, "/sdcard/videos"},
    };
    if (path.find("..") != std::string::npos || path.find('\\') != std::string::npos) {
        return false;
    }
    for (const auto& rule : rules) {
        if (path.rfind(rule.prefix, 0) != 0 || !EndsWith(LowerAscii(path), rule.extension)) {
            continue;
        }
        const std::string filename = path.substr(strlen(rule.prefix));
        if (filename.empty() || filename.find('/') != std::string::npos) {
            return false;
        }
        max_size = rule.max_size;
        directory = rule.directory;
        return true;
    }
    return false;
}

bool EnsureDirectory(const char* directory) {
    struct stat info = {};
    if (stat(directory, &info) == 0) {
        return S_ISDIR(info.st_mode);
    }
    return mkdir(directory, 0775) == 0;
}

void RemoveStalePartFiles() {
    constexpr const char* directories[] = {"/sdcard/images", "/sdcard/music", "/sdcard/videos"};
    for (const char* directory : directories) {
        DIR* handle = opendir(directory);
        if (!handle) {
            continue;
        }
        while (dirent* entry = readdir(handle)) {
            const std::string filename = entry->d_name;
            if (!EndsWith(filename, ".part")) {
                continue;
            }
            const std::string path = std::string(directory) + "/" + filename;
            unlink(path.c_str());
        }
        closedir(handle);
    }
}

bool IsSha256(const std::string& value) {
    return value.size() == 64 && std::all_of(value.begin(), value.end(), [](unsigned char ch) {
        return std::isxdigit(ch) != 0;
    });
}

std::string HexDigest(const unsigned char digest[32]) {
    char output[65];
    for (size_t index = 0; index < 32; ++index) {
        snprintf(output + index * 2, 3, "%02x", digest[index]);
    }
    output[64] = '\0';
    return output;
}

bool HashFile(const std::string& path, std::string& digest, uint64_t& size) {
    FILE* file = fopen(path.c_str(), "rb");
    if (!file) {
        return false;
    }
    auto* buffer = static_cast<unsigned char*>(heap_caps_malloc(kIoBufferSize, MALLOC_CAP_8BIT));
    if (!buffer) {
        fclose(file);
        return false;
    }
    mbedtls_sha256_context context;
    mbedtls_sha256_init(&context);
    bool success = mbedtls_sha256_starts(&context, 0) == 0;
    size = 0;
    while (success) {
        const size_t read = fread(buffer, 1, kIoBufferSize, file);
        if (read > 0) {
            success = mbedtls_sha256_update(&context, buffer, read) == 0;
            size += read;
        }
        if (read < kIoBufferSize) {
            if (ferror(file)) {
                success = false;
            }
            break;
        }
    }
    unsigned char hash[32] = {};
    if (success) {
        success = mbedtls_sha256_finish(&context, hash) == 0;
    }
    mbedtls_sha256_free(&context);
    heap_caps_free(buffer);
    success = fclose(file) == 0 && success;
    if (success) {
        digest = HexDigest(hash);
    }
    return success;
}
}

struct SdMediaManager::DownloadJob {
    std::string url;
    std::string dest_path;
    uint64_t expected_size = 0;
    std::string sha256;
    std::string transfer_id;
    std::string ack_url;
    std::string ack_token;
};

SdMediaManager::SdMediaManager() = default;
SdMediaManager::~SdMediaManager() = default;

bool SdMediaManager::Start(SdCard* sd_card, LcdDisplay* display) {
    std::lock_guard<std::mutex> lock(mutex_);
    sd_card_ = sd_card;
    display_ = display;
    if (worker_task_) {
        return true;
    }
    if (xTaskCreate(WorkerEntry, "sd_media", kWorkerStackSize, this, 2, &worker_task_) != pdPASS) {
        worker_task_ = nullptr;
        state_ = "error";
        error_ = "failed to create worker task";
        return false;
    }
    if (sd_card_ && sd_card_->IsMounted()) {
        RemoveStalePartFiles();
    }
    return true;
}

std::string SdMediaManager::EnqueueDownload(const std::string& url,
                                            const std::string& dest_path,
                                            uint64_t expected_size,
                                            const std::string& sha256,
                                            const std::string& transfer_id,
                                            const std::string& ack_url,
                                            const std::string& ack_token) {
    uint64_t max_size = 0;
    const char* directory = nullptr;
    const std::string server_base_url = MEDIA_SERVER_BASE_URL;
    if (url.rfind("https://", 0) != 0 && url.rfind("http://", 0) != 0) {
        return SimpleResult(false, "download URL must use HTTP or HTTPS");
    }
    if (server_base_url.rfind("https://", 0) != 0 && server_base_url.rfind("http://", 0) != 0) {
        return SimpleResult(false, "MEDIA_SERVER_BASE_URL must use HTTP or HTTPS");
    }
    if (!ValidatePath(dest_path, max_size, directory)) {
        return SimpleResult(false, "dest_path is not allowed");
    }
    if (expected_size == 0 || expected_size > max_size) {
        return SimpleResult(false, "expected_size is outside the allowed limit");
    }
    if (!IsSha256(sha256) || transfer_id.empty() || ack_token.empty()) {
        return SimpleResult(false, "invalid SHA-256 or transfer ticket");
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!worker_task_) {
            return SimpleResult(false, "SD media worker is not running");
        }
        if (active_ || pending_job_) {
            return SimpleResult(false, "another SD media download is active");
        }
        std::string configured_ack_url = server_base_url;
        while (!configured_ack_url.empty() && configured_ack_url.back() == '/') {
            configured_ack_url.pop_back();
        }
        configured_ack_url += "/api/media/transfers/" + transfer_id + "/ack";
        if (!ack_url.empty() && ack_url != configured_ack_url) {
            ESP_LOGW(kTag, "Ignoring ticket ACK URL; using hardcoded MEDIA_SERVER_BASE_URL");
        }
        pending_job_ = std::make_unique<DownloadJob>(DownloadJob{
            url, dest_path, expected_size, LowerAscii(sha256), transfer_id, configured_ack_url, ack_token
        });
        active_ = true;
        state_ = "queued";
        error_.clear();
        dest_path_ = dest_path;
        transfer_id_ = transfer_id;
        bytes_written_ = 0;
        expected_size_ = expected_size;
        acknowledged_ = false;
    }
    xTaskNotifyGive(worker_task_);

    cJSON* root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "success", true);
    cJSON_AddStringToObject(root, "status", "queued");
    cJSON_AddStringToObject(root, "transfer_id", transfer_id.c_str());
    cJSON_AddStringToObject(root, "dest_path", dest_path.c_str());
    return JsonString(root);
}

std::string SdMediaManager::GetStatusJson() const {
    std::lock_guard<std::mutex> lock(mutex_);
    cJSON* root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "active", active_);
    cJSON_AddStringToObject(root, "status", state_.c_str());
    cJSON_AddStringToObject(root, "error", error_.c_str());
    cJSON_AddStringToObject(root, "dest_path", dest_path_.c_str());
    cJSON_AddStringToObject(root, "transfer_id", transfer_id_.c_str());
    cJSON_AddNumberToObject(root, "bytes_written", static_cast<double>(bytes_written_));
    cJSON_AddNumberToObject(root, "expected_size", static_cast<double>(expected_size_));
    cJSON_AddBoolToObject(root, "acknowledged", acknowledged_);
    return JsonString(root);
}

void SdMediaManager::WorkerEntry(void* argument) {
    static_cast<SdMediaManager*>(argument)->WorkerLoop();
}

void SdMediaManager::WorkerLoop() {
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        std::unique_ptr<DownloadJob> job;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            job = std::move(pending_job_);
        }
        if (job) {
            RunJob(*job);
        }
        {
            std::lock_guard<std::mutex> lock(mutex_);
            active_ = false;
        }
    }
}

void SdMediaManager::SetStatus(const std::string& state, const std::string& error,
                               uint64_t bytes_written, uint64_t expected_size,
                               const std::string& dest_path, const std::string& transfer_id,
                               bool acknowledged) {
    std::lock_guard<std::mutex> lock(mutex_);
    state_ = state;
    error_ = error;
    bytes_written_ = bytes_written;
    expected_size_ = expected_size;
    dest_path_ = dest_path;
    transfer_id_ = transfer_id;
    acknowledged_ = acknowledged;
}

void SdMediaManager::RunJob(const DownloadJob& job) {
    SetStatus("validating", "", 0, job.expected_size, job.dest_path, job.transfer_id);
    if (!sd_card_ || !sd_card_->IsMounted()) {
        SetStatus("failed", "SD card is not mounted", 0, job.expected_size, job.dest_path, job.transfer_id);
        return;
    }
    RemoveStalePartFiles();

    auto& application = Application::GetInstance();
    auto* music = application.GetSdMusic();
    if (music && music->GetState() != Esp32SdMusic::PlayerState::Stopped) {
        SetStatus("failed", "SD music player is not idle", 0, job.expected_size, job.dest_path, job.transfer_id);
        return;
    }
    auto* video = application.GetVideo();
    if (video && video->GetState() != VideoPlayerState::Idle) {
        SetStatus("failed", "video player is not idle", 0, job.expected_size, job.dest_path, job.transfer_id);
        return;
    }

    uint64_t max_size = 0;
    const char* directory = nullptr;
    if (!ValidatePath(job.dest_path, max_size, directory) || !EnsureDirectory(directory)) {
        SetStatus("failed", "destination directory is unavailable", 0, job.expected_size, job.dest_path, job.transfer_id);
        return;
    }
    uint64_t total_space = 0;
    uint64_t free_space = 0;
    if (esp_vfs_fat_info("/sdcard", &total_space, &free_space) != ESP_OK ||
            free_space < job.expected_size + 64 * 1024) {
        SetStatus("failed", "insufficient SD card free space", 0, job.expected_size, job.dest_path, job.transfer_id);
        return;
    }

    struct stat existing = {};
    if (stat(job.dest_path.c_str(), &existing) == 0) {
        std::string existing_hash;
        uint64_t existing_size = 0;
        if (!HashFile(job.dest_path, existing_hash, existing_size) ||
                existing_size != job.expected_size || existing_hash != job.sha256) {
            SetStatus("failed", "destination file already exists with different content", existing_size,
                      job.expected_size, job.dest_path, job.transfer_id);
            return;
        }
        if (!RefreshLibrary(job.dest_path)) {
            SetStatus("failed", "download exists but media library refresh failed", existing_size,
                      job.expected_size, job.dest_path, job.transfer_id);
            return;
        }
        const bool acknowledged = SendAck(job);
        SetStatus(acknowledged ? "success" : "success_ack_pending",
                  acknowledged ? "" : "download verified but ACK failed",
                  existing_size, job.expected_size, job.dest_path, job.transfer_id, acknowledged);
        return;
    }

    std::string error;
    for (int attempt = 1; attempt <= kDownloadAttempts; ++attempt) {
        bool retryable = true;
        SetStatus("downloading", "", 0, job.expected_size, job.dest_path, job.transfer_id);
        if (DownloadOnce(job, error, retryable)) {
            if (!RefreshLibrary(job.dest_path)) {
                SetStatus("failed", "media library refresh failed", job.expected_size,
                          job.expected_size, job.dest_path, job.transfer_id);
                return;
            }
            const bool acknowledged = SendAck(job);
            SetStatus(acknowledged ? "success" : "success_ack_pending",
                      acknowledged ? "" : "download verified but ACK failed",
                      job.expected_size, job.expected_size, job.dest_path, job.transfer_id, acknowledged);
            return;
        }
        ESP_LOGW(kTag, "Download attempt %d/%d failed: %s", attempt, kDownloadAttempts, error.c_str());
        if (!retryable || attempt == kDownloadAttempts) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(1000 * attempt));
    }
    SetStatus("failed", error, 0, job.expected_size, job.dest_path, job.transfer_id);
}

bool SdMediaManager::DownloadOnce(const DownloadJob& job, std::string& error, bool& retryable) {
    const std::string part_path = job.dest_path + ".part";
    unlink(part_path.c_str());

    auto network = Board::GetInstance().GetNetwork();
    if (!network) {
        error = "network is unavailable";
        return false;
    }
    auto http = network->CreateHttp(0);
    http->SetTimeout(kHttpTimeoutMs);
    if (!http->Open("GET", job.url)) {
        error = "failed to open download URL";
        return false;
    }
    if (http->GetStatusCode() != 200) {
        error = "HTTP status " + std::to_string(http->GetStatusCode());
        http->Close();
        return false;
    }
    const size_t content_length = http->GetBodyLength();
    if (content_length != 0 && content_length != job.expected_size) {
        error = "Content-Length does not match expected_size";
        http->Close();
        return false;
    }

    FILE* file = fopen(part_path.c_str(), "wb");
    if (!file) {
        error = "failed to open temporary SD file";
        retryable = false;
        http->Close();
        return false;
    }
    auto* buffer = static_cast<unsigned char*>(heap_caps_malloc(kIoBufferSize, MALLOC_CAP_8BIT));
    if (!buffer) {
        error = "failed to allocate download buffer";
        retryable = false;
        fclose(file);
        http->Close();
        unlink(part_path.c_str());
        return false;
    }

    mbedtls_sha256_context context;
    mbedtls_sha256_init(&context);
    bool okay = mbedtls_sha256_starts(&context, 0) == 0;
    uint64_t total = 0;
    while (okay) {
        const int read = http->Read(reinterpret_cast<char*>(buffer), kIoBufferSize);
        if (read < 0) {
            error = "HTTP read failed";
            okay = false;
            break;
        }
        if (read == 0) {
            break;
        }
        if (total + static_cast<uint64_t>(read) > job.expected_size) {
            error = "download exceeds expected_size";
            okay = false;
            break;
        }
        if (fwrite(buffer, 1, read, file) != static_cast<size_t>(read)) {
            error = "SD card write failed";
            retryable = false;
            okay = false;
            break;
        }
        okay = mbedtls_sha256_update(&context, buffer, read) == 0;
        total += read;
        SetStatus("downloading", "", total, job.expected_size, job.dest_path, job.transfer_id);
        taskYIELD();
    }
    http->Close();

    unsigned char digest[32] = {};
    if (okay) {
        okay = mbedtls_sha256_finish(&context, digest) == 0;
    }
    mbedtls_sha256_free(&context);
    heap_caps_free(buffer);

    if (okay && total != job.expected_size) {
        error = "download ended before expected_size";
        okay = false;
    }
    if (okay && HexDigest(digest) != job.sha256) {
        error = "SHA-256 mismatch";
        okay = false;
    }
    if (okay && fflush(file) != 0) {
        error = "fflush failed";
        retryable = false;
        okay = false;
    }
    if (okay && fsync(fileno(file)) != 0) {
        error = "fsync failed";
        retryable = false;
        okay = false;
    }
    if (fclose(file) != 0) {
        error = "fclose failed";
        retryable = false;
        okay = false;
    }
    if (!okay) {
        unlink(part_path.c_str());
        return false;
    }
    if (rename(part_path.c_str(), job.dest_path.c_str()) != 0) {
        error = "atomic rename failed";
        retryable = false;
        unlink(part_path.c_str());
        return false;
    }
    return true;
}

bool SdMediaManager::RefreshLibrary(const std::string& dest_path) {
    auto& application = Application::GetInstance();
    if (dest_path.rfind("/sdcard/music/", 0) == 0) {
        auto* music = application.GetSdMusic();
        return music && music->RebuildPlaylist();
    }
    if (dest_path.rfind("/sdcard/videos/", 0) == 0) {
        auto* video = application.GetVideo();
        return video && video->ScanDirectory() > 0;
    }
    return true;
}

bool SdMediaManager::SendAck(const DownloadJob& job) {
    cJSON* root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "token", job.ack_token.c_str());
    cJSON_AddNumberToObject(root, "bytes_written", static_cast<double>(job.expected_size));
    cJSON_AddStringToObject(root, "sha256", job.sha256.c_str());
    cJSON_AddStringToObject(root, "dest_path", job.dest_path.c_str());
    const std::string body = JsonString(root);

    for (int attempt = 1; attempt <= kAckAttempts; ++attempt) {
        auto network = Board::GetInstance().GetNetwork();
        if (!network) {
            return false;
        }
        auto http = network->CreateHttp(0);
        http->SetTimeout(kHttpTimeoutMs);
        http->SetHeader("Content-Type", "application/json");
        http->SetContent(std::string(body));
        const bool opened = http->Open("POST", job.ack_url);
        const int status = opened ? http->GetStatusCode() : 0;
        if (opened) {
            http->ReadAll();
        }
        http->Close();
        if (opened && status >= 200 && status < 300) {
            return true;
        }
        ESP_LOGW(kTag, "ACK attempt %d/%d failed with HTTP %d", attempt, kAckAttempts, status);
        vTaskDelay(pdMS_TO_TICKS(1000 * attempt));
    }
    return false;
}

std::string SdMediaManager::ShowImage(const std::string& query) {
    if (!sd_card_ || !sd_card_->IsMounted() || !display_) {
        return SimpleResult(false, "SD card is not mounted");
    }
    const std::string search = LowerAscii(query);
    if (search.empty()) {
        return SimpleResult(false, "query is required");
    }

    DIR* directory = opendir("/sdcard/images");
    if (!directory) {
        return SimpleResult(false, "image directory is unavailable");
    }
    std::vector<std::string> exact;
    std::vector<std::string> partial;
    while (dirent* entry = readdir(directory)) {
        const std::string filename = entry->d_name;
        const std::string lower = LowerAscii(filename);
        if (!EndsWith(lower, ".png")) {
            continue;
        }
        const std::string stem = lower.substr(0, lower.size() - 4);
        if (stem == search) {
            exact.push_back(filename);
        } else if (stem.find(search) != std::string::npos) {
            partial.push_back(filename);
        }
    }
    closedir(directory);
    const auto& matches = exact.empty() ? partial : exact;
    if (matches.empty()) {
        return SimpleResult(false, "image not found");
    }
    if (matches.size() != 1) {
        cJSON* root = cJSON_CreateObject();
        cJSON_AddBoolToObject(root, "success", false);
        cJSON_AddStringToObject(root, "error", "ambiguous");
        cJSON* candidates = cJSON_AddArrayToObject(root, "candidates");
        for (const auto& filename : matches) {
            cJSON_AddItemToArray(candidates, cJSON_CreateString(filename.c_str()));
        }
        return JsonString(root);
    }

    const std::string path = "/sdcard/images/" + matches.front();
    struct stat info = {};
    if (stat(path.c_str(), &info) != 0 || info.st_size <= 0 || info.st_size > kImageMaxBytes) {
        return SimpleResult(false, "invalid PNG size");
    }
    FILE* file = fopen(path.c_str(), "rb");
    if (!file) {
        return SimpleResult(false, "failed to open PNG");
    }
    void* data = heap_caps_malloc(info.st_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!data) {
        fclose(file);
        return SimpleResult(false, "insufficient PSRAM for PNG");
    }
    const bool read_ok = fread(data, 1, info.st_size, file) == static_cast<size_t>(info.st_size);
    const bool close_ok = fclose(file) == 0;
    if (!read_ok || !close_ok) {
        heap_caps_free(data);
        return SimpleResult(false, "failed to read PNG");
    }
    std::unique_ptr<LvglAllocatedImage> image;
    try {
        image = std::make_unique<LvglAllocatedImage>(data, info.st_size);
    } catch (const std::exception& exception) {
        heap_caps_free(data);
        return SimpleResult(false, exception.what());
    }
    display_->SetPreviewImage(std::move(image));
    display_->KeepPreviewVisible(true);

    cJSON* root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "success", true);
    cJSON_AddStringToObject(root, "path", path.c_str());
    return JsonString(root);
}

std::string SdMediaManager::PrepareDownload(const std::string& query) {
    auto network = Board::GetInstance().GetNetwork();
    if (!network) {
        return SimpleResult(false, "network is unavailable");
    }

    cJSON* root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "query", query.c_str());
    std::string body = JsonString(root);

    std::string url = MEDIA_SERVER_BASE_URL;
    while (!url.empty() && url.back() == '/') {
        url.pop_back();
    }
    url += "/api/media/prepare-download";

    auto http = network->CreateHttp(0);
    http->SetTimeout(kHttpTimeoutMs);
    http->SetHeader("Content-Type", "application/json");
    http->SetContent(std::move(body));

    if (!http->Open("POST", url)) {
        http->Close();
        return SimpleResult(false, "failed to connect to media server");
    }

    int status = http->GetStatusCode();
    std::string response = http->ReadAll();
    http->Close();

    if (status != 200) {
        cJSON* err_json = cJSON_Parse(response.c_str());
        if (err_json) {
            cJSON* detail = cJSON_GetObjectItem(err_json, "detail");
            if (detail && cJSON_IsString(detail)) {
                std::string err_msg = detail->valuestring;
                cJSON_Delete(err_json);
                return SimpleResult(false, err_msg);
            }
            cJSON_Delete(err_json);
        }
        return SimpleResult(false, "media server returned HTTP " + std::to_string(status));
    }

    return response;
}
