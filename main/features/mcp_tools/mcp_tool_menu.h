#pragma once

#include "display/lvgl_display/lvgl_image.h"

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#ifndef CONFIG_USE_EMOTE_MESSAGE_STYLE
#include <lvgl.h>
#endif

struct McpToolIllustration {
    std::string url;
    std::string sha256;
    uint32_t bytes = 0;
    uint16_t width = 0;
    uint16_t height = 0;
    bool valid = false;
};

struct McpToolEntry {
    std::string name;
    std::string title;
    std::string description;
    bool selectable = false;
    McpToolIllustration illustration;
};

class McpToolMenu {
public:
    using InvokeHandler = std::function<void(
        const std::string& tool_name,
        const std::string& revision,
        const std::string& request_id)>;

    McpToolMenu();
    ~McpToolMenu();

    McpToolMenu(const McpToolMenu&) = delete;
    McpToolMenu& operator=(const McpToolMenu&) = delete;

    bool IsOpen() const { return open_.load(); }
    void SetInvokeHandler(InvokeHandler handler);
    void SetConnectionState(bool online);
    void SetCatalog(std::string revision, std::vector<McpToolEntry> tools,
                    bool ready, bool truncated);
    void SetResult(const std::string& request_id, bool success,
                   const std::string& message);

    void Toggle();
    void Close();
    void Next();
    void Previous();
    void TripleClick();

    // Fixed-size queue payload; public only so the translation-unit decoder
    // helper can consume it without retaining a pointer to the menu.
    struct ImageRequest {
        uint32_t generation;
        uint32_t expected_bytes;
        char revision[65];
        char sha256[65];
        char url[1025];
    };

private:
    enum class ViewState : uint8_t {
        kBrowse,
        kConfirm,
        kRunning,
        kResult,
    };

    struct CacheEntry {
        std::string sha256;
        std::unique_ptr<LvglAllocatedImage> image;
        uint32_t last_used = 0;
    };

    static void ImageWorkerEntry(void* arg);
    static void AnimateTranslate(void* object, int32_t value);
    static void AnimateOpacity(void* object, int32_t value);
    void ImageWorkerLoop();
    void Move(int direction);
    void BuildUiLocked();
    void RenderLocked(int direction = 0);
    void HideLocked();
    void AnimateCardLocked(int direction);
    void ScheduleImagesLocked();
    void QueueImageLocked(const McpToolEntry& tool);
    void ApplyCachedImageLocked(const std::string& sha256);
    void DetachImageLocked();
    void ClearCacheLocked();
    void FreePendingRequests();
    std::string NewRequestId() const;

    mutable std::mutex mutex_;
    std::atomic_bool open_{false};
    std::atomic_bool stop_worker_{false};
    std::atomic_bool worker_running_{false};
    bool ready_ = false;
    bool online_ = false;
    bool truncated_ = false;
    size_t selected_index_ = 0;
    ViewState view_state_ = ViewState::kBrowse;
    std::string revision_;
    std::vector<McpToolEntry> tools_;
    std::string active_request_id_;
    std::string result_message_;
    bool result_success_ = false;
    InvokeHandler invoke_handler_;
    QueueHandle_t image_queue_ = nullptr;
    TaskHandle_t image_worker_task_ = nullptr;
    uint32_t image_generation_ = 0;
    uint32_t cache_clock_ = 0;
    std::vector<CacheEntry> image_cache_;

#ifndef CONFIG_USE_EMOTE_MESSAGE_STYLE
    lv_obj_t* overlay_ = nullptr;
    lv_obj_t* card_ = nullptr;
    lv_obj_t* image_ = nullptr;
    lv_obj_t* placeholder_ = nullptr;
    lv_obj_t* placeholder_icon_ = nullptr;
    lv_obj_t* title_label_ = nullptr;
    lv_obj_t* name_pill_ = nullptr;
    lv_obj_t* name_label_ = nullptr;
    lv_obj_t* summary_label_ = nullptr;
    lv_obj_t* status_label_ = nullptr;
    lv_obj_t* left_chevron_ = nullptr;
    lv_obj_t* right_chevron_ = nullptr;
    lv_obj_t* left_hint_ = nullptr;
    lv_obj_t* counter_label_ = nullptr;
    lv_obj_t* right_hint_ = nullptr;
#endif
};
