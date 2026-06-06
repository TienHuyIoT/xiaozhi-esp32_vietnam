#include "file_browser_scan.h"

#include <ctype.h>
#include <dirent.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

extern "C" {
#include "esp_log.h"
#include "freertos/task.h"
}

namespace {
constexpr const char* kTag = "FB_SCAN";

class ScanService final {
 public:
  static ScanService& Instance() {
    static ScanService instance;
    return instance;
  }

  void Init() {
    if (queue_ == nullptr) {
      queue_ = xQueueCreate(8, sizeof(file_scan_event_t));
    }
    if (results_ == nullptr) {
      capacity_ = 128;
      results_ = static_cast<file_item_t*>(malloc(sizeof(file_item_t) * capacity_));
      count_ = 0;
    }
  }

  void Start(const char* folder_path) {
    if (folder_path == nullptr || busy_) {
      return;
    }

    char* path_copy = static_cast<char*>(malloc(strlen(folder_path) + 1));
    if (path_copy == nullptr) {
      SendError(ENOMEM, "malloc failed");
      return;
    }
    strcpy(path_copy, folder_path);

    cancel_ = false;
    busy_ = true;
    BaseType_t ok = xTaskCreate(ScanTaskEntry, "fb_scan", 4096, path_copy, 5, &task_);
    if (ok != pdPASS) {
      free(path_copy);
      busy_ = false;
      SendError(ENOMEM, "xTaskCreate failed");
    }
  }

  void Cancel() {
    cancel_ = true;
  }

  const file_item_t* Results(uint32_t* out_count) const {
    if (out_count != nullptr) {
      *out_count = count_;
    }
    return results_;
  }

  QueueHandle_t Queue() const {
    return queue_;
  }

 private:
  ScanService()
      : queue_(nullptr),
        task_(nullptr),
        results_(nullptr),
        count_(0),
        capacity_(0),
        cancel_(false),
        busy_(false) {}

  static void ScanTaskEntry(void* arg) {
    char* path = static_cast<char*>(arg);
    ScanService::Instance().RunScan(path);
    free(path);
    vTaskDelete(nullptr);
  }

  file_type_t DetectType(const char* filename) const {
    if (filename == nullptr) {
      return FILE_TYPE_UNSUPPORTED;
    }
    const char* dot = strrchr(filename, '.');
    if (dot == nullptr || *(dot + 1) == '\0') {
      return FILE_TYPE_UNSUPPORTED;
    }

    char ext[16] = {0};
    for (int i = 0; i < static_cast<int>(sizeof(ext) - 1) && dot[1 + i] != '\0'; ++i) {
      ext[i] = static_cast<char>(tolower(dot[1 + i]));
    }

    if (strcmp(ext, "mp3") == 0 || strcmp(ext, "wav") == 0 || strcmp(ext, "flac") == 0 ||
        strcmp(ext, "aac") == 0 || strcmp(ext, "ogg") == 0) {
      return FILE_TYPE_AUDIO;
    }
    if (strcmp(ext, "jpg") == 0 || strcmp(ext, "jpeg") == 0 || strcmp(ext, "png") == 0 ||
        strcmp(ext, "gif") == 0 || strcmp(ext, "bmp") == 0) {
      return FILE_TYPE_IMAGE;
    }
    if (strcmp(ext, "mp4") == 0 || strcmp(ext, "avi") == 0 || strcmp(ext, "mkv") == 0 ||
        strcmp(ext, "mov") == 0) {
      return FILE_TYPE_VIDEO;
    }
    if (strcmp(ext, "txt") == 0 || strcmp(ext, "json") == 0 || strcmp(ext, "yaml") == 0 ||
        strcmp(ext, "yml") == 0 || strcmp(ext, "ini") == 0 || strcmp(ext, "cfg") == 0 ||
        strcmp(ext, "log") == 0) {
      return FILE_TYPE_DOCUMENT;
    }
    if (strcmp(ext, "bin") == 0 || strcmp(ext, "elf") == 0) {
      return FILE_TYPE_EXECUTABLE;
    }
    return FILE_TYPE_UNSUPPORTED;
  }

  bool EnsureCapacity(uint32_t required) {
    if (required <= capacity_) {
      return true;
    }

    uint32_t new_capacity = (capacity_ == 0) ? 128 : capacity_;
    while (new_capacity < required) {
      new_capacity *= 2;
    }

    file_item_t* resized = static_cast<file_item_t*>(
        realloc(results_, sizeof(file_item_t) * new_capacity));
    if (resized == nullptr) {
      return false;
    }

    results_ = resized;
    capacity_ = new_capacity;
    return true;
  }

  void SendProgress(uint32_t loaded, uint32_t total) {
    if (queue_ == nullptr) {
      return;
    }
    file_scan_event_t evt = {};
    evt.type = SCAN_EVT_PROGRESS;
    evt.loaded_files = loaded;
    evt.total_files = total;
    xQueueSend(queue_, &evt, 0);
  }

  void SendComplete(uint32_t total) {
    if (queue_ == nullptr) {
      return;
    }
    file_scan_event_t evt = {};
    evt.type = SCAN_EVT_COMPLETE;
    evt.loaded_files = total;
    evt.total_files = total;
    xQueueSend(queue_, &evt, 0);
  }

  void SendError(int code, const char* message) {
    if (queue_ == nullptr) {
      return;
    }
    file_scan_event_t evt = {};
    evt.type = SCAN_EVT_ERROR;
    evt.error_code = code;
    snprintf(evt.error_msg, sizeof(evt.error_msg), "%s", message);
    xQueueSend(queue_, &evt, 0);
  }

  void RunScan(const char* folder_path) {
    DIR* dir = opendir(folder_path);
    if (dir == nullptr) {
      SendError(errno, "open dir failed");
      busy_ = false;
      task_ = nullptr;
      return;
    }

    count_ = 0;
    uint32_t batch = 0;
    struct dirent* entry = nullptr;
    while (!cancel_ && (entry = readdir(dir)) != nullptr) {
      if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) {
        continue;
      }

      if (!EnsureCapacity(count_ + 1)) {
        SendError(ENOMEM, "realloc failed");
        break;
      }

      file_item_t* item = &results_[count_];
      memset(item, 0, sizeof(file_item_t));
      snprintf(item->name, sizeof(item->name), "%s", entry->d_name);
      snprintf(item->full_path, sizeof(item->full_path), "%s/%s", folder_path, entry->d_name);

      struct stat st;
      if (stat(item->full_path, &st) == 0) {
        bool is_dir = S_ISDIR(st.st_mode);
        item->type = is_dir ? FILE_TYPE_FOLDER : DetectType(item->name);
        item->size_bytes = is_dir ? 0 : static_cast<uint32_t>(st.st_size);
        item->date_unix = static_cast<uint32_t>(st.st_mtime);
      } else {
        item->type = DetectType(item->name);
        item->size_bytes = 0;
        item->date_unix = 0;
      }
      item->is_accessible = true;

      ++count_;
      ++batch;
      if (batch >= FILE_BROWSER_SCAN_BATCH_SIZE) {
        SendProgress(count_, 0);
        batch = 0;
        vTaskDelay(pdMS_TO_TICKS(FILE_BROWSER_SCAN_YIELD_MS));
      }
    }

    closedir(dir);
    if (!cancel_) {
      SendComplete(count_);
      ESP_LOGI(kTag, "Scan complete: %lu", static_cast<unsigned long>(count_));
    }
    busy_ = false;
    task_ = nullptr;
  }

  QueueHandle_t queue_;
  TaskHandle_t task_;
  file_item_t* results_;
  uint32_t count_;
  uint32_t capacity_;
  bool cancel_;
  bool busy_;
};
}  // namespace

extern "C" void file_browser_scan_init(void) { ScanService::Instance().Init(); }

extern "C" void file_browser_scan_folder(const char* folder_path) {
  ScanService::Instance().Start(folder_path);
}

extern "C" void file_browser_scan_cancel(void) {
  ScanService::Instance().Cancel();
}

extern "C" const file_item_t* file_browser_scan_get_results(uint32_t* out_count) {
  return ScanService::Instance().Results(out_count);
}

extern "C" QueueHandle_t file_browser_scan_get_event_queue(void) {
  return ScanService::Instance().Queue();
}