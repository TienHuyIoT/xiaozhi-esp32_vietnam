#include "flash_fatfs.h"
#include <esp_log.h>
#include <esp_vfs_fat.h>
#include <sys/stat.h>

#define TAG "FlashFatFs"

FlashFatFs::FlashFatFs() {
    is_mounted_ = false;
    wl_handle_ = WL_INVALID_HANDLE;
}

FlashFatFs::~FlashFatFs() {
    Deinitialize();
}

esp_err_t FlashFatFs::Initialize() {
    if (is_mounted_) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Mounting FATFS to /sdcard from internal flash partition 'storage'");

    const esp_vfs_fat_mount_config_t mount_config = {
        .format_if_mount_failed = true,
        .max_files = 10,
        .allocation_unit_size = 4096,
        .disk_status_check_enable = false
    };

    esp_err_t ret = esp_vfs_fat_spiflash_mount_rw_wl("/sdcard", "storage", &mount_config, &wl_handle_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount FATFS (%s)", esp_err_to_name(ret));
        return ret;
    }

    is_mounted_ = true;
    ESP_LOGI(TAG, "FATFS mounted successfully");
    return ESP_OK;
}

esp_err_t FlashFatFs::Deinitialize() {
    if (!is_mounted_) {
        return ESP_OK;
    }

    esp_err_t ret = esp_vfs_fat_spiflash_unmount_rw_wl("/sdcard", wl_handle_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to unmount FATFS (%s)", esp_err_to_name(ret));
        return ret;
    }

    wl_handle_ = WL_INVALID_HANDLE;
    is_mounted_ = false;
    ESP_LOGI(TAG, "FATFS unmounted successfully");
    return ESP_OK;
}

bool FlashFatFs::IsMounted() const {
    return is_mounted_;
}

const char* FlashFatFs::GetMountPoint() const {
    return "/sdcard";
}

void FlashFatFs::PrintCardInfo() const {
    if (!is_mounted_) {
        ESP_LOGW(TAG, "PrintCardInfo: FATFS not mounted.");
        return;
    }
    
    uint64_t total_bytes = 0, free_bytes = 0;
    esp_err_t err = esp_vfs_fat_info("/sdcard", &total_bytes, &free_bytes);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "FATFS Size: %llu MB, Free: %llu MB", total_bytes / (1024 * 1024), free_bytes / (1024 * 1024));
    } else {
        ESP_LOGE(TAG, "Failed to get FATFS info (%s)", esp_err_to_name(err));
    }
}

// Basic file operations - stubbed or simplified for now since we rely on VFS standard functions (fopen etc)
esp_err_t FlashFatFs::WriteFile(const char* path, const char* data) { return ESP_FAIL; }
esp_err_t FlashFatFs::ReadFile(const char* path, char* buffer, size_t buffer_size) { return ESP_FAIL; }
esp_err_t FlashFatFs::DeleteFile(const char* path) { return ESP_FAIL; }
esp_err_t FlashFatFs::RenameFile(const char* old_path, const char* new_path) { return ESP_FAIL; }
bool FlashFatFs::FileExists(const char* path) { return false; }
esp_err_t FlashFatFs::Format() { return ESP_FAIL; }
