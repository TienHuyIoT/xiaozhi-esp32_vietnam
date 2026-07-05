#ifndef FLASH_FATFS_H
#define FLASH_FATFS_H

#include "sd_card.h"
#include <esp_vfs_fat.h>
#include <wear_levelling.h>

class FlashFatFs : public SdCard {
public:
    FlashFatFs();
    virtual ~FlashFatFs();

    virtual esp_err_t Initialize() override;
    virtual esp_err_t Deinitialize() override;
    virtual bool IsMounted() const override;
    virtual const char* GetMountPoint() const override;
    virtual void PrintCardInfo() const override;
    virtual esp_err_t WriteFile(const char* path, const char* data) override;
    virtual esp_err_t ReadFile(const char* path, char* buffer, size_t buffer_size) override;
    virtual esp_err_t DeleteFile(const char* path) override;
    virtual esp_err_t RenameFile(const char* old_path, const char* new_path) override;
    virtual bool FileExists(const char* path) override;
    virtual esp_err_t Format() override;

private:
    wl_handle_t wl_handle_;
};

#endif // FLASH_FATFS_H
