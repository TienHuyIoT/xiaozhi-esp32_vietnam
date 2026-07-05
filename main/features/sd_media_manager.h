#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

class LcdDisplay;
class SdCard;

class SdMediaManager {
public:
    SdMediaManager();
    ~SdMediaManager();

    bool Start(SdCard* sd_card, LcdDisplay* display);
    std::string EnqueueDownload(const std::string& url,
                                const std::string& dest_path,
                                uint64_t expected_size,
                                const std::string& sha256,
                                const std::string& transfer_id,
                                const std::string& ack_url,
                                const std::string& ack_token);
    std::string GetStatusJson() const;
    std::string ShowImage(const std::string& query);
    std::string PrepareDownload(const std::string& query);

private:
    struct DownloadJob;

    static void WorkerEntry(void* argument);
    void WorkerLoop();
    void RunJob(const DownloadJob& job);
    bool DownloadOnce(const DownloadJob& job, std::string& error, bool& retryable);
    bool SendAck(const DownloadJob& job);
    bool RefreshLibrary(const std::string& dest_path);
    void SetStatus(const std::string& state, const std::string& error,
                   uint64_t bytes_written, uint64_t expected_size,
                   const std::string& dest_path, const std::string& transfer_id,
                   bool acknowledged = false);

    SdCard* sd_card_ = nullptr;
    LcdDisplay* display_ = nullptr;
    TaskHandle_t worker_task_ = nullptr;
    std::unique_ptr<DownloadJob> pending_job_;

    mutable std::mutex mutex_;
    bool active_ = false;
    std::string state_ = "idle";
    std::string error_;
    std::string dest_path_;
    std::string transfer_id_;
    uint64_t bytes_written_ = 0;
    uint64_t expected_size_ = 0;
    bool acknowledged_ = false;
};
