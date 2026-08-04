---
name: code-review
description: Hướng dẫn review code cho dự án firmware nhúng (ESP32/ESP8266). Dùng khi xem PR, kiểm tra thay đổi liên quan đến phần cứng, RTOS, bộ nhớ, flash/OTA và an ninh.
---

# Embedded Firmware Code Review (ESP)

Hướng dẫn này tập trung vào các điểm quan trọng khi review firmware nhúng trên nền ESP (C / C++ / FreeRTOS / esp-idf / Arduino).

## Checklist nhanh

- Build & toolchain
  - Có cảnh báo compile không hợp lý không? (tắt hoặc sửa, không im lặng lỗi)
  - Sử dụng đúng flags tối ưu/strip (size) và không để secrets trong biến môi trường build.
- Runtime & stability
  - Rủi ro null / buffer overflow / integer overflow
  - Nguy cơ stack overflow (task stack sizes đủ không?)
  - Sử dụng heap đúng cách (malloc/free trên đường trục thời gian thực?)
- RTOS / concurrency
  - Không block trong ISR; dùng queue/semaphore/notification để chuyển tác vụ
  - Dùng API FreeRTOS đúng: xHigherPriorityTaskWoken, từ ISR dùng FromISR variants
  - Kiểm tra deadlock, priority inversion, watchdog feeding
- ISR & timing
  - ISR ngắn, tránh gọi hàm không an toàn (printf, malloc), thích hợp đặt IRAM_ATTR khi cần
  - Đảm bảo atomic access cho biến chia sẻ (volatile + guards)
- Peripherals & DMA
  - Bộ nhớ cho DMA không nằm trong vùng cache nếu cần; kiểm tra alignment
  - I2C/SPI/ADC được cấu hình và xử lý lỗi bus (nack, timeouts)
- Memory & flash
  - Kiểm tra leak, fragmentation, và footprint (flash + RAM)
  - Partition table/OTA: thay đổi partition có được cân nhắc không?
  - Không lưu secrets (API keys, Wi‑Fi pass) ở plaintext trong repo/firmware images
- Power & low-power modes
  - Deep sleep / light sleep: wake sources, state save/restore, RTC usage
  - Kiểm tra leakage / peripheral states khi wake
- OTA & boot
  - OTA có xác thực/validate signature không? rollback plan rõ ràng?
  - Kiểm tra bootloader, partition table, flash wear consideration
- Security
  - TLS: certificate validation, no insecure cipher, no skipping cert checks
  - Secure boot / flash encryption (nếu yêu cầu)
  - Không để hardcoded passwords, tokens, debug backdoors
- Performance & efficiency
  - Tránh busy loops trên main or high-priority tasks
  - Logging: kiểm soát mức độ log, tránh ghi flash quá nhiều
- Tests & CI
  - Unit tests (hosted hoặc firmware unit), integration/hardware tests, long-run tests
  - CI: build, unit test, static analysis, optionally auto-flash test device or emulator
- Long-term impact
  - Thay đổi partition/bootloader/ABI, driver/hardware pinout, hoặc đổi vendor library → yêu cầu review sâu và release notes

## Thiết kế & đánh giá thay đổi

- Thay đổi driver hoặc cấu hình pin: kiểm tra ảnh hưởng với board khác, document pinout.
- Thêm thư viện/binary bậc thấp: xem license, kích thước, phụ thuộc, maintenance.
- Thay đổi API public: có migration plan cho phần mềm khác dùng firmware không?

## Test coverage (đối với firmware)

- Unit tests cho logic (có thể mock HAL/driver).
- Integration tests trên phần cứng (smoke tests: boot, wifi connect, sensor read, OTA).
- Long-run tests (power cycle, memory leak, stability > X hours).
- Hardware-in-the-loop hoặc CI có target device để flash & run sanity checks (nếu có khả năng).

## Công cụ & static analysis

- Chạy: -Wall -Wextra -Werror (thận trọng với -Werror trên CI)
- Static analyzers: clang‑tidy, cppcheck, Coverity (nếu có)
- Size analysis: map/size report, strip symbols cho release
- Fuzzer / sanitizers: nếu có khả năng build host‑side tests với ASAN/UBSAN

## Mẫu lỗi phổ biến & ví dụ

- Không an toàn trong ISR (ví dụ gọi malloc/printf): tránh, dùng queue hoặc ghi vào buffer tĩnh.
- Sử dụng bộ nhớ cache cho DMA trên ESP32: đảm bảo dùng vùng không cache hoặc flush/invalidate cache khi cần.
- Quên thêm watchdog reset khi task dài → device reboot ngẫu nhiên.

Ví dụ C/C++ (ISR safe):
```c
// BAD: gọi malloc trong ISR
void IRAM_ATTR gpio_isr_handler(void* arg) {
    char *buf = malloc(64); // nguy hiểm
    ...
}

// GOOD: push event vào queue
void IRAM_ATTR gpio_isr_handler(void* arg) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_queue, &gpio_num, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}
