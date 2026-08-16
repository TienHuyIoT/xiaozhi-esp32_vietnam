# HƯỚNG DẪN NẠP FIRMWARE ROBOT SONIC (ESP32-S3)

Bản firmware này là file tổng hợp **All-in-One (`sonic_robot_firmware_all_in_one_16MB.bin`)** đã bao gồm:
* Bootloader (`0x0`)
* Bảng phân vùng Partition Table (`0x8000`)
* Dữ liệu khởi tạo OTA (`0xd000`)
* Chương trình chính Sonic App (`0x20000`)
* Bộ Emoji / Giao diện đồ họa (`0x800000`)

---

## Cách 1: Nạp nhanh bằng 1-Click Script (Khuyên dùng trên Windows)
1. Cắm Robot vào máy tính qua cáp Type-C.
2. Kiểm tra cổng COM trong **Device Manager** (ví dụ: `COM5`).
3. Nhấp đúp vào file **`flash_robot.bat`**.
4. Nhập cổng COM và nhấn Enter $\rightarrow$ Script sẽ tự động nạp toàn bộ vào robot ở địa chỉ `0x0`.

---

## Cách 2: Nạp qua Trình duyệt Web (ESP Web Flasher - Không cần cài phần mềm)
1. Dùng trình duyệt Chrome hoặc Edge, truy cập: **[https://espressif.github.io/esptool-js/](https://espressif.github.io/esptool-js/)**
2. Bấm **Connect** $\rightarrow$ Chọn cổng COM của robot $\rightarrow$ Bấm Kết nối.
3. Cấu hình nạp:
   * **Baudrate:** `921600`
   * **Flash Mode:** `DIO`
   * **Flash Size:** `16MB`
4. Ở dòng nạp file:
   * **Offset (Địa chỉ):** `0x0`
   * **File:** Chọn file `sonic_robot_firmware_all_in_one_16MB.bin`
5. Bấm **Program** và chờ thanh tiến trình chạy đến 100%.

---

## Cách 3: Nạp bằng Lệnh Terminal / Command Line (`esptool.py`)

Chạy lệnh sau trong PowerShell hoặc Command Prompt:

```powershell
python -m esptool --chip esp32s3 -p COM5 -b 921600 --before default_reset --after hard_reset write_flash --flash_mode dio --flash_size 16MB --flash_freq 80m 0x0 sonic_robot_firmware_all_in_one_16MB.bin
```
*(Thay `COM5` bằng cổng COM thực tế của robot)*.

---

## Cấu hình sau khi nạp xong:
1. Sau khi nạp xong, màn hình robot sẽ hiển thị giao diện Sonic và phát âm thanh chào mừng.
2. Nếu robot chưa có WiFi: Nhấn giữ nút hoặc cấu hình WiFi qua AP `Sonic-Robot-Setup` (mật khẩu mặc định nếu có).
3. Robot sẽ tự động kết nối về máy chủ Sonic Backend và sẵn sàng hoạt động!
