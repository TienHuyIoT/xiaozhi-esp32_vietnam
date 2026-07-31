# Báo thức

## Trạng thái

Báo thức được khởi tạo trong `Application::Start()` ở mọi profile, không có Kconfig toggle riêng. `AlarmManager` kiểm tra theo giây trong vòng lặp chính và đăng ký 7 MCP tools. Giờ được đọc bằng giờ địa phương, vì vậy thiết bị cần đồng bộ NTP trước khi đặt báo thức.

## Âm thanh có sẵn

Firmware hiện có đúng 4 ID âm thanh nhúng:

| `sound_id` | Ý nghĩa |
|---|---|
| `alarm` | Chuông mặc định |
| `school` | Nhắc đi học/đi làm |
| `wakeup` | Nhắc thức dậy |
| `medicine` | Nhắc uống thuốc |

Âm thanh được đóng gói qua cấu hình ngôn ngữ/assets; người dùng không cần chép file âm thanh vào thẻ SD.

## Sử dụng qua MCP

### Đặt báo thức

Ví dụ người dùng nói “Đặt báo thức 7 giờ đi học”:

```json
{
  "name": "self.alarm.set",
  "arguments": {
    "hour": 7,
    "minute": 0,
    "message": "đi học",
    "repeated": false,
    "sound_id": "school"
  }
}
```

`hour` nằm trong 0–23, `minute` trong 0–59. `repeated=true` lặp mỗi ngày; `false` chỉ chạy một lần. Nếu không chọn âm thanh, dùng `alarm`.

### Xem, hủy và đổi âm thanh

Các tình huống sử dụng giọng nói thực tế:

- **Xem danh sách báo thức:**
  - Người dùng: “Cho tôi xem danh sách báo thức” hoặc “Có những báo thức nào đã đặt?”
  - AI gọi: `{"name": "self.alarm.list", "arguments": {}}`
  - *Lưu ý:* `self.alarm.list` trả về các trường `time`, `message`, `sound_id`, `enabled`, `repeated`, bao gồm cả các mục đang ở trạng thái disabled.

- **Xem báo thức tiếp theo:**
  - Người dùng: “Báo thức tiếp theo là mấy giờ?” hoặc “Báo thức sắp tới”
  - AI gọi: `{"name": "self.alarm.next", "arguments": {}}`

- **Xem danh sách âm thanh báo thức có sẵn:**
  - Người dùng: “Có những âm thanh báo thức nào?” hoặc “Danh sách tiếng chuông báo thức”
  - AI gọi: `{"name": "self.alarm.available_sounds", "arguments": {}}`

- **Đổi âm thanh báo thức:**
  - Người dùng: “Đổi chuông báo thức 7 giờ thành tiếng nhắc đi học”
  - AI gọi:
    ```json
    {
      "name": "self.alarm.set_sound",
      "arguments": { "hour": 7, "minute": 0, "sound_id": "school" }
    }
    ```

- **Hủy một báo thức cụ thể:**
  - Người dùng: “Tắt báo thức 7 giờ” hoặc “Hủy báo thức 7 giờ đi học”
  - AI gọi:
    ```json
    {
      "name": "self.alarm.cancel",
      "arguments": { "hour": 7, "minute": 0 }
    }
    ```

- **Hủy tất cả báo thức:**
  - Người dùng: “Hủy tất cả báo thức” hoặc “Xóa toàn bộ báo thức”
  - AI gọi: `{"name": "self.alarm.cancel_all", "arguments": {}}`

### Giới hạn quan trọng của `cancel`

Tool `self.alarm.cancel` kiểm tra đúng giờ/phút nhưng implementation hiện tại gọi `clearAll()` khi tìm thấy mục phù hợp. Vì vậy lệnh này có thể xóa **toàn bộ** báo thức, không chỉ mục được yêu cầu. Muốn xóa tất cả, dùng `self.alarm.cancel_all` và coi `cancel` là hành vi có giới hạn đã biết.

## Lưu trữ và vòng đời

- Dữ liệu được lưu trong namespace NVS `alarms` dưới dạng cấu trúc tuần tự của `hour`, `minute`, `message`, `sound_id`, `enabled` và `repeated`.
- Alarm được nạp lại sau reboot; alarm một lần bị disable sau khi trigger và được dọn ở bước cleanup kế tiếp, không phải chờ người dùng khởi động lại.
- Giới hạn đọc hiện tại là tối đa 10 alarm từ NVS. Đây là giới hạn tải dữ liệu, không phải hard cap được kiểm tra trong `addAlarm()`.
- Khi trigger, firmware cập nhật trạng thái hiển thị và phát âm thanh nhiều lần theo logic hiện tại. Không nên dùng alarm như bộ hẹn giờ chính xác đến dưới một giây.

## Thêm âm thanh mới

Quy trình phải cập nhật cả asset âm thanh và registry trong `main/features/alarm_clock/alarm_sounds.h`, sau đó chạy đúng script sinh asset/ngôn ngữ của repository để cập nhật `lang_config.h`. Chỉ sửa bảng tài liệu không làm âm thanh xuất hiện trong firmware.

## Xử lý lỗi

| Hiện tượng | Kiểm tra |
|---|---|
| Báo thức chạy sai giờ | Wi-Fi/NTP đã đồng bộ và timezone hiện tại |
| `set_sound` thất bại | Alarm cùng `hour`/`minute` đã tồn tại chưa; `sound_id` có đúng 4 ID không |
| `list` vẫn hiện mục đã tắt | Đây là hành vi hiện tại: list trả cả alarm disabled |
| Hủy một mục làm mất các mục khác | Đây là hạn chế đã biết của `self.alarm.cancel`; xem `cancel_all` |
