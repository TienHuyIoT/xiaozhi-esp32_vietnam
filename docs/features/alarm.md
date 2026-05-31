# Alarm Clock Feature - Báo Thức

## Giới Thiệu

Tính năng **Alarm Clock** cho phép thiết bị ESP32-S3 đặt báo thức, nhắc nhở người dùng vào thời gian chỉ định. Hệ thống hỗ trợ 4 âm thanh khác nhau (chuông báo thức, đi học, thức dậy, uống thuốc), lưu alarm vào NVS để giữ sau khi reboot, và có 7 MCP tools để AI có thể tự động đặt/xem/hủy báo thức theo yêu cầu người dùng.

| Thành Phần | File Chính | Chức Năng |
|---|---|---|
| **AlarmManager** | `main/features/alarm_clock/alarm_manager.{h,cc}` | Quản lý alarm, kiểm tra trigger, lưu NVS |
| **AlarmSounds** | `main/features/alarm_clock/alarm_sounds.h` | Registry âm thanh báo thức |
| **MCP Tools** | `main/features/mcp_server_features.cc` | 7 tools cho AI tương tác |

---

## Kiến Trúc

```
Người dùng nói: "Đặt báo thức 7 giờ đi học"
        │
        ▼
  MCP Server nhận tool call
        │
        ▼
  self.alarm.set { hour:7, sound_id:"school" }
        │
        ▼
  AlarmManager::addAlarm() ──► Lưu vào NVS
        │
        ▼
  AlarmManager::checkAlarms() [mỗi giây, MainEventLoop]
        │
        ├─► Giờ giống = thời gian hiện tại? ──► KHÔNG ──► Thoát
        │                                                          ▲
        └─► CÓ ──► display->SetChatMessage()                     │
                   app.PlaySound(OGG_SCHOOL) x5 lần                │
                        │                                        │
                        ▼                                        │
                   Lưu: enabled=false (một lần)                   │
                        │                                        │
                        └────────── Nếu repeated=true ──► Giữ enabled ──► Quay lại kiểm tra ngày mai
```

### Luồng Khởi Động

```
1. Application::Start()
2. InitMusic()
3. InitRadio()
4. InitAlarm()
   a. alarm_manager_ = &AlarmManager::getInstance()
   b. alarm_manager_->init()
      - nvs_open("alarms")
      - loadFromNVS()
      - cleanupExpiredAlarms()
   c. McpFeatureTools::RegisterAlarmTools(alarm_manager_)
```

---

## Âm Thanh Báo Thức

### Các Âm Thanh Hỗ Trợ

| ID | File OGG | Mô Tả | Từ Khóa Nhận Diện |
|---|---|---|---|
| `alarm` | `alarm.ogg` | Chuông báo thức mặc định | mặc định khi không nhận diện được |
| `school` | `school.ogg` | Đã tới giờ đi học rồi đấy | đi học, đi làm, school, work |
| `wakeup` | `wakeup.ogg` | Dậy thôi, sáng rồi! | thức dậy, dậy, wake |
| `medicine` | `medicine.ogg` | Đã tới giờ uống thuốc | uống thuốc, thuốc, medicine |

### Tại Sao Cần 4 Âm Thanh Khác Nhau?

Khi người dùng nói "đặt báo thức 7 giờ đi học", AI sẽ:
1. Gọi `self.alarm.set` với `sound_id="school"`
2. Phát `school.ogg` thay vì chuông báo thức chung

Âm thanh `school` mang tính giao tiếp hơn - nó nói rõ ràng "đã tới giờ đi học" thay vì chỉ là một chuông.

---

## MCP Tools

Tất cả tools được đăng ký trong `mcp_server_features.cc` dòng 753-964.

### 1. `self.alarm.set` — Đặt Báo Thức Mới

**Khi nào dùng:** Khi người dùng nói "đặt báo thức", "báo thức lúc", "hẹn giờ", "nhắc tôi lúc".

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

**Thứ tự ưu tiên chọn sound_id (AI tự động):**

```
1. "đi học" / "đi làm" / "school" / "work"
   → sound_id = "school"

2. "thức dậy" / "dậy" / "wake" / "wake up"
   → sound_id = "wakeup"

3. "uống thuốc" / "thuốc" / "medicine"
   → sound_id = "medicine"

4. mặc định
   → sound_id = "alarm"
```

### 2. `self.alarm.list` — Xem Danh Sách

**Khi nào dùng:** Khi người dùng hỏi "có bao nhiêu báo thức", "xem danh sách báo thức", "kiểm tra báo thức".

```json
{ "name": "self.alarm.list", "arguments": {} }
```

**Trả về:** JSON array chứa tất cả alarm đang hoạt động.

### 3. `self.alarm.cancel` — Hủy Báo Thức Cụ Thể

**Khi nào dùng:** Hủy một báo thức theo giờ chỉ định.

```json
{
  "name": "self.alarm.cancel",
  "arguments": { "hour": 7, "minute": 0 }
}
```

### 4. `self.alarm.cancel_all` — Hủy Tất Cả

**Khi nào dùng:** Khi người dùng nói "hủy tất cả báo thức", "xóa hết báo thức", "tắt hết báo thức".

```json
{ "name": "self.alarm.cancel_all", "arguments": {} }
```

### 5. `self.alarm.next` — Xem Báo Thức Tiếp Theo

**Khi nào dùng:** Hỏi "báo thức tiếp theo là mấy giờ", "báo thức nào gần nhất".

```json
{ "name": "self.alarm.next", "arguments": {} }
```

### 6. `self.alarm.set_sound` — Đặt Âm Thanh Cho Báo Thức

Đặt âm thanh cho báo thức **đã tồn tại**. Thường dùng sau khi `self.alarm.set` nếu phát hiện người dùng muốn âm thanh đặc biệt.

**Ví dụ:** Người dùng nói "đặt báo thức 7h đi học" → AI gọi `self.alarm.set` (default alarm) → sau đó gọi `self.alarm.set_sound` để đổi thành `school`.

```json
{
  "name": "self.alarm.set_sound",
  "arguments": {
    "hour": 7,
    "minute": 0,
    "sound_id": "school"
  }
}
```

### 7. `self.alarm.available_sounds` — Xem Âm Thanh Khả Dụng

Trả về danh sách tất cả âm thanh báo thức cùng mô tả.

```json
{ "name": "self.alarm.available_sounds", "arguments": {} }
```

---

## Lưu Trữ NVS

Alarms được lưu vào NVS partition (`nvs_open("alarms")`) để giữ sau khi mất điện hoặc reboot.

### Cú Pháp Lưu

```
alarm_0 = "HH:MM|message|sound_id|enabled|repeated"
alarm_1 = "HH:MM|message|sound_id|enabled|repeated"
...
count = N
```

### Ví Dụ

```
alarm_0 = "07:00|đi học|school|1|0"
alarm_1 = "06:30|wake up alarm|wakeup|1|1"
count = 2
```

| Đại Diện | Giá Trị |
|---|---|
| `HH:MM` | Giờ và phút (00-23:00-59) |
| `message` | Nội dung nhắc nhở (VD: "đi học") |
| `sound_id` | ID âm thanh ("alarm", "school", "wakeup", "medicine") |
| `enabled` | 1 = bật, 0 = tắt |
| `repeated` | 1 = lặp hàng ngày, 0 = chỉ một lần |

### Các Trường Hợp Xử Lý

| TH | Hành Vi |
|---|---|
| Alarm một lần (repeated=false) | Sau khi trigger, `enabled=false`. `cleanupExpiredAlarms()` xóa khi khởi động lại |
| Alarm lặp hàng ngày (repeated=true) | `enabled=true` sau trigger, sẽ trigger lại vào ngày mai cùng giờ |
| Reset/thiết bị mất điện | Alarm load lại từ NVS khi `init()` |

---

## Thêm Âm Thanh Báo Thức Mới

### Bước 1: Thêm Vào Script Sinh OGG

File: `scripts/generate_alarm_sounds.py`, dòng 24-28

```python
ALARM_SOUNDS = [
    {"id": "school",  "text": "Đã tới giờ đi học rồi đấy",       "desc": "Báo thức đi học"},
    {"id": "wakeup",  "text": "Dậy thôi, sáng rồi!",             "desc": "Báo thức thức dậy"},
    {"id": "medicine","text": "Đã tới giờ uống thuốc",           "desc": "Nhắc uống thuốc"},
    # Thêm dòng mới
    {"id": "meeting", "text": "Đã tới giờ họp rồi!",             "desc": "Nhắc họp"},
]
```

Chạy script để tạo file OGG:

```bash
python scripts/generate_alarm_sounds.py
```

File `meeting.ogg` sẽ được tạo trong `main/assets/common/`.

### Bước 2: Thêm Vào alarm_sounds.h

File: `main/features/alarm_clock/alarm_sounds.h`, dòng 18-23

```cpp
static constexpr AlarmSound kAlarmSounds[] = {
    { "alarm",   "Chuông báo thức mặc định", OGG_PTR(ALARM) },
    { "school",  "Đã tới giờ đi học rồi đấy", OGG_PTR(SCHOOL) },
    { "wakeup",  "Dậy thôi, sáng rồi!",        OGG_PTR(WAKEUP) },
    { "medicine","Đã tới giờ uống thuốc",       OGG_PTR(MEDICINE) },
    // Thêm dòng mới
    { "meeting", "Đã tới giờ họp rồi!",       OGG_PTR(MEETING) },
};
```

---

## Cấu Trúc Dữ Liệu

### Alarm (alarm_manager.h dòng 18-25)

```cpp
struct Alarm {
    uint8_t hour;          // 0-23
    uint8_t minute;        // 0-59
    std::string message;   // Nội dung nhắc nhở
    std::string sound_id;  // ID âm thanh: "alarm", "school", "wakeup", "medicine"
    bool enabled;          // true = đang hoạt động
    bool repeated;         // true = lặp hàng ngày
};
```

---

## API Reference

### AlarmManager

```cpp
static AlarmManager& getInstance();
```

Singleton. Luôn trả về cùng instance.

---

```cpp
void init();
```

Khởi tạo AlarmManager:
1. `nvs_open("alarms")`
2. `loadFromNVS()`
3. `cleanupExpiredAlarms()`
4. `last_check_time_ = 0`

---

```cpp
void checkAlarms();
```

Kiểm tra alarm mỗi giây (được gọi từ `MainEventLoop`). Nếu giờ hiện tại khớp với alarm:
- Nếu `repeated=false`: đặt `enabled=false` sau khi trigger
- Nếu `repeated=true`: giữ `enabled=true` để trigger ngày mai

---

```cpp
void addAlarm(const Alarm& alarm);
```

Thêm alarm mới. Tự động kiểm tra trùng lặp (cùng giờ + phút + enabled). Tự động nhận diện sound_id từ keyword trong message nếu `sound_id="alarm"`.

---

```cpp
const std::vector<Alarm>& getAlarms() const;
```

Trả về danh sách tất cả alarm (kể cả disabled).

---

```cpp
bool isDuplicateAlarm(uint8_t hour, uint8_t minute);
```

Kiểm tra xem đã có alarm enabled cùng giờ chưa.

---

```cpp
void clearAll();
```

Xóa tất cả alarm và lưu NVS.

---

```cpp
std::string getNextAlarmInfo();
```

Trả về thông tin alarm gần nhất. Format: `"HH:MM - message"`.

---

```cpp
bool setAlarmSound(uint8_t hour, uint8_t minute, const std::string& sound_id);
```

Cập nhật âm thanh cho alarm cụ thể. Trả về `true` nếu tìm thấy alarm.

---

### AlarmSounds

```cpp
const AlarmSound* FindAlarmSound(const std::string& id);
```

Tìm âm thanh theo ID. Trả về `nullptr` nếu không tìm thấy.

---

```cpp
std::string_view GetAlarmSoundOgg(const std::string& sound_id);
```

Trả về OGG string_view cho sound_id. Fallback về `alarm` nếu không tìm thấy.

---

## Lưu Ý Sử Dụng

### Về Thời Gian

1. **Hệ thống dùng giờ cục bộ** (`localtime_r`) — phụ thuộc vào NTP sync. Nếu thiết bị chưa đồng bộ NTP, giờ có thể chưa chính xác.
2. **Kiểm tra mỗi giây** — `checkAlarms()` chạy trong `MainEventLoop`, trigger đúng giây khi `tm_sec == 0`.
3. **Trigger 5 lần** — Mỗi alarm phát âm thanh 5 lần, mỗi lần cách nhau 1 giây.

### Về Âm Thanh

1. **Định dạng OGG Opus** — File phải là OGG Opus (không phải Vorbis). Script `generate_alarm_sounds.py` dùng FFmpeg với `-c:a libopus -b:a 16k`.
2. **Vượt 50KB** — FFmpeg warning nếu file > 50KB. Cần rút ngắn text nếu cần.
3. **Fallback** — Nếu sound_id không tồn tại, fallback về `alarm`.

### Về Lưu Trữ

1. **Giới hạn 10 alarm** — Code chỉ load tối đa 10 alarm từ NVS (`i < 10`).
2. **Clear NVS** — `clearAll()` xóa hết alarm và lưu vào NVS ngay lập tức.
3. **Không cần ESP_RESTART** — Thay đổi alarm được lưu ngay, không cần khởi động lại.

### Về AI Tương Tác

1. **Keyword nhập nghĩa** — AI có thể nhận diện "7 giờ đi học" và tự đặt `sound_id="school"` nhưng vẫn nên rõ ràng trong prompt tool.
2. **AI không tự trigger** — `checkAlarms()` chỉ kiểm tra trigger, không phát hiện yêu cầu từ AI. AI cần gọi MCP tool để đặt/hủy báo thức.
3. **Không có âm thanh chuông** — `alarm.ogg` phải được tạo bằng script, không có sẵn trong repository.
