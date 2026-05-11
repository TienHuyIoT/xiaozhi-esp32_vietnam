# Internet Radio - Phát đài phát thanh qua mạng

## Giới thiệu

Tính năng **Internet Radio** cho phép thiết bị ESP32-S3 phát các đài phát thanh trực tuyến qua HTTP stream. Hệ thống hỗ trợ cả MP3 và AAC stream, kèm danh sách các đài VOV (Đài Tiếng nói Việt Nam) được preset sẵn.

Hệ thống kế thừa từ `AudioStreamPlayer` — base class chung cho streaming audio từ cả HTTP và file SD card:

| Thành phần | File chính | Chức năng |
|---|---|---|
| **Esp32Radio** | `main/features/music/esp32_radio.{h,cc}` | Player chính, danh sách đài, auto-detect decoder |
| **AudioStreamPlayer** | `main/features/music/audio_stream_player.{h,cc}` | Base class: HTTP streaming, decode, buffer, PCM output |
| **Radio (interface)** | `main/features/music/radio.h` | Abstract interface để tái sử dụng |

---

## Kiến trúc luồng dữ liệu

```
HTTP Stream (AAC/MP3)
       │
       ▼
┌─────────────┐
│ Source Task │  ── Producer: đọc chunk 4KB, đẩy vào buffer (PSRAM)
│  (Core 0)   │
└──────┬──────┘
       │  queue<StreamAudioChunk>
       ▼
┌─────────────┐
│ Audio Buffer│  ── Producer-Consumer queue, max 256KB, cần >= 32KB trước khi phát
│   (PSRAM)   │
└──────┬──────┘
       │  semaphore wake-up
       ▼
┌─────────────┐
│ Decoder Task│  ── Consumer: esp_audio_simple_dec (MP3/AAC/FLAC/WAV)
│  (Core 1)   │
└──────┬──────┘
       │  PCM 16-bit
       ▼
┌─────────────┐
│ Volume Amp  │  ── Nhân với volume_factor (mặc định 4.5 cho radio)
│  (float)   │
└──────┬──────┘
       │  int16_t PCM
       ▼
┌─────────────┐
│ AudioCodec  │  ── Xuất ra I2S / DAC
└─────────────┘
       │
       ▼  FFT callback
┌─────────────┐
│ Spectrum    │  ── Hiển thị spectrum analyzer trên LCD
│ Analyzer    │
└─────────────┘
```

### Task và thread

| Task | Core | Stack | Priority | Chức năng |
|---|---|---|---|---|
| `SourceTask` | Core 0 (PRO_CPU) | 8KB | 5 | Download HTTP stream |
| `PlayTask` | Core 1 (APP_CPU) | 6KB | 6 | Decode + output PCM |

---

## Cấu hình

### 1. Kconfig

Internet Radio không có flag Kconfig riêng — nó được build cùng với `CONFIG_ESP32_RADIO_ENABLE` (hoặc bật qua module music):

```
Xiaozhi Assistant
├── Enable Internet Radio Feature  ──→  CONFIG_ESP32_RADIO_ENABLE=y
```

### 2. Thiết lập trong code

Khởi tạo trong `application.cc`:

```cpp
Esp32Radio radio;
radio.Initialize(/* codec */ nullptr);  // nullptr = dùng Application pipeline
```

---

## Các đài preset

Hệ thống khởi tạo **14 đài VOV** trong `esp32_radio.cc` dòng 52-77:

### VOV - Kênh quốc gia

| Key | Tên hiển thị | URL | Thể loại | Volume |
|---|---|---|---|---|
| `VOV1` | VOV 1 - Thời sự | `stream.vovmedia.vn/vov-1` | News/Talk | 4.5 |
| `VOV2` | VOV 2 - Văn hoá & Giáo dục | `stream.vovmedia.vn/vov-2` | Culture/Education | 4.0 |
| `VOV3` | VOV 3 - Âm nhạc & Giải trí | `stream.vovmedia.vn/vov-3` | Music/Entertainment | 4.4 |
| `VOV5` | VOV 5 - Đối ngoại | `stream.vovmedia.vn/vov5` | International | 4.1 |

### VOV Giao thông

| Key | Tên hiển thị | URL | Thể loại | Volume |
|---|---|---|---|---|
| `VOV_GT_HN` | VOV Giao thông Hà Nội | `stream.vovmedia.vn/vovgt-hn` | Traffic | 4.7 |
| `VOV_GT_HCM` | VOV Giao thông TP.HCM | `stream.vovmedia.vn/vovgt-hcm` | Traffic | 4.7 |

### VOV4 - Đài phát thanh các vùng miền

| Key | Tên hiển thị | Thể loại | Volume |
|---|---|---|---|
| `VOV_MEKONG` | VOV Mekong FM | Regional | 4.6 |
| `VOV4_MIENTRUNG` | VOV4 Miền Trung | Regional | 4.3 |
| `VOV4_TAYBAC` | VOV4 Tây Bắc | Regional | 4.4 |
| `VOV4_DONGBAC` | VOV4 Đông Bắc | Regional | 4.4 |
| `VOV4_TAYNGUYEN` | VOV4 Tây Nguyên | Regional | 4.5 |
| `VOV4_DBSCL` | VOV4 ĐBSCL | Regional | 4.5 |
| `VOV4_HCM` | VOV4 TP.HCM | Regional | 4.5 |

### VOV quốc tế

| Key | Tên hiển thị | URL | Thể loại | Volume |
|---|---|---|---|---|
| `VOV5_ENGLISH` | VOV 5 – English 24/7 | `stream.vovmedia.vn/vov247` | International | 4.0 |

---

## MCP Tools

Tất cả tools được đăng ký trong `mcp_server_features.cc` dòng 149-187.

### 1. `self.radio.play_station` — Phát đài

```json
{
  "name": "self.radio.play_station",
  "arguments": { "station_name": "VOV1" }
}
```

**Khi nào dùng:** Khi người dùng yêu cầu nghe đài phát thanh, phát radio.

**Thuật toán tìm đài (theo thứ tự ưu tiên):**

1. **Tìm theo display name** — so khớp không phân biệt hoa thường, hỗ trợ tìm một phần
2. **Tìm theo exact key** — khớp chính xác với key đài (VD: `"VOV1"`)
3. **Tìm theo key (case-insensitive)** — `"vov1"`, `"VoV3"`
4. **Tìm theo keyword tiếng Việt:**
   - `"tây nguyên"` / `"tay nguyen"` → `VOV4_TAYNGUYEN`
   - `"giao thông"` → `VOV_GT_HN`
   - `"mê kông"` / `"mekong"` → `VOV_MEKONG`
5. **Shorthand VOV + số:** `"vov"` → mặc định `VOV1`

**Ví dụ:**

```json
{ "name": "self.radio.play_station", "arguments": { "station_name": "VOV3" } }
{ "name": "self.radio.play_station", "arguments": { "station_name": "giao thông Hà Nội" } }
{ "name": "self.radio.play_station", "arguments": { "station_name": "Tây Nguyên" } }
```

### 2. `self.radio.get_stations` — Liệt kê đài

```json
{ "name": "self.radio.get_stations", "arguments": {} }
```

Trả về JSON array chứa danh sách tất cả 14 đài với format `"KEY - Tên hiển thị"`.

---

## API Reference

### Esp32Radio

```cpp
void Initialize(AudioCodec* codec);
```

Khởi tạo player. Truyền `nullptr` để dùng Application pipeline cho audio output.

---

```cpp
bool PlayStation(const std::string& station_name);
```

Tìm và phát đài theo tên (hỗ trợ nhiều cách match: display name, key, keyword tiếng Việt). Trả về `true` nếu thành công.

---

```cpp
bool PlayUrl(const std::string& radio_url, const std::string& station_name = "");
```

Phát trực tiếp từ URL. Tự động detect decoder type (AAC cho `vovmedia.vn`, MP3 cho `.mp3` URL). Thường dùng để thêm đài tùy chỉnh không có trong preset.

---

```cpp
bool Stop();
```

Dừng phát. Trả về `true` nếu đang không phát.

---

```cpp
std::vector<std::string> GetStationList() const;
```

Trả về danh sách tất cả đài preset, format `"KEY - Tên hiển thị"`.

---

```cpp
bool IsPlaying() const;
std::string GetCurrentStation() const;
size_t GetBufferSize() const;
bool IsDownloading() const;
```

Các method trạng thái.

---

### AudioStreamPlayer (base class)

```cpp
bool StartStream(const std::string& source, AudioDecoderType type = AudioDecoderType::MP3);
bool StopStream();
void PauseStream();
void ResumeStream();
```

Điều khiển phát cơ bản.

---

```cpp
void SetVolume(float factor);  // 1.0 = 100%, mặc định 4.5 cho radio
float GetVolume() const;
```

Cường điệu âm lượng. Radio dùng `RADIO_DEFAULT_VOLUME = 4.5f` (450%) để bù cho đài phát thanh thường phát ở mức thấp.

---

```cpp
enum class AudioDecoderType { MP3, AAC, FLAC, WAV, AUTO };
```

Decoder type. `Esp32Radio` tự động detect dựa trên URL.

---

```cpp
void SetFftCallback(AudioFftCallback cb);
void SetPcmCallback(AudioPcmCallback cb);
void SetStateCallback(AudioStateCallback cb);
```

Callbacks cho spectrum analyzer, PCM processing, và state change.

---

## Luồng hoạt động chi tiết

### Phát một đài

```
1. PlayStation("VOV Giao thông Hà Nội")
2. Tìm thấy key "VOV_GT_HN" trong danh sách
3. Set current_station_volume_ = 4.7
4. Stop() nếu đang phát đài khác
5. GuessDecoderType("stream.vovmedia.vn/vovgt-hn") → AAC (vovmedia.vn)
6. StartStream(url, AAC)
   a. Tạo audio_buffer_ queue
   b. Tạo source task (Core 0): HTTP GET, đọc chunk 4KB
   c. Tạo play task (Core 1): decode AAC → PCM → volume × 4.7 → I2S
7. OnStreamInfoReady → log thông tin stream (sample_rate, bitrate...)
8. OnDisplayReady → hiển thị tên đài trên LCD
```

### Tự động reconnect

`AudioStreamPlayer` hỗ trợ reconnect tự động (tối đa 3 lần, delay 1500ms) khi stream bị ngắt. Xử lý qua `SourceDataLoop()`.

---

## Buffer và memory

| Tham số | Giá trị | Ý nghĩa |
|---|---|---|
| `AUDIO_BUF_MAX_SIZE` | 256 KB | Tổng buffer giới hạn trong PSRAM |
| `AUDIO_BUF_MIN_SIZE` | 32 KB | Cần buffer đủ 32KB mới bắt đầu phát (HTTP stream) |
| `AUDIO_HTTP_CHUNK_SIZE` | 4 KB | Kích thước mỗi chunk đọc từ HTTP |
| `AUDIO_DEC_INPUT_BUF_SIZE` | 8 KB | Decoder input buffer |
| `AUDIO_PCM_OUT_BUF_SIZE` | 4.6 KB | Output buffer đủ cho 1 MP3 frame |

---

## Lưu ý khi sử dụng

### Về đài và stream

1. **Tất cả đài preset đều là VOV** — stream từ `stream.vovmedia.vn`, định dạng AAC/AAC+.
2. **Thêm đài tùy chỉnh:** Dùng `PlayUrl(url, station_name)` để phát bất kỳ HTTP stream nào. Decoder type được auto-detect từ URL pattern hoặc fallback về AAC.
3. **Phân biệt với nhạc online:** Dùng `self.radio.play_station` cho radio. Dùng `self.music.play_song` cho nhạc online qua API.
4. **WiFi required:** Radio cần WiFi kết nối ổn định. Nếu mất kết nối, source task sẽ retry 3 lần rồi dừng.

### Về volume

1. **Radio volume mặc định 4.5x** — cao hơn bình thường vì đài phát thanh thường phát ở mức đầu ra thấp.
2. **Mỗi đài có volume riêng** — các đài giao thông (4.7) cao hơn đài văn hóa (4.0) để nghe rõ trong xe.
3. **Không ảnh hưởng music player** — volume factor của radio nằm trong `Esp32Radio`, không ảnh hưởng đến `AudioStreamPlayer` của các player khác.

### Về decoder

1. **AAC:** Được dùng mặc định cho hầu hết internet radio (bao gồm tất cả VOV).
2. **MP3:** Dùng khi URL chứa `.mp3` hoặc stream type được chỉ định.
3. **Auto-detect:** Nếu không xác định được từ URL, mặc định là AAC.

### Về hiển thị

1. **Spectrum analyzer:** Radio dùng chung spectrum display với music player. FFT callback gửi PCM data qua `SetFftCallback`.
2. **Display mode:** Có hai chế độ `DISPLAY_MODE_SPECTRUM` và `DISPLAY_MODE_INFO`, chuyển qua `SetDisplayMode()`.
3. **LCD hide:** Khi radio bắt đầu phát, `WeatherUI` idle card được ẩn đi.
