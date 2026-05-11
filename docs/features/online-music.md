# Online Music Player - Phát nhạc trực tuyến từ server

## Giới thiệu

Tính năng **Online Music Player** cho phép thiết bị ESP32-S3 phát nhạc trực tuyến từ server qua internet. Nhạc được tìm kiếm qua API của server, sau đó stream về ESP32 để giải mã và phát qua loa. Toàn bộ được điều khiển thông qua AI agent qua giao thức MCP.

Hệ thống bao gồm các thành phần chính:

| Thành phần | File chính | Chức năng |
|---|---|---|
| **Esp32Music** | `main/features/music/esp32_music.{h,cc}` | Tìm kiếm & phát nhạc online |
| **AudioStreamPlayer** | `main/features/music/audio_stream_player.{h,cc}` | Base class: HTTP streaming, decoder, buffer, FFT |
| **O3icManager** | `main/features/music/o3ic_manager.{h,cc}` | Quản lý lời bài hát (o3ics) |
| **MusicVisualizer** | `main/features/music/music_visualizer.{h,cc}` | Spectrum display & music UI overlay |

---

## Kiến trúc hệ thống

### Luồng phát nhạc online

```
User (voice/text)
    │
    ▼
AI Agent ──MCP──► self.music.play_song { song_name, artist_name }
    │
    ▼
Esp32Music::Download(song_name, artist_name)
    │
    ├── 1) HTTP GET /stream_pcm?song=...&artist=... (với auth headers)
    │
    ├── 2) Parse JSON response:
    │       { title, artist, audio_url, o3ic_url, duration }
    │
    ├── 3) StartStreaming(audio_url)
    │       │
    │       ▼
    │   AudioStreamPlayer (base class)
    │       ├── HTTP GET audio_url (chunked transfer)
    │       ├── MP3 Decoder (esp_audio_codec)
    │       ├── PCM Output → AudioCodec → I2S → Loa
    │       ├── FFT → MusicVisualizer (spectrum bars)
    │       └── O3icManager (hiển thị lời bài hát)
    │
    └── 4) O3icManager::Start(o3ic_url) (nếu ở chế độ lyrics)
```

### Authentication

ESP32 gửi các header đặc biệt để server xác thực thiết bị:

| Header | Mô tả |
|---|---|
| `X-MAC-Address` | Địa chỉ MAC của ESP32 |
| `X-Chip-ID` | Chip ID (MAC không có dấu `:`) |
| `X-Timestamp` | Unix timestamp (giây) |
| `X-Dynamic-Key` | SHA-256 hash của `MAC:ChipID:Timestamp:SecretKey` (16 ký tự đầu) |

Thuật toán tạo key sử dụng **mbedtls_sha256**, format:

```
data = MAC + ":" + ChipID + ":" + timestamp + ":" + "your-esp32-secret-key-2024"
key  = SHA-256(data)  // lấy 16 bytes hex đầu tiên
```

---

## Cấu hình

### 1. Server URL

**Mặc định:** `http://www.xiaozhihui.xyz:5005`

Có thể thay đổi qua Settings:

- Key: `music_url` trong namespace `wifi`
- File cấu hình: `settings.json` hoặc qua OTA web interface

### 2. Audio pipeline

Nhạc online đi qua pipeline hoàn chỉnh:

```
HTTP Stream (MP3)
    │
    ├── esp_audio_codec decoder (MP3/AAC/FLAC/WAV)
    │
    ├── PCM Frame (16-bit, stereo/mono)
    │       │
    │       ├── FFT (512-point, ~23ms/frame @22050Hz)
    │       │       └── MusicVisualizer → Spectrum bars trên LCD
    │       │
    │       └── AudioCodec → I2S DMA → Loa
    │
    └── Progress tracking (position_ms, duration_ms)
```

### 3. Display mode

Esp32Music hỗ trợ 2 chế độ hiển thị:

| Mode | Giá trị | Mô tả |
|---|---|---|
| `DISPLAY_MODE_SPECTRUM` | `spectrum` | Hiện spectrum analyzer (FFT bars) |
| `DISPLAY_MODE_LYRICS` | `o3ics` | Hiện lời bài hát (từ O3icManager) |

Chuyển đổi bằng `self.music.set_display_mode`.

---

## Các MCP Tools

### Online Music — Danh sách đầy đủ

Tất cả tools được đăng ký trong `mcp_server_features.cc` dòng 89-143.

#### 1. `self.music.play_song` — Tìm và phát nhạc online

```json
{
  "name": "self.music.play_song",
  "arguments": {
    "song_name": "tên bài hát",
    "artist_name": "tên ca sĩ (tùy chọn)"
  }
}
```

**Khi nào dùng:** Đây là tool **mặc định** khi người dùng nói "phát nhạc", "mở nhạc", "phát bài hát", "play music", "play song", "mở bài ...". **Không dùng** khi người dùng nói rõ "nhạc trong thẻ nhớ", "nhạc offline" (dùng `self.sdmusic.*`).

**Flow thực hiện:**

1. AI gọi `self.music.play_song` với tên bài hát
2. Esp32Music gửi HTTP GET đến server `/stream_pcm` kèm auth headers
3. Server trả JSON: `{ title, artist, audio_url, o3ic_url, duration }`
4. ESP32 stream audio từ `audio_url`, decode MP3, phát qua loa
5. Nếu đang ở chế độ `o3ics` → fetch và hiển thị lời bài hát

**Ví dụ:**

```json
{
  "name": "self.music.play_song",
  "arguments": { "song_name": "Nắng ấm xa dần", "artist_name": "Hồ Ngọc Hà" }
}
```

**Trả về:**

| Kết quả | Message |
|---|---|
| Thành công | `{"success": true, "message": "Music started playing"}` |
| Thất bại | `{"success": false, "message": "Failed to get music resource"}` |

#### 2. `self.music.set_display_mode` — Chuyển chế độ hiển thị

```json
{
  "name": "self.music.set_display_mode",
  "arguments": {
    "mode": "spectrum | o3ics"
  }
}
```

| mode | Mô tả |
|---|---|
| `spectrum` | Chuyển sang hiển thị spectrum analyzer (FFT bars) |
| `o3ics` | Chuyển sang hiển thị lời bài hát |

**Trả về:**

```json
{ "success": true, "message": "Switched to spectrum display mode" }
{ "success": true, "message": "Switched to o3ics display mode" }
{ "success": false, "message": "Invalid display mode, use 'spectrum' or 'o3ics'" }
```

---

## Cách sử dụng — Ví dụ thực tế

### Phát nhạc online (mặc định)

**Yêu cầu bằng giọng nói:**
> "Phát bài Chạy ngay đi"
> "Mở nhạc của Sơn Tùng"
> "Play song Hello"
> "Phát bài Nắng ấm xa dần"

**Flow thực hiện:**

1. AI nhận yêu cầu, gọi `self.music.play_song` với `song_name`
2. Hệ thống gửi request đến music server
3. Server trả URL stream + metadata (title, artist, duration, o3ic_url)
4. ESP32 bắt đầu stream và decode, hiển thị spectrum hoặc lyrics

### Chuyển chế độ hiển thị

```
User: "Hiện lời bài hát"
AI: → self.music.set_display_mode { mode: "o3ics" }

User: "Hiện spectrum"
AI: → self.music.set_display_mode { mode: "spectrum" }
```

---

## Chi tiết kỹ thuật

### AudioStreamPlayer — Base class

Base class `AudioStreamPlayer` cung cấp hạ tầng chung cho tất cả audio player (online music, SD music, radio):

**Kiến trúc buffer (Producer-Consumer):**

```
HTTP Download Task (Core 0)
    │
    ├── Chunk 4KB → PushToBuffer() → PSRAM queue
    │
Source Task ──────────────────────────► Audio Buffer (PSRAM, max 256KB)
    │                                         │
    ├── Min before play: 32KB (HTTP)          │
    ├── Min before play: 8KB (SD card)        │
    └── Max size: 256KB                       │
                                                ▼
                                          Play Task (Core 1)
                                                │
                                                ├── esp_audio_codec decode
                                                ├── PCM volume amp
                                                ├── FFT callback (spectrum)
                                                ├── PCM callback (o3ics)
                                                └── AudioCodec → I2S → Loa
```

**Codecs hỗ trợ:**

| Codec | Decoder | Ghi chú |
|---|---|---|
| MP3 | `esp_audio_simple_dec` | Mặc định cho nhạc online |
| AAC | `esp_audio_simple_dec` | Qua HTTP stream |
| FLAC | `esp_audio_simple_dec` | Qua HTTP stream |
| WAV | Passthrough (no decoder) | Raw PCM 16-bit |

**Macro quan trọng:**

```c
AUDIO_BUF_MAX_SIZE         256KB   // Max PSRAM cho buffer
AUDIO_BUF_MIN_SIZE          32KB   // Buffer trước khi bắt đầu phát (HTTP)
AUDIO_HTTP_CHUNK_SIZE        4KB    // Mỗi lần đọc từ HTTP
AUDIO_DEC_INPUT_BUF_SIZE     8KB   // Decoder input buffer
AUDIO_PCM_OUT_BUF_SIZE     4608B   // PCM output (1152 samples × 2ch × 2B)
AUDIO_MAX_RECONNECT           3    // Số lần thử lại khi mất kết nối
AUDIO_RECONNECT_DELAY_MS   1500ms  // Delay giữa các lần reconnect
```

### O3icManager — Quản lý lời bài hát

O3icManager tải và hiển thị lời bài hát theo thời gian thực:

- **Nguồn:** `o3ic_url` từ API server
- **Latency compensation:** `LYRIC_LATENCY_OFFSET_MS = 600ms` — bù trừ độ trễ hiển thị
- **Trigger:** Chỉ hoạt động khi `display_mode == DISPLAY_MODE_LYRICS`
- **Update:** Gọi qua `OnPcmFrame` hook mỗi khi có PCM frame mới

### MusicVisualizer — Spectrum & UI overlay

Component tự chứa (self-contained), không phụ thuộc vào player class cụ thể:

- **Spectrum:** FFT 512-point, 40 bars, render lên LVGL canvas
- **Music UI overlay:** Title, artist, progress bar, bitrate, next track
- **Thread-safe:** Audio feed methods an toàn khi gọi từ streaming task
- **Nguồn nhạc:** Phân biệt qua `SourceType` enum: `SD_CARD`, `ONLINE`, `RADIO`, `NONE`

### ESP32 Authentication — Chi tiết

```c
// Mỗi request gửi 4 header đặc biệt:
http->SetHeader("X-MAC-Address", "AA:BB:CC:DD:EE:FF");
http->SetHeader("X-Chip-ID",     "AABBCCDDEEFF");
http->SetHeader("X-Timestamp",   "1715000000");
http->SetHeader("X-Dynamic-Key", "1A2B3C4D5E6F7081...");
```

Thuật toán tạo key sử dụng **mbedtls_sha256**:

```
data  = "AA:BB:CC:DD:EE:FF:AABBCCDDEEFF:1715000000:your-esp32-secret-key-2024"
hash  = SHA-256(data)  // 32 bytes
key   = hex(hash[0..15]) // 16 bytes hex = 32 ký tự
```

---

## Lưu ý khi sử dụng

### Về kết nối mạng

1. **WiFi bắt buộc:** Nhạc online cần thiết bị kết nối WiFi. Kiểm tra log khởi động:
   ```
   I Esp32Music: Music player initialised (codec=app-pipeline)
   I Esp32Music: Searching for: bai-hat
   I Esp32Music: API URL: http://www.xiaozhihui.xyz:5005/stream_pcm?song=...
   ```
2. **Reconnect:** Nếu mất kết nối, tự động thử lại tối đa 3 lần, mỗi lần cách nhau 1.5s.
3. **Chunked transfer:** Server có thể dùng chunked encoding (Transfer-Encoding: chunked), ESP32 xử lý được.

### Về buffer & PSRAM

1. **PSRAM:** Buffer nằm trong PSRAM (external RAM), cần ESP32-S3 có PSRAM >= 8MB.
2. **Buffer size:** Tối đa 256KB. Nếu buffer đầy, source task sẽ chờ.
3. **Buffer trước phát:** Cần tối thiểu 32KB dữ liệu đã download trước khi bắt đầu decode — tránh giật.

### Về decoder & audio

1. **MP3 mặc định:** Tất cả nhạc online dùng `AudioDecoderType::MP3`.
2. **Bitrate:** Đọc từ decoder, trả về 0 nếu chưa xác định. Sau đó ước tính từ Content-Length + bitrate.
3. **Duration:** Lấy từ API server. Nếu server không trả, ước tính: `duration_ms = content_bytes * 8 / bitrate_kbps`.
4. **Sample rate:** Tự động detect từ stream header.
5. **Volume:** Mặc định 1.0x (100%), có thể điều chỉnh qua `SetVolume(float)`.

### Về MCP tools và AI

1. **Phân biệt nhạc online vs offline:** AI phải dùng đúng tool:
   - `self.music.play_song` — nhạc online (mặc định)
   - `self.sdmusic.*` — nhạc từ thẻ SD
2. **Tool luôn khả dụng:** Không có Kconfig toggle cho online music — luôn được biên dịch.
3. **Server không phản hồi:** Trả về `{"success": false, "message": "Failed to get music resource"}`.
4. **Auth thất bại:** Nếu server yêu cầu auth mà key không hợp lệ → HTTP 401/403.

### Về display

1. **Spectrum mode:** FFT bars hiển thị tần số âm thanh, cập nhật ~43 FPS (@60Hz LVGL refresh).
2. **Lyrics mode:** Lời bài hát đồng bộ theo thời gian thực, có latency compensation 600ms.
3. **Chuyển mode:** Không làm gián đoạn playback — chỉ thay đổi nội dung hiển thị.
4. **MusicVisualizer:** Self-contained, không phụ thuộc vào Esp32Music trực tiếp — host (Application) cung cấp data qua callback.

---

## So sánh nhạc Online vs SD Card

| Tiêu chí | Online Music | SD Card Music |
|---|---|---|
| **Tool** | `self.music.play_song` | `self.sdmusic.*` |
| **Nguồn** | HTTP stream từ server | File local trên thẻ SD |
| **Tốc độ mạng** | Cần WiFi, phụ thuộc băng thông | Không cần mạng |
| **Metadata** | Từ API server (title, artist, o3ic) | Từ ID3 tag file MP3 |
| **Lyrics** | Từ `o3ic_url` của server | Không hỗ trợ |
| **Decoder** | esp_audio_codec (MP3/AAC/FLAC/WAV) | esp_audio_codec |
| **Buffer** | 32KB min (HTTP) | 8KB min (file) |
| **Playlist** | Single song (1 request = 1 bài) | Full library với playlist.json |
