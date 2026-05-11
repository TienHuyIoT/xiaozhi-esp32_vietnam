# SD Media Player - Phát nhạc & video từ thẻ nhớ

## Giới thiệu

Tính năng **SD Media Player** cho phép thiết bị ESP32-S3 phát nhạc và video trực tiếp từ thẻ nhớ SD mà không cần kết nối mạng. Toàn bộ media được quản lý và điều khiển thông qua AI agent qua giao thức MCP.

Hệ thống bao gồm hai thành phần độc lập:

| Thành phần | File chính | Chức năng |
|---|---|---|
| **SD Music** | `main/features/music/esp32_sd_music.{h,cc}` | Phát nhạc từ thẻ SD |
| **SD Video** | `main/features/video/video_player.{h,cc}` | Phát video từ thẻ SD |

---

## Cấu hình

### 1. Kconfig (`idf.py menuconfig`)

Mở menu cấu hình và bật các mục sau:

```
Xiaozhi Assistant
├── Enable SD Card  ──────────────────→  CONFIG_SD_CARD_ENABLE=y
│
└── SD Card Interface Type
    ├── SDMMC Interface  (mặc định, 4-bit bus)
    └── SPI Interface    (1-bit bus, dùng khi hết chân) (sử dụng)


CONFIG_SD_CARD_INTERFACE_TYPE_SPI=y
```

> **Lưu ý:** Khi cấu hình `CONFIG_SD_CARD_ENABLE=y`, toàn bộ MCP tools cho SD music và SD video sẽ được đăng ký tự động tại thời điểm khởi động (`application.cc` dòng 437-446).

### 2. Cấu hình GPIO cho board `bread-compact-wifi-lcd`

File: `main/boards/bread-compact-wifi-lcd/config.h`

#### SPI Interface

```c
// SPI interface cho SD card
#define CARD_SPI_MOSI_GPIO  GPIO_NUM_10   // DI
#define CARD_SPI_MISO_GPIO  GPIO_NUM_11   // DO
#define CARD_SPI_SCLK_GPIO  GPIO_NUM_9    // CLK
#define CARD_SPI_CS_GPIO    GPIO_NUM_14   // CS
```

---

## Cấu trúc thẻ nhớ

### Thư mục cho nhạc (SD Music)

**Quét đệ quy toàn bộ thẻ SD** — không giới hạn thư mục cụ thể. File `.mp3` có thể đặt ở bất kỳ đâu trên thẻ.

```
/sdcard/
├── playlist.json        ← Tự tạo sau khi quét (lưu metadata)
├── Music/
│   ├── bai1.mp3
│   └── album/
│       └── bai2.mp3
├── Nhac/
│   └── bai3.mp3
└── bai4.mp3            ← File ở thư mục gốc cũng được quét
```

**Định dạng hỗ trợ:** `.mp3`, `.wav`, `.aac`, `.flac`

### Thư mục cho video (SD Video)

**Chỉ quét thư mục cố định** `/sdcard/videos/`, không quét đệ quy.

```
/sdcard/
├── videos/              ← Thư mục cố định cho video
│   ├── demo1.avi
│   └── clip.avi
└── Music/              ← Nhạc không bị ảnh hưởng
    └── song.mp3
```

**Định dạng hỗ trợ:** `.avi` (MJPEG + PCM audio)

---

## Các MCP Tools

### SD Music — Danh sách đầy đủ

Tất cả tools được đăng ký trong `mcp_server_features.cc` dòng 289-745.

#### 1. `self.sdmusic.playback` — Điều khiển phát nhạc

```json
{
  "name": "self.sdmusic.playback",
  "arguments": { "action": "play | pause | stop | next | prev" }
}
```

**Khi nào dùng:** Chỉ dùng khi người dùng nói rõ "nhạc trong thẻ nhớ", "nhạc offline", "SD card", "phát bài trong thẻ". **Không dùng** khi người dùng chỉ nói "phát nhạc" (đó là nhạc online, dùng `self.music.play_song`).

| action | Mô tả |
|---|---|
| `play` | Bắt đầu phát. Nếu chưa có playlist → tự quét thẻ → tạo playlist.json → phát |
| `pause` | Dừng tạm |
| `stop` | Dừng hoàn toàn |
| `next` | Chuyển bài tiếp theo |
| `prev` | Quay lại bài trước |

**Ví dụ:**

```json
{
  "name": "self.sdmusic.playback",
  "arguments": { "action": "play" }
}
```

#### 2. `self.sdmusic.mode` — Chế độ phát

```json
{
  "name": "self.sdmusic.mode",
  "arguments": {
    "action": "shuffle | repeat",
    "enabled": true,
    "mode": "none | one | all"
  }
}
```

| action | Tham số | Mô tả |
|---|---|---|
| `shuffle` | `enabled: bool` | Bật/tắt phát ngẫu nhiên |
| `repeat` | `mode: none \| one \| all` | Chế độ lặp: không lặp / lặp 1 bài / lặp toàn bộ |

**Ví dụ:**

```json
{ "name": "self.sdmusic.mode", "arguments": { "action": "shuffle", "enabled": true } }
{ "name": "self.sdmusic.mode", "arguments": { "action": "repeat", "mode": "all" } }
```

#### 3. `self.sdmusic.track` — Thao tác với bài hát

```json
{
  "name": "self.sdmusic.track",
  "arguments": {
    "action": "set | info | list | current",
    "index": 0
  }
}
```

| action | Tham số | Mô tả |
|---|---|---|
| `set` | `index: int` | Chọn bài hát theo số thứ tự |
| `info` | `index: int` | Xem thông tin chi tiết bài hát (title, artist, album, genre, duration, bitrate...) |
| `list` | — | Đếm tổng số bài hát trong thư viện |
| `current` | — | Lấy tên bài đang phát |

**Trả về của `info`:** JSON với các trường `name`, `path`, `title`, `artist`, `album`, `genre`, `year`, `track_number`, `duration_ms`, `bitrate_kbps`, `file_size`, `has_cover`, `cover_size`, `cover_mime`.

#### 4. `self.sdmusic.directory` — Thao tác thư mục

```json
{
  "name": "self.sdmusic.directory",
  "arguments": {
    "action": "play | list",
    "directory": "Music"
  }
}
```

| action | Tham số | Mô tả |
|---|---|---|
| `play` | `directory: string` | Phát tất cả bài trong thư mục con |
| `list` | — | Liệt kê các thư mục con trên thẻ SD |

#### 5. `self.sdmusic.search` — Tìm và phát theo tên

```json
{
  "name": "self.sdmusic.search",
  "arguments": {
    "action": "search | play",
    "keyword": "tên bài hát"
  }
}
```

| action | Mô tả |
|---|---|
| `search` | Tìm các bài khớp với từ khóa, trả về danh sách JSON |
| `play` | Phát bài đầu tiên khớp từ khóa |

#### 6. `self.sdmusic.library` — Thư viện & phân trang

```json
{
  "name": "self.sdmusic.library",
  "arguments": {
    "action": "count_dir | count_current | page",
    "directory": "Music",
    "page": 1,
    "page_size": 10
  }
}
```

| action | Mô tả |
|---|---|
| `count_dir` | Đếm số bài trong thư mục chỉ định |
| `count_current` | Đếm số bài trong thư mục hiện tại |
| `page` | Lấy danh sách bài theo trang (phân trang 1-indexed) |

#### 7. `self.sdmusic.reload` — Quét lại thư viện

```json
{ "name": "self.sdmusic.reload", "arguments": {} }
```

Quét lại toàn bộ thẻ SD và cập nhật `playlist.json`. Dùng khi đã thêm/bớt file nhạc trên thẻ mà không khởi động lại thiết bị.

#### 8. `self.sdmusic.suggest` — Gợi ý bài hát

```json
{
  "name": "self.sdmusic.suggest",
  "arguments": {
    "action": "next | similar",
    "keyword": "tên bài",
    "max_results": 5
  }
}
```

| action | Mô tả |
|---|---|
| `next` | Gợi bài tiếp theo dựa trên lịch sử nghe |
| `similar` | Tìm bài tương tự với từ khóa (cùng thể loại, thư mục, tần suất nghe) |

#### 9. `self.sdmusic.progress` — Tiến trình phát

```json
{ "name": "self.sdmusic.progress", "arguments": {} }
```

**Trả về:** JSON `{ position_ms, duration_ms, state, bitrate_kbps, position_str, duration_str, track_name, track_path }`

`state` có thể là: `stopped`, `preparing`, `playing`, `paused`, `error`.

#### 10. `self.sdmusic.genre` — Phát theo thể loại

```json
{
  "name": "self.sdmusic.genre",
  "arguments": {
    "action": "search | play | play_index | next",
    "genre": "Pop",
    "index": 0
  }
}
```

| action | Mô tả |
|---|---|
| `search` | Tìm các bài thuộc thể loại chỉ định |
| `play` | Phát bài đầu tiên của thể loại |
| `play_index` | Phát bài theo chỉ số trong danh sách thể loại |
| `next` | Phát bài tiếp theo cùng thể loại |

#### 11. `self.sdmusic.genre_list` — Liệt kê thể loại

```json
{ "name": "self.sdmusic.genre_list", "arguments": {} }
```

Trả về JSON array chứa tất cả thể loại có trong thư viện nhạc.

---

### SD Video — Danh sách đầy đủ

Tất cả tools được đăng ký trong `mcp_server_features.cc` dòng 194-284.

#### 1. `self.sdvideo.playback` — Điều khiển phát video

```json
{
  "name": "self.sdvideo.playback",
  "arguments": { "action": "play | pause | stop | next | prev | shuffle | repeat" }
}
```

**Khi nào dùng:** Chỉ dùng khi người dùng nói rõ "video trong thẻ nhớ", "video offline", "phát video trong thẻ". **Không dùng** khi người dùng chỉ nói "phát video" (mặc định là phát online).

| action | Mô tả |
|---|---|
| `play` | Phát video đầu tiên trong playlist |
| `pause` | Dừng tạm video |
| `stop` | Dừng phát video |
| `next` | Phát video tiếp theo |
| `prev` | Phát video trước đó |
| `shuffle` | Phát ngẫu nhiên một video |
| `repeat` | Lặp lại video hoặc toàn bộ playlist |

#### 2. `self.sdvideo.search_play` — Tìm và phát video

```json
{
  "name": "self.sdvideo.search_play",
  "arguments": { "video_name": "tên video" }
}
```

Tìm video theo tên (có thể là tên đầy đủ hoặc một phần) trong playlist và phát.

---

## Cách sử dụng — Ví dụ thực tế

### Phát nhạc offline

**Yêu cầu bằng giọng nói:**
> "Phát nhạc trong thẻ nhớ"
> "Mở bài hát Tết trong thẻ SD"
> "Bật shuffle nhạc offline"
> "Phát thư mục Music từ thẻ nhớ"

**Flow thực hiện:**

1. AI nhận yêu cầu, gọi `self.sdmusic.playback` với `action: "play"`
2. Hệ thống kiểm tra playlist → chưa có → tự động quét thẻ SD
3. Tạo `playlist.json` với danh sách tất cả file `.mp3`
4. Phát bài đầu tiên

### Phát video offline

**Yêu cầu bằng giọng nói:**
> "Phát video trong thẻ nhớ"
> "Mở clip từ SD card"

**Flow thực hiện:**

1. AI gọi `self.sdvideo.playback` với `action: "play"`
2. Hệ thống quét `/sdcard/videos/` tìm file `.avi`
3. Phát video đầu tiên trong danh sách

### Tìm bài hát cụ thể

```
AI: "Bạn muốn nghe bài nào?"
User: "Phát bài Đom Đóm trong thẻ nhớ"
AI: → self.sdmusic.search { action: "play", keyword: "Đom Đóm" }
```

### Chuyển chế độ phát

```
User: "Bật lặp lại tất cả bài"
AI: → self.sdmusic.mode { action: "repeat", mode: "all" }

User: "Tắt shuffle"
AI: → self.sdmusic.mode { action: "shuffle", enabled: false }
```

---

## Lưu ý khi sử dụng

### Về thẻ nhớ

1. **Định dạng thẻ:** FAT32 là định dạng được khuyến nghị. exFAT cũng được hỗ trợ nhưng FAT32 ổn định hơn trên ESP32.
2. **Dung lượng:** Không giới hạn, nhưng thẻ <= 32GB FAT32 cho hiệu năng tốt nhất.
3. **Tốc độ thẻ:** Nên dùng thẻ tốc độ Class 10 trở lên để tránh giật lag khi phát video.
4. **Mount thành công:** Kiểm tra log khởi động:

   ```
   I ESP32SdMusic: Scanning SD card: /sdcard
   I Esp32SdMusic: Playlist file (JSON) saved: /sdcard/playlist.json
   I Application: InitSdMusic: SD card music player ready
   ```

### Về nhạc

1. **ID3 Tag:** File `.mp3` nên có ID3 tag đầy đủ (title, artist, album, genre) để AI hiểu và gợi ý chính xác. Không có tag → dùng tên file làm tiêu đề.
2. **Cover art:** Ảnh bìa album nhúng trong file MP3 (ID3v2) sẽ được đọc và hiển thị.
3. **Bitrate:** Hệ thống đọc bitrate từ metadata, trả về qua `self.sdmusic.progress`.
4. **Lịch sử phát:** Hệ thống ghi lại lịch sử nghe để gợi ý bài tiếp theo qua `self.sdmusic.suggest`.

### Về video

1. **Định dạng bắt buộc:** Chỉ hỗ trợ `.avi` với codec MJPEG (video) + PCM (audio).
2. **Độ phân giải:** Tối đa 320x240 pixel (`VIDEO_MAX_WIDTH=320`, `VIDEO_MAX_HEIGHT=240`).
3. **Thư mục cố định:** Video chỉ được quét trong `/sdcard/videos/`, không quét đệ quy.
4. **Render mode:** Mặc định dùng `LvglCanvas` (qua LVGL). Có thể đổi sang `DirectLcd` trong `application.cc` dòng 1593 để bypass LVGL, đạt FPS cao hơn nhưng có thể không ổn định.
5. **Frame buffer:** Double-buffer RGB565 được cấp phát trong PSRAM (8MB external RAM).
6. **Decode:** JPEG decode qua `esp_new_jpeg` (software decoder).

### Về MCP tools và AI

1. **Phân biệt nhạc online vs offline:** AI phải dùng đúng tool:
   - `self.music.play_song` — nhạc online (mặc định)
   - `self.sdmusic.*` — nhạc từ thẻ SD
2. **Tool không khả dụng** nếu `CONFIG_SD_CARD_ENABLE` không được bật → code sẽ không biên dịch các tool này (nằm trong `#ifdef CONFIG_SD_CARD_ENABLE`).
3. **Fallback:** Khi thẻ SD không mount được, log sẽ hiển thị `Failed to mount SD card` và các MCP tools vẫn đăng ký nhưng sẽ trả về lỗi khi gọi.
