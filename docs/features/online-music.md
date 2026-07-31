# Nhạc online

## Trạng thái

Online music được khởi tạo và đăng ký MCP trong `Application::InitMusic()` ở mọi profile. Hai tool runtime là:

- `self.music.play_song`
- `self.music.set_display_mode`

Đường online hiện luôn decode stream bằng MP3. Cần Wi-Fi, music server còn hoạt động, audio codec và đủ PSRAM cho buffer/task; source không áp đặt cứng yêu cầu “PSRAM tối thiểu 8 MB”.

## Luồng và endpoint

Khi người dùng nói “phát nhạc”, AI gọi `self.music.play_song`. Firmware gửi:

```text
GET <music-server>/stream_pcm?song=<song>&artist=<artist>
```

Response JSON được đọc các trường `title`, `artist`, `audio_url`, `lyric_url` và `duration`. `audio_url` được ghép vào server URL, stream được decode MP3 và đưa tới `AudioCodec`.

URL mặc định trong source là:

```text
http://171.234.93.103:5000
```

Firmware chỉ đọc override từ `Settings("wifi").GetString("music_url")`. Không có đường `settings.json` hoặc setter OTA được kiểm chứng trong repo; muốn đổi server cần dùng cơ chế Settings thực tế hoặc sửa cấu hình/source.

## Xác thực request

Request music gửi `X-MAC-Address`, `X-Chip-ID`, `X-Timestamp` và `X-Dynamic-Key`.

- `X-Timestamp` là uptime của thiết bị tính bằng giây từ `esp_timer`, không phải Unix epoch.
- Dynamic key được tạo từ MAC, chip ID, timestamp và secret nội bộ, băm SHA-256; 16 byte đầu được biểu diễn thành 32 ký tự hex.

Không copy secret vào tài liệu hoặc log công khai.

## MCP tools

### Phát bài

```json
{
  "name": "self.music.play_song",
  "arguments": {
    "song_name": "Nắng ấm xa dần",
    "artist_name": "Sơn Tùng"
  }
}
```

`artist_name` là tùy chọn. Chỉ dùng tool SD khi người dùng nói rõ “nhạc trong thẻ”, “nhạc offline” hoặc “SD card”.

Response thành công hiện có thể là `{"success":true,"message":"Music started playing"}` ngay sau khi API trả URL. Code không kiểm tra giá trị trả về của `StartStreaming()`, nên success không chứng minh rằng server đã trả audio hoặc decoder đã phát tiếng; hãy xác nhận bằng log/audio state.

### Chế độ hiển thị

```json
{
  "name": "self.music.set_display_mode",
  "arguments": { "mode": "spectrum" }
}
```

Giá trị hợp lệ:

| `mode` | Hành vi |
|---|---|
| `spectrum` | Music spectrum visualizer |
| `lyrics` | Lời bài hát qua `LyricManager` |

Tên token đúng là `lyrics`; `lyric_url` là key từ response server.

Để có lyrics cho bài mới, nên gọi `set_display_mode(lyrics)` **trước** `play_song`. Setter hiện chỉ đổi state; lyrics được tải khi bài mới được nhận và mode lúc đó đã là lyrics. Không hứa chuyển giữa lyrics/spectrum giữa bài mà không có độ trễ hoặc reload.

## Spectrum và lyrics

- Spectrum dùng FFT 512, khoảng 40 cột và callback định kỳ khoảng 30 Hz theo cấu hình visualizer hiện tại.
- `LyricManager` tải và parse LRC từ `lyric_url`, sau đó đồng bộ theo timestamp PCM.
- Màn hình thực tế còn phụ thuộc display/LVGL của board; online music không tự biến board không có display thành lyrics UI.

## Cách sử dụng bằng giọng nói

Các tình huống sử dụng giọng nói thực tế:

- **Phát bài hát theo tên bài & ca sĩ:**
  - Người dùng: “Phát bài Nắng ấm xa dần của Sơn Tùng”
  - AI gọi:
    ```json
    {
      "name": "self.music.play_song",
      "arguments": { "song_name": "Nắng ấm xa dần", "artist_name": "Sơn Tùng" }
    }
    ```

- **Phát bài hát chỉ có tên bài:**
  - Người dùng: “Mở bài hát Phố đã lên đèn”
  - AI gọi:
    ```json
    {
      "name": "self.music.play_song",
      "arguments": { "song_name": "Phố đã lên đèn" }
    }
    ```

- **Chuyển chế độ hiển thị lời bài hát:**
  - Người dùng: “Hiển thị lời bài hát” hoặc “Xem lời bài hát”
  - AI gọi: `{"name": "self.music.set_display_mode", "arguments": {"mode": "lyrics"}}`
  - *Lưu ý:* Nên đổi mode `lyrics` trước khi phát bài mới để server tải file LRC kịp thời.

- **Chuyển chế độ hiển thị phổ âm thanh:**
  - Người dùng: “Hiển thị phổ âm thanh” hoặc “Mở sóng âm nhạc”
  - AI gọi: `{"name": "self.music.set_display_mode", "arguments": {"mode": "spectrum"}}`

*Lưu ý phân biệt:* Câu lệnh “Phát nhạc” hoặc “Phát bài [tên bài]” không kèm từ khóa thẻ nhớ sẽ tự động phát bằng nhạc online. Nếu muốn nghe nhạc offline trên thẻ SD, cần nói rõ “Phát nhạc trong thẻ nhớ”.

## Xử lý lỗi

| Hiện tượng | Kiểm tra |
|---|---|
| API không trả bài | Wi-Fi, `music_url`, endpoint `/stream_pcm`, HTTP status và auth |
| Tool báo success nhưng không có tiếng | Log `StartStreaming`, buffer PSRAM, MP3 decoder và stream URL |
| Lyrics không hiện | Response có `lyric_url`, đã đặt mode `lyrics` trước khi phát bài mới chưa |
| Dynamic key bị từ chối | Server có đang mong timestamp uptime và đúng secret/key format không |
