# Media từ thẻ SD

## Trạng thái

Profile hiện tại bật `CONFIG_SD_CARD_ENABLE=y` và `CONFIG_SD_CARD_SPI_INTERFACE=y`. Application chỉ khởi tạo `Esp32SdMusic` và đăng ký MCP tools sau khi board mount SD thành công. Không có thẻ hoặc mount lỗi thì nhạc SD không khả dụng.

Video SD là trạng thái khác: source `VideoPlayer` và các tool `self.sdvideo.*` vẫn tồn tại, nhưng lời gọi `InitVideo()`/`InitMp4Video()` hiện bị vô hiệu hóa trong `Application`. Vì vậy video chưa được đăng ký runtime và không được quảng bá là tính năng đang dùng được.

## Nhạc SD

### Định dạng được phát

Đường playback hiện xử lý:

- MP3
- WAV
- AAC/M4A
- FLAC

Scanner có thể nhận diện thêm `.ogg` và `.opus` ở bước phân loại file, nhưng decoder playback không ánh xạ hai định dạng này vào đường phát hiện tại. Không hướng dẫn người dùng coi OGG/Opus là định dạng được hỗ trợ.

Thẻ nên dùng FAT/FAT32. exFAT không được xem là tương thích trong profile ESP-IDF hiện tại. Playlist được đọc hoặc tạo tại `<mount-point>/playlist.json`.

### MCP tools và hướng dẫn sử dụng bằng giọng nói

Khi mount thành công, 11 tool nhạc SD được đăng ký:

`self.sdmusic.playback`, `self.sdmusic.mode`, `self.sdmusic.track`, `self.sdmusic.directory`, `self.sdmusic.search`, `self.sdmusic.library`, `self.sdmusic.reload`, `self.sdmusic.suggest`, `self.sdmusic.progress`, `self.sdmusic.genre`, `self.sdmusic.genre_list`.

Hướng dẫn tình huống sử dụng bằng giọng nói thực tế:

- **Phát / Tạm dừng / Dừng / Chuyển bài trong thẻ nhớ:**
  - Người dùng: “Phát nhạc trong thẻ nhớ” hoặc “Nghe nhạc SD”
    `{"name": "self.sdmusic.playback", "arguments": {"action": "play"}}`
  - Người dùng: “Tạm dừng nhạc thẻ nhớ”
    `{"name": "self.sdmusic.playback", "arguments": {"action": "pause"}}`
  - Người dùng: “Dừng phát nhạc thẻ nhớ”
    `{"name": "self.sdmusic.playback", "arguments": {"action": "stop"}}`
  - Người dùng: “Bài tiếp theo trong thẻ”
    `{"name": "self.sdmusic.playback", "arguments": {"action": "next"}}`
  - Người dùng: “Bài trước đó”
    `{"name": "self.sdmusic.playback", "arguments": {"action": "prev"}}`
  - *Lưu ý:* Cần nói rõ ngữ cảnh “thẻ nhớ” hoặc “SD card”; nếu chỉ nói “phát nhạc”, AI sẽ ưu tiên chọn nhạc online (`self.music.play_song`).

- **Tìm kiếm và phát bài hát trên thẻ:**
  - Người dùng: “Phát bài Đom Đóm trong thẻ nhớ” hoặc “Tìm nhạc Sơn Tùng trên thẻ”
    `{"name": "self.sdmusic.search", "arguments": {"action": "play", "keyword": "Đom Đóm"}}`

- **Cấu hình chế độ phát (Ngẫu nhiên / Lặp lại):**
  - Người dùng: “Bật phát ngẫu nhiên nhạc trong thẻ”
    `{"name": "self.sdmusic.mode", "arguments": {"action": "shuffle", "enabled": true}}`
  - Người dùng: “Lặp lại tất cả bài hát”
    `{"name": "self.sdmusic.mode", "arguments": {"action": "repeat", "mode": "all"}}`

- **Xem thư viện và danh sách thể loại:**
  - Người dùng: “Cho tôi xem thư viện nhạc trong thẻ nhớ”
    `{"name": "self.sdmusic.library", "arguments": {"action": "page", "page": 1, "page_size": 20}}`
  - Người dùng: “Danh sách thể loại nhạc trong thẻ”
    `{"name": "self.sdmusic.genre_list", "arguments": {}}`

- **Xem tiến độ bài hát đang phát:**
  - Người dùng: “Bài hát đang phát chạy đến phút thứ mấy rồi?”
    `{"name": "self.sdmusic.progress", "arguments": {}}`

- **Gợi ý nhạc & Quét lại thẻ nhớ:**
  - Người dùng: “Gợi ý cho tôi vài bài nhạc trong thẻ”
    `{"name": "self.sdmusic.suggest", "arguments": {}}`
  - Người dùng: “Quét lại bài hát trong thẻ nhớ”
    `{"name": "self.sdmusic.reload", "arguments": {}}`

### Reload và lịch sử

`self.sdmusic.reload` quét lại thư viện và có thể ghi lại `playlist.json`. Response hiện tại có cờ `success` không phản ánh nhất quán kết quả thực tế; sau khi gọi hãy xác nhận bằng `self.sdmusic.library`, `genre_list`, log hoặc số lượng track, không dựa riêng vào cờ đó.

Lịch sử phát và gợi ý được giữ trong RAM, tối đa theo giới hạn nội bộ, và bị xóa khi playlist được load/rebuild. Không mô tả đây là lịch sử bền vững sau reboot.

Metadata ID3 được dùng cho title/artist/album/genre. Playlist không lưu dữ liệu cover và code hiện tại không có bộ đọc cover nhúng; các trường cover trong metadata không phải bằng chứng rằng ảnh bìa sẽ được hiển thị.

## Video SD: source-only

Nếu sau này nối lại runtime trên board LCD:

- AVI được quét không đệ quy trong `/sdcard/videos/`.
- Source có hai tool `self.sdvideo.playback` và `self.sdvideo.search_play`.
- Đường AVI thử nghiệm dùng MJPEG + PCM và renderer LVGL canvas/direct LCD tùy code.
- MP4 có helper riêng nhưng hiện không được khởi tạo trong application.

Trong profile hiện tại, người dùng gọi `self.sdvideo.*` sẽ không có tool runtime tương ứng. Đây là giới hạn triển khai, không phải lỗi hướng dẫn sử dụng.

## Xử lý lỗi

| Hiện tượng | Kiểm tra |
|---|---|
| Không có tool `self.sdmusic.*` | `CONFIG_SD_CARD_ENABLE`, wiring SPI và log mount |
| Library rỗng | Thẻ đã mount, file đúng thư mục/định dạng, `playlist.json` hợp lệ |
| File được scan nhưng không phát | Định dạng có nằm trong MP3/WAV/AAC/M4A/FLAC không |
| Reload báo success nhưng thư viện không đổi | Dùng `library`/log để xác minh; cờ response hiện không đáng tin |
| Mất gợi ý sau reload/reboot | Lịch sử chỉ ở RAM và bị reset khi load/rebuild |
