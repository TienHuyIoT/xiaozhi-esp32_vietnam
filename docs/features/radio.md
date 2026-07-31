# Internet radio

## Trạng thái

Internet radio được khởi tạo trực tiếp trong `Application::InitRadio()` và không có cờ bật/tắt riêng. Hai MCP tools được đăng ký ở mọi profile có application chạy:

- `self.radio.play_station`
- `self.radio.get_stations`

Radio cần Wi-Fi, stream HTTP còn hoạt động, audio codec của board và đủ PSRAM cho các task/buffer. Implementation hiện tại ghim các task theo mô hình dual-core; board single-core hoặc thiếu PSRAM không được xem là tương thích mặc định.

## Pipeline thực tế

`Esp32Radio` kế thừa `AudioStreamPlayer`. Application truyền `Board::GetInstance().GetAudioCodec()` khi khởi tạo và nối callback output/spectrum của application. Tài liệu cũ hướng dẫn truyền `nullptr` không phản ánh đường chạy hiện tại; không dùng ví dụ đó khi tích hợp radio vào application.

Decoder được đoán từ URL:

- URL VOV (`vovmedia.vn`) được coi là AAC/AAC+.
- URL có `.aac`, `aacp` hoặc `aac+` dùng AAC.
- URL có `.mp3` dùng MP3.
- URL không nhận diện được mặc định về AAC.

Firmware không kiểm tra `Content-Type` để xác nhận giả định này. Một stream có URL không chuẩn có thể cần chỉnh heuristic trong source.

## Đài preset

Source hiện có 14 preset VOV, gồm VOV1, VOV2, VOV3, VOV5, VOV Giao thông Hà Nội/TP.HCM, các kênh VOV4 vùng miền và VOV5 English. `PlayStation` chấp nhận key, tên hiển thị không phân biệt hoa thường và một số từ khóa tiếng Việt; tên đài thực tế vẫn phụ thuộc URL stream còn khả dụng.

## MCP tools

### Phát đài

```json
{
  "name": "self.radio.play_station",
  "arguments": { "station_name": "VOV3" }
}
```

Có thể nói “Mở VOV3”, “Phát VOV giao thông Hà Nội” hoặc “Nghe radio”. Kết quả thành công chỉ cho biết `PlayStation` chấp nhận yêu cầu và bắt đầu đường stream; hãy xem log/buffer để xác nhận server thực sự trả audio.

### Liệt kê đài

```json
{ "name": "self.radio.get_stations", "arguments": {} }
```

Kết quả là object, không phải array thuần:

```json
{
  "success": true,
  "stations": ["VOV1", "VOV2", "VOV3"]
}
```

## Dừng và reconnect

Dừng radio do application quản lý khi chuyển sang media khác hoặc khi gọi stop. Reconnect của lớp stream chỉ được kích hoạt khi thao tác đọc HTTP trả lỗi (`n < 0`); kết thúc stream bình thường (`n == 0`) không tự động được coi là lỗi mạng. Khi radio im lặng, kiểm tra Wi-Fi, URL stream và log decoder trước khi kết luận là lỗi reconnect.

## Hiển thị

Radio dùng chung đường spectrum visualizer với audio. Không nên mô tả một “INFO display mode” riêng cho radio: trạng thái hiện tại không chuyển radio sang một renderer thông tin độc lập theo MCP. Nội dung hiển thị còn phụ thuộc loại display của board.

## Ví dụ sử dụng bằng giọng nói

Các tình huống sử dụng giọng nói thực tế:

- **Phát đài radio theo tên hoặc kênh:**
  - Người dùng: “Mở đài VOV1” hoặc “Bật VOV3”
    `{"name": "self.radio.play_station", "arguments": {"station_name": "VOV1"}}`
  - Người dùng: “Phát VOV giao thông Hà Nội” hoặc “Bật đài giao thông TP.HCM”
    `{"name": "self.radio.play_station", "arguments": {"station_name": "VOV Giao thông HN"}}`
  - Người dùng: “Nghe radio đi”
    `{"name": "self.radio.play_station", "arguments": {"station_name": "VOV1"}}` *(AI chọn đài mặc định)*

- **Xem danh sách các đài phát thanh có sẵn:**
  - Người dùng: “Có những đài phát thanh nào?” hoặc “Danh sách kênh radio”
    `{"name": "self.radio.get_stations", "arguments": {}}`

*Lưu ý phân biệt:* Nếu người dùng nói “phát bài hát” hoặc “mở nhạc”, hãy sử dụng `self.music.play_song` hoặc `self.sdmusic.*`; tính năng `self.radio.*` chỉ dành riêng cho đài phát thanh radio broadcast.

## Xử lý lỗi

| Hiện tượng | Kiểm tra |
|---|---|
| Không tìm thấy đài | Dùng key/tên có trong `get_stations`; kiểm tra URL preset |
| Có success nhưng không nghe tiếng | Wi-Fi, buffer PSRAM, codec AAC/MP3 và stream server |
| Stream dừng nhưng không reconnect | Có thể là EOS bình thường; kiểm tra log `n == 0` và gọi lại tool |
| Board treo hoặc thiếu bộ nhớ | Kiểm tra PSRAM và mô hình dual-core của board |
