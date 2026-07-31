# Halo Xiaozhi

Firmware trợ lý giọng nói ESP32 của Halo, phát triển từ Xiaozhi Vietnam. Tài liệu này mô tả đúng trạng thái của source và profile `sdkconfig` đang có trong nhánh `main`; một tính năng chỉ có source hoặc bị tắt trong cấu hình không được xem là đã sẵn sàng cho người dùng.

- Phiên bản firmware: `2.0.5.06`
- ESP-IDF: `>=5.4.0`
- Giấy phép: MIT

## Trạng thái hiện tại

| Tính năng | Trạng thái | Điều kiện chính |
|---|---|---|
| Trợ lý giọng nói, MCP và hội thoại | Hoạt động mặc định | Wi-Fi, backend Xiaozhi và board được hỗ trợ |
| Nhạc online | Hoạt động mặc định | Wi-Fi, music server, codec và PSRAM đủ dùng |
| Internet radio | Hoạt động mặc định | Wi-Fi, stream còn hoạt động, codec và PSRAM |
| Báo thức | Hoạt động mặc định | Đồng bộ NTP để có giờ chính xác |
| Nhạc từ thẻ SD | Có điều kiện | `CONFIG_SD_CARD_ENABLE`, mount SD thành công |
| Spectrum khi phát audio | Có điều kiện | Media đang phát và board có màn hình phù hợp |
| Weather idle card | Tắt trong profile hiện tại | Bật `CONFIG_WEATHER_IDLE_DISPLAY_ENABLE` để thử nghiệm |
| Video AVI/MP4 từ SD | Mã nguồn thử nghiệm, chưa nối runtime | Các lời gọi khởi tạo video đang bị vô hiệu hóa khi build |
| OTA/assets/hiển thị ảnh qua web cục bộ | Hoạt động theo board | Thiết bị đã vào mạng; khả năng hiển thị phụ thuộc display |

## Kiến trúc ngắn

Thiết bị nhận âm thanh từ microphone, hiển thị trạng thái trên LCD/OLED và kết nối Wi-Fi. Cấu hình OTA chọn giao thức MQTT+UDP hoặc WebSocket để giao tiếp với backend ASR/LLM/TTS. Các chức năng thiết bị được expose dưới dạng MCP tools, gồm media, radio, alarm và các thao tác hệ thống.

Ngoài kết nối backend, firmware luôn khởi động web server cục bộ cổng 80 với các endpoint cho OTA, assets và ảnh màn hình:

- `GET /ota` và `POST /ota_upload`: cập nhật firmware qua trình duyệt.
- `GET /assets` và `POST /assets_upload`: quản lý custom assets.
- `POST /api/display_image`: gửi ảnh hiển thị tạm thời nếu board hỗ trợ.

## Profile đang được kiểm chứng

Các giá trị dưới đây lấy từ `sdkconfig` hiện tại, không phải mặc định cho mọi board:

| Mục | Giá trị |
|---|---|
| Chip | ESP32-S3, dual-core, có PSRAM |
| Board | `bread-compact-wifi-lcd` |
| Ngôn ngữ | `CONFIG_LANGUAGE_VI_VN=y` |
| Màn hình | ST7789 240×240 |
| SD card | Bật, giao tiếp SPI |
| Weather | Tắt |
| Wake word | AFE, model “Hey Ivy” |
| OTA URL | `https://api.tenclass.net/xiaozhi/ota/` |

## Bắt đầu nhanh

### 1. Lấy source

Remote `origin` có default branch khác với profile đang kiểm chứng, vì vậy clone rõ nhánh `main`:

```powershell
git clone --branch main https://github.com/halotech95/halo_xiaozhi.git
cd halo_xiaozhi
```

### 2. Nạp môi trường ESP-IDF và build profile hiện tại

Project yêu cầu ESP-IDF `>=5.4.0`. Trên PowerShell:

```powershell
& 'C:\Espressif\tools\Microsoft.v5.5.3.PowerShell_profile.ps1'
idf.py build
idf.py -p COMx flash monitor
```

Không chạy `idf.py set-target` trước khi build profile hiện tại; lệnh đó tái tạo cấu hình và có thể thay đổi `sdkconfig`. Hãy thay `COMx` bằng cổng serial của thiết bị.

### 3. Đổi chip hoặc board

```powershell
idf.py set-target esp32s3
idf.py menuconfig
idf.py build
idf.py -p COMx flash monitor
```

Các target được project chuẩn bị gồm ESP32, ESP32-C3, ESP32-S3, ESP32-C6 và ESP32-P4. Sau khi đổi board, phải kiểm tra lại codec, PSRAM, màn hình, SD và các tính năng có điều kiện.

`scripts/release.py` có thể liệt kê/tạo cấu hình release cho các board có `config.json`:

```powershell
python -X utf8 scripts/release.py --list-boards
python -X utf8 scripts/release.py <board> --name <variant>
```

Script này thay đổi target và cấu hình build. Board `bread-compact-wifi-lcd` trong profile hiện tại không có `config.json`, nên hãy build trực tiếp bằng `sdkconfig` thay vì giả định rằng release script sẽ hỗ trợ board đó.

## Sử dụng thiết bị

1. Khởi động thiết bị, mở Wi-Fi setup theo hướng dẫn trên màn hình và kết nối vào mạng gia đình. AP cấu hình do firmware tạo có tiền tố `TienHuyIoT`.
2. Chờ thiết bị đồng bộ mạng/backend. Dùng câu lệnh tự nhiên, ví dụ “đặt báo thức 7 giờ”, “mở VOV3” hoặc “phát bài …”.
3. Khi biết địa chỉ IP, mở `http://<device-ip>/ota` để kiểm tra web server cục bộ và cập nhật firmware/assets theo nhu cầu.
4. Để phát nhạc trong thẻ, phải nói rõ “nhạc trong thẻ nhớ” hoặc “nhạc offline”; câu “phát nhạc” mặc định đi vào online music.

Hướng dẫn chi tiết và các giới hạn đã kiểm chứng:

- [Báo thức](docs/features/alarm.md)
- [Internet radio](docs/features/radio.md)
- [Weather idle card](docs/features/weather.md)
- [Nhạc và media từ SD](docs/features/media-player.md)
- [Nhạc online](docs/features/online-music.md)

## Tài liệu hệ thống

Các tài liệu hệ thống nằm trong thư mục [`docs`](docs):

- [Custom board](docs/custom-board.md)
- [MCP usage](docs/mcp-usage.md) và [MCP protocol](docs/mcp-protocol.md)
- [MQTT + UDP](docs/mqtt-udp.md)
- [WebSocket](docs/websocket.md)
- [AV render architecture](docs/av_render_architecture.md)

Các tài liệu trên không nằm trong đợt audit tính năng này; hãy đối chiếu source nếu cần dùng làm đặc tả giao thức. Tài liệu board/component cục bộ vẫn được giữ cạnh source của component.

## Cấu trúc repository

```text
main/                Application, board, audio, display và MCP tools
docs/                Tài liệu hệ thống và tính năng
scripts/              Script build/release và sinh assets
boards/               Cấu hình board
components/           Managed/local ESP-IDF components
sdkconfig             Profile build đang được kiểm chứng
```

## Nguồn dự án

- Repository: <https://github.com/halotech95/halo_xiaozhi>
- Issues: <https://github.com/halotech95/halo_xiaozhi/issues>
- Upstream tham khảo: <https://github.com/TienHuyIoT/xiaozhi-esp32_vietnam>

## Phạm vi tài liệu

README này và năm tài liệu trong `docs/features` được đối chiếu tĩnh với CMake, Kconfig, `sdkconfig`, `Application` và các lớp feature hiện tại. Không có firmware/API nào được thay đổi trong đợt cập nhật tài liệu này.
