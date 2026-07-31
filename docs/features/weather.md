# Weather idle card

## Trạng thái hiện tại

Weather có source, service và UI trong firmware nhưng **đang tắt** ở profile hiện tại (`sdkconfig` không đặt `CONFIG_WEATHER_IDLE_DISPLAY_ENABLE`). Kconfig mặc định cũng tắt. Source vẫn nằm trong build; không được mô tả là bị loại hoàn toàn khỏi binary.

Để thử nghiệm, bật `CONFIG_WEATHER_IDLE_DISPLAY_ENABLE=y` trong `idf.py menuconfig`, dùng board có `LcdDisplay`, Wi-Fi và đủ tài nguyên UI, sau đó build lại.

## Luồng chạy

Khi flag được bật:

1. Application tạo weather idle task sau khi network sẵn sàng.
2. Task chỉ cập nhật khi thiết bị ở trạng thái idle và không có media đang phát.
3. Lần fetch đầu được gọi ở khoảng giây thứ 5; các lần sau theo điều kiện hard-code 1800 giây (30 phút) trong `Application`.
4. Service gọi OpenWeatherMap cho current weather và cố gắng lấy forecast.

`WEATHER_UPDATE_INTERVAL_MS` trong `weather_config.h` được dùng bởi `NeedsUpdate()`, nhưng scheduler hiện tại không gọi hàm đó để quyết định chu kỳ. Chỉ đổi macro này không thay đổi lịch fetch thực tế; muốn đổi chu kỳ phải sửa caller trong `Application`.

## Cấu hình

| Cấu hình | Hành vi thực tế |
|---|---|
| `CONFIG_WEATHER_IDLE_DISPLAY_ENABLE` | Bật/tắt task và card idle |
| `wifi/weather_city` | Thành phố sử dụng khi setting có giá trị; rỗng/`auto` sẽ dò theo IP |
| `wifi/weather_api_key` | Có đường đọc setting, nhưng constructor hiện khởi tạo key mặc định khác rỗng nên setting thường không ghi đè được |
| API endpoint | OpenWeatherMap current + forecast, đơn vị metric |

Dò IP đọc trường `region` từ dịch vụ định vị, không phải trường `city`. Nếu dò thất bại hoặc Wi-Fi chưa sẵn sàng, fetch bị bỏ qua/giữ trạng thái cũ.

## Dữ liệu và UI

Current weather lưu thành phố, nhiệt độ, feels-like, độ ẩm, áp suất, gió, icon và mô tả. Forecast parser lấy các mốc 12:00 và có thể lưu tối đa 5 `ForecastItem`. Việc fetch forecast thất bại không làm current weather mất hiệu lực nếu current request đã thành công.

Idle card hiện ưu tiên đồng hồ/ngày, vị trí, icon, nhiệt độ và thông tin current. Không quảng bá rằng màn hình hiện tại chắc chắn render đầy đủ 5 ngày forecast; vector forecast có trong model nhưng phần UI phải được kiểm tra riêng trên board.

Board không có battery gauge sẽ hiển thị mức pin không xác định thay vì số phần trăm thực.

## Cách sử dụng

Tình huống vận hành và sử dụng:

- **Tự động hiển thị thẻ thời tiết khi idle:**
  - Tính năng thời tiết **không có MCP tool độc lập** và không nhận câu lệnh giọng nói trực tiếp để xem thời tiết.
  - Khi được bật cấu hình (`CONFIG_WEATHER_IDLE_DISPLAY_ENABLE=y`) và thiết bị ở trạng thái rảnh (Idle), màn hình sẽ tự động hiển thị thẻ thời tiết cùng đồng hồ/ngày tháng.
  - Thiết bị tự động cập nhật dữ liệu thời tiết định kỳ (khoảng 30 phút một lần) từ OpenWeatherMap qua Wi-Fi.

- **Ẩn thẻ thời tiết trong các tình huống hoạt động:**
  - Khi thiết bị đang phát nhạc, phát đài radio hoặc đang trong phiên hội thoại giọng nói với người dùng, thẻ thời tiết sẽ tự động bị ẩn để nhường màn hình cho giao diện tương ứng và việc tải dữ liệu thời tiết sẽ tạm hoãn.

## Giới hạn đã biết

- Profile hiện tại không hiển thị weather vì flag đang tắt.
- Cần LCD; tài liệu không được hứa hỗ trợ OLED/cellular như một UI weather tương đương.
- API key thay đổi qua setting chưa đáng tin cậy do giá trị mặc định trong constructor; muốn dùng key khác cần sửa cấu hình/source rồi build.
- Chu kỳ “30 phút” nằm trong application loop, không tự lấy từ `WEATHER_UPDATE_INTERVAL_MS`.
- Forecast là best-effort; current weather có thể hợp lệ dù forecast không tải được.

## Xử lý lỗi

1. Kiểm tra `CONFIG_WEATHER_IDLE_DISPLAY_ENABLE=y` trong `sdkconfig`.
2. Kiểm tra Wi-Fi đã connected và timezone/network đã sẵn sàng.
3. Kiểm tra log HTTP/status OpenWeatherMap và thành phố đã URL-encode đúng.
4. Nếu card không xuất hiện, xác nhận board trả về `LcdDisplay` và thiết bị đang idle, không phát media.
