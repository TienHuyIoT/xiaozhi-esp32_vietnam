# Weather Feature - Dự báo thời tiết

## Giới thiệu

Tính năng **Weather** cho phép thiết bị ESP32-S3 hiển thị thời tiết hiện tại và dự báo 5 ngày trên màn hình LCD khi ở trạng thái idle (không sử dụng). Thời tiết được lấy từ API **OpenWeatherMap** và có thể tự động phát hiện vị trí qua địa chỉ IP.

Hệ thống bao gồm các thành phần:

| Thành phần | File chính | Chức năng |
|---|---|---|
| **Weather Service** | `main/features/weather/weather_service.{h,cc}` | Gọi API, parse dữ liệu thời tiết |
| **Weather UI** | `main/features/weather/weather_ui.{h,cc}` | Giao diện LCD với đồng hồ lật + thời tiết |
| **Weather Model** | `main/features/weather/weather_model.h` | Cấu trúc dữ liệu WeatherInfo, IdleCardInfo |
| **Weather Config** | `main/features/weather/weather_config.h` | Hằng số cấu hình, API endpoint |

---

## Cấu hình

### 1. Kconfig (`idf.py menuconfig`)

Mở menu cấu hình và bật:

```
Xiaozhi Assistant
└── Enable Weather Feature (Idle Display for LCD)  ──→  CONFIG_WEATHER_IDLE_DISPLAY_ENABLE=y
```

Khi bật `CONFIG_WEATHER_IDLE_DISPLAY_ENABLE=y`, toàn bộ thành phần weather sẽ được biên dịch:

- `WeatherService` được khởi tạo trong `application.cc` dòng 1649
- Task `weather_idle_task` chạy độc lập, cập nhật đồng hồ mỗi giây và fetch thời tiết mỗi 30 phút
- `WeatherUI` được tạo trong `lcd_display.cc` dòng 200
- Giao diện idle card hiển thị khi không có media đang phát

### 2. Cấu hình trong Settings

Có thể đặt city và API key qua Settings (key-value store), hoặc để mặc định:

| Key | Giá trị mặc định | Mô tả |
|---|---|---|
| `weather_city` | `""` (auto detect) | Tên thành phố cần xem thời tiết |
| `weather_api_key` | `""` (dùng key mặc định) | OpenWeatherMap API key |

> **Lưu ý:** Nếu `weather_city` rỗng hoặc là `"auto"`, hệ thống sẽ tự động phát hiện thành phố qua IP (`ipwho.is`). Nếu phát hiện thất bại, mặc định fallback về **Hanoi**.

### 3. weather_config.h

File: `main/features/weather/weather_config.h`

```c
// Khoảng thời gian cập nhật thời tiết (30 phút)
#define WEATHER_UPDATE_INTERVAL_MS (30 * 60 * 1000)

// OpenWeatherMap API key mặc định (demo key, giới hạn usage)
#define OPEN_WEATHERMAP_API_KEY_DEFAULT "ae8d3c2fda691593ce3e84472ef25784"

// API endpoints
#define WEATHER_API_ENDPOINT "https://api.openweathermap.org/data/2.5/"
#define IP_LOCATION_API_ENDPOINT "https://ipwho.is"

// Thành phố mặc định khi không phát hiện được IP
#define CITY_LOCATION_DEFAULT "Hanoi"

// Timeout cho HTTP request (10 giây)
#define WEATHER_HTTP_TIMEOUT_MS 10000
```

---

## Luồng hoạt động

### Khởi động

```
1. application.cc → StartWeatherIdleTask() (dòng 1649)
2. Tạo task "weather_idle_task" với stack 6KB, priority 2
3. Sau 5 giây đầu tiên → gọi FetchWeatherData()
4. Mỗi 30 phút → gọi lại FetchWeatherData()
5. Mỗi giây → gọi UpdateIdleDisplay() để cập nhật đồng hồ
```

### Fetch dữ liệu thời tiết

```
1. Kiểm tra WiFi đã kết nối (chờ tối đa 20 giây)
2. Lấy city từ settings hoặc auto-detect qua IP
3. Gọi API: GET /weather?q={city}&appid={key}&units=metric&lang=vi
4. Parse JSON → WeatherInfo (temp, humidity, feels_like, wind, icon...)
5. Gọi API: GET /forecast?q={city}&appid={key}&units=metric&cnt=40
6. Parse JSON → lấy 5 mốc dự báo (12:00 mỗi ngày)
7. Lưu vào WeatherInfo, đánh dấu valid=true
8. Gọi UpdateIdleDisplay() để render lên LCD
```

---

## Giao diện Idle Card

### Màn hình chính

```
┌─────────────────────────────────────┐
│  🔋 85%            📶 -45 dBm      │  ← Header (pin + wifi)
│                                     │
│    ┌──┐  ┌──┐  ┌──┐  ┌──┐         │
│    │ 1│ :│ 2│  │ 3│ :│ 4│         │  ← Đồng hồ lật (HH:MM)
│    └──┘  └──┘  └──┘  └──┘         │
│                                     │
│     Thứ Năm, 08/05/2025            │  ← Ngày tháng
│     📍 Hanoi                       │  ← Vị trí
│     ☀️  32°C                       │  ← Thời tiết hiện tại
└─────────────────────────────────────┘
```

### Thành phần UI (weather_ui.cc)

| Thành phần | Mô tả |
|---|---|
| **Header** | Pin (biểu tượng + %), Wifi (RSSI + icon) |
| **Đồng hồ lật** | 4 thẻ flip card cho HH:MM, animation lật khi đổi số |
| **Ngày tháng** | Thứ, ngày/tháng/năm (tiếng Việt) |
| **Vị trí** | Tên thành phố từ OpenWeatherMap |
| **Khối thời tiết** | Icon thời tiết + nhiệt độ hiện tại |

### Mã icon thời tiết (Font Awesome)

| Mã OpenWeatherMap | Icon | Mô tả |
|---|---|---|
| `01d`, `01n` | ☀️ | Trời quang |
| `02d`, `02n` | 🌤️ | Ít mây |
| `03d`, `03n`, `04d`, `04n` | ☁️ | Mây |
| `09d`, `09n` | 🌧️ | Mưa |
| `10d`, `10n` | 🌦️ | Mưa rào |
| `11d`, `11n` | ⛈️ | Giông |

---

## Cấu trúc dữ liệu

### WeatherInfo (weather_model.h)

Dữ liệu thời tiết từ API:

```c
struct WeatherInfo {
    std::string city;         // Tên thành phố
    std::string description;  // Mô tả thời tiết ("Nhiều Mây")
    std::string icon_code;    // Mã icon ("03d", "01n"...)
    float temp = 0.0f;       // Nhiệt độ hiện tại (°C)
    int humidity = 0;         // Độ ẩm (%)
    float feels_like = 0.0f; // Nhiệt độ cảm giác (°C)
    int pressure = 0;         // Áp suất khí quyển (hPa)
    float wind_speed = 0.0f; // Tốc độ gió (m/s)
    bool valid = false;       // Dữ liệu có hợp lệ không

    std::vector<ForecastItem> forecast;  // Dự báo 5 ngày
};

struct ForecastItem {
    std::string day_name;   // "T2", "T3", "CN"...
    std::string icon_code;  // Mã icon dự báo
    float temp;             // Nhiệt độ dự báo (°C)
};
```

### IdleCardInfo (weather_model.h)

Dữ liệu truyền vào UI để hiển thị:

```c
struct IdleCardInfo {
    std::string city;
    std::string temperature_text;   // "32°C"
    std::string description_text;    // "Nhiều Mây"
    std::string humidity_text;       // "75%"
    std::string wind_text;          // "3.5 m/s"
    std::string pressure_text;
    std::string battery_icon;       // Icon Font Awesome
    std::string network_icon;       // Icon Font Awesome
    const char* icon = nullptr;     // Icon thời tiết
    int battery_level = 100;
    bool is_charging = false;
    int8_t rssi = 0;
    std::vector<ForecastItem> forecast;
};
```

---

## Lưu ý khi sử dụng

### Về API

1. **OpenWeatherMap API key:** Key mặc định có giới hạn usage. Khuyến nghị đăng ký key riêng tại [openweathermap.org/api](https://openweathermap.org/api) (miễn phí với 1000 calls/ngày).
2. **Ngôn ngữ:** API gọi với `lang=vi` nên mô tả thời tiết trả về tiếng Việt.
3. **Đơn vị:** Dùng `units=metric` (nhiệt độ °C, tốc độ m/s).
4. **Auto-detect city:** Hệ thống dùng `ipwho.is` để phát hiện vị trí. Nếu thất bại → fallback về Hanoi.

### Về hiển thị

1. **Idle display:** Giao diện thời tiết chỉ hiển thị khi không có media đang phát. Khi bắt đầu phát nhạc/video → tự động ẩn (`HideIdleCard()`).
2. **Đồng hồ lật:** Animation flip card mỗi phút. Mỗi card có 3 lớp chồng (shadow, highlight, white) tạo hiệu ứng 3D.
3. **Task priority:** `weather_idle_task` chạy ở priority 2, stack 6KB. Task cập nhật mỗi giây nên cần đủ stack.
4. **Double-buffer:** Giao diện weather không dùng double-buffer riêng, dùng chung với LVGL display.

### Về cấu hình build

1. **Khi không bật** `CONFIG_WEATHER_IDLE_DISPLAY_ENABLE`:
   - Toàn bộ code weather bị exclude qua `#ifdef`
   - Không ảnh hưởng binary size
   - Các thành phần UI vẫn build nhưng `WeatherUI` không được khởi tạo
2. **WiFi required:** Weather cần WiFi kết nối để gọi API. Nếu không có WiFi, task sẽ chờ tối đa 20 giây rồi bỏ qua.

### Về tối ưu

1. **Update interval:** Mặc định 30 phút. Có thể thay đổi `WEATHER_UPDATE_INTERVAL_MS` trong `weather_config.h`.
2. **Forecast:** API `/forecast` trả về 40 mốc (5 ngày × 8 mốc/ngày). Hệ thống chỉ lấy mốc 12:00 của mỗi ngày → 5 ngày dự báo.
3. **Memory:** Response forecast có thể lên tới 20KB. Hệ thống đọc chunk 1KB để tránh tràn buffer.
