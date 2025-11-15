# FFT Display Refactoring

## Tổng quan

Dự án này đã được refactor để tách biệt logic hiển thị FFT khỏi các class display cụ thể, tạo ra một hệ thống linh hoạt và có thể tái sử dụng cho nhiều loại display khác nhau.

## Kiến trúc mới

### 1. FFTDisplay Class (fft_display.h/.cc)
- **Chức năng chính**: Xử lý tất cả logic FFT và rendering
- **Tính năng**:
  - FFT processing với Hanning window
  - Multiple effect types (Spectrum bars, Waveform, Circular, Waterfall)  
  - Thread-safe audio data processing
  - Memory management cho PSRAM
  - Real-time visualization với 20 FPS

### 2. DisplayAdapter Interface
- **Abstract base class** cho các loại display khác nhau
- **Methods cần implement**:
  ```cpp
  virtual void drawPixel(int x, int y, uint16_t color) = 0;
  virtual void drawLine(int x0, int y0, int x1, int y1, uint16_t color) = 0;  
  virtual void drawRect(int x, int y, int width, int height, uint16_t color, bool filled) = 0;
  virtual void clearDisplay() = 0;
  virtual void updateDisplay() = 0;
  ```

### 3. Concrete Adapters

#### LCDDisplayAdapter
- Dành cho LCD displays sử dụng LVGL
- Support full color (RGB565)
- Hardware acceleration thông qua LVGL canvas

#### OLEDDisplayAdapter  
- Dành cho OLED displays (SSD1306/SH1106)
- Monochrome framebuffer
- Software rendering với Bresenham algorithms

## Cách sử dụng

### 1. Tích hợp với LCD Display

```cpp
// Trong LcdDisplay constructor
fft_adapter_ = std::make_unique<LCDDisplayAdapter>(width_, height_);
if (fft_adapter_->initialize()) {
    fft_display_ = std::make_unique<FFTDisplay>(std::move(fft_adapter_));
}

// Sử dụng
display->start();                           // Bắt đầu FFT
auto buffer = display->createAudioDataBuffer(1152);
display->updateAudioDataBuffer(pcm_data, size);
display->stopFft();                         // Dừng FFT
```

### 2. Tích hợp với OLED Display

```cpp  
// Tương tự LCD nhưng sử dụng LVGL adapter cho OLED
fft_adapter_ = std::make_unique<LCDDisplayAdapter>(width_, height_);
// OLED cũng có thể dùng OLEDDisplayAdapter cho performance tốt hơn
```

### 3. Tích hợp với Radio/Music Players

```cpp
// Trong esp32_radio.cc PlayRadioStream method
if (display && display_mode_ == DISPLAY_MODE_SPECTRUM) {
    auto audio_buffer = display->createAudioDataBuffer(pcm_size_bytes);
    display->updateAudioDataBuffer(amplified_buffer.data(), pcm_size_bytes);
}
```

## Effect Types

### 1. SPECTRUM_BARS (Default)
- Traditional vertical bars
- Color gradient based on frequency and magnitude
- Smooth animation với rise/fall rates

### 2. WAVEFORM  
- Real-time waveform display
- Shows audio signal amplitude over time

### 3. CIRCULAR
- Circular spectrum visualization  
- Bars radiate from center
- Great cho round displays

### 4. WATERFALL
- Scrolling spectrum history
- Shows frequency content over time
- (Chưa fully implemented)

## Performance Optimization

### Memory Management
- Sử dụng PSRAM cho large buffers
- Efficient buffer reuse
- Proper cleanup trong destructors

### Threading
- Separate FFT processing thread
- 50ms update interval (20 FPS)
- Non-blocking audio data updates

### Display Optimization
- Minimal screen redraws
- Hardware acceleration khi available
- Adaptive quality dựa trên display capabilities

## API Changes

### LcdDisplay/OledDisplay
- **Removed**: Direct FFT implementation
- **Added**: FFT proxy methods to FFTDisplay
- **Maintained**: Same public interface

### New Methods
```cpp
// FFT control
virtual void start() override;
virtual void stopFft() override;
virtual void clearScreen() override;

// Audio data interface  
virtual int16_t* createAudioDataBuffer(size_t sample_count) override;
virtual void updateAudioDataBuffer(int16_t* data, size_t sample_count) override;
virtual void releaseAudioDataBuffer(int16_t* buffer) override;
```

## Migration Guide

### Existing Code
Không cần thay đổi code sử dụng Display interface. Tất cả methods vẫn hoạt động như trước.

### New Projects
Có thể sử dụng FFTDisplay directly cho custom displays:

```cpp
auto adapter = std::make_unique<CustomDisplayAdapter>(width, height);
auto fft_display = std::make_unique<FFTDisplay>(std::move(adapter));
fft_display->setEffect(FFTEffect::CIRCULAR);
fft_display->start();
```

## Extensibility

### Adding New Display Types
1. Implement DisplayAdapter interface
2. Handle color vs monochrome rendering  
3. Optimize cho hardware capabilities

### Adding New Effects
1. Add enum value to FFTEffect
2. Implement rendering method trong FFTDisplay
3. Add effect switching logic

### Custom Visualizations
Có thể extend FFTDisplay hoặc create custom classes sử dụng DisplayAdapter pattern.

## Files Changed

### New Files
- `main/display/fft_display.h` - FFT display class và adapters
- `main/display/fft_display.cc` - Implementation  
- `main/display/fft_display_example.cc` - Usage examples

### Modified Files
- `main/display/lcd_display.h` - Added FFT integration
- `main/display/lcd_display.cc` - Replaced FFT implementation
- `main/display/oled_display.h` - Added FFT integration  
- `main/display/oled_display.cc` - Added FFT methods
- `main/CMakeLists.txt` - Added new source files

### Removed Code
- Old FFT implementation trong lcd_display.cc
- Duplicate FFT processing logic
- Display-specific FFT variables

## Benefits

1. **Modularity**: FFT logic tách biệt, dễ maintain
2. **Reusability**: Cùng FFT engine cho tất cả display types
3. **Extensibility**: Dễ dàng add effects và display types mới
4. **Performance**: Optimized threading và memory usage
5. **Maintainability**: Single source of truth cho FFT logic

## Future Enhancements

1. **Hardware Acceleration**: GPU-based FFT processing
2. **Advanced Effects**: 3D visualization, particle systems
3. **Configuration**: Runtime effect switching, color themes
4. **Audio Analysis**: Beat detection, frequency analysis
5. **Multiple Displays**: Simultaneous FFT trên multiple screens