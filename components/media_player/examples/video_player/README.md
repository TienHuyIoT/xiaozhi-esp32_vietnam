# ESP Video Player Example

A straightforward video player demonstration showcasing the essential features of the ESP Media Player component. This example provides a simple, automated playback demo that's perfect for getting started with multimedia applications on ESP32 platforms.

## Overview

The Video Player example demonstrates the core functionality of the ESP Media Player component through a clean, minimal implementation. Unlike the feature-rich CLI example, this focuses on the essential API usage patterns, making it ideal for understanding the basic concepts and integrating media playback into your own applications.

## Features

### 🎬 **Automated Playback Demo**
- **Single-file playbook** with hardcoded test video
- **Automatic playback sequence** - plays once, then loops 5 times
- **Full audio/video rendering** with optimized thread management
- **Resource cleanup** demonstration

### 🎯 **Core API Demonstration**
- **Player lifecycle** - create, configure, play, cleanup
- **Event handling** - comprehensive event callback implementation
- **Loop playback** - demonstrates both single-play and loop modes
- **Render setup** - audio (I2S) and video (LCD) configuration

### ⚡ **Performance Optimized**
- **Multi-core thread scheduling** - optimized for ESP32-P4 dual-core
- **Memory management** - SPIRAM utilization and performance tuning
- **Hardware acceleration** - uses ESP32-P4 video processing capabilities

### 🔧 **Hardware Integration**
- **ESP32-P4 Dev Board** support with DSI display interface
- **ESP32-S3** compatibility with SPI display
- **Audio codec** integration via I2S interface
- **SD card** storage for media files

## Quick Start

### 1. Hardware Setup

#### ESP32-P4 Development Board (Default)
- **Display**: DSI panel interface (automatically detected)
- **Audio**: I2S DAC/codec via development board audio jack
- **Storage**: SD card slot for media files

#### ESP32-S3 Alternative Setup  
- **Display**: SPI-connected LCD panel
- **Audio**: I2S DAC/codec
- **Storage**: SD card via SPI interface

### 2. Prepare Media File
Place your test video file at the expected location:
```bash
# Default test file path
/sdcard/video/xizao.mp4
```

**Supported formats:**
- **Video**: H.264, MJPEG
- **Audio**: AAC, MP3, PCM
- **Container**: MP4, AVI, TS, FLV etc

### 3. Build and Flash
```bash
# For ESP32-P4 (default)
idf.py set-target esp32p4
idf.py build flash monitor

# For ESP32-S3
idf.py set-target esp32s3
idf.py build flash monitor
```

### 4. Expected Behavior
The application will automatically:
1. **Initialize** hardware and codecs
2. **Play** the test video once to completion
3. **Loop play** the same video 5 times
4. **Clean up** and terminate

## Code Structure

### Main Application Flow
```c
void app_main(void)
{
    // 1. System initialization
    media_lib_add_default_adapter();
    set_codec_board_type(TEST_BOARD);
    mount_sdcard();
    init_codec(&init_cfg);
    board_lcd_init();
    
    // 2. Component registration
    register_components();
    
    // 3. Single playback test
    play_file(TEST_VIDEO_FILE, PLAY_MASK_AUDIO | PLAY_MASK_VIDEO, false);
    
    // 4. Loop playback test (5 times)
    play_file(TEST_VIDEO_FILE, PLAY_MASK_AUDIO | PLAY_MASK_VIDEO, true);
    
    // 5. Cleanup
    unregister_components();
}
```

### Player Implementation Pattern
```c
static int play_file(char* file, uint32_t play_mask, bool loop)
{
    // 1. Configure player
    player_cfg_t cfg = { .play_mask = play_mask };
    
    // 2. Setup audio render (if needed)
    if (play_mask & PLAY_MASK_AUDIO) {
        i2s_render_cfg_t i2s_cfg = { .play_handle = get_playback_handle() };
        cfg.audio_render = av_render_alloc_i2s_render(&i2s_cfg);
    }
    
    // 3. Setup video render (if needed)
    if (play_mask & PLAY_MASK_VIDEO) {
        lcd_render_cfg_t lcd_cfg = { .lcd_handle = board_get_lcd_handle() };
        cfg.video_render = av_render_alloc_lcd_render(&lcd_cfg);
    }
    
    // 4. Create and configure player
    media_player_handle_t player = media_player_open(&cfg);
    media_player_set_callback(player, player_event_cb, NULL);
    media_player_set_loop(player, loop);
    media_player_set_source(player, MEDIA_SRC_TYPE_STORAGE, file);
    
    // 5. Start playback and wait for completion
    media_player_play(player);
    // ... event handling loop ...
    
    // 6. Cleanup resources
    media_player_close(player);
    audio_render_free_handle(audio_render);
    video_render_free_handle(video_render);
}
```

## Event Handling

The example demonstrates comprehensive event handling:

```c
static int player_event_cb(player_event_t event, void *ctx)
{
    switch (event) {
        case PLAYER_EVENT_PREPARED:
            // Media file parsed successfully
            break;
        case PLAYER_EVENT_PLAYED_DONE:
            // Playback started successfully
            break;
        case PLAYER_EVENT_AUDIO_EOS:
            // End of stream reached
            break;
        case PLAYER_EVENT_STOPPED:
            // Playback stopped
            break;
        case PLAYER_EVENT_PREPARE_ERROR:
        case PLAYER_EVENT_PLAY_ERROR:
            // Error occurred
            break;
    }
    return 0;
}
```

## Configuration

### Thread Management
Optimized thread priorities for smooth playback:

| Thread | Core | Priority | Stack Size | Purpose |
|--------|------|----------|------------|---------|
| Vdec | 1 | 15 | 10KB | Video decoder |
| ARender | 1 | 21 | 10KB | Audio rendering |
| Adec | 0 | 10 | 15KB | Audio decoder |

### Memory Configuration
- **SPIRAM**: Enabled for large buffer handling
- **Performance optimization**: Compiler optimized for speed
- **FatFS**: Long filename support for media files

### Hardware-Specific Settings

#### ESP32-P4
- **Flash**: QIO mode, 4MB
- **SPIRAM**: 200MHz speed
- **Display**: DSI panel interface
- **Camera**: SC2336 MIPI support (optional)

#### ESP32-S3
- **Display**: SPI-connected LCD
- **Audio**: Standard I2S interface
- **Memory**: SPIRAM configuration optimized

## Customization

### 1. Change Test File
Edit `settings.h`:
```c
#define TEST_VIDEO_FILE "/sdcard/your_video.mp4"
```

### 2. Modify Playback Behavior
Adjust loop settings in `video_player.c`:
```c
static uint32_t loop_limits = 10;  // Change loop count
```

### 3. Audio-Only Mode
```c
uint32_t play_mask = PLAY_MASK_AUDIO;  // Audio only
```

### 4. Video-Only Mode
```c
uint32_t play_mask = PLAY_MASK_VIDEO;  // Video only
```

### 5. Add Network Support
Include network streaming capabilities:
```c
// Add to register_components()
media_src_register_network();

// Use network source
media_player_set_source(player, MEDIA_SRC_TYPE_NETWORK, "http://example.com/video.mp4");
```

## Integration Guide

### Using in Your Application

1. **Copy the core pattern**:
```c
#include "player.h"
#include "av_render_default.h"
// ... other includes

// Use the play_file() function as a template
int your_player_function(char* media_file) {
    // Follow the same pattern as play_file()
}
```

2. **Event-driven approach**:
```c
// Implement your own event callback
static int my_event_callback(player_event_t event, void *ctx) {
    // Handle events according to your application needs
    switch (event) {
        case PLAYER_EVENT_EOS:
            // Your end-of-stream handling
            break;
        // ... other events
    }
}
```

3. **Resource management**:
```c
// Always clean up resources properly
if (player) media_player_close(player);
if (audio_render) audio_render_free_handle(audio_render);
if (video_render) video_render_free_handle(video_render);
```

## Troubleshooting

### Common Issues

1. **"Fail to create player"**
   - **Cause**: Insufficient memory or codec initialization failure
   - **Solution**: Check SPIRAM configuration and codec board settings

2. **"Fail to set data source"**
   - **Cause**: File not found or SD card not mounted
   - **Solution**: Verify file path and SD card mounting

3. **No video output**
   - **Cause**: LCD initialization failure or incorrect display configuration
   - **Solution**: Check display connections and board type settings

4. **No audio output**
   - **Cause**: I2S configuration or audio codec issues
   - **Solution**: Verify audio codec initialization and I2S connections

### Debug Information
The example provides detailed logging:
```
I (123) Player_Test: Got player event:3  // PLAYER_EVENT_PREPARED
I (456) Player_Test: Got player event:4  // PLAYER_EVENT_PLAYED_DONE
I (789) Player_Test: Replay count 1      // Loop iteration
```

## Build Requirements

### Dependencies
- **ESP-IDF**: Version 5.0 or later
- **Media Player Component**: Local override path configured
- **Codec Board Support**: Hardware-specific codec libraries

### Memory Requirements
- **Flash**: Minimum 4MB
- **RAM**: SPIRAM strongly recommended for video playback
- **Heap**: Sufficient free heap for media buffers

## Next Steps

After understanding this example:
1. **Explore CLI example** for advanced features
2. **Integrate into your application** using the provided patterns
3. **Customize for your hardware** setup and requirements
4. **Add network streaming** capabilities as needed

## License

SPDX-License-Identifier: Apache-2.0

---

This example provides the perfect starting point for understanding ESP Media Player basics and integrating multimedia capabilities into your ESP32 applications.
