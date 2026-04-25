# ESP Media Player Component

A comprehensive multimedia player component for ESP32 platforms, supporting audio and video playback with advanced features like seeking, speed control, and multiple data sources.

## Features

### 🎵 **Multi-Media Support**
- **Audio playback** with various codec support
- **Video playback** with hardware acceleration
- **Subtitle support** (future enhancement)
- **Configurable media types** via play masks

### 📡 **Flexible Data Sources**
- **Network streaming** (HTTP/HTTPS URLs)
- **Local storage** (SD card, flash storage)
- **Callback-based input** for custom data sources
- **FIFO buffer input** for real-time streaming
- **Push mode** for manual data feeding (planned)

### 🎮 **Playback Control**
- **Basic controls**: Play, pause, stop, resume
- **Speed control**: Variable playback speed (0.5x, 1x, 2x, etc.)
- **Seeking**: Time-based seeking with millisecond precision
- **Loop playback**: Auto-replay functionality
- **Position tracking**: Get current playback position and duration

### 🔧 **Advanced Features**
- **Event-driven architecture** with comprehensive callback system
- **Stream information** querying and management
- **Interrupt/recover** functionality for resource optimization
- **Configurable FIFO buffers** for async processing
- **Debug utilities** for performance monitoring

## Architecture

```
┌─────────────────┐    ┌──────────────┐    ┌─────────────────┐
│   Data Source   │───▶│   Extractor  │───▶│    Decoder      │
│ (Network/File)  │    │  (Demuxer)   │    │ (Audio/Video)   │
└─────────────────┘    └──────────────┘    └─────────────────┘
                                                      │
                                                      ▼
┌─────────────────┐    ┌──────────────┐    ┌─────────────────┐
│   Application   │◀───│   Events     │    │    Renderer     │
│   (Your Code)   │    │  Callbacks   │    │ (Audio/Video)   │
└─────────────────┘    └──────────────┘    └─────────────────┘
```

## Quick Start

### 1. Basic Setup

```c
#include "player.h"
#include "av_render.h"

// Initialize audio and video renderers
audio_render_handle_t audio_render = audio_render_create(...);
video_render_handle_t video_render = video_render_create(...);

// Configure player
player_cfg_t cfg = {
    .play_mask = PLAY_MASK_AUDIO | PLAY_MASK_VIDEO,
    .audio_render = audio_render,
    .video_render = video_render,
    .no_accurate_seek = false
};

// Create player instance
media_player_handle_t player = media_player_open(&cfg);
```

### 2. Set Event Callback

```c
int player_event_callback(player_event_t event, void *ctx) {
    switch (event) {
        case PLAYER_EVENT_PREPARED:
            printf("Media file prepared, ready to play\n");
            break;
        case PLAYER_EVENT_PLAYED_DONE:
            printf("Playback started successfully\n");
            break;
        case PLAYER_EVENT_EOS:
            printf("Playback finished\n");
            break;
        case PLAYER_EVENT_PLAY_ERROR:
            printf("Playback error occurred\n");
            break;
        // Handle other events...
    }
    return 0;
}

media_player_set_callback(player, player_event_callback, NULL);
```

### 3. Play Media File

```c
// Set media source (local file)
media_player_set_source(player, MEDIA_SRC_TYPE_FILE, "/sdcard/video.mp4");

// Or network stream
media_player_set_source(player, MEDIA_SRC_TYPE_HTTP, "http://example.com/stream.mp4");

// Start playback
media_player_play(player);
```

## API Reference

### Core Functions

#### `media_player_open(player_cfg_t *cfg)`
Creates a new player instance with specified configuration.

#### `media_player_set_source(player, src_type, uri)`
Sets the media source (file path or URL).

#### `media_player_play(player)`
Starts or resumes playback.

#### `media_player_stop(player)`
Stops playback (can be resumed with play).

#### `media_player_close(player)`
Destroys the player instance and frees resources.

### Playback Control

#### `media_player_set_speed(player, speed)`
Controls playback speed:
- `0.0` - Pause
- `1.0` - Normal speed
- `0.5` - Half speed
- `2.0` - Double speed

#### `media_player_seek_time(player, time_ms)`
Seeks to specific time position in milliseconds.

#### `media_player_set_loop(player, loop)`
Enables/disables loop playback.

### Information Retrieval

#### `media_player_get_position(player, &position)`
Gets current playback position in milliseconds.

#### `media_player_get_duration(player, &duration)`
Gets total media duration in milliseconds.

#### `media_player_get_stream_info(player, stream_type, idx, &info)`
Retrieves information about specific audio/video streams.

## Advanced Usage

### Custom Data Source

```c
int my_read_callback(void *data, uint32_t size, void *ctx) {
    // Read data from custom source
    return bytes_read;
}

int my_seek_callback(uint32_t position, void *ctx) {
    // Seek in custom source
    return 0; // success
}

media_player_set_source_by_callback(player, my_read_callback,
                                   my_seek_callback, my_context);
```

### Performance Optimization

```c
player_fifo_cfg_t fifo_cfg = {
    .adec_fifo_size = 8192,    // Audio decoder buffer
    .vdec_fifo_size = 16384,   // Video decoder buffer
    .arender_fifo_size = 4096, // Audio render buffer
    .vrender_fifo_size = 8192, // Video render buffer
    .extractor_pool_size = 10  // Frame pool size
};

media_player_set_fifo_size(player, &fifo_cfg);
```

### Resource Management

```c
// Temporarily free resources while keeping playback position
media_player_interrupt(player);

// ... other operations ...

// Resume from interrupted position
media_player_recover(player);
```

## Event Types

| Event | Description |
|-------|-------------|
| `PLAYER_EVENT_SRC_CONNECTING` | Connecting to data source |
| `PLAYER_EVENT_SRC_CONNECTED` | Successfully connected |
| `PLAYER_EVENT_PREPARED` | Media file parsed and ready |
| `PLAYER_EVENT_PLAYED_DONE` | Playback started |
| `PLAYER_EVENT_EOS` | End of stream reached |
| `PLAYER_EVENT_SEEK_DONE` | Seek operation completed |
| `PLAYER_EVENT_PLAY_ERROR` | Playback error occurred |

## Error Codes

| Code | Constant | Description |
|------|----------|-------------|
| `-1` | `PLAYER_ERR_INVAID_ARG` | Invalid argument |
| `-2` | `PLAYER_ERR_NOT_ALLOW` | Operation not allowed |
| `-3` | `PLAYER_ERR_NO_MEM` | Out of memory |

## Dependencies

- **ESP-IDF** framework
- **esp_extractor** - Media parsing and demuxing
- **av_render** - Audio/video rendering
- **media_src** - Data source abstraction

## Examples

Check the `examples/` directory for complete working examples:
- `video_player/` - Basic video player implementation
- `player_cli/` - Command-line media player

## Performance Considerations

1. **Memory Usage**: Configure FIFO sizes based on available RAM
2. **Threading**: Larger FIFO buffers enable async processing but use more resources
3. **Seeking**: Disable accurate seek (`no_accurate_seek = true`) for faster seeking
4. **Network Streaming**: Ensure stable network connection for smooth playback

## Supported Formats

- **Video**: H.264, MJPEG
- **Audio**: AAC, MP3, PCM
- **Containers**: MP4, AVI, MPEG-TS, WAV, FLV, OGG etc

## License

SPDX-License-Identifier: LicenseRef-Espressif-Modified-MIT

See LICENSE file for details.
