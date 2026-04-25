# ESP Player CLI Example

A comprehensive command-line interface example demonstrating the full capabilities of the ESP Media Player component. This interactive CLI provides extensive media playback control, debugging tools, and system monitoring capabilities.

## Overview

The Player CLI example transforms your ESP32 device into a powerful media player with a full-featured command-line interface accessible via UART, USB CDC, or USB Serial JTAG. It supports advanced features like dual player instances, file system browsing, network streaming, and comprehensive debugging tools.

## Features

### 🎮 **Dual Player Support**
- **Two simultaneous players** (player 0 and player 1)
- **Independent control** of each player instance
- **Configurable play masks** (audio-only, video-only, or both)

### 📁 **File System Management**
- **Directory browsing** with filtering
- **Recursive folder scanning** with extension filtering
- **Pattern-based file search** and playlist management
- **Auto-play sequences** with next/previous navigation

### 🌐 **Network Capabilities**
- **HTTP/HTTPS streaming** support
- **Network speed testing** utilities
- **Remote file server** integration
- **Download performance monitoring**

### 🎵 **Advanced Playback Control**
- **Variable speed playback** (0.5x to 2x speed)
- **Precision seeking** (millisecond accuracy)
- **Loop playback** functionality
- **Pause/resume** with position tracking
- **Interrupt/restore** for resource management

### 🔧 **Data Source Flexibility**
- **File-based playback** (SD card, flash storage)
- **Network streaming** (HTTP URLs)
- **Callback-based input** (custom data sources)
- **Buffer-based input** (FIFO streaming)

### 🛠️ **Debugging & Monitoring**
- **Memory leak detection** and tracing
- **CPU usage monitoring** and profiling
- **Heap analysis** with detailed reports
- **Pipeline data dumping** for analysis
- **Event tracking** and status monitoring

## Hardware Requirements

### Supported Boards
- **ESP32-P4 Development Board** (default configuration)
- **ESP32-S3** with display and audio codec
- **Custom boards** with appropriate codec and display hardware

### Hardware Components
- **Audio Output**: I2S DAC/Codec (automatically detected)
- **Video Output**: LCD display via SPI/DSI interface
- **Storage**: SD card for local media files
- **Network**: WiFi connectivity for streaming

## Quick Start

### 1. Build and Flash
```bash
idf.py set-target esp32p4  # or esp32s3
idf.py build
idf.py flash monitor
```

### 2. Connect to Console
The CLI interface is available via:
- **UART**: Serial console (default)
- **USB CDC**: USB serial interface
- **USB Serial JTAG**: Built-in USB debugging

### 3. Basic Usage
```
esp> play -uri /sdcard/video.mp4 -mask 3  # Play local file
esp> pause                          # Pause playback
esp> resume                         # Resume playback
esp> seek 30000                     # Seek to 30 seconds
esp> stop                           # Stop playback
esp> close                          # Close playback
```

## CLI Commands Reference

### **Player Management**
| Command | Parameters | Description |
|---------|------------|-------------|
| `open` | `-mask 1/2/3` `-sel 0/1` | Create player instance |
| `close` | `-sel 0/1` | Close player instance |
| `play` | `-uri path` `-sel 0/1` | Start playback |
| `stop` | `-sel 0/1` | Stop playback |
| `pause` | `-sel 0/1` | Pause playback |
| `resume` | `-sel 0/1` | Resume playback |
| `close` | `-sel 0/1` | Clsoe playback |

### **Playback Control**
| Command | Parameters | Description |
|---------|------------|-------------|
| `seek` | `time_ms` `-sel 0/1` | Seek to position |
| `speed` | `0.5/1.0/2.0` `-sel 0/1` | Set playback speed |
| `loop` | `0/1` `-sel 0/1` | Enable/disable loop playback|
| `vol` | `volume` | Set audio volume |

### **File System Navigation**
| Command | Parameters | Description |
|---------|------------|-------------|
| `dir` | `[path] [filter]` | List directory contents |
| `filter` | `dirname -ext extensions` | Set up folder filtering |
| `file` | `pattern [-ip address]` | Find files by pattern |
| `n` | | Play next file in sequence |
| `p` | | Play previous file in sequence |
| `recur` | `[-timeout ms]` | Play all files recursively |

### **Advanced Features**
| Command | Parameters | Description |
|---------|------------|-------------|
| `playby` | `-cb/-buf -uri path` | Play via callback/buffer |
| `int` | `-sel 0/1` | Interrupt playback |
| `res` | `-sel 0/1` | Restore from interrupt |
| `param` | `-dynamic 0/1` | Configure MP4 parser |

### **Debugging & Monitoring**
| Command | Parameters | Description |
|---------|------------|-------------|
| `q` | | Query player status |
| `i` | | Show CPU usage |
| `heap` | `start/stop/use/leak` | Memory analysis |
| `dump` | `-mask value` | Dump pipeline data |
| `speed` | `url` | Test network speed |
| `leak` | | Check memory leaks |

### **Utility Commands**
| Command | Parameters | Description |
|---------|------------|-------------|
| `wait` | `time_ms` | Wait for specified time |
| `waitmsg` | `-msg event -timeout ms` | Wait for player event |
| `waiteos` | `[-timeout ms]` | Wait for end of stream |
| `quit` | | Exit application |

## Configuration Examples

### 1. Audio-Only Player
```
esp> play -uri /sdcard/music.mp3 -mask 1
```

### 2. Video-Only Player
```
esp> play -uri /sdcard/video.mp4 -mask 2
```

### 3. Full Media Player
```
esp> play -uri http://example.com/movie.mp4 -mask 3
```

### 4. Dual Player Setup
```
esp> open -mask 1 -sel 0          # Audio player
esp> open -mask 2 -sel 1          # Video player
esp> play -uri /sdcard/audio.mp3 -sel 0
esp> play -uri /sdcard/video.mp4 -sel 1
```

## Advanced Usage

### File System Filtering
```bash
# Set up folder filtering for media files
esp> filter /sdcard -ext mp3;mp4;avi;wav

# Find files containing "music"
esp> file music

# Play files sequentially
esp> n    # Next file
esp> p    # Previous file
esp> recur    # Play all files recursively
```

### Network Streaming
```bash
# Play network stream
esp> play -uri http://192.168.1.100/video.mp4

# Test network speed
esp> speed http://192.168.1.100/testfile.bin
```

### Custom Data Sources
```bash
# Play via callback interface
esp> playby -cb -uri /sdcard/video.mp4

# Play via buffer interface
esp> playby -buf -uri /sdcard/audio.mp3
```

### Memory Monitoring
```bash
# Start memory tracing
esp> heap start trace.log

# Check memory usage
esp> heap use

# Stop tracing and check leaks
esp> heap stop
esp> leak
```

### Performance Analysis
```bash
# Show system information
esp> i

# Query player status
esp> q

# Dump pipeline data
esp> dump -mask 7
```

## Event System

The CLI monitors and responds to player events:

- **PREPARED**: Media file parsed successfully
- **PLAYED**: Playback started
- **STOPPED**: Playback stopped
- **SEEKED**: Seek operation completed
- **EOS**: End of stream reached
- **ERROR**: Playback error occurred

### Event Waiting
```bash
# Wait for specific events
esp> waitmsg -msg prepared -timeout 5000
esp> waitmsg -msg played
esp> waiteos -timeout 60000
```

## Troubleshooting

### Common Issues

1. **"Player already closed"**
   - Solution: Use `open` command to create player instance first

2. **File not found errors**
   - Solution: Use `dir` command to verify file paths
   - Ensure SD card is properly mounted

3. **Network streaming fails**
   - Solution: Check WiFi connection and URL accessibility
   - Use `speed` command to test network performance

4. **Audio/video not working**
   - Solution: Verify hardware connections and codec initialization
   - Check play mask settings (`-mask` parameter)

### Debug Commands
```bash
# Check system status
esp> i                    # CPU usage
esp> heap use            # Memory usage
esp> q                   # Player status

# Enable detailed logging
esp> heap start debug.log
esp> dump -mask 15       # Dump all pipeline data
```

## Build Configuration

### Target Selection
- **ESP32-P4**: Default target with DSI display support
- **ESP32-S3**: Alternative target with SPI display

### Console Interface
Configure in `sdkconfig`:
- `CONFIG_ESP_CONSOLE_UART=y` (default)
- `CONFIG_ESP_CONSOLE_USB_CDC=y`
- `CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y`

### Memory Configuration
Adjust based on your application needs:
- **Stack sizes**: Configured per thread type
- **FIFO buffers**: Adjustable for performance tuning
- **Memory pools**: Configurable for different media types

## Example Scripts

### Automated Testing
Create script files for automated testing:

```bash
# test_script.txt
open -mask 3
play -uri /sdcard/test.mp4
waiteos -timeout 30000
close
quit
```

Run script: `player_cli_run("test_script.txt")`

### Performance Benchmarking
```bash
heap start benchmark.log
filter /sdcard -ext mp4
file .
recur -timeout 10000
heap stop
leak
```

## License

SPDX-License-Identifier: Apache-2.0

---

This example demonstrates the full power of the ESP Media Player component through an interactive CLI interface, providing both basic playback capabilities and advanced debugging tools for development and testing.
