# Media Player Component — Architecture

## Overview

The media component provides MP4 (and other format) playback with audio output
via I2S and video output to an LCD panel. It wraps the `tempotian/media_player`
library behind a thread-safe service layer.

## Block Diagram

```mermaid
graph TD
    subgraph Application Layer
        APP[Application / MCP / UI]
    end

    subgraph Media Service Layer
        SVC[MediaPlayerService]
        QUEUE[FreeRTOS Command Queue]
        WORKER[Worker Task<br/>Core 0 · Priority 5]
    end

    subgraph Media Player Library
        MP[media_player<br/>tempotian/media_player v0.5.0]
        SRC[media_src<br/>SD Card / HTTP]
        EXT[esp_extractor<br/>MP4 · WAV · OGG · FLV · HLS]
    end

    subgraph AV Render Layer
        AVR[av_render<br/>tempotian/av_render v0.9.1]
        ADEC[Audio Decoder<br/>AAC · MP3 · PCM]
        VDEC[Video Decoder<br/>H.264 · MJPEG]
        AFIFO[Audio FIFO]
        VFIFO[Video FIFO]
    end

    subgraph Render Factory
        FACTORY[media_render_factory]
        ARENDER[I2S Audio Render<br/>esp_codec_dev]
        VRENDER[LCD Video Render<br/>esp_lcd_panel]
    end

    subgraph Hardware
        CODEC[Audio Codec<br/>ES8311 / ES8388]
        I2S[I2S Bus]
        LCD[LCD Panel<br/>ILI9341 SPI]
        SPK[Speaker]
    end

    APP -->|Play / Pause / Seek| SVC
    SVC -->|enqueue| QUEUE
    QUEUE -->|dequeue| WORKER
    WORKER -->|API calls| MP

    MP --> SRC
    SRC -->|raw stream| EXT
    EXT -->|demuxed frames| AVR

    AVR --> ADEC
    AVR --> VDEC
    ADEC --> AFIFO
    VDEC --> VFIFO

    FACTORY -.->|creates| ARENDER
    FACTORY -.->|creates| VRENDER

    AFIFO --> ARENDER
    VFIFO --> VRENDER

    ARENDER -->|esp_codec_dev_write| CODEC
    CODEC --> I2S --> SPK
    VRENDER -->|esp_lcd_panel_draw_bitmap| LCD

    MP -->|player_event_t callback| SVC
    SVC -->|MediaPlayerEvent| APP

    style SVC fill:#4fc3f7,color:#000
    style MP fill:#81c784,color:#000
    style AVR fill:#ffb74d,color:#000
    style FACTORY fill:#ce93d8,color:#000
    style CODEC fill:#ef9a9a,color:#000
    style LCD fill:#ef9a9a,color:#000
```

## Data Flow

1. **Command path** (top → down):
   `Application` → `MediaPlayerService::Play()` → command queue → worker task
   → `media_player_play()` → media_player library

2. **Media data path** (left → right):
   `SD Card / HTTP` → `media_src` → `esp_extractor` (demux MP4) → `av_render`
   → audio decoder + video decoder → FIFOs → I2S render / LCD render → hardware

3. **Event path** (bottom → up):
   `media_player` callback → `HandlePlayerEvent()` → `MediaPlayerEvent`
   → application listener

## Key Files

| File | Purpose |
|------|---------|
| `media_player_service.h` | Public API: Init, Play, Pause, Seek, SetSource, events |
| `media_player_service.cc` | Command queue, worker task, player lifecycle |
| `media_render_factory.h` | Factory interface for audio/video render creation |
| `media_render_factory.cc` | Creates I2S render and LCD render from board handles |

## Initialization Sequence

```mermaid
sequenceDiagram
    participant App as Application
    participant Svc as MediaPlayerService
    participant Fac as media_render_factory
    participant MP as media_player
    participant AVR as av_render

    App->>Svc: Init(codec, panel, config)
    Svc->>Fac: CreateAudioRender(codec)
    Fac-->>Svc: audio_render_handle_t
    Svc->>Fac: CreateVideoRender(panel)
    Fac-->>Svc: video_render_handle_t
    Svc->>MP: media_player_open(cfg with renders)
    MP->>AVR: av_render_open(audio+video renders)
    MP-->>Svc: player handle
    Svc->>MP: media_player_set_callback()
    Svc->>Svc: Start worker task
    Svc-->>App: true (success)
```

## Thread Model

| Task | Core | Priority | Role |
|------|------|----------|------|
| Worker task (`media_svc`) | 0 | 5 | Processes command queue, calls media_player API |
| media_player internal | 1 | — | Demux, decode (managed by library) |
| av_render decode threads | — | — | Audio/video decode into FIFOs |
| av_render render threads | — | — | Drain FIFOs into hardware |

## Usage Example

```cpp
#include "media_player_service.h"

// Get board hardware handles
auto* codec = Board::GetInstance().GetAudioCodec();
auto* display = static_cast<LcdDisplay*>(Board::GetInstance().GetDisplay());

// Configure for MP4 with audio + video
MediaPlayerConfig config;
config.enable_audio = true;
config.enable_video = true;

// Initialize
auto& player = MediaPlayerService::GetInstance();
player.Init(codec, display->GetPanelHandle(), config);

// Set event listener
player.SetEventCallback([](MediaPlayerEvent event, MediaPlayerState state) {
    ESP_LOGI("App", "Event: %d State: %d", (int)event, (int)state);
});

// Play MP4 from SD card
player.SetSource(MediaSourceType::kFile, "/sdcard/video.mp4");
player.Play();
```
