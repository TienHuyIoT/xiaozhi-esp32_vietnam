# MP4 Player av_render Architecture and Local Modifications

## 1. Scope

This document describes:
- How the MP4 player is implemented with av_render in this repository
- What local modifications were applied to the av_render component and integration layer
- Why those changes were made

Main related files:
- main/features/mp4/mp4_player.h
- main/features/mp4/mp4_player.cc
- managed_components/tempotian__av_render/src/audio_decoder.c
- main/idf_component.yml
- main/main.cc

## 2. High-Level Design

The player uses a push-based pipeline:

1. MP4 demux:
- esp_extractor reads MP4 frames from SD file
- Output frame types: audio/video elementary frames

2. Render/decode pipeline:
- av_render owns decode and render worker threads internally
- App pushes compressed frames into av_render using:
  - av_render_add_audio_data
  - av_render_add_video_data

3. Device output:
- Audio callback path: av_render -> AudioRenderWrite -> OutputPcmToCodec -> AudioCodec
- Video callback path: av_render -> VideoRenderWrite -> DrawFrameToLcd -> esp_lcd_panel_draw_bitmap

No app-level software H264 or simple audio decode loop is used anymore. Decode is delegated to av_render internal decoder resources.

## 3. Threading and Responsibilities

### 3.1 App playback task (mp4_play)

PlaybackTaskLoop in mp4_player.cc is responsible for:
- Read frames from extractor
- Push frames into av_render queues
- Handle stop/pause flags
- Handle extractor EOS and timeout conditions
- Push EOS packet to av_render on playback end
- Execute final cleanup and state transition

### 3.2 av_render internal workers

Managed by av_render component. Typical workers:
- Audio decoder thread
- Video decoder thread
- Audio render thread
- Video render thread

These workers are started and stopped by av_render stream/open/reset lifecycle, not by app-created worker tasks.

## 4. Startup and Init Sequence

1. app_main:
- media_lib_add_default_adapter() is called before media pipeline usage
- This is required for media_lib memory/OS adapter setup used by av_render/media stack

2. Mp4Player::Initialize:
- Register extractor and decoder defaults:
  - esp_extractor_register_default
  - esp_audio_dec_register_default
  - esp_video_dec_register_default
- Create audio_render and video_render handles with app callbacks
- Open av_render with tuned fifo and sync config

3. ConfigureRenderStreams:
- Reset av_render
- Map extractor stream format to av_render codec type
- Add audio stream then video stream

## 5. Stream Configuration Details

### 5.1 Audio

- Codec mapping supports AAC/MP3/FLAC
- For AAC in MP4:
  - Parse AudioSpecificConfig (ASC) from spec_info
  - Derive sample rate/channels from ASC when extractor fields are not reliable
  - Fallback defaults if still zero:
    - sample_rate = 44100
    - channel = 2
    - bits_per_sample = 16
  - Set aac_no_adts = 1 only when spec_info exists

### 5.2 Video

- Codec mapping supports H264 and MJPEG
- Frame output format support is RGB565 / RGB565_BE in VideoRenderFormatSupported
- DrawFrameToLcd handles centering, chunk draw, and byte swap when needed

## 6. End-of-Playback and Cleanup Flow

When playback loop exits:

1. PushEos:
- Sends zero-size EOS packets for both audio and video
- Retries briefly if queue is full

2. Force av_render worker stop:
- av_render_reset is called to send CLOSE to internal threads and wait for exit
- This prevents lingering video render activity from continuing after extractor is closed

3. Final app cleanup:
- Close extractor/file resources
- Clear state and invoke end callback/state callback
- Mark playback task finished

## 7. Local Modifications in av_render Component

File:
- managed_components/tempotian__av_render/src/audio_decoder.c

### 7.1 Added state fields in decoder context

In adec_t:
- av_render_audio_info_t stream_info
- bool aac_fallback_tried

Purpose:
- Keep mutable stream config for runtime AAC fallback behavior
- Prevent repeated reopen loops

### 7.2 AAC open-parameter sanitization

In _open_audio_dec, AAC case:
- If sample_rate == 0, set 44100
- If channel == 0, set 2
- If bits_per_sample == 0, set 16

Purpose:
- Avoid opening AAC decoder with invalid/zero format hints

### 7.3 One-time AAC no_adts -> ADTS fallback

In decoder_one_frame:
- If AAC decode fails while aac_no_adts is enabled:
  - retry once with aac_no_adts = 0
  - close and reopen decoder
  - decode again

Purpose:
- Handle streams where payload format does not match expected no-ADTS mode

### 7.4 Stream info persistence on open

In adec_open:
- adec->stream_info is copied from cfg->audio_info

Purpose:
- Make runtime reopen/fallback possible using retained audio stream settings

## 8. Local Modifications in MP4 Integration Layer

File:
- main/features/mp4/mp4_player.cc

### 8.1 Architecture migration to av_render

- Removed app-managed decode/render worker model
- Added av_render integration with custom audio_render/video_render callbacks

### 8.2 AAC configuration hardening

- Added ASC parser
- Added safe defaults for rate/channels/bits
- Added clear AAC config logging and no_adts decision based on spec_info presence

### 8.3 Playback loop robustness

- Added WAITING_OUTPUT consecutive timeout guard (5 seconds)
- Preserve EOS flag before releasing extractor frame
- Always break loop on EOS frame even if frame push fails

### 8.4 EOS and teardown robustness

- PushEos uses retry instead of one-shot send
- av_render_reset is called before final state transition to ensure worker exit

## 9. Dependency and Bootstrapping Changes

File:
- main/idf_component.yml

Relevant media dependency alignment:
- Added tempotian/av_render dependency
- Aligned esp_audio_codec version to integration-compatible one
- Disabled direct esp_h264 dependency in this app path (decode delegated through av_render stack)

File:
- main/main.cc

- Added media_lib_add_default_adapter call in app_main for media stack initialization

## 10. Operational Notes

1. managed_components patches can be overwritten by dependency refresh/reinstall
- Re-apply local patches in audio_decoder.c if component is re-fetched

2. App partition headroom is low (about 2 percent in recent builds)
- Track binary growth when adding more media/UI features

3. If playback-end issues reappear, inspect logs around:
- EOS frame received
- PushEos result
- av_render_reset done
- Playback finished
