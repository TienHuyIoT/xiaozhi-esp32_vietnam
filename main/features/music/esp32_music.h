#ifndef ESP32_MUSIC_H
#define ESP32_MUSIC_H

/**
 * @file esp32_music.h
 * @brief Online music player – inherits AudioStreamPlayer.
 *
 * Responsibilities:
 *   - Song search via HTTP API (with ESP32 auth)
 *   - Builds streaming URL and delegates playback to base class
 *   - Lyrics management (via LyricManager)
 *   - Display of song metadata on LCD
 */

#include <string>
#include <atomic>

#include "audio_stream_player.h"
#include "lyric_manager.h"
#include "music.h"

/* ------------------------------------------------------------------ */
/*  Macros                                                            */
/* ------------------------------------------------------------------ */

/** Default music server URL */
#define DEFAULT_MUSIC_URL       "https://trinh-khanh-mp3-new-soundcloud.hf.space"

/** Lyric buffer latency compensation (ms) */
#define LYRIC_LATENCY_OFFSET_MS 600

/* ------------------------------------------------------------------ */
/*  Esp32Music                                                        */
/* ------------------------------------------------------------------ */

class Esp32Music : public Music, public AudioStreamPlayer {
public:
    Esp32Music();
    virtual ~Esp32Music();

    /**
     * @brief Initialize the music player.
     * @param codec  AudioCodec for direct PCM output (nullptr = use Application pipeline)
     */
    void Initialize(AudioCodec* codec);

    /* ---- Music interface ---- */
    bool Download(const std::string& song_name, const std::string& artist_name = "") override;
    std::string GetDownloadResult() override;

    bool StartStreaming(const std::string& music_url) override;
    bool StopStreaming() override;

    size_t GetBufferSize() const override { return AudioStreamPlayer::GetBufferSize(); }
    bool IsDownloading() const override   { return AudioStreamPlayer::IsDownloading(); }
    bool IsPlaying() const override       { return AudioStreamPlayer::IsPlaying(); }

    /* ---- Display mode (re-expose base enum with LYRICS alias) ---- */
    using AudioStreamPlayer::DisplayMode;
    static constexpr DisplayMode DISPLAY_MODE_LYRICS = DISPLAY_MODE_INFO;

    void SetDisplayMode(DisplayMode mode) { AudioStreamPlayer::SetDisplayMode(mode); }
    DisplayMode GetDisplayMode() const    { return AudioStreamPlayer::GetDisplayMode(); }

    /** Get music server URL from settings (or default). */
    std::string GetCheckMusicServerUrl();

    /* ---- Metadata getters (for BuildMusicInfo) ---- */
    std::string GetTitle() const;
    std::string GetArtist() const;
    int64_t     GetPositionMs() const  { return AudioStreamPlayer::GetPlayTimeMs(); }
    int64_t     GetDurationMs() const  { return duration_ms_; }
    int         GetBitrateKbps() const { return bitrate_kbps_; }

protected:
    /* ---- AudioStreamPlayer hooks ---- */
    void OnPrepareHttp(void* http_ptr) override;
    void OnStreamInfoReady(int sample_rate, int bits_per_sample, int channels, int bitrate, int frame_size) override;
    void OnPcmFrame(int64_t play_time_ms, int sample_rate, int channels) override;
    bool OnPlaybackFinishedAndContinue() override;
    void OnDisplayReady() override;

private:
    /* ---- Auth helpers ---- */
    static std::string GetDeviceMac();
    static std::string GetDeviceChipId();
    static std::string GenerateDynamicKey(int64_t timestamp);
    static void AddAuthHeaders(void* http_ptr);

    /* ---- URL helpers ---- */
    static std::string UrlEncode(const std::string& str);
    static std::string BuildUrlWithParams(const std::string& base,
                                          const std::string& path,
                                          const std::string& query);

    /* ---- State ---- */
    std::string last_downloaded_data_;
    std::string current_music_url_;
    std::string artist_name_;
    std::string title_name_;
    std::string current_song_name_;
    std::string current_lyric_url_;

    bool song_name_displayed_;
    bool full_info_displayed_;
    int  bitrate_kbps_ = 0;       ///< Bitrate (kbps) from decoder
    int64_t duration_ms_ = 0;     ///< Duration (ms) from API

    /* ---- Lyrics ---- */
    LyricManager lyric_mgr_;
};

#endif // ESP32_MUSIC_H
