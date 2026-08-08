"""Hop dong regression cho panic khi Device TTS chuyen sang server Opus.

Trace robot 09/08/2026 cho thay TransitionSpeechAudio bi khoa 5.106 giay trong
DeviceTtsClient::Abort -> AudioStreamPlayer::StopStream, sau do CPU0 Interrupt
WDT. Day la host contract test vi firmware chua co C++ unit runner native.
"""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PLAYER_HEADER = ROOT / "main" / "features" / "music" / "audio_stream_player.h"
PLAYER_SOURCE = ROOT / "main" / "features" / "music" / "audio_stream_player.cc"
DEVICE_TTS_HEADER = ROOT / "main" / "protocols" / "device_tts_client.h"
DEVICE_TTS_SOURCE = ROOT / "main" / "protocols" / "device_tts_client.cc"


def _text(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def _function(source: str, signature: str, next_signature: str) -> str:
    start = source.index(signature)
    end = source.index(next_signature, start)
    return source[start:end]


def test_abort_device_tts_chi_huy_pipeline_khong_doi_worker_5_giay():
    source = _text(DEVICE_TTS_SOURCE)
    abort = _function(
        source,
        "void DeviceTtsClient::Abort()",
        "bool DeviceTtsClient::IsBusy()",
    )

    assert "InterruptStream()" in abort
    assert "StopStream()" not in abort
    assert "CloseReadySocket()" not in abort
    assert "vTaskDelay" not in abort


def test_ket_noi_cu_hoan_tat_sau_abort_khong_duoc_gui_ssml():
    source = _text(DEVICE_TTS_SOURCE)
    synthesize = _function(
        source,
        "bool DeviceTtsClient::SynthesizeOne",
        "void DeviceTtsClient::HandleData",
    )

    assert "synthesis_generation" in synthesize
    assert "turn_generation_.load() != synthesis_generation" in synthesize
    assert synthesize.index("turn_generation_.load() != synthesis_generation") < synthesize.index(
        "websocket_->Send(body)"
    )


def test_interrupt_stream_dung_generation_de_loai_pcm_cu_ma_khong_dung_task():
    header = _text(PLAYER_HEADER)
    source = _text(PLAYER_SOURCE)
    interrupt = _function(
        source,
        "void AudioStreamPlayer::InterruptStream()",
        "void AudioStreamPlayer::RegisterAudioTraceSegment",
    )

    assert "std::atomic<uint32_t> stream_generation_" in header
    assert "stream_generation_.fetch_add(1)" in interrupt
    assert "DiscardQueuedAudio()" in interrupt
    assert "is_playing_ = false" not in interrupt
    assert "is_source_active_ = false" not in interrupt
    assert "StopStream()" not in interrupt


def test_byte_tts_den_muon_khong_duoc_gan_generation_moi_sau_abort():
    header = _text(DEVICE_TTS_HEADER)
    source = _text(DEVICE_TTS_SOURCE)
    set_trace = _function(
        source,
        "void DeviceTtsClient::SetActiveTrace",
        "void DeviceTtsClient::ClearActiveTrace",
    )
    handle = source[source.index("void DeviceTtsClient::HandleData") :]

    assert "active_stream_generation_" in header
    assert "GetStreamGeneration()" in set_trace
    assert "active_stream_generation_.load()" in handle
    assert "PushToBuffer(p + off, len - off, trace.trace_sequence," in handle


def test_playback_bo_frame_decode_neu_abort_xay_ra_khi_decoder_dang_chay():
    source = _text(PLAYER_SOURCE)
    compressed = _function(
        source,
        "void AudioStreamPlayer::PlayLoopCompressed()",
        "void AudioStreamPlayer::PlayLoopWav()",
    )

    assert "playback_generation" in compressed
    assert "stream_generation_.load()" in compressed
    assert compressed.index("stream_generation_.load()") < compressed.index(
        "OutputPcmFrame"
    )
    assert "ResetCompressedPlaybackForGeneration" in compressed


def test_stopstream_khong_free_static_stack_trong_vong_doi_runtime():
    source = _text(PLAYER_SOURCE)
    start = _function(
        source,
        "bool AudioStreamPlayer::StartStream",
        "bool AudioStreamPlayer::StopStream",
    )
    stop = _function(
        source,
        "bool AudioStreamPlayer::StopStream",
        "void AudioStreamPlayer::PauseStream",
    )

    assert "if (!StopStream())" in start
    assert "heap_caps_free(source_task_stack_)" not in stop
    assert "heap_caps_free(source_task_buffer_)" not in stop
    assert "heap_caps_free(play_task_stack_)" not in stop
    assert "heap_caps_free(play_task_buffer_)" not in stop
    assert "return false" in stop


def test_worker_suspend_de_stopstream_xoa_task_tu_ben_ngoai_roi_moi_tai_su_dung_stack():
    header = _text(PLAYER_HEADER)
    source = _text(PLAYER_SOURCE)
    source_entry = _function(
        source,
        "void AudioStreamPlayer::SourceTaskEntry",
        "void AudioStreamPlayer::PlayTaskEntry",
    )
    play_entry = _function(
        source,
        "void AudioStreamPlayer::PlayTaskEntry",
        "void AudioStreamPlayer::SourceDataLoop",
    )
    stop = _function(
        source,
        "bool AudioStreamPlayer::StopStream",
        "void AudioStreamPlayer::PauseStream",
    )

    assert "source_task_exited_" in header
    assert "play_task_exited_" in header
    assert "source_task_handle_ = nullptr" not in source_entry
    assert "play_task_handle_ = nullptr" not in play_entry
    assert "source_task_exited_ = true" in source_entry
    assert "play_task_exited_ = true" in play_entry
    assert "vTaskSuspend(nullptr)" in source_entry
    assert "vTaskSuspend(nullptr)" in play_entry
    assert "vTaskDelete(source_task_handle_)" in stop
    assert "vTaskDelete(play_task_handle_)" in stop


def test_interrupt_va_i2s_duoc_tuan_tu_hoa_de_pcm_cu_khong_lot_sau_abort():
    header = _text(PLAYER_HEADER)
    source = _text(PLAYER_SOURCE)
    interrupt = _function(
        source,
        "void AudioStreamPlayer::InterruptStream()",
        "void AudioStreamPlayer::RegisterAudioTraceSegment",
    )
    output = _function(
        source,
        "void AudioStreamPlayer::OutputPcmFrame",
        "void AudioStreamPlayer::OutputPcmDirect",
    )

    assert "output_generation_mutex_" in header
    assert "output_generation_mutex_" in interrupt
    assert "output_generation_mutex_" in output
    assert "expected_generation != stream_generation_.load()" in output
    assert output.index("expected_generation != stream_generation_.load()") < output.index(
        "OutputPcmDirect"
    )


def test_static_stack_chi_duoc_free_sau_stopstream_thanh_cong_o_destructor():
    source = _text(PLAYER_SOURCE)
    destructor = _function(
        source,
        "AudioStreamPlayer::~AudioStreamPlayer()",
        "bool AudioStreamPlayer::StartStream",
    )

    assert "const bool stopped = StopStream()" in destructor
    assert "if (stopped)" in destructor
    for field in (
        "source_task_stack_",
        "source_task_buffer_",
        "play_task_stack_",
        "play_task_buffer_",
    ):
        assert f"heap_caps_free({field})" in destructor


def test_destructor_khong_tra_ve_neu_worker_van_con_giu_this():
    source = _text(PLAYER_SOURCE)
    destructor = _function(
        source,
        "AudioStreamPlayer::~AudioStreamPlayer()",
        "bool AudioStreamPlayer::StartStream",
    )
    failure = destructor[destructor.index("if (!stopped)") :]

    assert "esp_restart()" in failure
    assert "return;" not in failure


def test_device_tts_join_worker_truoc_khi_member_lop_con_bi_huy():
    source = _text(DEVICE_TTS_SOURCE)
    destructor = _function(
        source,
        "DeviceTtsClient::~DeviceTtsClient()",
        "bool DeviceTtsClient::Configure",
    )

    assert destructor.index("Shutdown()") < destructor.index("StopStream()")
    assert destructor.index("StopStream()") < destructor.index("CloseSocket()")
    assert "if (!StopStream())" in destructor
    assert "esp_restart()" in destructor


def test_shutdown_khong_dong_active_socket_cheo_task_va_join_preconnect_an_toan():
    header = _text(DEVICE_TTS_HEADER)
    source = _text(DEVICE_TTS_SOURCE)
    shutdown = _function(
        source,
        "void DeviceTtsClient::Shutdown()",
        "bool DeviceTtsClient::EnsureStarted()",
    )
    preconnect = _function(
        source,
        "void DeviceTtsClient::PreconnectTaskRoutine()",
        "bool DeviceTtsClient::SynthesizeOne",
    )

    assert "preconnect_task_exited_" in header
    assert "CloseSocket()" not in shutdown
    assert "eTaskGetState(preconnect_task_handle_) == eSuspended" in shutdown
    assert "vTaskDelete(preconnect_task_handle_)" in shutdown
    assert "esp_restart()" in shutdown
    assert "preconnect_task_handle_ = nullptr" not in preconnect
    assert "preconnect_task_exited_ = true" in preconnect
    assert "vTaskSuspend(nullptr)" in preconnect
