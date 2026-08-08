"""Hop dong host cho telemetry audio Phase 0.

ESP-IDF project chua co unit-test runner tren host. Cac test nay khoa giao thuc
metadata va diem dat trace ma khong can phan cung; build dung board va serial
trace tren robot that van la cong xac nhan E2E bat buoc.
"""

from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[1]
APPLICATION_HEADER = ROOT / "main" / "application.h"
APPLICATION_SOURCE = ROOT / "main" / "application.cc"
DEVICE_HEADER = ROOT / "main" / "protocols" / "device_tts_client.h"
DEVICE_SOURCE = ROOT / "main" / "protocols" / "device_tts_client.cc"
PLAYER_HEADER = ROOT / "main" / "features" / "music" / "audio_stream_player.h"
PLAYER_SOURCE = ROOT / "main" / "features" / "music" / "audio_stream_player.cc"


def _text(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def _function(source: str, signature: str, next_signature: str) -> str:
    start = source.index(signature)
    end = source.index(next_signature, start)
    return source[start:end]


def test_sentence_start_nhan_metadata_tuong_thich_nguoc_ma_khong_doi_routing():
    app = _text(APPLICATION_SOURCE)
    app_header = _text(APPLICATION_HEADER)

    assert "ReadAudioTraceContext" in app
    assert '"turn_id"' in app
    assert '"segment_id"' in app
    assert '"audio_source"' in app
    assert 'has_device_tts_body ? "device_tts" : "server_opus"' in app
    assert "device_tts_client_->Enqueue(body, trace)" in app
    assert "std::mutex audio_trace_mutex_" in app_header
    assert app.count("lock(audio_trace_mutex_)") >= 2

    # Routing cu van chi phu thuoc tts_body; metadata chi di vao telemetry.
    assert "if (has_device_tts_body && device_tts_client_ != nullptr)" in app


def test_metadata_log_duoc_allowlist_va_gioi_han_do_dai():
    player_header = _text(PLAYER_HEADER)
    player_source = _text(PLAYER_SOURCE)

    assert "struct AudioTraceContext" in player_header
    for field in ("turn_id", "segment_id", "audio_source"):
        assert f"std::string {field}" in player_header

    assert "kMaxAudioTraceFieldLength" in player_source
    assert "SanitizeAudioTraceField" in player_source
    assert "std::isalnum" in player_source
    assert "value.size(), kMaxAudioTraceFieldLength" in player_source


def test_hang_doi_giu_context_va_log_depth_khi_enqueue_dequeue():
    header = _text(DEVICE_HEADER)
    source = _text(DEVICE_SOURCE)

    assert "struct QueuedTtsSegment" in header
    assert "AudioTraceContext trace" in header
    assert "std::deque<QueuedTtsSegment> queue_" in header
    assert "const AudioTraceContext &trace" in header
    assert 'LogAudioTraceQueue("queue", "enqueue"' in source
    assert 'LogAudioTraceQueue("queue", "dequeue"' in source


def test_trace_co_day_du_event_va_dung_dong_ho_monotonic():
    app = _text(APPLICATION_SOURCE)
    device = _text(DEVICE_SOURCE)
    player = _text(PLAYER_SOURCE)
    all_sources = app + device + player

    for event in (
        "json_receive",
        "queue",
        "connect_begin",
        "connect_ready",
        "first_provider_byte",
        "first_decoded_pcm",
        "last_pcm",
        "source_transition",
        "abort",
    ):
        assert f'"{event}"' in all_sources

    assert "esp_timer_get_time()" in player
    assert (
        '"event=%s ts_us=%lld seq=%lu turn_id=%s segment_id=%s audio_source=%s"'
        in player
    )


def test_first_last_pcm_do_playback_so_huu_va_khong_log_moi_frame():
    player_header = _text(PLAYER_HEADER)
    player_source = _text(PLAYER_SOURCE)
    device_source = _text(DEVICE_SOURCE)

    assert "trace_sequence" in player_header
    assert "ObserveDecodedPcm" in player_header
    assert "FinishAudioTraceSegment" in player_header
    assert player_source.count('"first_decoded_pcm"') == 1
    assert player_source.count('"last_pcm"') == 1
    assert "last_pcm_timestamp_us_" in player_source
    assert "decoded_trace_sequence != active_pcm_trace_sequence_" in player_source
    # Source task khong duoc ket luan segment da phat het chi tu compressed buffer.
    # Source chi request; playback se ghi last_pcm sau khi input trace da consume het.
    assert "FinishAudioTraceSegment(segment.trace.trace_sequence)" not in device_source
    assert "RequestAudioTraceSegmentFinish(segment.trace.trace_sequence)" in device_source
    assert "MaybeFinishRequestedAudioTrace" in player_header
    assert "input_trace_spans_" in player_source
    assert "EmitLastPcm(previous, previous_last_pcm_us)" in player_source
    assert "FinishActiveAudioTrace();" in player_source

    output_pcm = _function(
        player_source,
        "void AudioStreamPlayer::OutputPcmFrame",
        "void AudioStreamPlayer::OutputPcmDirect",
    )
    assert '"first_decoded_pcm"' not in output_pcm
    assert '"last_pcm"' not in output_pcm


def test_abort_va_provider_trace_khong_log_payload_hay_secret():
    app = _text(APPLICATION_SOURCE)
    device = _text(DEVICE_SOURCE)

    assert 'LogAudioTraceEvent("abort"' in app
    assert 'LogAudioTraceEvent("connect_begin"' in device
    assert 'LogAudioTraceEvent("connect_ready"' in device
    assert 'LogAudioTraceEvent("first_provider_byte"' in device
    assert "first_provider_byte_seen_.exchange(true)" in device

    forbidden_log_values = (
        "tts_body.c_str()",
        "body.c_str()",
        "snapshot.url.c_str()",
        "snapshot.config_frame.c_str()",
    )
    trace_calls = re.findall(r"LogAudioTrace(?:Event|Queue)\([^;]+", device)
    for value in forbidden_log_values:
        assert all(value not in call for call in trace_calls)
