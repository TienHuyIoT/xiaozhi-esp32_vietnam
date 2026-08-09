"""Hop dong regression cho Device TTS latency va listen/VAD hygiene.

Robot that 09/08/2026 ghi nhan Listening mo luc 13:20:22 nhung
playback_finished cua cung segment toi 13:22:25. Host contract nay khoa hai
nguyen nhan firmware co the xac minh khong can phan cung: decoder resync tung
byte co sleep, va source bao idle khi compressed input van con cho decode/phat.
"""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PLAYER_HEADER = ROOT / "main" / "features" / "music" / "audio_stream_player.h"
PLAYER_SOURCE = ROOT / "main" / "features" / "music" / "audio_stream_player.cc"
DEVICE_SOURCE = ROOT / "main" / "protocols" / "device_tts_client.cc"
APPLICATION_HEADER = ROOT / "main" / "application.h"
APPLICATION_SOURCE = ROOT / "main" / "application.cc"


def _text(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def _function(source: str, signature: str, next_signature: str) -> str:
    start = source.index(signature)
    end = source.index(next_signature, start)
    return source[start:end]


def test_compressed_decoder_error_resync_nhanh_va_consume_trace_span():
    header = _text(PLAYER_HEADER)
    source = _text(PLAYER_SOURCE)
    loop = _function(
        source,
        "void AudioStreamPlayer::PlayLoopCompressed()",
        "void AudioStreamPlayer::PlayLoopWav()",
    )
    error = loop[
        loop.index("if (ret != ESP_AUDIO_ERR_OK)") : loop.index(
            "/* Successful decode */"
        )
    ]

    assert "FindCompressedSyncOffset" in header
    assert "FindCompressedSyncOffset" in error
    assert "ConsumeTraceInputBytes(discard)" in error
    assert "input_bytes_left_ -= discard" in error
    # Khong duoc bien N byte rac thanh N * 20ms im lang.
    assert "vTaskDelay" not in error
    assert "input_bytes_left_--" not in error


def test_compressed_resync_dung_sync_word_rieng_cho_flac_va_aac():
    source = _text(PLAYER_SOURCE)
    sync = _function(
        source,
        "size_t AudioStreamPlayer::FindCompressedSyncOffset",
        "AudioDecoderType AudioStreamPlayer::DetectStreamType",
    )

    # FLAC frame dung sync code 14-bit 0x3FFE; `fLaC` chi la stream marker.
    assert 'memcmp(data + offset, "fLaC", 4)' in sync
    assert "(data[offset + 1] & 0xFE) == 0xF8" in sync
    # ADTS can sync 12-bit va layer == 00, khong dung predicate MP3 11-bit.
    assert "decoder_type_ == AudioDecoderType::AAC" in sync
    assert "(data[offset + 1] & 0xF6) == 0xF0" in sync


def test_device_tts_chua_idle_khi_decoder_con_input_hoac_frame_dang_output():
    header = _text(PLAYER_HEADER)
    player = _text(PLAYER_SOURCE)
    device = _text(DEVICE_SOURCE)
    busy = _function(
        device,
        "bool DeviceTtsClient::IsBusy()",
        "void DeviceTtsClient::Shutdown()",
    )
    source_loop = _function(
        device,
        "void DeviceTtsClient::SourceDataLoop",
        "AudioTraceContext DeviceTtsClient::GetActiveTrace",
    )
    compressed = _function(
        player,
        "void AudioStreamPlayer::PlayLoopCompressed()",
        "void AudioStreamPlayer::PlayLoopWav()",
    )

    assert "pending_decoder_input_bytes_" in header
    assert "decoder_output_in_flight_" in header
    assert "HasPendingDecoderPlayback()" in header
    assert "IsPlaybackDrained()" in header
    assert "std::atomic<size_t> buffer_size_" in header
    assert "HasPendingDecoderPlayback()" in busy
    assert "IsPlaybackDrained()" in source_loop
    assert source_loop.count("IsPlaybackDrained()") >= 2
    assert source_loop.rindex("IsPlaybackDrained()") < source_loop.index(
        "on_idle_()"
    )
    assert compressed.index("decoder_output_in_flight_ = true") < compressed.index(
        "OutputPcmFrame("
    )
    assert compressed.index("OutputPcmFrame(") < compressed.rindex(
        "decoder_output_in_flight_ = false"
    )
    pop_last_chunk = compressed[
        compressed.index("audio_buffer_.pop()") : compressed.index(
            "got = true", compressed.index("audio_buffer_.pop()")
        )
    ]
    assert pop_last_chunk.index("pending_decoder_input_bytes_.store(") < (
        pop_last_chunk.index("buffer_size_.fetch_sub")
    )


def test_trace_span_mutation_duoc_bao_ve_cung_trace_mutex():
    source = _text(PLAYER_SOURCE)
    consume = _function(
        source,
        "void AudioStreamPlayer::ConsumeTraceInputBytes",
        "void AudioStreamPlayer::RequestAudioTraceSegmentFinish",
    )
    compressed = _function(
        source,
        "void AudioStreamPlayer::PlayLoopCompressed()",
        "void AudioStreamPlayer::PlayLoopWav()",
    )

    assert "lock(trace_mutex_)" in consume
    append = compressed[
        compressed.index("input_trace_spans_.back()") - 160 : compressed.index(
            "total_played +="
        )
    ]
    assert "lock(trace_mutex_)" in append


def test_enter_speaking_khong_reset_lan_hai_sau_khi_audio_owner_da_commit():
    header = _text(APPLICATION_HEADER)
    source = _text(APPLICATION_SOURCE)
    set_state = _function(
        source,
        "void Application::SetDeviceState",
        "void Application::Reboot",
    )

    assert "ResetDecoderBeforeSpeakingIfUnowned" in header
    assert "ResetDecoderBeforeSpeakingIfUnowned();" in set_state
    speaking = set_state[
        set_state.index("case kDeviceStateSpeaking") : set_state.index(
            "default:", set_state.index("case kDeviceStateSpeaking")
        )
    ]
    assert "audio_service_.ResetDecoder();" not in speaking

    helper = _function(
        source,
        "void Application::ResetDecoderBeforeSpeakingIfUnowned()",
        "void Application::SetListeningMode",
    )
    transition_lock = helper.index("speech_audio_transition_mutex_")
    owner_check = helper.index("SpeechAudioSource::kNone")
    reset = helper.index("audio_service_.ResetDecoder()")
    assert transition_lock < owner_check < reset


def test_chunk_lon_hon_decoder_buffer_duoc_giu_phan_du_khong_cat_mat():
    source = _text(PLAYER_SOURCE)
    compressed = _function(
        source,
        "void AudioStreamPlayer::PlayLoopCompressed()",
        "void AudioStreamPlayer::PlayLoopWav()",
    )

    # Provider co the giao mot WebSocket payload lon hon input buffer. Phan chua
    # copy phai duoc giu lai cho vong decode sau, khong free ca chunk ngay.
    assert "pending_chunk_offset" in compressed
    assert "pending_chunk.size - pending_chunk_offset" in compressed
    assert "pending_chunk.data + pending_chunk_offset" in compressed
    assert "pending_chunk_offset += copy" in compressed
    assert "if (pending_chunk_offset == pending_chunk.size)" in compressed
    assert "size_t copy  = std::min(chunk.size, space)" not in compressed


def test_data_lack_cuoi_segment_co_fallback_huu_han_khong_ket_speaking():
    header = _text(PLAYER_HEADER)
    player = _text(PLAYER_SOURCE)
    device_header = _text(ROOT / "main" / "protocols" / "device_tts_client.h")
    device = _text(DEVICE_SOURCE)
    compressed = _function(
        player,
        "void AudioStreamPlayer::PlayLoopCompressed()",
        "void AudioStreamPlayer::PlayLoopWav()",
    )
    data_lack = compressed[
        compressed.index("if (ret == ESP_AUDIO_ERR_DATA_LACK)") : compressed.index(
            "if (ret == ESP_AUDIO_ERR_CONTINUE)"
        )
    ]
    source_loop = _function(
        device,
        "void DeviceTtsClient::SourceDataLoop",
        "AudioTraceContext DeviceTtsClient::GetActiveTrace",
    )

    assert "RequestIncompleteDecoderTailDiscard" in header
    assert "discard_incomplete_decoder_tail_" in header
    assert "kDecoderTailStallMs" in device_header
    assert "RequestIncompleteDecoderTailDiscard()" in source_loop
    assert "discard_incomplete_decoder_tail_.exchange(false)" in data_lack
    assert "ConsumeTraceInputBytes(input_bytes_left_)" in data_lack
    assert "input_bytes_left_ = 0" in data_lack
    assert "pending_decoder_input_bytes_ = 0" in data_lack


def test_yeu_cau_bo_tail_cu_khong_duoc_ro_sang_segment_moi():
    source = _text(PLAYER_SOURCE)
    compressed = _function(
        source,
        "void AudioStreamPlayer::PlayLoopCompressed()",
        "void AudioStreamPlayer::PlayLoopWav()",
    )
    consumed = compressed[
        compressed.index("if (raw.consumed > 0)") : compressed.index(
            "if (ret == ESP_AUDIO_ERR_DATA_LACK)"
        )
    ]
    pop = compressed[
        compressed.index("audio_buffer_.pop()") : compressed.index(
            "got = true", compressed.index("audio_buffer_.pop()")
        )
    ]

    # Neu decoder lai tien hoac chunk moi den truoc luc task playback doc co,
    # request cu phai bi huy; neu khong DATA_LACK dau cua cau sau se bi cat oan.
    assert "discard_incomplete_decoder_tail_ = false" in consumed
    assert "discard_incomplete_decoder_tail_ = false" in pop


def _starved_branch(compressed: str) -> str:
    """Nhanh 'khong lay duoc chunk moi' trong PlayLoopCompressed."""
    start = compressed.index("if (!got) {")
    end = compressed.index("if (pending_chunk.data && pending_chunk.size > 0)")
    return compressed[start:end]


def test_het_chunk_ma_con_tail_thi_van_phai_goi_decoder():
    """Robot that 09/08 ket Speaking: queue rong nhung input_bytes_left_ > 0.

    Vong playback bo qua decoder khi khong lay duoc chunk moi, nen ca nhanh
    DATA_LACK lan fallback bo tail deu khong bao gio chay -> pending khong ve 0
    -> IsPlaybackDrained() sai vinh vien -> on_idle_() khong bao gio goi.
    """
    source = _text(PLAYER_SOURCE)
    compressed = _function(
        source,
        "void AudioStreamPlayer::PlayLoopCompressed()",
        "void AudioStreamPlayer::PlayLoopWav()",
    )
    starved = _starved_branch(compressed)

    # Chi duoc quay lai dau vong khi decoder khong con gi de nhai.
    assert "if (input_bytes_left_ <= 0)" in starved
    assert starved.index("if (input_bytes_left_ <= 0)") < starved.index(
        "xSemaphoreTake(buffer_data_sem_"
    )
    # Dung mot `continue` duy nhat va no phai nam trong guard tren; `break` cua
    # stream ket thuc that su khong tinh.
    assert starved.count("continue;") == 1


def test_source_bao_het_byte_segment_thi_decode_cuoi_duoc_dat_eos():
    """Khong co eos, decoder giu lai frame cuoi cho sync word cua frame ke tiep.

    `is_source_active_` dung cho ca vong doi client nen eos cu khong bao gio bat
    giua turn; phai co tin hieu rieng theo segment, neu khong moi cau deu de lai
    tail va phai cho het kDecoderTailStallMs.
    """
    header = _text(PLAYER_HEADER)
    player = _text(PLAYER_SOURCE)
    device = _text(DEVICE_SOURCE)
    compressed = _function(
        player,
        "void AudioStreamPlayer::PlayLoopCompressed()",
        "void AudioStreamPlayer::PlayLoopWav()",
    )
    source_loop = _function(
        device,
        "void DeviceTtsClient::SourceDataLoop",
        "AudioTraceContext DeviceTtsClient::GetActiveTrace",
    )
    eos = compressed[
        compressed.index("bool eos =") : compressed.index("esp_audio_simple_dec_raw_t")
    ]

    assert "source_segment_complete_" in header
    assert "MarkSourceSegmentComplete" in header
    assert "ClearSourceSegmentComplete" in header
    assert "source_segment_complete_.load()" in eos
    # Chi duoc coi la het segment khi khong con byte nao dang cho vao decoder.
    assert "buffer_size_.load() == 0" in eos
    assert "pending_chunk.data == nullptr" in eos
    # Nguon phai bao SAU khi SynthesizeOne tra ve (turn.end = het byte) va TRUOC
    # khi doi drain, neu khong tin hieu vo dung.
    assert "MarkSourceSegmentComplete()" in source_loop
    assert source_loop.index("SynthesizeOne(") < source_loop.index(
        "MarkSourceSegmentComplete()"
    )
    assert source_loop.index("MarkSourceSegmentComplete()") < source_loop.index(
        "IsPlaybackDrained()"
    )
    # Segment moi phai xoa co, neu khong eos cu ro sang cau sau va cat frame dau.
    assert "ClearSourceSegmentComplete()" in source_loop
    assert source_loop.index("ClearSourceSegmentComplete()") < source_loop.index(
        "SynthesizeOne("
    )
