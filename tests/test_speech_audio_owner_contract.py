"""Hop dong Phase 1: chi mot duong speech duoc quyen phat loa.

Day la host contract test vi firmware ESP-IDF chua co unit-test runner native.
Robot that + AudioTrace van la cong E2E bat buoc sau khi build/flash.
"""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
APPLICATION_HEADER = ROOT / "main" / "application.h"
APPLICATION_SOURCE = ROOT / "main" / "application.cc"
AUDIO_SERVICE_HEADER = ROOT / "main" / "audio" / "audio_service.h"
AUDIO_SERVICE_SOURCE = ROOT / "main" / "audio" / "audio_service.cc"
PROTOCOL_HEADER = ROOT / "main" / "protocols" / "protocol.h"
WEBSOCKET_PROTOCOL_SOURCE = ROOT / "main" / "protocols" / "websocket_protocol.cc"


def _text(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def _function(source: str, signature: str, next_signature: str) -> str:
    start = source.index(signature)
    end = source.index(next_signature, start)
    return source[start:end]


def test_owner_co_lease_theo_source_turn_va_generation():
    header = _text(APPLICATION_HEADER)

    assert "enum class SpeechAudioSource" in header
    for state in ("kNone", "kSwitching", "kServerOpus", "kDeviceTts"):
        assert state in header
    assert "struct SpeechAudioLease" in header
    assert "std::string turn_id" in header
    assert "uint32_t generation" in header
    assert "std::mutex speech_audio_mutex_" in header


def test_transition_thu_hoi_nguon_cu_truoc_khi_cap_lease_moi():
    source = _text(APPLICATION_SOURCE)
    transition = _function(
        source,
        "Application::TransitionSpeechAudio",
        "Application::IsSpeechAudioLeaseCurrent",
    )

    switching = transition.index("SpeechAudioSource::kSwitching")
    abort_device = transition.index("device_tts_client_->Abort()")
    reset_opus = transition.index("audio_service_.ResetDecoder()")
    commit_new = transition.index("speech_audio_source_ = desired_source")

    assert switching < abort_device < reset_opus < commit_new
    # Cung mot source nhung turn moi van phai thu hoi turn cu.
    assert "speech_audio_turn_id_ == trace.turn_id" in transition
    assert "speech_audio_source_ == desired_source" in transition


def test_server_opus_chi_duoc_vao_decode_queue_khi_dang_so_huu_loa():
    source = _text(APPLICATION_SOURCE)
    incoming = _function(
        source,
        "protocol_->OnIncomingAudio",
        "protocol_->OnAudioChannelOpened",
    )

    assert "AcceptServerOpusPacket(std::move(packet))" in incoming
    assert "audio_service_.PushPacketToDecodeQueue" not in incoming

    admission = _function(
        source,
        "Application::AcceptServerOpusPacket",
        "Application::EnqueueDeviceTtsIfLeaseCurrent",
    )
    transition_lock = admission.index("speech_audio_transition_mutex_")
    owner_check = admission.index(
        "speech_audio_source_ != SpeechAudioSource::kServerOpus"
    )
    push = admission.index("audio_service_.PushPacketToDecodeQueue")
    assert transition_lock < owner_check < push


def test_server_opus_packet_mang_token_luot_va_bi_loai_neu_den_muon():
    protocol_header = _text(PROTOCOL_HEADER)
    websocket_source = _text(WEBSOCKET_PROTOCOL_SOURCE)
    application_source = _text(APPLICATION_SOURCE)
    admission = _function(
        application_source,
        "Application::AcceptServerOpusPacket",
        "Application::EnqueueDeviceTtsIfLeaseCurrent",
    )

    assert "speech_turn_token" in protocol_header
    assert "speech_turn_token_present" in protocol_header
    assert '"Speech-Audio-Frame-Version"' in websocket_source
    assert "SOP1" in websocket_source
    assert "packet->speech_turn_token_present" in admission
    assert "packet->speech_turn_token !=" in admission
    assert admission.index("packet->speech_turn_token !=") < admission.index(
        "audio_service_.PushPacketToDecodeQueue"
    )


def test_device_tts_den_muon_bi_loai_bang_lease_generation():
    source = _text(APPLICATION_SOURCE)
    sentence = _function(
        source,
        '} else if (strcmp(state->valuestring, "sentence_start") == 0)',
        "auto text = cJSON_GetObjectItem(root, \"text\")",
    )

    assert "TransitionSpeechAudio(trace)" in sentence
    assert "EnqueueDeviceTtsIfLeaseCurrent(lease, body, trace)" in sentence
    assert "device_tts_client_->Enqueue" not in sentence

    admission = _function(
        source,
        "Application::EnqueueDeviceTtsIfLeaseCurrent",
        "Application::RevokeSpeechAudio",
    )
    transition_lock = admission.index("speech_audio_transition_mutex_")
    lease_check = admission.index("IsSpeechAudioLeaseCurrent(lease)")
    enqueue = admission.index("device_tts_client_->Enqueue(body, trace)")
    assert transition_lock < lease_check < enqueue


def test_abort_thu_hoi_ca_device_tts_va_server_opus():
    source = _text(APPLICATION_SOURCE)
    abort = _function(
        source,
        "void Application::AbortSpeaking",
        "void Application::CheckSpeakingFinished",
    )

    assert "RevokeSpeechAudio()" in abort
    assert "device_tts_client_->Abort()" not in abort
    assert "audio_service_.ResetDecoder()" not in abort


def test_normal_tts_stop_khong_revoke_va_cat_duoi_opus_som():
    source = _text(APPLICATION_SOURCE)
    set_state = _function(
        source,
        "void Application::SetDeviceState",
        "void Application::Reboot",
    )

    assert "RevokeSpeechAudio()" not in set_state


def test_dong_audio_channel_thu_hoi_loa_truoc_khi_ve_idle():
    source = _text(APPLICATION_SOURCE)
    closed = _function(
        source,
        "protocol_->OnAudioChannelClosed",
        "device_tts_client_ = std::make_unique<DeviceTtsClient>()",
    )

    assert closed.index("RevokeSpeechAudio()") < closed.index(
        "SetDeviceState(kDeviceStateIdle)"
    )


def test_reset_decoder_huy_decode_cu_va_cho_frame_dang_ra_loa():
    header = _text(AUDIO_SERVICE_HEADER)
    source = _text(AUDIO_SERVICE_SOURCE)
    reset = _function(
        source,
        "void AudioService::ResetDecoder",
        "void AudioService::CheckAndUpdateAudioPowerState",
    )
    codec_loop = _function(
        source,
        "void AudioService::OpusCodecTask",
        "void AudioService::SetDecodeSampleRate",
    )
    output_loop = _function(
        source,
        "void AudioService::AudioOutputTask",
        "void AudioService::OpusCodecTask",
    )

    for field in (
        "audio_decode_generation_",
        "audio_decode_in_flight_",
        "audio_output_in_flight_",
    ):
        assert field in header

    assert "++audio_decode_generation_" in reset
    assert "audio_queue_cv_.wait" in reset
    assert "!audio_decode_in_flight_ && !audio_output_in_flight_" in reset
    assert reset.index("audio_queue_cv_.wait") < reset.index(
        "opus_decoder_->ResetState()"
    )

    assert "decode_generation = audio_decode_generation_" in codec_loop
    assert "decode_generation == audio_decode_generation_" in codec_loop
    assert "audio_decode_in_flight_ = true" in codec_loop
    assert "audio_decode_in_flight_ = false" in codec_loop

    assert "audio_output_in_flight_ = true" in output_loop
    assert output_loop.index("audio_output_in_flight_ = true") < output_loop.index(
        "codec_->OutputData(task->pcm)"
    )
    assert output_loop.index("codec_->OutputData(task->pcm)") < output_loop.index(
        "audio_output_in_flight_ = false"
    )
