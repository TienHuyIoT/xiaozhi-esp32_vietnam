"""Hop dong host M1 cho playback acknowledgement cua firmware.

ESP-IDF project chua co unit-test runner native. Cac test doc source de khoa
capability, duong drain tren main task va payload metadata-only; build dung board
va trace robot that van la cong xac nhan E2E bat buoc.
"""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
APPLICATION_HEADER = ROOT / "main" / "application.h"
APPLICATION_SOURCE = ROOT / "main" / "application.cc"
PROTOCOL_HEADER = ROOT / "main" / "protocols" / "protocol.h"
PROTOCOL_SOURCE = ROOT / "main" / "protocols" / "protocol.cc"
WEBSOCKET_PROTOCOL_SOURCE = ROOT / "main" / "protocols" / "websocket_protocol.cc"
PLAYER_HEADER = ROOT / "main" / "features" / "music" / "audio_stream_player.h"
PLAYER_SOURCE = ROOT / "main" / "features" / "music" / "audio_stream_player.cc"
DEVICE_TTS_SOURCE = ROOT / "main" / "protocols" / "device_tts_client.cc"
AUDIO_SERVICE_HEADER = ROOT / "main" / "audio" / "audio_service.h"
AUDIO_SERVICE_SOURCE = ROOT / "main" / "audio" / "audio_service.cc"


def _text(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def _function(source: str, signature: str, next_signature: str) -> str:
    start = source.index(signature)
    end = source.index(next_signature, start)
    return source[start:end]


def test_websocket_khai_bao_audio_state_capability_version_1():
    source = _text(WEBSOCKET_PROTOCOL_SOURCE)

    assert 'SetHeader("Audio-State-Version", "1")' in source
    assert source.index('SetHeader("Speech-Audio-Frame-Version", "1")') < source.index(
        'SetHeader("Audio-State-Version", "1")'
    )


def test_protocol_gui_ack_metadata_only_khong_ro_noi_dung_tts():
    header = _text(PROTOCOL_HEADER)
    source = _text(PROTOCOL_SOURCE)

    assert "void SendAudioPlaybackState(" in header
    body = _function(
        source,
        "void Protocol::SendAudioPlaybackState",
        "bool Protocol::IsTimeout",
    )
    for field in (
        '"audio_state"',
        '"state"',
        '"turn_id"',
        '"segment_id"',
        '"audio_source"',
        '"generation"',
    ):
        assert field in body
    assert "SendText(message)" in body
    assert 'segment_id[0] != \'\\0\'' in body

    for forbidden in ("tts_body", "body", "url", "config_frame"):
        assert forbidden not in body


def test_ack_mac_dinh_tat_va_chi_interaction_config_duoc_doi_co():
    header = _text(APPLICATION_HEADER)
    source = _text(APPLICATION_SOURCE)

    assert "std::atomic<bool> playback_ack_enabled_{false}" in header
    interaction = _function(
        source,
        '} else if (strcmp(type->valuestring, "interaction_config") == 0)',
        '} else if (strcmp(type->valuestring, "tts") == 0)',
    )
    assert "version" in interaction
    assert "cJSON_IsNumber" in interaction
    assert "valueint == 1" in interaction
    assert "valuedouble == 1.0" in interaction
    assert "playback_ack" in interaction
    assert "cJSON_IsBool" in interaction
    assert "playback_ack_enabled_.store" in interaction
    # Chi config hop le va lifecycle mo session moi duoc thay doi co runtime.
    assert source.count("playback_ack_enabled_.store") == 2


def test_reconnect_thu_hoi_ack_truoc_khi_websocket_nhan_config_phien_moi():
    protocol_header = _text(PROTOCOL_HEADER)
    protocol_source = _text(PROTOCOL_SOURCE)
    websocket_source = _text(WEBSOCKET_PROTOCOL_SOURCE)
    app = _text(APPLICATION_SOURCE)

    assert "OnAudioChannelOpening" in protocol_header
    assert "on_audio_channel_opening_" in protocol_header
    assert "void Protocol::OnAudioChannelOpening" in protocol_source
    opening = _function(
        websocket_source,
        "bool WebsocketProtocol::OpenAudioChannel()",
        "std::string WebsocketProtocol::GetHelloMessage()",
    )
    assert "on_audio_channel_opening_()" in opening
    assert opening.index("on_audio_channel_opening_()") < opening.index(
        "CreateWebSocket"
    )

    registration = _function(
        app,
        "protocol_->OnAudioChannelOpening",
        "protocol_->OnAudioChannelOpened",
    )
    assert "playback_ack_enabled_.store(false)" in registration


def test_observer_chi_day_ring_va_event_bit_khong_block_allocate_hay_gui_mang():
    header = _text(APPLICATION_HEADER)
    source = _text(APPLICATION_SOURCE)
    observer = _function(
        source,
        "void Application::ObserveAudioTrace",
        "void Application::DrainAudioPlaybackStates",
    )

    assert "MAIN_EVENT_AUDIO_PLAYBACK_STATE" in header
    assert "xEventGroupSetBits(" in observer
    assert "MAIN_EVENT_AUDIO_PLAYBACK_STATE" in observer
    assert "audio_playback_state_ring_" in observer
    assert "Schedule(" not in observer
    assert "main_tasks_" not in observer
    assert "SendAudioPlaybackState(" not in observer
    assert "SendText(" not in observer
    assert "protocol_" not in observer
    assert "->OnData(" not in observer

    drain = _function(
        source,
        "void Application::DrainAudioPlaybackStates",
        "void Application::Schedule",
    )
    assert "protocol_->SendAudioPlaybackState" in drain

    main_loop = _function(
        source,
        "void Application::MainEventLoop()",
        "void Application::OnWakeWordDetected()",
    )
    assert "MAIN_EVENT_AUDIO_PLAYBACK_STATE" in main_loop
    assert "DrainAudioPlaybackStates();" in main_loop


def test_ring_ack_co_tran_cung_va_dem_goi_bi_drop():
    header = _text(APPLICATION_HEADER)
    source = _text(APPLICATION_SOURCE)

    assert "kAudioPlaybackStateRingCapacity" in header
    assert "audio_playback_state_drop_count_" in header
    assert "std::array<AudioPlaybackState" in header
    record = header[
        header.index("struct AudioPlaybackState") : header.index(
            "static constexpr size_t kAudioPlaybackStateRingCapacity"
        )
    ]
    assert "std::string" not in record
    assert "std::array<char" in record
    assert "audio_playback_state_count_" in source
    assert "kAudioPlaybackStateRingCapacity" in source
    assert "audio_playback_state_drop_count_.fetch_add" in source

    observer = _function(
        source,
        "void Application::ObserveAudioTrace",
        "void Application::DrainAudioPlaybackStates",
    )
    for allocating_container in ("push_back", "emplace_back", "std::string"):
        assert allocating_container not in observer


def test_trace_observer_anh_xa_du_5_moc_playback():
    header = _text(PLAYER_HEADER)
    source = _text(PLAYER_SOURCE)
    app = _text(APPLICATION_SOURCE)

    assert "using AudioTraceObserver" in header
    assert "SetAudioTraceObserver" in header
    assert "SetAudioTraceObserver" in app
    assert "audio_trace_observer_" in source
    for trace_event, playback_state in (
        ("queue", "playback_queued"),
        ("first_decoded_pcm", "playback_started"),
        ("last_pcm", "playback_finished"),
        ("abort", "playback_aborted"),
        ("drop_stale", "playback_dropped_stale"),
    ):
        assert f'"{trace_event}"' in app + source + _text(DEVICE_TTS_SOURCE)
        assert f'"{playback_state}"' in app


def test_generation_duoc_dong_dau_theo_lease_va_khong_doc_global_khi_event_den_muon():
    header = _text(PLAYER_HEADER)
    source = _text(APPLICATION_SOURCE)
    observer = _function(
        source,
        "void Application::ObserveAudioTrace",
        "void Application::DrainAudioPlaybackStates",
    )
    sentence = _function(
        source,
        '} else if (strcmp(state->valuestring, "sentence_start") == 0)',
        'auto text = cJSON_GetObjectItem(root, "text")',
    )

    assert "uint32_t playback_generation" in header
    assert "bool playback_generation_present" in header
    assert "trace.playback_generation = lease.generation" in sentence
    assert "trace.playback_generation_present = true" in sentence
    assert "ack.generation = trace.playback_generation" in observer
    assert "speech_audio_generation_" not in observer


def test_drop_stale_duoc_trace_o_ca_hai_duong_loai_bo():
    app = _text(APPLICATION_SOURCE)
    device = _text(DEVICE_TTS_SOURCE)

    assert 'LogAudioTraceEvent("drop_stale"' in app
    assert 'LogAudioTraceEvent("drop_stale"' in device


def test_server_opus_packet_giu_trace_sequence_pod_den_output_task():
    protocol_header = _text(PROTOCOL_HEADER)
    audio_header = _text(AUDIO_SERVICE_HEADER)
    audio_source = _text(AUDIO_SERVICE_SOURCE)
    app = _text(APPLICATION_SOURCE)

    assert "uint32_t audio_trace_sequence" in protocol_header
    assert "uint32_t audio_trace_sequence" in audio_header
    assert "packet->audio_trace_sequence" in app

    codec = _function(
        audio_source,
        "void AudioService::OpusCodecTask()",
        "void AudioService::SetDecodeSampleRate",
    )
    assert "task->audio_trace_sequence = packet->audio_trace_sequence" in codec


def test_server_opus_dang_ky_metadata_mot_lan_trong_bang_bounded():
    header = _text(AUDIO_SERVICE_HEADER)
    source = _text(AUDIO_SERVICE_SOURCE)
    app = _text(APPLICATION_SOURCE)

    assert "RegisterServerOpusTrace" in header
    assert "struct ServerOpusTraceSlot" in header
    assert "kMaxServerOpusTraceSlots" in header
    assert "std::array<ServerOpusTraceSlot" in header
    assert "RegisterServerOpusTrace(trace)" in app
    assert "next_server_opus_trace_sequence_" in _text(APPLICATION_HEADER)

    register = _function(
        source,
        "bool AudioService::RegisterServerOpusTrace",
        "void AudioService::RequestServerOpusTraceFinish",
    )
    assert 'LogAudioTraceQueue("queue", "enqueue"' in register
    for forbidden in ("tts_body", "body.c_str()", '"payload"', '"text"'):
        assert forbidden not in register


def test_server_opus_khong_correlation_van_drain_va_completion_cho_backend_cu():
    """ACK co the bo qua metadata thieu, nhung lifecycle playback khong duoc ket."""
    source = _text(AUDIO_SERVICE_SOURCE)
    register = _function(
        source,
        "bool AudioService::RegisterServerOpusTrace",
        "void AudioService::RequestServerOpusTraceFinish",
    )

    assert "trace.trace_sequence == 0" in register
    assert "!trace.playback_generation_present" in register
    assert 'trace.audio_source != "server_opus"' in register
    assert "trace.turn_id.empty()" not in register
    assert 'trace.turn_id == "-"' not in register


def test_server_opus_bang_trace_day_van_gui_completion_de_khong_ket_speaking():
    header = _text(AUDIO_SERVICE_HEADER)
    source = _text(AUDIO_SERVICE_SOURCE)
    register = _function(
        source,
        "bool AudioService::RegisterServerOpusTrace",
        "void AudioService::RequestServerOpusTraceFinish",
    )
    finish = _function(
        source,
        "void AudioService::MaybeFinishServerOpusTraceLocked",
        "bool AudioService::RegisterServerOpusTrace",
    )

    assert "server_opus_playback_generation_" in header
    assert "server_opus_playback_generation_ = trace.playback_generation" in register
    assert "server_opus_playback_generation_" in finish
    assert finish.index("slot != nullptr") < finish.index(
        "on_server_opus_playback_finished"
    )


def test_server_opus_started_finished_bam_pcm_that_va_tts_stop_request_finish():
    header = _text(AUDIO_SERVICE_HEADER)
    source = _text(AUDIO_SERVICE_SOURCE)
    app = _text(APPLICATION_SOURCE)

    assert "RequestServerOpusTraceFinish" in header
    assert "RequestServerOpusTraceFinish();" in app
    output = _function(
        source,
        "void AudioService::AudioOutputTask()",
        "void AudioService::OpusCodecTask()",
    )
    assert "task->audio_trace_sequence" in output
    assert "esp_timer_get_time()" in output
    assert "NotifyAudioTraceEvent" in output
    assert output.count("NotifyAudioTraceEvent(") == 2
    assert '"first_decoded_pcm"' in output
    assert '"last_pcm"' in output
    assert output.index("NotifyAudioTraceEvent") < output.index(
        "codec_->OutputData(task->pcm)"
    )
    assert output.index('NotifyAudioTraceEvent("last_pcm"') < output.index(
        'NotifyAudioTraceEvent("first_decoded_pcm"'
    )
    assert output.index("codec_->OutputData(task->pcm)") < output.rindex(
        '"first_decoded_pcm"'
    )
    assert "false);" in output


def test_server_opus_multi_segment_ket_thuc_cau_cu_truoc_khi_start_cau_moi():
    source = _text(AUDIO_SERVICE_SOURCE)
    output = _function(
        source,
        "void AudioService::AudioOutputTask()",
        "void AudioService::OpusCodecTask()",
    )

    assert "active_server_opus_trace_sequence_ !=" in output
    assert "previous_last_pcm_us" in output
    assert "output_finished_us" in output
    assert output.index('"last_pcm"') < output.index('"first_decoded_pcm"')
    assert "*previous_trace, previous_last_pcm_us" in output
    assert "*current_trace, output_started_us" in output


def test_server_opus_stamp_packet_nam_trong_cung_transition_lock_voi_owner():
    app = _text(APPLICATION_SOURCE)
    admission = _function(
        app,
        "Application::AcceptServerOpusPacket",
        "Application::EnqueueDeviceTtsIfLeaseCurrent",
    )

    transition_lock = admission.index("speech_audio_transition_mutex_")
    owner_check = admission.index("SpeechAudioSource::kServerOpus")
    trace_lock = admission.index("lock(audio_trace_mutex_)")
    stamp = admission.index("packet->audio_trace_sequence")
    push = admission.index("PushPacketToDecodeQueue")
    assert transition_lock < owner_check < trace_lock < stamp < push


def test_server_opus_dang_ky_trace_khong_lot_qua_abort_sau_khi_cap_lease():
    app = _text(APPLICATION_SOURCE)
    sentence = _function(
        app,
        '} else if (strcmp(state->valuestring, "sentence_start") == 0)',
        'auto text = cJSON_GetObjectItem(root, "text")',
    )

    transition_lock = sentence.index("speech_audio_transition_mutex_")
    lease_check = sentence.index("IsSpeechAudioLeaseCurrent(lease)")
    trace_commit = sentence.index("active_audio_trace_ = trace")
    register = sentence.index("RegisterServerOpusTrace(trace)")
    assert transition_lock < lease_check < trace_commit < register


def test_server_opus_reset_abort_khong_phat_finished_muon_sang_generation_moi():
    source = _text(AUDIO_SERVICE_SOURCE)
    reset = _function(
        source,
        "void AudioService::ResetDecoder()",
        "void AudioService::CheckAndUpdateAudioPowerState",
    )

    assert "server_opus_finish_requested_ = false" in reset
    assert "server_opus_trace_event_in_flight_" in reset
    assert reset.index("audio_queue_cv_.wait") < reset.index(
        "ClearServerOpusTracesLocked"
    )


def test_tts_stop_cho_ca_device_va_server_opus_drain_truoc_khi_doi_state():
    app = _text(APPLICATION_SOURCE)
    audio_header = _text(AUDIO_SERVICE_HEADER)
    stop = _function(
        app,
        '} else if (strcmp(state->valuestring, "stop") == 0)',
        '} else if (strcmp(state->valuestring, "sentence_start") == 0)',
    )
    check = _function(
        app,
        "void Application::CheckSpeakingFinished()",
        "void Application::SetListeningMode",
    )

    assert "RequestServerOpusTraceFinish();" in stop
    assert "CheckSpeakingFinished();" in stop
    assert "SetDeviceState(" not in stop
    assert "IsServerOpusPlaybackBusy()" in check
    assert "device_tts_client_->IsBusy()" in check
    assert "aborted_" in check
    assert "IsServerOpusPlaybackBusy" in audio_header


def test_device_tts_stop_khong_cho_nham_audio_service_queue_khi_khong_co_opus():
    audio = _text(AUDIO_SERVICE_SOURCE)
    request = _function(
        audio,
        "void AudioService::RequestServerOpusTraceFinish()",
        "bool AudioService::IsServerOpusPlaybackBusy()",
    )

    no_trace_guard = request.index("server_opus_playback_generation_ == 0")
    mark_finish = request.index("server_opus_finish_requested_ = true")
    assert no_trace_guard < mark_finish


def test_server_opus_completion_ve_main_task_dung_generation_va_chi_mot_lan():
    app_header = _text(APPLICATION_HEADER)
    app = _text(APPLICATION_SOURCE)
    audio_header = _text(AUDIO_SERVICE_HEADER)
    audio = _text(AUDIO_SERVICE_SOURCE)

    assert "on_server_opus_playback_finished" in audio_header
    assert "server_opus_completed_generation_" in app_header
    assert "MAIN_EVENT_SERVER_OPUS_FINISHED" in app_header
    assert "MAIN_EVENT_SERVER_OPUS_FINISHED" in app
    assert "playback_generation" in audio

    finish = _function(
        audio,
        "void AudioService::MaybeFinishServerOpusTraceLocked",
        "bool AudioService::RegisterServerOpusTrace",
    )
    assert finish.count("on_server_opus_playback_finished") == 2
    assert finish.index("server_opus_finish_requested_ = false") < finish.index(
        "on_server_opus_playback_finished"
    )

    handler = _function(
        app,
        "void Application::HandleServerOpusPlaybackFinished",
        "void Application::Schedule",
    )
    # HOP DONG DOI 16/08 -- ghi ly do vao day chu khong phai sua test cho vua
    # code: ban cu bat main task BO QUA completion khi lease da doi generation.
    # Review 16/08 dem duoc do la chot #4 trong nam duong gay ket Speaking --
    # sau mot lan doi nguon giua luot thi khong con su kien nao goi lai
    # CheckSpeakingFinished. Bo dieu kien do an toan vi CheckSpeakingFinished
    # doc lai trang thai SONG (tts_stop_received_, hai co busy) chu khong tin
    # vao danh tinh cua completion. AudioService van chuyen generation qua
    # callback de dam bao "mot lan mot" -- hai assert phia tren van khoa cho do.
    assert "server_opus_completed_generation_" in handler
    assert "CheckSpeakingFinished();" in handler
    assert "speech_audio_generation_ !=" not in handler


def test_abort_reset_khong_phat_completion_stale():
    audio = _text(AUDIO_SERVICE_SOURCE)
    reset = _function(
        audio,
        "void AudioService::ResetDecoder()",
        "void AudioService::CheckAndUpdateAudioPowerState",
    )

    assert "server_opus_finish_requested_ = false" in reset
    assert "on_server_opus_playback_finished" not in reset
