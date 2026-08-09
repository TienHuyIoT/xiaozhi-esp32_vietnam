"""T1 (firmware) — cau tong hop hong phai BAO LEN SERVER, khong duoc im lang.

⚠️ TEST RED CO Y (viet 09/08/2026, task T1 cua
`.claude/plans/natural-audio-smart-turn.m1.plan.md` §12.2). Implementation CHUA co
nen ca file se FAIL. Do la trang thai ban giao dung cua TDD.

Do 09/08 tren robot COM5: 34 `connect_begin` -> 27 `first_decoded_pcm`. 21% so cau
be KHONG NGHE THAY GI, va backend khong he biet -> khong co duong phuc hoi. Doi phia
backend da xong (`sonic_backend/tests/unit/test_device_tts_fallback.py`); day la nua
con lai.

Bon bat bien khoa o day, theo thu tu nguy hiem giam dan:

1. **Khong duoc gui khi server chua khai la hieu.** Backend cu khong biet type
   `tts_segment_failed` -> goi roi xuong nhanh `payload = raw_text` -> nguyen cuc
   JSON di vao LLM nhu loi be (dung loi MCP 18/07 da tung xay ra). Thu tu deploy
   firmware/backend khong kiem soat duoc nen day la luoi duy nhat.
2. **Khong duoc gui tu source task.** Callback no tren task nguon cua DeviceTtsClient;
   dung WebSocket tu do la data race. Phai hop ve main task qua `Schedule(`.
3. **Khong duoc mang noi dung hoi thoai len.** Chi turn_id/segment_id/enum ly do.
4. **Capability chi song trong mot phien.** Mo lai kenh phai thu hoi co, giong het
   `playback_ack_enabled_`.

Test doc thang ma nguon C++ dang text — dung quy uoc san co cua 7 file test khac
trong thu muc nay (ESP-IDF khong co host unit runner).
"""

from pathlib import Path

MAIN = Path(__file__).resolve().parents[1] / "main"
APPLICATION_CC = (MAIN / "application.cc").read_text(encoding="utf-8", errors="replace")
APPLICATION_H = (MAIN / "application.h").read_text(encoding="utf-8", errors="replace")
PROTOCOL_CC = (MAIN / "protocols" / "protocol.cc").read_text(encoding="utf-8", errors="replace")
PROTOCOL_H = (MAIN / "protocols" / "protocol.h").read_text(encoding="utf-8", errors="replace")
CLIENT_CC = (MAIN / "protocols" / "device_tts_client.cc").read_text(encoding="utf-8", errors="replace")
CLIENT_H = (MAIN / "protocols" / "device_tts_client.h").read_text(encoding="utf-8", errors="replace")


def _slice(text: str, start: str, end: str) -> str:
    start_at = text.index(start)
    return text[start_at:text.index(end, start_at)]


# ------------------------------------------- 1. capability tu interaction_config


def test_doc_co_device_tts_fallback_tu_interaction_config():
    assert "device_tts_fallback" in APPLICATION_CC, (
        "chua doc capability -> firmware se gui mu quang toi backend cu"
    )
    branch = _slice(APPLICATION_CC, 'strcmp(type->valuestring, "interaction_config")',
                    'strcmp(type->valuestring, "tts")')
    assert "device_tts_fallback" in branch
    assert "device_tts_fallback_enabled_" in branch, (
        "phai luu vao co rieng, khong duoc dung chung `playback_ack_enabled_`: "
        "hai tinh nang, hai co doc lap o backend"
    )
    assert ".store(" in branch


def test_co_capability_la_atomic_va_bi_thu_hoi_khi_mo_lai_kenh():
    assert "std::atomic<bool> device_tts_fallback_enabled_" in APPLICATION_H, (
        "co bi ghi tu task giao thuc, doc tu task nguon -> phai atomic"
    )
    opening = _slice(APPLICATION_CC, "OnAudioChannelOpening(", "OnAudioChannelOpened(")
    assert "device_tts_fallback_enabled_.store(false)" in opening, (
        "capability chi co gia tri trong MOT phien WebSocket; khong thu hoi truoc "
        "connect thi backend cu se nhan bao hong va bien no thanh loi be"
    )


# ---------------------------------------------------- 2. hop dong message gui len


def test_protocol_co_ham_gui_bao_hong():
    assert "SendTtsSegmentFailed" in PROTOCOL_H
    assert "void Protocol::SendTtsSegmentFailed" in PROTOCOL_CC


def test_message_bao_hong_dung_allowlist_va_khong_cho_noi_dung():
    body = _slice(PROTOCOL_CC, "void Protocol::SendTtsSegmentFailed", "\n}\n")
    assert '"tts_segment_failed"' in body
    for field in ("session_id", "turn_id", "segment_id", "reason"):
        assert f'"{field}"' in body, f"thieu field `{field}` trong message"
    for forbidden in ("text", "tts_body", "payload"):
        assert f'"{forbidden}"' not in body, (
            f"`{forbidden}` la noi dung hoi thoai, khong duoc di duong telemetry"
        )


def test_segment_id_bat_buoc_khac_voi_audio_state():
    """`audio_state` cho phep thieu segment_id; bao hong thi khong.

    Backend loai bo goi thieu `segment_id` (parse_tts_segment_failed) vi khong biet
    doc bu cau nao. Gui ra ma khong co la lang phi mot lan roundtrip.
    """
    body = _slice(PROTOCOL_CC, "void Protocol::SendTtsSegmentFailed", "\n}\n")
    ack_body = _slice(PROTOCOL_CC, "void Protocol::SendAudioPlaybackState", "\n}\n")
    assert 'strcmp(segment_id, "-")' in ack_body, "test doc sai ham ACK"
    assert body.count("cJSON_AddStringToObject") >= 5, (
        "segment_id phai duoc them vo dieu kien, khong bo trong nhanh if nhu ACK"
    )


# ------------------------------------------------- 3. duong bao tu DeviceTtsClient


def test_client_co_callback_bao_hong():
    assert "OnSegmentFailed" in CLIENT_H, (
        "DeviceTtsClient khong duoc phu thuoc truc tiep vao Protocol; bao ra bang "
        "callback giong `OnIdle`"
    )
    assert "on_segment_failed_" in CLIENT_H


def test_client_ban_bao_hong_dung_tai_cho_synthesize_that_bai():
    failure = _slice(CLIENT_CC, "const bool ok = SynthesizeOne(", "if (!HasQueuedSentence())")
    assert "on_segment_failed_" in failure, (
        "phai ban ngay tai cho `!ok`, cho toi vong sau thi trace da doi sang cau khac"
    )
    assert "if (!ok)" in failure


def test_ly_do_hong_nam_trong_allowlist_backend_chap_nhan():
    """4 chuoi nay phai khop het `_TTS_SEGMENT_FAILURE_REASONS` cua backend.

    Backend co y KHONG co "unknown": ly do la = hop dong da lech giua hai dau. Lech
    chinh ta o day thi bao hong bi vut im lang o server, dung loi ma T1 di sua.
    """
    for reason in ("connect_failed", "no_audio_timeout", "stream_start_failed",
                   "synthesis_failed"):
        assert f'"{reason}"' in CLIENT_CC, f"firmware chua bao gio bao `{reason}`"


# ----------------------------------------------- 4. wiring an toan o Application


def test_wiring_gac_capability_va_hop_ve_main_task():
    wiring_at = APPLICATION_CC.index("OnSegmentFailed(")
    wiring = APPLICATION_CC[wiring_at:wiring_at + 2200]
    assert "device_tts_fallback_enabled_" in wiring, (
        "BAT BIEN SO 1: khong gac capability -> gui toi backend cu -> luot ma vao LLM"
    )
    assert "Schedule(" in wiring, (
        "callback no tren task nguon cua DeviceTtsClient; dung protocol_ tu do la "
        "data race -> phai hop ve main task"
    )
    assert "SendTtsSegmentFailed" in wiring


def test_khong_gui_khi_chua_co_protocol():
    wiring_at = APPLICATION_CC.index("OnSegmentFailed(")
    wiring = APPLICATION_CC[wiring_at:wiring_at + 2200]
    assert "protocol_ ==" in wiring or "protocol_ !=" in wiring, (
        "kenh co the dong truoc khi Schedule chay -> phai kiem tra protocol_"
    )
