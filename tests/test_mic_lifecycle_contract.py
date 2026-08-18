"""M3.2 -- bat mic KHONG duoc reset duong phat.

Bat bien: `ResetDecoder()` thuoc ve **chuyen quyen so huu tieng noi** va **abort
tuong minh**, KHONG thuoc ve vong doi mic. Truoc M3.2,
`AudioService::EnableVoiceProcessing(true)` goi `ResetDecoder()` ngam, ma
`ResetDecoder()` xoa `audio_decode_queue_` + `audio_playback_queue_`. Nghia la
"mo mic" va "vut audio dang phat" bi buoc vao nhau -- dung cai M3.3 se dam vao
khi bat mic trong luc robot dang noi.

Vi sao bo duoc: khao 19/08 cho thay MOI duong vao kDeviceStateListening da co
chot rieng, khong duong nao dua vao cai reset nay:

  - qua `AbortSpeaking()`/`RevokeSpeechAudio()` -> da co `ResetDecoder()` that;
  - qua `CheckSpeakingFinished()` -> khong reset, ma **guard**: ham tu choi doi
    state khi `device_tts_client_->IsBusy()` hoac
    `audio_service_.IsServerOpusPlaybackBusy()`;
  - tu kDeviceStateIdle -> Idle da `EnableVoiceProcessing(false)`, khong co gi
    dang phat.

Hau qua: bo reset khoi mic lifecycle lam hai guard cua `CheckSpeakingFinished`
tro thanh **chiu luc**. Nen file nay khoa CA HAI phia -- bo reset dung cho, va
giu guard dung cho. Go mot trong hai la cat duoi cau ngay.

Day la host contract test vi firmware chua co C++ unit runner native; build
ESP-IDF va phep do robot that van la cong E2E bat buoc.
"""

from __future__ import annotations

from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[1]
AUDIO_SERVICE = ROOT / "main" / "audio" / "audio_service.cc"
APPLICATION = ROOT / "main" / "application.cc"


def _text(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def _code_only(text: str) -> str:
    """Bo chu thich truoc khi soi.

    Test nay tung TU BAT CHINH NO: chu thich giai thich "reset thuoc ve
    RevokeSpeechAudio()" co chua chuoi `ResetDecoder()`, va phep kiem chuoi tho
    coi do la mot loi goi. Hop dong noi ve MA, khong noi ve van xuoi.
    """
    text = re.sub(r"/\*.*?\*/", " ", text, flags=re.DOTALL)
    return re.sub(r"//[^\n]*", " ", text)


def _function_body(source: str, signature: str) -> str:
    """Than ham tinh theo do sau ngoac nhon, khong dung regex ngay tho."""
    start = source.index(signature)
    brace = source.index("{", start)
    depth = 0
    for index in range(brace, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[brace : index + 1]
    raise AssertionError(f"khong tim duoc than ham cua {signature}")


# --------------------------------------------------------------------------
# 1. Bat mic khong duoc dung toi ResetDecoder
# --------------------------------------------------------------------------

def test_enable_voice_processing_khong_goi_ResetDecoder():
    body = _code_only(_function_body(
        _text(AUDIO_SERVICE), "void AudioService::EnableVoiceProcessing(bool enable)"
    ))
    assert "ResetDecoder()" not in body, (
        "Mo mic dang keo theo viec vut sach audio_decode_queue_ + "
        "audio_playback_queue_. Reset thuoc ve chuyen quyen so huu / abort, "
        "khong thuoc ve vong doi mic."
    )


def test_enable_voice_processing_van_giu_warmup_va_start_processor():
    """Bo reset thoi -- khong duoc tien tay bo phan khoi dong capture."""
    body = _function_body(
        _text(AUDIO_SERVICE), "void AudioService::EnableVoiceProcessing(bool enable)"
    )
    assert "audio_input_need_warmup_ = true" in body
    assert "audio_processor_->Start()" in body
    assert "audio_processor_->Stop()" in body


def test_khong_tao_duong_reset_gian_tiep_trong_EnableVoiceProcessing():
    """Chan cach lach: goi thang cac ham xoa hang doi thay vi ResetDecoder()."""
    body = _code_only(_function_body(
        _text(AUDIO_SERVICE), "void AudioService::EnableVoiceProcessing(bool enable)"
    ))
    for banned in (
        "audio_decode_queue_.clear()",
        "audio_playback_queue_.clear()",
        "ClearServerOpusTracesLocked()",
        "++audio_decode_generation_",
    ):
        assert banned not in body, f"{banned} trong EnableVoiceProcessing = reset tra hinh"


# --------------------------------------------------------------------------
# 2. Reset phai CON o dung cho cua no
# --------------------------------------------------------------------------

def test_chuyen_quyen_so_huu_va_thu_hoi_van_con_ResetDecoder():
    source = _text(APPLICATION)
    for signature in (
        "Application::TransitionSpeechAudio",
        "Application::RevokeSpeechAudio",
    ):
        body = _code_only(_function_body(source, signature))
        assert "audio_service_.ResetDecoder()" in body, (
            f"{signature} mat ResetDecoder -> audio cua luot cu se chen sang luot moi"
        )


# --------------------------------------------------------------------------
# 3. Hai guard tro thanh CHIU LUC sau khi bo reset -- khoa lai
# --------------------------------------------------------------------------

def test_CheckSpeakingFinished_giu_du_hai_guard_ban_ron():
    body = _code_only(
        _function_body(_text(APPLICATION), "void Application::CheckSpeakingFinished()")
    )
    assert "device_tts_client_->IsBusy()" in body, (
        "Mat guard nay thi mic mo trong luc device TTS con cau trong hang doi."
    )
    assert "audio_service_.IsServerOpusPlaybackBusy()" in body, (
        "Mat guard nay thi mic mo trong luc Opus con decode/cho phat -- va vi "
        "M3.2 da bo reset khoi duong mic, khong con gi chan cat duoi nua."
    )
    # Ca hai guard phai la duong THOAT SOM, khong phai chi ghi log.
    for guard in ("device_tts_client_->IsBusy()", "IsServerOpusPlaybackBusy()"):
        index = body.index(guard)
        assert "return" in body[index : index + 200], f"guard {guard} khong return"


def test_moi_duong_vao_Listening_deu_co_chot():
    """Khong duoc them duong vao Listening ma khong qua mot trong ba chot.

    Ba chot hop le: RevokeSpeechAudio/AbortSpeaking (co reset), guard cua
    CheckSpeakingFinished, hoac xuat phat tu Idle. Test nay dem so cho dat state
    Listening -- them cho moi thi phai doc lai va cap nhat co y thuc, chu khong
    de no lot im lang.
    """
    source = _code_only(_text(APPLICATION))
    # Chi dem cho DAT state, khong dem cho SO SANH state. Ca bon cho deu nam
    # trong mot loi goi SetDeviceState(...), ke ca dang ternary.
    setters = re.findall(
        r"SetDeviceState\((?:[^();]|\([^()]*\))*kDeviceStateListening", source
    )
    assert len(setters) == 4, (
        f"So cho DAT kDeviceStateListening doi tu 4 thanh {len(setters)}. "
        "Doc lai tung duong vao Listening: no phai qua Revoke/Abort (co reset), "
        "hoac qua guard cua CheckSpeakingFinished, hoac xuat phat tu Idle."
    )
