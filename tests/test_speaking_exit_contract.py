"""Hop dong host: robot phai LUON co duong ra khoi trang thai Speaking.

Review 16/08 tim ra trieu chung "robot noi mai, khong mo mic" khong phai mot loi
ma la mot LOP loi: `CheckSpeakingFinished()` co nam guard noi tiep, moi guard la
mot chot cung, va ham chi duoc goi tu ba diem su kien -- khong co timeout nao.
Vi the va hai lan (84292a3, 40c0d12) van con ket: moi lan chi dong mot chot.

Cac test o day khoa hai thu:

  1. Tung chot da biet: `aborted_` (khong bao gio duoc xoa ngoai `tts:start`) va
     completion Opus bi nuot khi generation da doi.
  2. Mot luoi chung: watchdog do KHONG CO TIEN TRIEN, log dich danh guard nao
     dang chan, roi cuong buc roi Speaking. Chot nao chua biet thi lan sau con
     bang chung tren serial thay vi phai doc lai mot nghin dong ma nguon.

ESP-IDF project chua co unit-test runner native nen test doc van ban ma nguon,
cung khuon voi test_audio_state_contract.py. Build dung board + trace robot that
van la cong xac nhan E2E bat buoc.
"""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
APPLICATION_HEADER = ROOT / "main" / "application.h"
APPLICATION_SOURCE = ROOT / "main" / "application.cc"
DEVICE_TTS_SOURCE = ROOT / "main" / "protocols" / "device_tts_client.cc"


def _text(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def _function(source: str, signature: str, next_signature: str) -> str:
    start = source.index(signature)
    end = source.index(next_signature, start)
    return source[start:end]


def _stall_body() -> str:
    return _function(
        _text(APPLICATION_SOURCE),
        "void Application::CheckSpeakingStall",
        "void Application::ResetDecoderBeforeSpeakingIfUnowned",
    )


# --------------------------------------------------------------------------
# Chot #1 -- `aborted_`
# --------------------------------------------------------------------------


def test_abort_speaking_tu_roi_trang_thai_speaking():
    """`AbortSpeaking` phai tu doi state, khong duoc trong cho ai goi ho.

    Ba ly do doc lap, moi ly do du de ket:
      - `aborted_` chi duoc xoa o nhanh `tts:start`; con `CheckSpeakingFinished`
        thi `return` ngay khi thay co do.
      - `DeviceTtsClient::SourceDataLoop` chi ban `on_idle_()` khi `!abort_`, ma
        `Abort()` vua dat co do -> callback duy nhat con lai cung im.
      - Ba trong bon diem goi `AbortSpeaking` (nut BOOT, wake-word x2) khong he
        doi state sau do.
    """
    source = _text(APPLICATION_SOURCE)
    body = _function(
        source,
        "void Application::AbortSpeaking",
        "void Application::CheckSpeakingFinished",
    )

    assert "SetDeviceState(" in body, "AbortSpeaking phai roi Speaking"
    # Cung luat voi CheckSpeakingFinished: manual-stop ve Idle, con lai mo mic.
    assert "kListeningModeManualStop" in body
    assert "kDeviceStateIdle" in body
    assert "kDeviceStateListening" in body
    # Chi doi state khi dang o Speaking; abort tu Idle/Listening khong duoc keo
    # robot vao Listening ngoai y muon.
    assert "kDeviceStateSpeaking" in body

    # Thu tu song con: thu hoi ca hai producer TRUOC khi mo mic, neu khong PCM
    # cua turn vua huy con chay trong luc robot da nghe.
    assert body.index("RevokeSpeechAudio()") < body.index("SetDeviceState(")


def test_device_tts_khong_ban_on_idle_sau_abort_nen_khong_the_trong_vao_no():
    """Khoa dung tien de cua test tren: `on_idle_` bi `abort_` chan."""
    source = _text(DEVICE_TTS_SOURCE)

    idle_call = source.index("on_idle_()")
    guard = source.rindex("!abort_.load()", 0, idle_call)
    # Dieu kien abort nam ngay truoc loi goi -> khong co duong nao ban idle sau
    # khi Abort() da dat co.
    assert idle_call - guard < 400


# --------------------------------------------------------------------------
# Chot #4 -- completion Opus bi nuot
# --------------------------------------------------------------------------


def test_completion_opus_stale_van_phai_kiem_lai_guard():
    """Completion cua generation cu VAN co nghia: duong Opus da drain xong.

    Ban cu `return` khi generation da doi -> sau mot lan doi nguon giua luot,
    khong con su kien nao goi lai `CheckSpeakingFinished`. Bo qua stale la an
    toan vi chinh `CheckSpeakingFinished` doc lai trang thai song, chu khong tin
    vao danh tinh cua completion.
    """
    source = _text(APPLICATION_SOURCE)
    body = _function(
        source,
        "void Application::HandleServerOpusPlaybackFinished",
        "// Add a async task to MainLoop",
    )

    assert "CheckSpeakingFinished();" in body
    assert "speech_audio_generation_ !=" not in body, (
        "khong duoc nuot completion theo generation"
    )


# --------------------------------------------------------------------------
# Luoi chung -- watchdog thoat Speaking
# --------------------------------------------------------------------------


def test_watchdog_duoc_goi_moi_giay_tu_clock_tick():
    header = _text(APPLICATION_HEADER)
    source = _text(APPLICATION_SOURCE)

    assert "void CheckSpeakingStall();" in header
    tick = _function(
        source,
        "if (bits & MAIN_EVENT_CLOCK_TICK)",
        "void Application::OnWakeWordDetected",
    )
    assert "CheckSpeakingStall();" in tick


def test_watchdog_log_dich_danh_tung_guard():
    """Muc dich chinh cua watchdog la BANG CHUNG, khong phai viec cuong buc.

    Lan sau con ket thi serial phai noi duoc guard nao dang giu.
    """
    body = _stall_body()

    for guard in ("aborted=", "tts_stop=", "device_tts_busy=", "opus_busy="):
        assert guard in body, f"log thieu guard {guard}"
    assert "ESP_LOGW" in body


def test_watchdog_do_TIEN_TRIEN_chu_khong_do_tong_thoi_gian_noi():
    """Mot luot bai hoc dai la BINH THUONG, khong phai ket.

    Neu watchdog dem tong thoi gian o Speaking thi no se cat oan cau dai. No
    phai dem khoang KHONG CO GI NHUC NHICH: PCM khong chay them, byte khong ve,
    hai co busy khong doi.
    """
    body = _stall_body()

    assert "GetPlayTimeMs()" in body, "thieu tin hieu PCM dang chay"
    assert "GetBufferSize()" in body, "thieu tin hieu byte dang ve"
    # Co tien trien thi phai reset dong ho, neu khong van la dem tong thoi gian.
    assert "speaking_stall_ticks_ = 0" in body


def test_watchdog_chi_cuong_buc_khi_server_da_het_luot():
    """Server con dang nha cau thi im lang la viec cua server, khong cuong buc.

    Cuong buc trong luc server con gui se lam segment ke tiep bi
    `EnqueueDeviceTtsIfLeaseCurrent` bo (no doi state == Speaking) -- tu chua
    ket thanh mat cau.
    """
    body = _stall_body()

    force = body.index("kSpeakingStallTimeoutSeconds")
    gate = body.index("tts_stop_received_ || aborted_")
    assert gate < force, "phai chan dieu kien het luot truoc khi cuong buc"
    # Thu hoi producer roi moi mo mic, giong AbortSpeaking.
    assert body.index("RevokeSpeechAudio()") < body.rindex("SetDeviceState(")


def test_watchdog_khong_duoc_lam_io_mang():
    """Chay tren main task canh moi viec khac cua vong su kien."""
    body = _stall_body()

    for forbidden in ("protocol_->Send", "OpenAudioChannel", "vTaskDelay"):
        assert forbidden not in body


def test_nguong_watchdog_la_hang_so_co_ten_khong_rai_so_tran():
    header = _text(APPLICATION_HEADER)

    assert "kSpeakingStallLogSeconds" in header
    assert "kSpeakingStallTimeoutSeconds" in header
