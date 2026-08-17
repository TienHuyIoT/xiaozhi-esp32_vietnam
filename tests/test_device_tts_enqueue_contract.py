"""Hop dong: `Enqueue()` KHONG duoc chan duong cua Application, va KHONG duoc vut cau.

Do tren robot COM5 ngay 16/08 (log `m1_retest_1608.log`, 7 luot / 39 segment):

    16631.889  queue enqueue  segment-63e7882b   <- Enqueue() ghi trace roi goi EnsureStarted()
    16631.918  queue dequeue  segment-63e7882b
    16631.939  connect_begin  segment-63e7882b
    16637.432  E khong bat duoc duong phat ... reason=stream_start_failed   <- CUNG segment do
    16639.006  vong nguon ket thuc

Tu enqueue den dong bao that bai la **5543 ms** -- dung bang vong cho
`for (i < 50) vTaskDelay(100ms)` trong `AudioStreamPlayer::StopStream()`.
Duong di: `Enqueue -> EnsureStarted -> StartStream -> StopStream(cho worker cu)`.

Hai hau qua, ca hai deu do duoc:

1. **Treo may.** `Application::EnqueueDeviceTtsIfLeaseCurrent` giu
   `speech_audio_transition_mutex_` trong luc goi `Enqueue()`, ma
   `RevokeSpeechAudio`/`TransitionSpeechAudio` cung can khoa do -> ca cum so huu
   audio dung hinh toi 5,5 giay.
2. **Mat cau.** `EnsureStarted()` tra false thi cau bi bo luon. Do duoc 7 lan
   `reason=stream_start_failed`, roi vao **6/7 luot**, va lan nao cung la cau
   **DAU** cua luot -- dung cau be dang doi. Day la phan lon cua ~7s doi ra
   ngoai ngan sach (san 5,1s, quan sat 12,1s).

Vi sao khong phai T3 (ha nguong dem 4KB): do tren cung log,
`first_provider_byte -> first_decoded_pcm` o segment DAU luot chi 231ms p50 /
468ms max, va khe im lang giua hai cau khi byte da co trong tay la **21ms p50 /
22ms p90**. Nguong 4KB khong phai cho nghen. Con so 3515ms p90 ghi trong plan la
do do nham: no gom ca thoi gian **cau truoc dang duoc doc** (`ConsumeTraceInputBytes`
la FIFO tuyet doi nen `first_decoded_pcm` cua cau N+1 khong the no truoc
`last_pcm` cua cau N).

Day la host contract test vi firmware chua co C++ unit runner native; build
ESP-IDF va phep do robot that van la cong E2E bat buoc.
"""

from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[1]
DEVICE_HEADER = ROOT / "main" / "protocols" / "device_tts_client.h"
DEVICE_SOURCE = ROOT / "main" / "protocols" / "device_tts_client.cc"


def _text(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def _function(source: str, signature: str, next_signature: str) -> str:
    start = source.index(signature)
    end = source.index(next_signature, start)
    return source[start:end]


def _enqueue_body(source: str) -> str:
    """Than cua overload `Enqueue(body, trace)` -- ban lam viec that."""
    return _function(
        source,
        "void DeviceTtsClient::Enqueue(const std::string &tts_body,",
        "void DeviceTtsClient::Abort()",
    )


def _code_only(text: str) -> str:
    """Bo chu thich.

    Bat bien can khoa la "khong con LOI GOI", khong phai "khong con CHU". Chinh
    cho sua nay bat buoc phai ghi lai bang chung (`stream_start_failed`,
    `EnsureStarted`) trong chu thich de nguoi sau khong khoi phuc lai bug --
    tim tren van ban tho se bao duong tinh gia.
    """
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.S)
    return re.sub(r"//[^\n]*", "", text)


def test_enqueue_khong_goi_ensure_started():
    """`EnsureStarted` di qua `StopStream`, cho worker cu toi 5s -- cam tren duong Application."""
    enqueue = _code_only(_enqueue_body(_text(DEVICE_SOURCE)))

    assert "EnsureStarted()" not in enqueue, (
        "Enqueue goi EnsureStarted -> chan Application 5,5s (do 16/08)"
    )


def test_enqueue_khong_vut_cau_khi_chua_bat_duoc_player():
    """Cau da nam trong hang doi thi phai duoc doc, du player chua san sang.

    Ban cu ghi mot dong ESP_LOGE roi bo mac cau do: 7 cau mat han, 6/7 luot mat
    dung cau DAU. Hang doi la noi giu cau; khong duoc co duong nao lam no boc hoi.
    """
    enqueue = _code_only(_enqueue_body(_text(DEVICE_SOURCE)))

    assert "stream_start_failed" not in enqueue, (
        "Enqueue van con duong bo cau khi khong bat duoc player"
    )
    assert "RequestStart()" in enqueue, (
        "Enqueue phai nho task khac bat player thay vi tu bat/tu bo"
    )


def test_start_duoc_chay_tren_task_preconnect_chu_khong_phai_worker_audio():
    """`StopStream` tu choi khi bi goi tu chinh source/play task.

    Task preconnect duoc tao trong constructor, doc lap voi stream, nen no la
    cho duy nhat vua KHONG phai duong Application vua KHONG phai worker audio.
    """
    source = _text(DEVICE_SOURCE)
    routine = _function(
        source,
        "void DeviceTtsClient::PreconnectTaskRoutine()",
        "bool DeviceTtsClient::ShouldRetrySegment",
    )
    loop = _function(
        source,
        "void DeviceTtsClient::SourceDataLoop",
        "AudioTraceContext DeviceTtsClient::GetActiveTrace",
    )

    assert "EnsureStarted()" in routine, "task preconnect phai la noi bat player"
    assert "EnsureStarted()" not in loop, (
        "source task goi EnsureStarted -> StopStream tu choi (deadlock/log loi)"
    )


def test_start_request_co_duong_danh_thuc_rieng_khong_di_nho_preconnect():
    """`RequestPreconnect()` thoat som khi socket am con dung duoc.

    Neu gui yeu cau bat player qua duong do thi dung luc socket am con tot --
    tuc luc thuong gap nhat -- task se khong bao gio duoc danh thuc.
    """
    source = _text(DEVICE_SOURCE)
    header = _text(DEVICE_HEADER)
    request_start = _function(
        source,
        "void DeviceTtsClient::RequestStart()",
        "void DeviceTtsClient::RequestPreconnect()",
    )

    assert "start_requested_" in header
    assert "kPreconnectWakeBit" in request_start, (
        "RequestStart phai tu set bit danh thuc, khong duoc dua vao RequestPreconnect"
    )
    assert "ready_websocket_" not in request_start, (
        "RequestStart khong duoc thoat som theo trang thai socket am"
    )


def test_thu_lai_bat_player_co_chan_va_chi_khi_con_cau_trong_hang_doi():
    """Thu lai mai se quay nong CPU; khong thu lai thi cau ket trong hang doi.

    Dieu kien dung: con cau cho. Co nghi giua hai lan de khong quay nong khi
    ly do that bai la tuc thi (vi du chua Configure).
    """
    source = _text(DEVICE_SOURCE)
    header = _text(DEVICE_HEADER)
    routine = _function(
        source,
        "void DeviceTtsClient::PreconnectTaskRoutine()",
        "bool DeviceTtsClient::ShouldRetrySegment",
    )

    start = routine.index("EnsureStarted()")
    retry = routine[start : start + 700]

    assert "HasQueuedSentence()" in retry, (
        "chi thu lai khi con cau cho -- neu khong se quay hoai sau Abort"
    )
    assert "kStartRetryDelayMs" in retry, "phai co nghi giua hai lan thu"
    match = re.search(r"kStartRetryDelayMs\s*=\s*(\d+)", header)
    assert match, "thieu hang so nghi giua hai lan thu bat player"
    assert 0 < int(match.group(1)) <= 500


def test_bang_chung_5543ms_duoc_ghi_lai_trong_ma_nguon():
    """Nguoi sau se thay `Enqueue` khong tu bat player va tuong la thieu sot.

    Con so do phai nam ngay canh cho sua, khong phai chi trong plan.
    """
    source = _text(DEVICE_SOURCE)
    enqueue = _enqueue_body(source)

    assert "StopStream" in enqueue, "phai noi ro cho nao gay treo"
    assert re.search(r"5[,.]5|5543", enqueue), "phai ghi so do that"
