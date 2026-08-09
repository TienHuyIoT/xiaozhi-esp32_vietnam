"""Hop dong chong tut first-audio khi turn moi den luc connect cu con block.

Do tren robot 09/08/2026 (retest `78a022a`) cho thay CAU DAU cua MOI turn phai
tra ~1,6 giay bat tay TLS, trong khi cac cau sau chi ton ~20ms:

    turn      enq-deq  deq-cb  cb-ready  ready-byte  byte-pcm   TOTAL
    fa97ca03       38      19      1654         246       147    2104
    d0562300     1230      19      2272        1701        14    5237
    0385666d       10      38      1532         215        14    1810
    a84de34b       19      29      1595        1237        14    2894
    f5e709fa       23      25      1597         300        14    1960

Hai nguyen nhan trong lop nay:

1. `Abort()` tang `turn_generation_` -> socket warm bi coi la stale -> worker
   preconnect DONG no roi mo lai tu dau. Socket warm KHONG mang danh tinh turn
   (chua gui SSML, chua gan OnData) nen chi `config_generation_` moi duoc phep
   vo hieu hoa no.
2. Sau abort, source worker van chay het mot lan bat tay dong bo ~1,5 giay cho
   segment da chet roi moi dequeue turn moi (do duoc 1230ms o turn d0562300).

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


def test_socket_warm_song_qua_ranh_gioi_turn():
    """Promote khong duoc vut socket warm chi vi turn da doi.

    Socket warm chua gui SSML va chua gan OnData -> no khong thuoc turn nao.
    Chi URL/token moi (config_generation_) moi lam no vo nghia.
    """
    source = _text(DEVICE_SOURCE)
    promote = _function(
        source,
        "bool DeviceTtsClient::PromoteReadySocket()",
        "void DeviceTtsClient::RequestPreconnect()",
    )

    assert "config_generation_.load()" in promote
    assert "turn_generation_" not in promote


def test_preconnect_khong_dong_socket_warm_khi_turn_doi():
    """Worker preconnect khong duoc lay turn_generation lam ly do dong socket."""
    source = _text(DEVICE_SOURCE)
    request = _function(
        source,
        "void DeviceTtsClient::RequestPreconnect()",
        "void DeviceTtsClient::PreconnectTaskRoutine()",
    )
    routine = _function(
        source,
        "void DeviceTtsClient::PreconnectTaskRoutine()",
        "bool DeviceTtsClient::SynthesizeOne",
    )

    assert "config_generation_.load()" in request
    assert "turn_generation_" not in request
    assert "turn_generation_" not in routine


def test_socket_warm_co_han_tuoi_de_khong_dung_ket_noi_nua_song():
    """Giu socket qua nhieu turn thi phai chan TCP half-open.

    IsConnected() van true tren socket nua song; gui SSML vao do se an tron
    watchdog 7 giay. Han tuoi phai nho hon khoi token 5 phut cua nha cung cap.
    """
    header = _text(DEVICE_HEADER)
    source = _text(DEVICE_SOURCE)
    promote = _function(
        source,
        "bool DeviceTtsClient::PromoteReadySocket()",
        "void DeviceTtsClient::RequestPreconnect()",
    )

    match = re.search(r"kReadySocketMaxAgeMs\s*=\s*(\d+)", header)
    assert match, "thieu hang so tuoi toi da cua socket warm"
    assert 0 < int(match.group(1)) < 300_000

    assert "kReadySocketMaxAgeMs" in promote
    assert "ready_socket_opened_us_" in header


def test_khong_bat_tay_dong_bo_cho_segment_da_bi_huy():
    """Segment da chet khong duoc keo source worker qua mot lan TLS 1,5 giay."""
    source = _text(DEVICE_SOURCE)
    ensure = _function(
        source,
        "bool DeviceTtsClient::EnsureConnected",
        "void DeviceTtsClient::CloseSocket()",
    )

    assert "segment_generation" in ensure, "EnsureConnected phai biet generation"
    fallback = ensure.index("OpenConfiguredSocket(snapshot, false)")
    guard = ensure.rindex("turn_generation_.load() != segment_generation")
    assert guard < fallback, "phai kiem tra generation TRUOC khi bat tay dong bo"


def test_cho_preconnect_dung_generation_chu_khong_dung_co_abort():
    """`Enqueue()` xoa `abort_`, nen co do khong phai tin hieu huy dang tin cay."""
    source = _text(DEVICE_SOURCE)
    ensure = _function(
        source,
        "bool DeviceTtsClient::EnsureConnected",
        "void DeviceTtsClient::CloseSocket()",
    )
    anchor = ensure.index("kPreconnectWaitMs")
    wait_loop = ensure[max(0, anchor - 400) : anchor]

    assert "turn_generation_.load() == segment_generation" in wait_loop


def test_segment_trong_queue_mang_generation_de_bo_khi_turn_doi():
    """Abort giua dequeue va synth khong duoc lot mot cau cua turn cu ra loa."""
    header = _text(DEVICE_HEADER)
    source = _text(DEVICE_SOURCE)
    loop = _function(
        source,
        "void DeviceTtsClient::SourceDataLoop",
        "AudioTraceContext DeviceTtsClient::GetActiveTrace",
    )

    assert re.search(
        r"struct QueuedTtsSegment\s*\{[^}]*turn_generation", header, re.S
    ), "QueuedTtsSegment phai mang generation luc enqueue"
    assert "segment.turn_generation" in loop
    assert "SynthesizeOne(segment.body, segment.turn_generation)" in loop


def test_socket_warm_khong_gan_ondata_va_khong_gui_ssml_truoc_promote():
    """Bat bien lam cho viec giu socket qua turn la an toan.

    Neu socket warm co the nhan byte hoac da gui SSML thi no MANG danh tinh turn
    va luc do turn_generation se lai la dieu kien bat buoc.
    """
    source = _text(DEVICE_SOURCE)
    open_socket = _function(
        source,
        "std::unique_ptr<WebSocket> DeviceTtsClient::OpenConfiguredSocket",
        "bool DeviceTtsClient::PromoteReadySocket()",
    )
    synthesize = _function(
        source,
        "bool DeviceTtsClient::SynthesizeOne",
        "void DeviceTtsClient::HandleData",
    )

    # Chi bat loi goi that; chu "OnData" trong comment giai thich la hop le.
    assert "->OnData(" not in open_socket
    assert "Send(snapshot.config_frame)" in open_socket
    # SSML chi duoc gui sau khi socket da thanh socket active cua segment.
    assert "websocket_->Send(body)" in synthesize
    assert synthesize.index("EnsureConnected") < synthesize.index(
        "websocket_->Send(body)"
    )
