"""Hop dong: khong duoc co duong abort CAM.

Do 17/08 tren COM5 (log `m1_retest_1708b.log`): 3 dong `Abort speaking` nhung chi
2 dong `Stopped speaking by user`, va 0 dong `Wake word detected`. Lan abort thu
ba khong the truy ra nguon, vi hai trong bon cho goi `AbortSpeaking` khong ghi gi:

    application.cc:415   StartListening()   -> AbortSpeaking(kAbortReasonNone);
    application.cc:1592  WakeWordInvoke()   -> AbortSpeaking(kAbortReasonNone);

Hau qua khi doc log: **"co abort" bi hieu nham thanh "wake word chay"**. Chinh
cho nay tung suyt lam ket luan sai ve "Hi Lily". Ghi chu them: nhanh Idle cua
`WakeWordInvoke()` CO log "Wake word detected" con nhanh Speaking thi khong --
nen dem so dong do de suy ra wake word co no hay khong la khong dung.

Cach chan: nguon la **tham so bat buoc** cua `AbortSpeaking`, khong co gia tri
mac dinh. Them duong goi moi ma quen nguon thi khong bien dich duoc -- manh hon
quy uoc "nho ghi log", vi quy uoc thi lan sau lai quen.

Day la host contract test vi firmware chua co C++ unit runner native; build
ESP-IDF va phep do robot that van la cong E2E bat buoc.
"""

from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[1]
APP_SOURCE = ROOT / "main" / "application.cc"
APP_HEADER = ROOT / "main" / "application.h"
COLLECTOR = ROOT.parent / ".claude" / "plans" / "safe-audio-serial-collector.py"


def _text(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def _call_sites(source: str) -> list[str]:
    """Cac cho GOI AbortSpeaking (bo dinh nghia ham va cac ten khac)."""
    return [
        m.group(0)
        for m in re.finditer(r"(?<!::)\bAbortSpeaking\([^;]*\);", source)
        if "Application::AbortSpeaking" not in m.group(0)
    ]


def test_nguon_abort_la_tham_so_bat_buoc_khong_co_mac_dinh():
    """Co mac dinh thi lan sau van them duoc duong cam ma trinh dich im lang."""
    header = _text(APP_HEADER)

    match = re.search(r"void AbortSpeaking\(([^)]*)\);", header)
    assert match, "khong tim thay khai bao AbortSpeaking"
    params = match.group(1)

    assert "const char*" in params or "const char *" in params, (
        "AbortSpeaking phai nhan nguon abort"
    )
    assert "=" not in params, "nguon abort khong duoc co gia tri mac dinh"


def test_moi_cho_goi_deu_khai_bao_nguon():
    """Bon cho goi: hai nut + hai duong wake word. Khong cho nao duoc de trong."""
    sites = _call_sites(_text(APP_SOURCE))

    assert len(sites) >= 4, f"chi thay {len(sites)} cho goi, mong doi >= 4"
    nguon = []
    for site in sites:
        found = re.search(r',\s*"([a-z0-9_]+)"\s*\)', site)
        assert found, f"cho goi thieu nguon dang chuoi: {site}"
        nguon.append(found.group(1))

    assert len(set(nguon)) == len(nguon), (
        f"nguon abort bi trung nen khong phan biet duoc: {nguon}"
    )


def test_log_in_ra_nguon_bang_khoa_greppable():
    """Log phai mang khoa co dinh de loc bang grep, khong phai cau van xuoi."""
    source = _text(APP_SOURCE)
    start = source.index("void Application::AbortSpeaking(")
    body = source[start : start + 400]

    assert "abort_src=%s" in body, "log phai in nguon voi khoa abort_src="
    assert "Abort speaking" in body, (
        "giu nguyen chuoi cu -- bo thu serial dang loc theo no"
    )


def test_bo_thu_serial_van_cho_dong_nay_di_qua():
    """Log dung ma bi bo loc thi van khong doc duoc gi o lan do sau."""
    collector = _text(COLLECTOR)

    assert "Abort speaking" in collector, (
        "SAFE_LINE phai cho dong abort di qua"
    )
