"""Hop dong hoi quy cho luong TTS thiet bi.

Firmware ESP-IDF khong co test runner host san. Cac test nay khoa ba bat bien nguon
quan trong; build ESP-IDF va phep do robot that van la cong E2E bat buoc.
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


def test_preconnect_bat_dau_truoc_khi_cho_turn_end():
    """Bat tay cau N+1 phai overlap voi synth/phat cau N."""
    header = _text(DEVICE_HEADER)
    source = _text(DEVICE_SOURCE)
    synth = _function(
        source,
        "bool DeviceTtsClient::SynthesizeOne",
        "void DeviceTtsClient::HandleData",
    )

    assert "RequestPreconnect" in header
    assert "PreconnectTaskRoutine" in header
    assert "RequestPreconnect();" in synth
    assert synth.index("RequestPreconnect();") < synth.index(
        "while (running_.load() && !abort_.load())"
    )


def test_drain_thuc_day_ngay_khi_cau_tiep_theo_vao_queue():
    """Khong cho buffer can ve 0 neu LLM vua day them cau moi."""
    header = _text(DEVICE_HEADER)
    source = _text(DEVICE_SOURCE)
    loop = _function(
        source,
        "void DeviceTtsClient::SourceDataLoop",
        "bool DeviceTtsClient::EnsureConnected",
    )

    assert "HasQueuedSentence" in header
    assert "!HasQueuedSentence()" in loop
    assert re.search(
        r"GetBufferSize\(\)\s*>\s*0[\s\S]{0,160}!HasQueuedSentence\(\)",
        loop,
    )
    assert re.search(
        r"if\s*\(!HasQueuedSentence\(\)[\s\S]{0,200}on_idle_",
        loop,
    )
