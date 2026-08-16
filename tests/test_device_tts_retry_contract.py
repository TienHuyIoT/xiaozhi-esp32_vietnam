"""Hop dong host T4: han cho byte dau, va thu lai cau chua ra tieng.

BANG CHUNG dan toi thiet ke nay (do 16/08, robot COM5 + doi chung tu PC, cung
mang, cung URL/header/SSML do backend render):

    connect_begin -> connect_ready   p50    27ms   (socket am, preconnect an)
    connect_ready -> first_byte      p50  3082ms   p90 4884  max 5377
    first_byte    -> first_pcm       p50   400ms   p90 3515
    Doi chung tu PC: first_byte      p50  3281ms   min 2319  max 6394

=> ~3s cho byte dau la cua EDGE/MANG, KHONG phai cua ngan xep TLS/WS tren ESP32.
   Vi vay "rut ngan watchdog" nhu ke hoach T4 mo ta la SAI HUONG: cat o 3000ms
   se giet oan 14/26 cau von se ra tieng. Nguong an toan do duoc la 6000ms
   (giet oan 0/26).

Cai THAT SU dau: 7/33 segment bat tay xong nhung KHONG nhan mot byte nao, moi
cai dot tron 7s roi cau bi VUT LUON. Doi lai la thu lai tren socket moi -- ne
duoc ca hai: be van nghe du cau, va khong lap tieng vi chi thu lai khi chua co
byte nao ra loa.
"""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DEVICE_TTS_HEADER = ROOT / "main" / "protocols" / "device_tts_client.h"
DEVICE_TTS_SOURCE = ROOT / "main" / "protocols" / "device_tts_client.cc"


def _text(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def _function(source: str, signature: str, next_signature: str) -> str:
    start = source.index(signature)
    end = source.index(next_signature, start)
    return source[start:end]


def _int_const(header: str, name: str) -> int:
    line = next(l for l in header.splitlines() if name in l and "=" in l)
    return int(line.split("=")[1].strip().rstrip(";").replace("*", " ").split()[0])


def test_han_cho_byte_dau_tach_khoi_han_cho_byte_tiep():
    """Hai giai doan, hai ban chat khac nhau -> khong duoc dung chung mot so."""
    header = _text(DEVICE_TTS_HEADER)

    assert "kFirstByteTimeoutMs" in header
    assert "kNoAudioTimeoutMs" in header

    first_byte = _int_const(header, "kFirstByteTimeoutMs")
    stall = _int_const(header, "kNoAudioTimeoutMs")

    # >= 6000: cau tot cham nhat do duoc la 5377ms. Ha xuong 3000 la giet oan
    # 14/26 cau. Ai muon sua so nay thi phai DO lai truoc.
    assert first_byte >= 6000, "ha duoi 6000ms se giet oan cau von se ra tieng"
    assert first_byte <= stall, "cho byte dau khong duoc lau hon cho byte tiep"


def test_vong_cho_chon_han_theo_da_co_byte_dau_chua():
    source = _text(DEVICE_TTS_SOURCE)
    body = _function(
        source,
        "bool DeviceTtsClient::SynthesizeOne",
        "void DeviceTtsClient::HandleData",
    )

    assert "first_provider_byte_seen_" in body, "khong biet dang o giai doan nao"
    assert "kFirstByteTimeoutMs" in body
    assert "kNoAudioTimeoutMs" in body


def test_thu_lai_dung_mot_lan_va_chi_khi_chua_co_tieng():
    """Thu lai khi da phat duoc vai chu = be nghe cau do hai lan."""
    source = _text(DEVICE_TTS_SOURCE)
    guard = _function(
        source,
        "bool DeviceTtsClient::ShouldRetrySegment",
        "bool DeviceTtsClient::SynthesizeOne",
    )

    assert "first_provider_byte_seen_" in guard, "phai chan lap tieng"
    assert "abort_" in guard
    assert "turn_generation_" in guard
    # nullptr = huy co y (abort / doi turn / tat may), khong phai hong -> khong thu lai.
    assert "nullptr" in guard

    loop = _function(
        source,
        "void DeviceTtsClient::SourceDataLoop",
        "AudioTraceContext DeviceTtsClient::GetActiveTrace",
    )
    assert loop.count("SynthesizeOne(") == 2, "dung mot lan thu lai, khong hon"


def test_thu_lai_nam_truoc_khi_danh_dau_het_byte():
    """`MarkSourceSegmentComplete()` dat eos cho decoder.

    Goi no giua hai lan thu = decoder flush nua frame roi lan hai lai nhet byte
    vao -> tieng rac. Thu lai phai xong HET truoc do.
    """
    source = _text(DEVICE_TTS_SOURCE)
    loop = _function(
        source,
        "void DeviceTtsClient::SourceDataLoop",
        "AudioTraceContext DeviceTtsClient::GetActiveTrace",
    )

    retry = loop.index("ShouldRetrySegment(")
    mark = loop.index("MarkSourceSegmentComplete()")
    assert retry < mark

    # Socket hong phai dong truoc khi thu lai, neu khong EnsureConnected thay
    # `IsConnected()` con true va gui SSML vao cai xac.
    close = loop.index("CloseSocket()", retry)
    second = loop.index("SynthesizeOne(", retry)
    assert close < second


def test_van_dem_duoc_so_lan_thu_lai_qua_log():
    """Khong co so nay thi lan do sau khong biet thu lai co an hay khong."""
    source = _text(DEVICE_TTS_SOURCE)
    loop = _function(
        source,
        "void DeviceTtsClient::SourceDataLoop",
        "AudioTraceContext DeviceTtsClient::GetActiveTrace",
    )

    assert "thu lai" in loop.lower()
    assert "segment=" in loop
