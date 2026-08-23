"""Hop dong cua DUONG BANG CHUNG cho Gate M3 va phep do do tre tren robot that.

Day KHONG phai test bat bien firmware cua M3.2 (cho do la
`test_mic_lifecycle_contract.py`, chua ton tai). File nay khoa hai thu khac:

1. Collector serial co bat duoc dong bang chung mic lifecycle hay khong.
   Do 18/08: regex `SAFE_LINE` hien tai KHONG khop
   "AudioService: Enabling voice processing" -- ma do la dong DUY NHAT cho biet
   `ResetDecoder()` no luc nao (audio_service.cc:759 goi no ben trong
   `EnableVoiceProcessing(true)`; khong co AudioTrace event nao cho viec nay).
   Thu 100 luot ma thieu dong nay thi khong cham duoc gate, va phai cam robot lai.

2. Bo phan tich cham gate: do tre `turn_final -> first device PCM` (so tran M0),
   dem reset no giua luc PCM dang phat, dem doan bi cat duoi -- TACH THEO NGUON.

   Vi sao tach theo nguon: `ResetDecoder()` xoa `audio_decode_queue_` +
   `audio_playback_queue_` cua AudioService. Duong `device_tts` KHONG di qua hai
   queue do -- no ghi PCM thang vao codec (`OutputPcmDirect`,
   audio_stream_player.cc:958) va giu trace state rieng. Duong bi cat that la
   `server_opus`: `ClearServerOpusTracesLocked()` xoa slot trace dang cho nen
   `last_pcm` khong bao gio phat ra. Gop hai nguon lai se bao cao 0 loi tren mot
   he thong dang co loi.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
PLANS = ROOT.parent / ".claude" / "plans"
COLLECTOR_PATH = PLANS / "safe-audio-serial-collector.py"
ANALYZER_PATH = PLANS / "analyze-m3-gate.py"


def _load(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise AssertionError(f"khong nap duoc module tu {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def collector():
    return _load(COLLECTOR_PATH, "safe_audio_serial_collector")


@pytest.fixture(scope="module")
def analyzer():
    return _load(ANALYZER_PATH, "analyze_m3_gate")


# --------------------------------------------------------------------------
# 1. Collector phai bat duoc dong mic lifecycle
# --------------------------------------------------------------------------

MIC_LINES = [
    "I (37514) AudioService: Enabling voice processing",
    "I (33394) AudioService: Disabling voice processing",
]


@pytest.mark.parametrize("line", MIC_LINES)
def test_collector_bat_duoc_dong_mic_lifecycle(collector, line):
    assert collector.SAFE_LINE.search(line), (
        "Dong nay la bang chung duy nhat cho ResetDecoder do mic lifecycle; "
        "loc mat thi Gate M3 khong cham duoc."
    )


@pytest.mark.parametrize("line", MIC_LINES)
def test_sanitize_giu_nguyen_moc_thoi_gian_esp(collector, line):
    # Moc `(37514)` la ms tu luc boot, dung de doi chieu voi ts_us cua AudioTrace.
    assert collector.sanitize(line) == line


def test_collector_van_bat_duoc_audiotrace(collector):
    line = (
        "I (33390) AudioTrace: event=first_decoded_pcm ts_us=33390123 seq=7 "
        "turn_id=T1 segment_id=S1 audio_source=device_tts"
    )
    assert collector.SAFE_LINE.search(line)


def test_collector_khong_bat_dong_vo_can(collector):
    assert not collector.SAFE_LINE.search("I (100) wifi:state: run -> init")


# --------------------------------------------------------------------------
# 2. Bo phan tich cham gate
# --------------------------------------------------------------------------

def _trace(host: float, esp_ms: int, event: str, ts_us: int, seq: int,
           turn: str, seg: str, source: str) -> str:
    return (
        f"host_monotonic={host:.6f} I ({esp_ms}) AudioTrace: event={event} "
        f"ts_us={ts_us} seq={seq} turn_id={turn} segment_id={seg} "
        f"audio_source={source}"
    )


def _mic(host: float, esp_ms: int, enable: bool) -> str:
    verb = "Enabling" if enable else "Disabling"
    return (
        f"host_monotonic={host:.6f} I ({esp_ms}) "
        f"AudioService: {verb} voice processing"
    )


def test_enabling_giua_luc_pcm_dang_phat_bi_dem_la_cat_tieng(analyzer):
    log = "\n".join([
        _trace(100.0, 10_000, "first_decoded_pcm", 10_000_000, 1, "T1", "S1", "server_opus"),
        _mic(100.5, 10_500, True),
        _trace(101.0, 11_000, "last_pcm", 11_000_000, 2, "T1", "S1", "server_opus"),
    ])
    report = analyzer.build_report(serial_text=log, backend_lines=[])
    mic = report["mic_lifecycle"]
    assert mic["enable_events"] == 1
    assert mic["enable_while_pcm_active"] == 1
    assert mic["enable_while_pcm_active_by_source"]["server_opus"] == 1


def test_enabling_sau_khi_last_pcm_khong_bi_dem(analyzer):
    log = "\n".join([
        _trace(100.0, 10_000, "first_decoded_pcm", 10_000_000, 1, "T1", "S1", "server_opus"),
        _trace(101.0, 11_000, "last_pcm", 11_000_000, 2, "T1", "S1", "server_opus"),
        _mic(101.5, 11_500, True),
    ])
    report = analyzer.build_report(serial_text=log, backend_lines=[])
    assert report["mic_lifecycle"]["enable_events"] == 1
    assert report["mic_lifecycle"]["enable_while_pcm_active"] == 0


def test_doan_thieu_last_pcm_bi_dem_theo_tung_nguon(analyzer):
    log = "\n".join([
        # server_opus: bat dau roi mat hut -> cat duoi
        _trace(100.0, 10_000, "first_decoded_pcm", 10_000_000, 1, "T1", "S1", "server_opus"),
        _mic(100.5, 10_500, True),
        # device_tts: tron ven
        _trace(200.0, 20_000, "first_decoded_pcm", 20_000_000, 3, "T2", "S2", "device_tts"),
        _trace(201.0, 21_000, "last_pcm", 21_000_000, 4, "T2", "S2", "device_tts"),
    ])
    report = analyzer.build_report(serial_text=log, backend_lines=[])
    trunc = report["truncation"]
    assert trunc["segments_started"] == 2
    assert trunc["segments_without_last_pcm"] == 1
    assert trunc["by_source"]["server_opus"]["without_last_pcm"] == 1
    assert trunc["by_source"]["device_tts"]["without_last_pcm"] == 0


def test_run_sach_bao_cao_toan_so_khong(analyzer):
    log = "\n".join([
        _mic(99.0, 9_000, False),
        _trace(100.0, 10_000, "first_decoded_pcm", 10_000_000, 1, "T1", "S1", "device_tts"),
        _trace(101.0, 11_000, "last_pcm", 11_000_000, 2, "T1", "S1", "device_tts"),
        _mic(101.5, 11_500, True),
    ])
    report = analyzer.build_report(serial_text=log, backend_lines=[])
    assert report["mic_lifecycle"]["enable_while_pcm_active"] == 0
    assert report["truncation"]["segments_without_last_pcm"] == 0
    assert report["failures"]["panic"] == 0
    assert report["failures"]["reset_marker"] == 0


def test_do_tre_tinh_giong_M0_tren_luot_device_tts(analyzer):
    # turn_final o 1_000_000 ms; sentence_cmd o 1_000_500 ms; robot nhan json luc
    # ts_us=500_000, first_decoded_pcm luc ts_us=800_000 -> +300 ms
    # => do tre = (1_000_500 + 300) - 1_000_000 = 800 ms
    backend = [
        '[AUDIO_OBS] {"event":"turn_final","ts_unix_ms":1000000,"turn_id":"T1"}',
        '[AUDIO_OBS] {"event":"sentence_cmd","ts_unix_ms":1000500,"turn_id":"T1",'
        '"segment_id":"S1","audio_source":"device_tts"}',
    ]
    log = "\n".join([
        _trace(10.0, 100, "json_receive", 500_000, 1, "T1", "S1", "device_tts"),
        _trace(10.3, 400, "first_decoded_pcm", 800_000, 2, "T1", "S1", "device_tts"),
    ])
    report = analyzer.build_report(serial_text=log, backend_lines=backend)
    lat = report["latency_first_device_pcm"]
    assert lat["count"] == 1
    assert lat["p50_ms"] == pytest.approx(800.0, abs=0.5)
    assert lat["baseline_m0_p50_ms"] == 2610.8
    assert lat["delta_vs_m0_p50_ms"] == pytest.approx(800.0 - 2610.8, abs=0.5)


def test_luot_mo_man_bang_server_opus_bi_LOAI_khoi_chi_so(analyzer):
    """Giu nguyen dinh nghia cua M0, ke ca khi no lam mat mau.

    analyze-m0-retest.py chon doan PHAT SOM NHAT cua luot roi moi loc
    `audio_source == device_tts`. Luot nao mo man bang `server_opus` thi bi loai
    khoi chi so -- KHONG nhay sang doan device_tts di sau. Doi luat nay se lam
    con so khong con so duoc voi baseline M0 p50 2610,8 ms, tuc pha chinh cai
    ma phep do nay ton tai de lam.

    Hau qua thuc dung: neu robot mo dau moi luot bang audio kho (bai hoc), chi so
    nay se it mau -- phai bao cao `count`, khong duoc im lang khoe p50.
    """
    backend = [
        '[AUDIO_OBS] {"event":"turn_final","ts_unix_ms":1000000,"turn_id":"T1"}',
        '[AUDIO_OBS] {"event":"sentence_cmd","ts_unix_ms":1000100,"turn_id":"T1",'
        '"segment_id":"S0","audio_source":"server_opus"}',
        '[AUDIO_OBS] {"event":"sentence_cmd","ts_unix_ms":1000500,"turn_id":"T1",'
        '"segment_id":"S1","audio_source":"device_tts"}',
    ]
    log = "\n".join([
        _trace(9.0, 50, "json_receive", 200_000, 1, "T1", "S0", "server_opus"),
        _trace(9.1, 60, "first_decoded_pcm", 300_000, 2, "T1", "S0", "server_opus"),
        _trace(10.0, 100, "json_receive", 500_000, 3, "T1", "S1", "device_tts"),
        _trace(10.3, 400, "first_decoded_pcm", 800_000, 4, "T1", "S1", "device_tts"),
    ])
    report = analyzer.build_report(serial_text=log, backend_lines=backend)
    assert report["latency_first_device_pcm"]["count"] == 0


def test_dem_luot_theo_turn_final(analyzer):
    backend = [
        f'[AUDIO_OBS] {{"event":"turn_final","ts_unix_ms":{1000 + i},"turn_id":"T{i}"}}'
        for i in range(7)
    ]
    report = analyzer.build_report(serial_text="", backend_lines=backend)
    assert report["turns"]["turn_final_count"] == 7


def test_reboot_giua_run_khong_gay_interval_am(analyzer):
    # Sau `rst:` dong ho thiet bi ve 0; dung host_monotonic thi interval van dung.
    log = "\n".join([
        _trace(100.0, 30_000, "first_decoded_pcm", 30_000_000, 9, "T1", "S1", "server_opus"),
        _trace(101.0, 31_000, "last_pcm", 31_000_000, 10, "T1", "S1", "server_opus"),
        "host_monotonic=102.000000 rst:0xc (RTC_SW_CPU_RST),boot:0x8",
        _trace(110.0, 500, "first_decoded_pcm", 500_000, 1, "T2", "S2", "server_opus"),
        _mic(110.5, 700, True),
        _trace(111.0, 1_000, "last_pcm", 1_000_000, 2, "T2", "S2", "server_opus"),
    ])
    report = analyzer.build_report(serial_text=log, backend_lines=[])
    assert report["failures"]["reset_marker"] == 1
    # Chi lan thu hai co Enabling chen giua -> dung 1, khong phai 0 va khong phai 2
    assert report["mic_lifecycle"]["enable_while_pcm_active"] == 1
    for interval in report["intervals"]:
        if interval["end_host"] is not None:
            assert interval["end_host"] >= interval["start_host"]


# --------------------------------------------------------------------------
# 3. Collector phai SONG SOT khi robot re-enumerate USB
# --------------------------------------------------------------------------
# Do that 23/08: robot reboot -> USB-Serial/JTAG re-enumerate -> pyserial nem
# `SerialException: ClearCommError failed (PermissionError(13))` -> collector
# CHET HAN. Hau qua: mat toan bo bang chung firmware cho phan con lai cua run.
# Trong phien 100 luot thi mot lan reboot la hong ca buoi. Reboot la chuyen
# BINH THUONG trong phep do nay (watchdog, abort, cam lai day) nen collector
# phai noi lai duoc, va phai de lai dau cho biet co khoang trong.


class _FakePort:
    """Cong gia: tra vai dong roi nem SerialException nhu USB rot that."""

    def __init__(self, lines, fail_after=None):
        self._lines = [l.encode() + b"\n" for l in lines]
        self._fail_after = fail_after
        self._reads = 0
        self.closed = False

    def readline(self):
        if self._fail_after is not None and self._reads >= self._fail_after:
            import serial
            raise serial.SerialException("ClearCommError failed")
        if not self._lines:
            import serial
            raise serial.SerialException("cong bien mat")
        self._reads += 1
        return self._lines.pop(0)

    def close(self):
        self.closed = True

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False


def test_collector_noi_lai_duoc_khi_usb_rot(collector, tmp_path):
    out = tmp_path / "serial.log"
    stop = tmp_path / "stop.flag"
    truoc = "I (100) Application: STATE: speaking"
    sau = "I (200) Application: STATE: listening"
    ports = [
        _FakePort([truoc], fail_after=1),   # tra 1 dong roi rot USB
        _FakePort([sau]),                    # cong moi sau khi robot boot lai
    ]

    def factory():
        if not ports:
            stop.touch()          # het cong gia -> ket thuc vong lap
            return _FakePort([])
        return ports.pop(0)

    collector.run_collector(
        port_factory=factory, output_path=out, stop_file=stop, sleep=lambda _s: None
    )
    text = out.read_text(encoding="utf-8")
    assert "STATE: speaking" in text, "mat dong TRUOC khi rot USB"
    assert "STATE: listening" in text, "collector khong noi lai duoc sau khi USB rot"
    assert "collector_reconnect" in text, (
        "phai de lai dau khoang trong, neu khong nguoi doc log tuong la robot im"
    )


def test_collector_dung_khi_co_stop_file(collector, tmp_path):
    out = tmp_path / "serial.log"
    stop = tmp_path / "stop.flag"
    stop.touch()
    collector.run_collector(
        port_factory=lambda: _FakePort(["x"]), output_path=out, stop_file=stop,
        sleep=lambda _s: None,
    )
    assert "collector_stop_utc" in out.read_text(encoding="utf-8")


def test_mo_cong_that_bai_van_thu_lai_chu_khong_chet(collector, tmp_path):
    """Robot boot mat vai giay -- mo cong that bai la BINH THUONG, khong duoc thoat."""
    out = tmp_path / "serial.log"
    stop = tmp_path / "stop.flag"
    lan = {"n": 0}

    def factory():
        import serial
        lan["n"] += 1
        if lan["n"] < 3:
            raise serial.SerialException("could not open port")
        stop.touch()
        return _FakePort(["I (300) Application: STATE: idle"])

    collector.run_collector(
        port_factory=factory, output_path=out, stop_file=stop, sleep=lambda _s: None
    )
    assert lan["n"] >= 3, "phai thu lai khi mo cong that bai"
