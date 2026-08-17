"""Hop dong: EOS cua MOT CAU khong duoc giet ca duong phat.

Do tren robot COM5 ngay 17/08 (log `m1_retest_1708.log`):

    88661.225  AudioStreamPlayer: EOS reached
    88661.232  AudioStreamPlayer: PlayLoop finished           <- play task chet
    88668.147  Application: STATE: speaking                   <- luot moi
    88669.616  AudioStreamPlayer: StopStream: src=1, play=0   <- source VAN SONG
    88673.909  DeviceTTS: vong nguon ket thuc                 <- 4,3s moi go xong

`eos` o dau vong decode duoc tinh la:

    eos = buffer rong && (!is_source_active_ || source_segment_complete_)

`source_segment_complete_` chi noi **het mot cau** -- `DeviceTtsClient` dat no
sau moi segment. Nhung `break` o cuoi vong lai hieu no la **het phien**, nen mot
phien Device TTS nhieu cau bi cat ngay sau cau dau. Hau qua do duoc: luot sau
`EnsureStarted()` thay `IsPlaying()` false -> `StartStream -> StopStream`, va
`StopStream` phai cho source task (dang ket cho Edge) thoat: **2,2-4,3s cong
thang vao am dau**. Hai luot dinh do duoc 9743ms va 15919ms, trong khi p50 cua
cac luot khac la 2161ms.

Radio/nhac KHONG bi anh huong: chung khong bao gio goi
`MarkSourceSegmentComplete()`, nen `eos` cua chung luon di kem
`!is_source_active_` -> van thoat y nhu cu. Day la ly do sua duoc o lop cha ma
khong dung toi duong nhac.

Pham vi: file nay khoa cho **thoat vong**. Cach TINH `eos` da duoc khoa san o
`test_audio_latency_vad_contract.py::test_source_bao_het_byte_segment_thi_decode_cuoi_duoc_dat_eos`
-- dung sua sang do, no la hang rao chong cut tail cua cau cuoi.

Day la host contract test vi firmware chua co C++ unit runner native; build
ESP-IDF va phep do robot that van la cong E2E bat buoc.
"""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PLAYER_SOURCE = ROOT / "main" / "features" / "music" / "audio_stream_player.cc"


def _text(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def _function(source: str, signature: str, next_signature: str) -> str:
    start = source.index(signature)
    end = source.index(next_signature, start)
    return source[start:end]


def _compressed_loop(source: str) -> str:
    return _function(
        source,
        "void AudioStreamPlayer::PlayLoopCompressed()",
        "void AudioStreamPlayer::PlayLoopWav()",
    )


def _eos_block(source: str) -> str:
    """Khoi xu ly khi decoder da nhai het byte cua doan hien tai."""
    loop = _compressed_loop(source)
    start = loop.index("if (eos && input_bytes_left_ == 0)")
    end = loop.index("Compressed loop done", start)
    return loop[start:end]


def test_eos_cua_mot_cau_khong_thoat_vong_phat():
    """Phien Device TTS con song thi phai o lai cho cau ke tiep."""
    block = _eos_block(_text(PLAYER_SOURCE))

    assert "is_source_active_" in block, (
        "EOS phai phan biet het-mot-cau voi het-phien bang is_source_active_"
    )
    ket_thuc_phien = block.index('"EOS reached"')
    o_lai = block.index("continue;")
    assert o_lai < ket_thuc_phien, (
        "duong o-lai phai duoc xet TRUOC duong thoat, neu khong cau dau van cat phien"
    )


def test_van_con_duong_thoat_that_khi_het_phien():
    """`StopStream`/`Shutdown` tat `is_source_active_` -- vong phai thoat that.

    Mat duong nay thi play task song mai, `StopStream` het 5s roi bao timeout.
    """
    block = _eos_block(_text(PLAYER_SOURCE))

    assert '"EOS reached"' in block
    assert "break;" in block


def test_nap_lai_decoder_truoc_khi_nhan_cau_moi():
    """`raw.eos = true` da flush decoder -- nhet byte cau moi vao la ra tieng rac.

    Giong het nhanh bo tail loi sau DATA_LACK o tren: CleanupDecoder + InitDecoder.
    """
    block = _eos_block(_text(PLAYER_SOURCE))

    assert "CleanupDecoder();" in block
    assert "InitDecoder(decoder_type_)" in block
    assert block.index("CleanupDecoder();") < block.index("continue;"), (
        "phai nap lai decoder truoc khi quay lai vong"
    )


def test_nap_lai_decoder_that_bai_thi_dung_han_chu_khong_quay_mu():
    """Khong nap lai duoc decoder ma van `continue` la quay nong vo tan."""
    block = _eos_block(_text(PLAYER_SOURCE))

    assert "is_playing_ = false;" in block, (
        "nhanh nap lai decoder that bai phai ha co va thoat"
    )


def test_bang_chung_do_duoc_ghi_ngay_canh_cho_sua():
    """Nguoi sau se thay nhanh o-lai va tuong la thua."""
    block = _eos_block(_text(PLAYER_SOURCE))

    assert "17/08" in block, "phai ghi ngay do"
    assert "4,3s" in block or "4300" in block, "phai ghi gia phai tra do duoc"
