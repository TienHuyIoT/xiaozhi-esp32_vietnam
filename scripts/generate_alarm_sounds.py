#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
import io
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding='utf-8')

"""
Generate alarm sound OGG files for ESP32.
Uses Edge TTS for high-quality Vietnamese text-to-speech.
Requirements: pip install edge-tts
Output: 16000Hz, mono, OGG Opus (bitrate 16k), < 50KB

The ESP32 PlaySound() function parses OGG pages and only decodes Opus packets
(OpusHead/OpusTags format). Vorbis-encoded OGG files will NOT play.
"""
import asyncio
import edge_tts
import subprocess
import os

ASSETS_DIR = os.path.join(os.path.dirname(__file__), "..", "main", "assets", "common")

ALARM_SOUNDS = [
    {"id": "school",  "text": "Đã tới giờ đi học rồi đấy",       "desc": "Báo thức đi học"},
    {"id": "wakeup",  "text": "Dậy thôi, sáng rồi!",             "desc": "Báo thức thức dậy"},
    {"id": "medicine","text": "Đã tới giờ uống thuốc",           "desc": "Nhắc uống thuốc"},
]


async def generate_mp3(sound: dict) -> bool:
    """Generate MP3 audio from text using Edge TTS with retry."""
    mp3_path = os.path.join(ASSETS_DIR, f"{sound['id']}_temp.mp3")

    voices = ["vi-VN-NamMinhNeural", "vi-VN-HoaiMyNeural"]
    max_retries = 3
    delay = 5

    for attempt in range(max_retries):
        for voice in voices:
            try:
                communicate = edge_tts.Communicate(sound["text"], voice)
                await communicate.save(mp3_path)
                return True
            except Exception as e:
                err_str = str(e)
                if "No audio was received" in err_str or "429" in err_str:
                    print(f"    Rate-limited, retrying in {delay}s...")
                    await asyncio.sleep(delay)
                else:
                    print(f"    TTS error ({voice}): {err_str}")
        if attempt < max_retries - 1:
            print(f"    Attempt {attempt+1} failed, waiting {delay}s before retry...")
            await asyncio.sleep(delay)
            delay *= 2
    return False


def convert_to_ogg(mp3_path: str, ogg_path: str) -> bool:
    """Convert MP3 to OGG (Opus): 16000Hz, mono, --bitrate 16k."""
    if not os.path.exists(mp3_path):
        return False
    if os.path.exists(ogg_path):
        os.remove(ogg_path)
    try:
        result = subprocess.run(
            [
                "ffmpeg", "-y", "-i", mp3_path,
                "-ar", "16000", "-ac", "1",
                "-c:a", "libopus", "-b:a", "16k",
                "-application", "voip",
                ogg_path
            ],
            capture_output=True, text=True
        )
        if result.returncode != 0:
            print(f"    FFmpeg error: {result.stderr[-300:]}")
            return False
        size_kb = os.path.getsize(ogg_path) / 1024
        print(f"    OGG: {ogg_path} ({size_kb:.1f} KB)")
        return True
    except FileNotFoundError:
        print("    ERROR: ffmpeg not found!")
        return False


def main():
    print("=" * 60)
    print("Edge TTS -> OGG Generator for ESP32 Alarm Sounds")
    print("=" * 60)

    os.makedirs(ASSETS_DIR, exist_ok=True)
    print(f"Output: {ASSETS_DIR}\n")

    # Check ffmpeg
    try:
        subprocess.run(["ffmpeg", "-version"], capture_output=True, check=True)
        print("[OK] FFmpeg found\n")
    except (subprocess.CalledProcessError, FileNotFoundError):
        print("[ERROR] FFmpeg not found! Install: winget install ffmpeg")
        sys.exit(1)

    print("Available Vietnamese voices:")
    voices = asyncio.run(edge_tts.list_voices())
    for v in voices:
        if v["Locale"].startswith("vi-"):
            print(f"  {v['Name']} ({v['Gender']})")
    print()

    results = []
    for i, sound in enumerate(ALARM_SOUNDS):
        print(f"[{i+1}/{len(ALARM_SOUNDS)}] {sound['desc']}")
        print(f"    Text: \"{sound['text']}\"")

        # Generate MP3
        ok = asyncio.run(generate_mp3(sound))
        if not ok:
            results.append((sound["id"], False))
            print("    FAILED to generate audio\n")
            continue

        # Convert to OGG
        mp3_path = os.path.join(ASSETS_DIR, f"{sound['id']}_temp.mp3")
        ogg_path = os.path.join(ASSETS_DIR, f"{sound['id']}.ogg")
        ok = convert_to_ogg(mp3_path, ogg_path)

        if os.path.exists(mp3_path):
            os.remove(mp3_path)

        results.append((sound["id"], ok))
        print()

    # Summary
    print("=" * 60)
    print("SUMMARY")
    print("=" * 60)
    for sound_id, ok in results:
        ogg_path = os.path.join(ASSETS_DIR, f"{sound_id}.ogg")
        if ok and os.path.exists(ogg_path):
            size = os.path.getsize(ogg_path) / 1024
            status = "OK" if size <= 50 else f"OK but {size:.1f}KB > 50KB"
            print(f"  {sound_id}.ogg: {status}")
        else:
            print(f"  {sound_id}.ogg: FAILED")

    if all(ok for _, ok in results):
        print("\nAll alarm sounds generated!")
    else:
        print("\nSome files failed. Run script again to retry.")


if __name__ == "__main__":
    main()
