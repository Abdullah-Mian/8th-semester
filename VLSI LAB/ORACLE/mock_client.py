"""
Mock ESP client for the local voice assistant server.

Usage:
  python mock_esp.py --record 3
  python mock_esp.py --file input.wav

This sends 16 kHz int16 PCM frames over websocket using the server framing:
  [0xAA][0x55][LEN_H][LEN_L][PCM bytes]
EOS is sent as: [0xAA][0x55][0x00][0x00]

Then it receives text JSON back and audio frames from the server.
"""

import argparse
import asyncio
import json
import numpy as np
import sounddevice as sd
import soundfile as sf
import websockets

WS_URI = "ws://localhost:8765"
MIC_RATE = 16000
EOS_FRAME = bytes([0xAA, 0x55, 0x00, 0x00])


def make_frame(pcm_bytes: bytes) -> bytes:
    n = len(pcm_bytes)
    return bytes([0xAA, 0x55, (n >> 8) & 0xFF, n & 0xFF]) + pcm_bytes


def read_wav_file(path: str) -> bytes:
    audio, sr = sf.read(path, dtype="int16")
    if sr != MIC_RATE:
        raise ValueError(
            f"WAV file must be {MIC_RATE} Hz, but file is {sr} Hz."
        )
    if audio.ndim > 1:
        audio = audio.mean(axis=1).astype(np.int16)
    return audio.tobytes()


def record_audio(duration: float) -> bytes:
    print(f"[ESP] Recording {duration:.1f}s of audio at {MIC_RATE} Hz...")
    recording = sd.rec(int(duration * MIC_RATE), samplerate=MIC_RATE, channels=1, dtype="int16")
    sd.wait()
    print("[ESP] Recording complete")
    return recording.ravel().tobytes()


async def run_client(pcm_bytes: bytes) -> None:
    async with websockets.connect(WS_URI, max_size=None) as ws:
        print(f"[ESP] Connected to {WS_URI}")

        # --- THE FIX: Send in chunks of 8000 bytes ---
        CHUNK_SIZE = 8000 
        print(f"[ESP] Sending {len(pcm_bytes)} bytes in chunks...")
        
        for i in range(0, len(pcm_bytes), CHUNK_SIZE):
            chunk = pcm_bytes[i : i + CHUNK_SIZE]
            await ws.send(make_frame(chunk))
            
        # Send EOS (End of Stream)
        await ws.send(EOS_FRAME)
        # --------------------------------------------

        audio_buffer = bytearray()
        print("[ESP] Waiting for server response...")

        while True:
            message = await ws.recv()
            if isinstance(message, str):
                payload = json.loads(message)
                print("[ESP] Server heard:", repr(payload.get("heard", "")))
                print("[ESP] Server answer:", repr(payload.get("answer", "")))
            elif isinstance(message, (bytes, bytearray)):
                if len(message) >= 4 and message[2] == 0 and message[3] == 0:
                    print("[ESP] Received EOS from server")
                    break
                audio_buffer.extend(message[4:])

        if audio_buffer:
            samples = np.frombuffer(audio_buffer, dtype=np.int16).astype(np.float32) / 32768.0
            sd.play(samples, samplerate=MIC_RATE)
            sd.wait()


def main() -> None:
    parser = argparse.ArgumentParser(description="Mock ESP32 websocket client")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--record", type=float, help="Record microphone audio duration in seconds")
    group.add_argument("--file", type=str, help="Send an existing WAV file")
    args = parser.parse_args()

    if args.record is not None:
        pcm_bytes = record_audio(args.record)
    else:
        pcm_bytes = read_wav_file(args.file)

    asyncio.run(run_client(pcm_bytes))


if __name__ == "__main__":
    main()