#!/usr/bin/env python3
"""Record a headless Jetson CSI camera stream to an MP4 file."""
from __future__ import annotations

import argparse
import time
from pathlib import Path


def record(
    output: Path,
    *,
    sensor_id: int = 0,
    capture_size: tuple[int, int] = (1280, 720),
    output_size: tuple[int, int] = (640, 360),
    fps: int = 30,
) -> tuple[int, float]:
    import cv2

    from mav.io.camera import csi_pipeline

    output.parent.mkdir(parents=True, exist_ok=True)
    pipeline = csi_pipeline(
        sensor_id=sensor_id,
        capture_size=capture_size,
        output_size=output_size,
        fps=fps,
    )
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        raise RuntimeError(f"cannot open CSI camera sensor {sensor_id}")

    width, height = output_size
    writer = cv2.VideoWriter(
        str(output),
        cv2.VideoWriter_fourcc(*"mp4v"),
        float(fps),
        (width, height),
    )
    if not writer.isOpened():
        cap.release()
        raise RuntimeError(f"cannot open output video: {output}")

    frames = 0
    started = time.monotonic()
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            if frame.shape[1] != width or frame.shape[0] != height:
                frame = cv2.resize(frame, (width, height))
            writer.write(frame)
            frames += 1
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        writer.release()

    elapsed = time.monotonic() - started
    actual_fps = frames / elapsed if elapsed > 0 else 0.0
    return frames, actual_fps


def _size(value: str) -> tuple[int, int]:
    try:
        width, height = (int(part) for part in value.lower().split("x", 1))
    except ValueError as exc:
        raise argparse.ArgumentTypeError("size must be WIDTHxHEIGHT") from exc
    if width <= 0 or height <= 0:
        raise argparse.ArgumentTypeError("size dimensions must be positive")
    return width, height


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path, help="output MP4 path")
    parser.add_argument("--sensor-id", type=int, default=0)
    parser.add_argument("--capture-size", type=_size, default=(1280, 720))
    parser.add_argument("--output-size", type=_size, default=(640, 360))
    parser.add_argument("--fps", type=int, default=30)
    args = parser.parse_args()
    if args.fps <= 0:
        parser.error("--fps must be positive")

    frames, actual_fps = record(
        args.output,
        sensor_id=args.sensor_id,
        capture_size=args.capture_size,
        output_size=args.output_size,
        fps=args.fps,
    )
    print(f"frames: {frames}")
    print(f"actual_fps: {actual_fps:.2f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
