#!/usr/bin/env python3
"""Generate synthetic test sequences for optical flow evaluation.

Each sequence simulates a known motion pattern. Outputs PGM frames
plus a ground-truth CSV file.
"""

import os
import struct
import numpy as np
from typing import List, Tuple

W = 320
H = 240


def write_pgm(path: str, data: np.ndarray):
    assert data.dtype == np.uint8
    with open(path, 'wb') as f:
        f.write(f"P5\n{data.shape[1]} {data.shape[0]}\n255\n".encode())
        f.write(data.tobytes())


def checkerboard(w: int, h: int, phase_x: int = 0, phase_y: int = 0, size: int = 16) -> np.ndarray:
    x = np.arange(w) + phase_x
    y = np.arange(h) + phase_y
    grid = (x // size + y[:, None] // size) & 1
    return np.where(grid, 200, 50).astype(np.uint8)


def random_texture(w: int, h: int, dx: int = 0, dy: int = 0, seed: int = 42) -> np.ndarray:
    rng = np.random.RandomState(seed)
    pattern = rng.randint(0, 256, (h, w), dtype=np.uint8)
    return np.roll(pattern, (-dy, -dx), axis=(0, 1))


def gradient_x(w: int, h: int, phase: int = 0) -> np.ndarray:
    x = np.arange(w) + phase
    return (x % 256).astype(np.uint8).reshape(1, -1).repeat(h, axis=0)


def rotation_pattern(w: int, h: int, angle_deg: float = 0) -> np.ndarray:
    """Radial checkerboard (looks like a dartboard) for rotation testing."""
    cy, cx = h // 2, w // 2
    y, x = np.ogrid[:h, :w]
    r = np.sqrt((x - cx) ** 2 + (y - cy) ** 2)
    theta = (np.arctan2(y - cy, x - cx) * 180 / np.pi + angle_deg) % 360
    pattern = ((r // 16).astype(int) + (theta // 30).astype(int)) & 1
    return np.where(pattern, 220, 40).astype(np.uint8)


def generate_sequence(name: str, num_frames: int,
                      gen_func, gen_kwargs: dict,
                      flow_gt: List[Tuple[float, float]]):
    """Generate a test sequence.

    Args:
        name: Directory name under test_sequences/
        num_frames: Number of frames to generate
        gen_func: Function(w, h, frame, **kwargs) -> np.ndarray
        gen_kwargs: Extra kwargs for gen_func
        flow_gt: List of (vx, vy) for each frame (length num_frames)
    """
    out_dir = os.path.join(os.path.dirname(__file__), name)
    os.makedirs(out_dir, exist_ok=True)

    gt_path = os.path.join(out_dir, "ground_truth.csv")
    with open(gt_path, 'w') as f:
        f.write("frame,vx,vy\n")

        for i in range(num_frames):
            frame = gen_func(W, H, i, **gen_kwargs)
            write_pgm(os.path.join(out_dir, f"frame_{i:04d}.pgm"), frame)

            vx, vy = flow_gt[i] if i < len(flow_gt) else (0, 0)
            f.write(f"{i},{vx:.2f},{vy:.2f}\n")

    print(f"Generated {num_frames} frames in {out_dir}/")


def main():
    num = 100

    # 1. Hover - no motion
    generate_sequence("hover", num,
                      lambda w, h, i: random_texture(w, h, 0, 0),
                      {},
                      [(0, 0)] * num)

    # 2. Forward - constant X flow
    generate_sequence("forward", num,
                      lambda w, h, i: random_texture(w, h, i * 2, i * 1),
                      {},
                      [(2.0, 1.0)] * num)

    # 3. Backward - negative X flow
    generate_sequence("backward", num,
                      lambda w, h, i: random_texture(w, h, -i * 2, -i * 1),
                      {},
                      [(-2.0, -1.0)] * num)

    # 4. Rotation - pure yaw
    generate_sequence("rotation", num,
                      lambda w, h, i: rotation_pattern(w, h, i * 1.5),
                      {},
                      [(0, 0)] * num)  # rotation is not translational

    # 5. Landing - divergence toward center
    generate_sequence("landing", num,
                      lambda w, h, i: random_texture(w, h, 0, 0),
                      {},
                      [(0, 0)] * num)  # will add divergence logic

    # 6. Synthetic checkerboard (structured texture)
    generate_sequence("checkerboard", num,
                      lambda w, h, i: checkerboard(w, h, i * 2, i * 1),
                      {},
                      [(2.0, 1.0)] * num)

    # 7. Gradient - worst case (no texture)
    generate_sequence("gradient", num,
                      lambda w, h, i: gradient_x(w, h, i * 2),
                      {},
                      [(2.0, 0.0)] * num)

    print("\nAll sequences generated.")


if __name__ == "__main__":
    main()
