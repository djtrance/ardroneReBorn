#!/usr/bin/env python3
"""Visualize optical flow results from the simulator.

Usage:
  python3 visualizer.py results/results.csv --plot
  python3 visualizer.py results/results.csv --animate test_sequences/forward/
"""

import sys
import csv
import os
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np


def read_results(csv_path: str):
    frames, vx, vy, qual, sad, gx, gy, ex, ey = [], [], [], [], [], [], [], [], []
    with open(csv_path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            frames.append(int(row['frame']))
            vx.append(float(row['flow_x_est']))
            vy.append(float(row['flow_y_est']))
            qual.append(int(row['quality']))
            sad.append(int(row['sad_score']))
            gx.append(float(row['flow_x_gt']))
            gy.append(float(row['flow_y_gt']))
            ex.append(float(row['error_x']))
            ey.append(float(row['error_y']))
    return frames, vx, vy, qual, sad, gx, gy, ex, ey


def plot_results(csv_path: str):
    frames, vx, vy, qual, sad, gx, gy, ex, ey = read_results(csv_path)

    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)

    # Flow vectors
    ax = axes[0]
    ax.plot(frames, vx, 'b-', label='Estimated Vx', linewidth=1)
    ax.plot(frames, gx, 'b--', label='Ground Truth Vx', alpha=0.5)
    ax.plot(frames, vy, 'r-', label='Estimated Vy', linewidth=1)
    ax.plot(frames, gy, 'r--', label='Ground Truth Vy', alpha=0.5)
    ax.set_ylabel('Flow (px/frame)')
    ax.legend(loc='upper right', ncol=2)
    ax.grid(True, alpha=0.3)
    ax.set_title('Optical Flow Estimation')

    # Errors
    ax = axes[1]
    ax.plot(frames, ex, 'b-', label='Error X', linewidth=1)
    ax.plot(frames, ey, 'r-', label='Error Y', linewidth=1)
    ax.axhline(0, color='gray', linestyle='--')
    ax.set_ylabel('Error (px/frame)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_title('Estimation Error')

    # Quality
    ax = axes[2]
    ax.plot(frames, qual, 'g-', linewidth=1)
    ax.set_ylabel('Quality (0-255)')
    ax.set_ylim(0, 260)
    ax.grid(True, alpha=0.3)
    ax.set_title('Flow Quality Metric')

    # SAD
    ax = axes[3]
    ax.plot(frames, sad, 'm-', linewidth=1)
    ax.set_ylabel('Mean SAD')
    ax.set_xlabel('Frame')
    ax.grid(True, alpha=0.3)
    ax.set_title('Block Matching Error (SAD)')

    plt.tight_layout()
    plt.savefig(csv_path.replace('.csv', '_plot.png'), dpi=150)
    print(f"Plot saved to {csv_path.replace('.csv', '_plot.png')}")
    plt.show()


def load_pgm(path: str) -> np.ndarray:
    with open(path, 'rb') as f:
        header = f.readline().strip()
        if header != b'P5':
            raise ValueError(f"Not a PGM: {path}")
        width, height = [int(x) for x in f.readline().split()]
        max_val = int(f.readline().strip())
        data = np.frombuffer(f.read(), dtype=np.uint8).reshape(height, width)
    return data


def animate_sequence(csv_path: str, pgm_dir: str):
    """Animate flow vectors overlaid on actual frames."""
    frames, vx, vy, qual, sad, gx, gy, ex, ey = read_results(csv_path)

    # Load first frame to get dimensions
    first_pgm = os.path.join(pgm_dir, "frame_0000.pgm")
    if not os.path.exists(first_pgm):
        print(f"No PGM frames found in {pgm_dir}")
        return

    img_shape = load_pgm(first_pgm).shape
    fig, ax = plt.subplots(figsize=(10, 8))

    def update(frame_idx):
        ax.clear()

        # Load PGM frame
        pgm_path = os.path.join(pgm_dir, f"frame_{frame_idx:04d}.pgm")
        if not os.path.exists(pgm_path):
            return

        img = load_pgm(pgm_path)
        ax.imshow(img, cmap='gray', vmin=0, vmax=255)

        h, w = img.shape
        scale = 20  # pixels per unit flow (for visualization)

        # Draw estimated flow
        fx = vx[frame_idx]
        fy = vy[frame_idx]
        ax.arrow(w // 2, h // 2, fx * scale, fy * scale,
                 head_width=8, head_length=6, fc='red', ec='red', alpha=0.8,
                 label=f'Estimated ({fx:.1f}, {fy:.1f})')

        # Draw ground truth
        gx_v = gx[frame_idx]
        gy_v = gy[frame_idx]
        ax.arrow(w // 2, h // 2, gx_v * scale, gy_v * scale,
                 head_width=8, head_length=6, fc='green', ec='green', alpha=0.5,
                 label=f'GT ({gx_v:.1f}, {gy_v:.1f})')

        ax.set_title(f'Frame {frame_idx} | Quality={qual[frame_idx]} | '
                     f'SAD={sad[frame_idx]}')
        ax.legend(loc='lower right')
        ax.axis('off')

    anim = animation.FuncAnimation(fig, update, frames=len(frames),
                                   interval=100, repeat=True)

    out_path = csv_path.replace('.csv', '_animation.gif')
    anim.save(out_path, writer='pillow', fps=10)
    print(f"Animation saved to {out_path}")
    plt.show()


def analyze_quality(csv_path: str):
    """Analyze relationship between quality metric and error."""
    frames, vx, vy, qual, sad, gx, gy, ex, ey = read_results(csv_path)

    errors = np.sqrt(np.array(ex) ** 2 + np.array(ey) ** 2)
    qualities = np.array(qual)

    # Bin by quality
    print("\n=== Quality vs Error Analysis ===")
    for q_min in range(0, 256, 32):
        q_max = min(q_min + 32, 255)
        mask = (qualities >= q_min) & (qualities < q_max)
        if np.any(mask):
            mean_err = np.mean(errors[mask])
            count = np.sum(mask)
            print(f"  Quality {q_min:3d}-{q_max:3d}: {count:4d} frames, "
                  f"mean error = {mean_err:.3f} px/frame")

    # Summary stats
    print(f"\n  Total frames: {len(frames)}")
    print(f"  Mean error (all): {np.mean(errors):.3f} px/frame")
    print(f"  Median error: {np.median(errors):.3f} px/frame")
    print(f"  95th percentile: {np.percentile(errors, 95):.3f} px/frame")

    valid_mask = qualities > 50
    if np.any(valid_mask):
        print(f"  Mean error (quality > 50): {np.mean(errors[valid_mask]):.3f} px/frame")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    csv_path = sys.argv[1]
    if not os.path.exists(csv_path):
        print(f"File not found: {csv_path}")
        sys.exit(1)

    if "--plot" in sys.argv:
        plot_results(csv_path)
    elif "--animate" in sys.argv:
        pgm_dir = sys.argv[sys.argv.index("--animate") + 1] if "--animate" in sys.argv and len(sys.argv) > sys.argv.index("--animate") + 1 else None
        if pgm_dir:
            animate_sequence(csv_path, pgm_dir)
        else:
            animate_sequence(csv_path, ".")
    elif "--analyze" in sys.argv:
        analyze_quality(csv_path)
    else:
        analyze_quality(csv_path)
        plot_results(csv_path)
