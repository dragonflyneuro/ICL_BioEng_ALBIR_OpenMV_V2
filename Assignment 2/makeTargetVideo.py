"""
Script to make target tracking videos for MVBot
Equivalent to the original MATLAB script by DK (17/02/2021)

Dependencies:
    pip install opencv-python numpy

Usage:
    # Run all default frequencies with default square size:
    python makeTargetVideo.py

    # Run all default frequencies with custom square size:
    python makeTargetVideo.py --width 200 --height 400

    # Run a single frequency with custom square size:
    python makeTargetVideo.py --width 200 --height 400 --freq 1.0

    # Run multiple specific frequencies with custom square size:
    python makeTargetVideo.py --width 200 --height 400 --freq 1.0 2.0 3.0
"""

import cv2
import numpy as np
import argparse


def vid_maker(f, target_width, target_height):
    """
    Generate a target tracking video at a given frequency.

    The video has three phases:
        1. Calibration  — Blue square oscillates at a fixed 0.2 Hz for 2 seconds
        2. Intermission — Green square held still for 2.5 seconds
        3. Target       — Red square oscillates at frequency f for `reps` periods

    Args:
        f (float):            Stimulus oscillation frequency in Hz (red target only).
        target_width (int):   Width of the stimulus square in pixels.
        target_height (int):  Height of the stimulus square in pixels.
    """
    # --- Video parameters ---
    fps             = 120
    reps            = 10
    vid_w           = 1920
    vid_h           = 1080
    boundary        = max(target_width, target_height) / 2 + 50
    calibration_hz  = 0.2   # Fixed slow speed for blue calibration — matches MATLAB 0.2/fps step

    # --- Stimulus square pixel ranges ---
    half_w     = target_width  // 2
    half_h     = target_height // 2
    sq_rows    = np.arange(-half_h + 1, half_h + 1)
    sq_cols    = np.arange(-half_w + 1, half_w + 1)

    centre_row = int(3 * vid_h / 4)
    centre_col = vid_w // 2
    row_region = centre_row + sq_rows
    max_travel = vid_w / 2 - boundary

    # --- Output file ---
    filename   = f"targetVideo_{f}Hz_w{target_width}_h{target_height}.mp4"
    fourcc     = cv2.VideoWriter_fourcc(*'mp4v')
    writer     = cv2.VideoWriter(filename, fourcc, fps, (vid_w, vid_h))

    print(f"  Writing: {filename}")

    def get_col_centre(phase):
        """Compute horizontal centre column of the square at a given phase."""
        return int(round(-np.cos(2 * np.pi * phase) * max_travel + centre_col))

    def draw_square(frame_rgb, col_centre, zero_channels):
        """
        Draw the stimulus square on a white frame by zeroing specified channels.

        Args:
            frame_rgb (ndarray):    H x W x 3 float32 frame (values 0–1).
            col_centre (int):       Horizontal centre of the square.
            zero_channels (list):   Channel indices to zero out (0=R, 1=G, 2=B).
        """
        col_region = col_centre + sq_cols
        col_region = col_region[(col_region >= 0) & (col_region < vid_w)]
        row_region_clipped = row_region[(row_region >= 0) & (row_region < vid_h)]
        rows_idx, cols_idx = np.meshgrid(row_region_clipped, col_region, indexing='ij')
        for ch in zero_channels:
            frame_rgb[rows_idx, cols_idx, ch] = 0.0
        return frame_rgb

    def write_frame(frame_float):
        """Convert float [0,1] RGB frame to uint8 BGR and write to video."""
        frame_uint8 = (frame_float * 255).astype(np.uint8)
        frame_bgr   = cv2.cvtColor(frame_uint8, cv2.COLOR_RGB2BGR)
        writer.write(frame_bgr)

    # ----------------------------------------------------------
    # PHASE 1 — Calibration: BLUE square at fixed 0.2 Hz for 2 seconds
    # ----------------------------------------------------------
    print("  Phase 1: Calibration (blue square, fixed 0.2 Hz)...")
    t = 0.0
    while t <= 2.0:
        frame = np.ones((vid_h, vid_w, 3), dtype=np.float32)
        col   = get_col_centre(t)
        frame = draw_square(frame, col, zero_channels=[0, 1])   # zero R and G → Blue
        write_frame(frame)
        t += calibration_hz / fps   # ✅ Fixed 0.2 Hz — independent of f

    # ----------------------------------------------------------
    # PHASE 2 — Intermission: GREEN square held still for 2.5 seconds
    # ----------------------------------------------------------
    print("  Phase 2: Intermission (green square, stationary)...")
    frame = np.ones((vid_h, vid_w, 3), dtype=np.float32)
    col   = get_col_centre(0.0)
    frame = draw_square(frame, col, zero_channels=[0, 2])       # zero R and B → Green
    for _ in range(int(fps * 2.5)):
        write_frame(frame)

    # ----------------------------------------------------------
    # PHASE 3 — Target: RED square oscillates at frequency f for `reps` periods
    # ----------------------------------------------------------
    print(f"  Phase 3: Target (red square, {f} Hz)...")
    t = 0.0
    while t <= reps:
        frame = np.ones((vid_h, vid_w, 3), dtype=np.float32)
        col   = get_col_centre(t)
        frame = draw_square(frame, col, zero_channels=[1, 2])   # zero G and B → Red
        write_frame(frame)
        t += f / fps                # ✅ Variable frequency f — set by user

    writer.release()
    print(f"  Done: {filename}\n")


if __name__ == "__main__":

    # --- Default frequencies matching original MATLAB script ---
    default_freqs = [round(f, 1) for f in np.arange(0.1, 1.1, 0.1)] + \
                    [round(f, 1) for f in np.arange(1.2, 3.2, 0.2)]

    # --- Argument parser ---
    parser = argparse.ArgumentParser(
        description="Generate target tracking videos for MVBot.",
        formatter_class=argparse.RawTextHelpFormatter
    )
    parser.add_argument(
        "--width",
        type=int,
        default=300,
        help="Width of the stimulus square in pixels (default: 300)"
    )
    parser.add_argument(
        "--height",
        type=int,
        default=300,
        help="Height of the stimulus square in pixels (default: 300)"
    )
    parser.add_argument(
        "--freq",
        type=float,
        nargs="+",
        default=None,
        help=(
            "One or more frequencies in Hz for the red target (default: all MATLAB frequencies)\n"
            "Note: blue calibration square always moves at fixed 0.2 Hz\n"
            "Examples:\n"
            "  --freq 1.0          (single frequency)\n"
            "  --freq 1.0 2.0 3.0  (multiple frequencies)"
        )
    )

    args = parser.parse_args()

    frequencies = args.freq if args.freq is not None else default_freqs

    print(f"\n{'='*50}")
    print(f"  Target size        : {args.width} x {args.height} pixels")
    print(f"  Calibration speed  : 0.2 Hz (fixed, blue square)")
    print(f"  Target frequencies : {frequencies} Hz (red square)")
    print(f"{'='*50}\n")

    for freq in frequencies:
        print(f"Starting {freq} Hz...")
        vid_maker(freq, target_width=args.width, target_height=args.height)
        print(f"Finished {freq} Hz.")