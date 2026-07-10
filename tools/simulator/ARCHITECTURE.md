# Vision Pipeline Architecture — Bottom Camera Optical Flow

## Multi-Threaded Pipeline

```
                    ┌─────────────────────────────────┐
                    │       V4L2 Capture Thread        │
                    │   (320x240 UYVY @ 60fps max)     │
                    └────────────┬────────────────────┘
                                 │
                                 ▼
                    ┌─────────────────────────────────┐
                    │       Frame Distributor          │
                    │   Y-extract + downsample (si)    │
                    └────┬──────────┬──────────┬───────┘
                         │          │          │
                  ┌──────▼──┐  ┌───▼────┐  ┌──▼────────┐
                  │ Stage 1 │  │ Stage 2│  │  Stage 3  │
                  │ FAST    │  │ ROBUST │  │ ROTATION  │
                  │ Flow    │  │ Flow   │  │ + PATTERN │
                  │ @60fps  │  │ @15fps │  │ @5-30fps  │
                  └────┬────┘  └───┬────┘  └────┬──────┘
                       │           │            │
                       ▼           ▼            ▼
                  ┌──────────────────────────────────────┐
                  │          Sensor Fusion (EKF)          │
                  │  gyro + flow_stage1 + flow_stage2     │
                  │  + rotation + altitude                │
                  └──────────┬───────────────────────────┘
                             │
                             ▼
                  ┌──────────────────────┐
                  │  Output: vx, vy, vz, │
                  │  yaw_rate, quality,  │
                  │  distance_traveled   │
                  └──────────────────────┘
```

## Stage Details

### Stage 1: Ultra-Fast Flow (@camera fps)
- **Goal**: Sub-millisecond latency, every frame
- **Algorithm**: PX4-OpticalFlow SAD block matching on 8x8 tiles
- **Window**: 64x64 (out of 320x240), search ±6px
- **Arithmetic**: Pure integer (uint8_t pixels, int32_t accumulators)
- **Output**: vx, vy (integer pixels/frame), quality metric
- **Why**: Proven on STM32F4 @ 168MHz at 250Hz. On OMAP3530 at 600MHz, can run full 320x240 at 60fps with margin

### Stage 2: Robust Flow (@reduced fps)
- **Goal**: Higher accuracy, subpixel resolution, drift correction for Stage 1
- **Algorithm**: Paparazzi FAST9 corners + pyramidal Lucas-Kanade
- **Parameters**: 25-50 corners, 3 pyramid levels, window 10px, subpixel factor 10
- **Frequency**: Every 3-5 frames (~12-20fps)
- **Arithmetic**: int32_t gradients, float in matrix solve
- **Output**: Subpixel flow vectors, divergence, FoE

### Stage 3: Rotation Detection (visual + gyro fusion)
- **Goal**: Pure-visual yaw estimation for when gyro drifts
- **Algorithm**: Block matching between annular regions (rotation-invariant)
- **Or**: Phase correlation between consecutive frames
- **Fusion**: Complementary filter with gyro from navdata
- **Output**: yaw_rate, cumulative yaw

### Stage 4: Pattern Detection / Navigation
- **Goal**: Landing pad detection, visual return-to-home, markers
- **Algorithm**: 
  - **Landing pad**: Color threshold + blob detection (AprilTag H or simple cross)
  - **Return-to-home**: Feature snapshot at takeoff (FAST corners + BRIEF-like binary descriptors)
  - **Precision landing**: Size-based descent using divergence
- **Frequency**: On-demand or ~1-5 fps
- **Output**: Landing pad position (px, py), home vector

## Portability & Simulation

### Zero architecture-specific code
- All algorithms in portable C99
- NEON optimizations go in separate `*_neon.c` files (not yet)
- SIMD is abstracted via preprocessor if needed

### Desktop simulation framework

```
tools/simulator/
├── sim_main.c              # Test harness (loads images, runs pipeline)
├── Makefile                # Builds with gcc on desktop
├── test_sequences/         # Test image sequences
│   ├── hover/              # Drone hovering (low flow)
│   ├── forward/            # Forward motion
│   ├── rotation/           # Pure yaw rotation
│   ├── landing/            # Approach to landing pad
│   └── synthetic/          # Generated patterns for unit tests
├── visualizer.py           # Python visualization (matplotlib + OpenCV)
├── ground_truth.py         # Generate ground truth flow from known motion
└── analyze_results.py      # Compare output vs ground truth
```

### Test image capture
- Capture raw UYVY frames from AR.Drone bottom camera via FTP
- Script: `tools/capture_bottom_sequence.sh`
- Also: Synthetic test patterns (translating checkerboard, rotating starfield)

## File Structure

```
src/vision/
├── ARCHITECTURE.md         # This file
├── types.h                 # Common types (point_t, flow_t, image_t, result_t)
├── image.h / image.c       # Portable image ops (grayscale, resize, integral)
├── flow_stage1.h / .c      # SAD block matching (PX4Flow-derived)
├── flow_stage2.h / .c      # FAST9 + LK sparse (Paparazzi-derived)
├── rotation.h / .c         # Visual rotation estimation
├── pattern.h / .c          # Landing pad + marker detection
├── pipeline.h / .c         # Pipeline orchestrator + threading
└── test/                   # Unit tests
    ├── test_flow.c
    ├── test_rotation.c
    └── test_pattern.c
```

## Output Data Structure

```c
typedef struct {
    // Stage 1 (every frame)
    int32_t flow_x_fast;       // Fast flow X (subpixel * factor)
    int32_t flow_y_fast;       // Fast flow Y (subpixel * factor)
    uint8_t  quality_fast;      // 0-255: correlation quality
    
    // Stage 2 (when available)
    int32_t flow_x_robust;     // Robust flow X (subpixel * factor)
    int32_t flow_y_robust;     // Robust flow Y (subpixel * factor)
    int32_t divergence;        // Expansion/contraction
    int32_t focus_x;           // Focus of expansion X
    int32_t focus_y;           // Focus of expansion Y
    uint8_t  corner_cnt;       // Number of tracked corners
    uint8_t  quality_robust;   // 0-255
    
    // Stage 3
    int32_t yaw_rate;          // Visual yaw rate (millidegrees/s)
    int32_t cumulative_yaw;    // Visual yaw (millidegrees)
    
    // Stage 4 (when triggered)
    int16_t  landing_x;        // Landing pad center X (-1 if none)
    int16_t  landing_y;        // Landing pad center Y (-1 if none)
    uint16_t landing_size;     // Landing pad apparent size
    int16_t  home_angle;       // Angle to home (millidegrees)
    int16_t  home_distance;    // Distance to home (cm)
    
    // Fused output
    int32_t velocity_x;        // cm/s in body frame
    int32_t velocity_y;        // cm/s in body frame
    int32_t velocity_z;        // cm/s (up positive)
    
    // Metadata
    uint32_t frame_id;
    float    fps_estimate;
} vision_result_t;
```
