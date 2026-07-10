#ifndef FLOW_STAGE1_H
#define FLOW_STAGE1_H

#include "types.h"

/* Stage 1: Ultra-fast SAD block matching optical flow
 *
 * Derived from PX4-OpticalFlow algorithm. Uses grid-based SAD matching
 * between consecutive frames with subpixel refinement.
 *
 * Typical configuration: 8x8 tiles, ±6 search range, on 320x240 grayscale.
 * Running at 60fps, this consumes ~3-5% of a 600MHz Cortex-A8.
 */

/* Context for stage 1 flow (holds previous frame, state) */
typedef struct flow_stage1_s flow_stage1_t;

/* Create stage 1 context */
flow_stage1_t* flow_stage1_create(const flow_stage1_config_t *cfg);

/* Destroy stage 1 context */
void flow_stage1_destroy(flow_stage1_t *ctx);

/* Process a new greyscale frame.
 * Returns flow in subpixel units (factor = 10).
 * result->flow_x_fast and result->flow_y_fast are set.
 * Other fields remain unchanged.
 */
void flow_stage1_process(flow_stage1_t *ctx, const vision_image_t *frame,
                          vision_result_t *result);

/* Set new configuration at runtime (e.g., from ground station) */
void flow_stage1_reconfigure(flow_stage1_t *ctx, const flow_stage1_config_t *cfg);

/* Get current performance stats */
void flow_stage1_stats(const flow_stage1_t *ctx, uint32_t *avg_process_us,
                       uint32_t *tiles_matched, uint8_t *quality);

/* Get per-tile flow vectors from the last flow_stage1_process() call.
 *
 * Returns the number of matched tiles. Pointers can be NULL to skip that data.
 * tile_flow_x/y are in subpixel units (factor = 10).
 *
 * Use this for diagnostics or for passing per-tile data to Stage 3 (rotation).
 */
uint16_t flow_stage1_get_tile_flows(const flow_stage1_t *ctx,
    const uint16_t **positions_x, const uint16_t **positions_y,
    const int16_t **tile_flow_x, const int16_t **tile_flow_y,
    const uint32_t **tile_sad);

#endif /* FLOW_STAGE1_H */
