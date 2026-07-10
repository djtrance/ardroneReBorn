#ifndef FLOW_STAGE2_H
#define FLOW_STAGE2_H

#include "types.h"

/* Stage 2: Robust optical flow (FAST9 + pyramidal LK)
 *
 * Runs at reduced frame rate (every N frames) and provides
 * subpixel-accurate flow with outlier rejection, divergence,
 * and focus of expansion estimation.
 */

/* Stage 2 context */
typedef struct flow_stage2_s flow_stage2_t;

/* Create stage 2 context */
flow_stage2_t* flow_stage2_create(const flow_stage2_config_t *cfg,
                                   uint16_t image_width,
                                   uint16_t image_height);

/* Destroy stage 2 context */
void flow_stage2_destroy(flow_stage2_t *ctx);

/* Process frame pair.
 * Returns flow in subpixel units (factor = 10).
 * Fills result fields: flow_x_robust, flow_y_robust, divergence,
 *   focus_x, focus_y, corner_cnt, quality_robust.
 */
void flow_stage2_process(flow_stage2_t *ctx,
                          const vision_image_t *curr,
                          vision_result_t *result);

/* Reconfigure at runtime */
void flow_stage2_reconfigure(flow_stage2_t *ctx,
                              const flow_stage2_config_t *cfg);

#endif /* FLOW_STAGE2_H */
