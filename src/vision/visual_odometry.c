#include "visual_odometry.h"
#include "image.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* ================================================================== */
/*  Internal state                                                     */
/* ================================================================== */

struct vo_context_s {
    vo_config_t cfg;
    uint16_t image_width;
    uint16_t image_height;

    /* LK tracker */
    lk_context_t *lk;

    /* Feature map */
    vo_feature_map_t fmap;

    /* Previous frame (for LK) */
    vision_image_t prev_frame;
    bool has_prev;

    /* Accumulated position (body frame, NED) */
    vo_position_t pos;

    /* Scale factor (meters per unit translation) */
    float scale_factor;

    /* Statistics */
    uint32_t total_frames;
    uint32_t valid_frames;
    uint32_t total_matches;
    uint32_t total_inliers;
};

/* ================================================================== */
/*  SVD (compact, for 3x3 matrices)                                   */
/* ================================================================== */

/* Compute SVD of 3x3 matrix A = U * S * V^T
 * Using Jacobi rotation method (sufficient for 3x3).
 * All arrays must be pre-allocated (3x3 for U, V; 3 for S).
 */
static void svd3x3(float A_in[3][3], float U[3][3], float S[3], float V[3][3]) {
    /* Copy A to work buffer */
    double a[3][3];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            a[i][j] = A_in[i][j];

    /* Initialize U and V as identity */
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            U[i][j] = (i == j) ? 1.0f : 0.0f;
            V[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }

    /* Jacobi iterations for SVD */
    for (int iter = 0; iter < 40; iter++) {
        /* Find largest off-diagonal element */
        double max_val = 0.0;
        int p = 0, q = 1;
        for (int i = 0; i < 3; i++) {
            for (int j = i + 1; j < 3; j++) {
                double val = fabs(a[i][j]);
                if (val > max_val) {
                    max_val = val;
                    p = i;
                    q = j;
                }
            }
        }

        if (max_val < 1e-10) break;

        /* Compute rotation angle */
        double theta;
        if (fabs(a[p][p] - a[q][q]) < 1e-10) {
            theta = M_PI / 4.0;
        } else {
            theta = 0.5 * atan2(2.0 * a[p][q], a[p][p] - a[q][q]);
        }

        double c = cos(theta);
        double s = sin(theta);

        /* Apply Givens rotation: A' = A * G */
        for (int i = 0; i < 3; i++) {
            double aip = a[i][p];
            double aiq = a[i][q];
            a[i][p] = c * aip + s * aiq;
            a[i][q] = -s * aip + c * aiq;
        }

        /* Apply Givens rotation: A'' = G^T * A' */
        for (int j = 0; j < 3; j++) {
            double apj = a[p][j];
            double aqj = a[q][j];
            a[p][j] = c * apj + s * aqj;
            a[q][j] = -s * apj + c * aqj;
        }

        /* Update V */
        for (int i = 0; i < 3; i++) {
            double vip = V[i][p];
            double viq = V[i][q];
            V[i][p] = c * vip + s * viq;
            V[i][q] = -s * vip + c * viq;
        }
    }

    /* Extract singular values and ensure positive */
    for (int i = 0; i < 3; i++) {
        S[i] = (float)fabs(a[i][i]);
    }

    /* Sort S descending and swap columns of U, V accordingly */
    for (int i = 0; i < 3; i++) {
        for (int j = i + 1; j < 3; j++) {
            if (S[i] < S[j]) {
                float tmp = S[i]; S[i] = S[j]; S[j] = tmp;
                for (int k = 0; k < 3; k++) {
                    tmp = U[k][i]; U[k][i] = U[k][j]; U[k][j] = tmp;
                    tmp = V[k][i]; V[k][i] = V[k][j]; V[k][j] = tmp;
                }
            }
        }
    }

    /* Ensure U has det = +1 (proper rotation) */
    float det_u = U[0][0]*(U[1][1]*U[2][2] - U[1][2]*U[2][1])
                - U[0][1]*(U[1][0]*U[2][2] - U[1][2]*U[2][0])
                + U[0][2]*(U[1][0]*U[2][1] - U[1][1]*U[2][0]);
    if (det_u < 0) {
        for (int i = 0; i < 3; i++) {
            U[i][0] = -U[i][0];
            U[i][1] = -U[i][1];
            U[i][2] = -U[i][2];
        }
    }

    /* Ensure V has det = +1 */
    float det_v = V[0][0]*(V[1][1]*V[2][2] - V[1][2]*V[2][1])
                - V[0][1]*(V[1][0]*V[2][2] - V[1][2]*V[2][0])
                + V[0][2]*(V[1][0]*V[2][1] - V[1][1]*V[2][0]);
    if (det_v < 0) {
        for (int i = 0; i < 3; i++) {
            V[i][0] = -V[i][0];
            V[i][1] = -V[i][1];
            V[i][2] = -V[i][2];
        }
    }
}

/* ================================================================== */
/*  8-Point Essential Matrix                                           */
/* ================================================================== */

/* Normalize points to zero mean and unit variance */
static void normalize_points(float pts[][2], int n,
                             float pts_norm[][2], float T[3][3]) {
    float mx = 0, my = 0;
    for (int i = 0; i < n; i++) {
        mx += pts[i][0];
        my += pts[i][1];
    }
    mx /= n;
    my /= n;

    float sx = 0, sy = 0;
    for (int i = 0; i < n; i++) {
        sx += (pts[i][0] - mx) * (pts[i][0] - mx);
        sy += (pts[i][1] - my) * (pts[i][1] - my);
    }
    sx = sqrtf(sx / n);
    sy = sqrtf(sy / n);

    if (sx < 1e-6f) sx = 1.0f;
    if (sy < 1e-6f) sy = 1.0f;

    /* Normalization matrix */
    T[0][0] = 1.0f / sx;  T[0][1] = 0.0f;      T[0][2] = -mx / sx;
    T[1][0] = 0.0f;       T[1][1] = 1.0f / sy;  T[1][2] = -my / sy;
    T[2][0] = 0.0f;       T[2][1] = 0.0f;       T[2][2] = 1.0f;

    for (int i = 0; i < n; i++) {
        float w = T[2][0] * pts[i][0] + T[2][1] * pts[i][1] + T[2][2];
        pts_norm[i][0] = (T[0][0] * pts[i][0] + T[0][1] * pts[i][1] + T[0][2]) / w;
        pts_norm[i][1] = (T[1][0] * pts[i][0] + T[1][1] * pts[i][1] + T[1][2]) / w;
    }
}

bool vo_compute_essential(float pts1[][2], float pts2[][2],
                          int n, float E[3][3]) {
    if (n < 8) return false;

    /* Normalize points */
    float T1[3][3], T2[3][3];
    float (*pn1)[2] = malloc(n * sizeof(float[2]));
    float (*pn2)[2] = malloc(n * sizeof(float[2]));
    if (!pn1 || !pn2) { free(pn1); free(pn2); return false; }

    normalize_points(pts1, n, pn1, T1);
    normalize_points(pts2, n, pn2, T2);

    /* Build 8x9 matrix for Ah = 0 */
    float *A = calloc(n * 9, sizeof(float));
    if (!A) { free(pn1); free(pn2); return false; }

    for (int i = 0; i < n; i++) {
        float u1 = pn1[i][0], v1 = pn1[i][1];
        float u2 = pn2[i][0], v2 = pn2[i][1];
        A[i*9+0] = u2*u1;
        A[i*9+1] = u2*v1;
        A[i*9+2] = u2;
        A[i*9+3] = v2*u1;
        A[i*9+4] = v2*v1;
        A[i*9+5] = v2;
        A[i*9+6] = u1;
        A[i*9+7] = v1;
        A[i*9+8] = 1.0f;
    }

    /* Solve via SVD of A^T * A (9x9 is too big, use power iteration for smallest singular vector) */
    /* Instead, use the fact that for n >= 8, we can use the pseudo-inverse approach */
    /* For simplicity, use the normalized 8-point with SVD of the 8x9 matrix */

    /* For n=8 exactly, solve directly. For n>8, use least squares via normal equations */
    float ATA[9][9];
    memset(ATA, 0, sizeof(ATA));

    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 9; j++) {
            float sum = 0;
            for (int k = 0; k < n; k++) {
                sum += A[k*9+i] * A[k*9+j];
            }
            ATA[i][j] = sum;
        }
    }

    /* Find null space of ATA via power iteration (smallest eigenvalue) */
    float h[9];
    for (int i = 0; i < 9; i++) h[i] = 1.0f / 3.0f;

    for (int iter = 0; iter < 100; iter++) {
        float Ah[9];
        for (int i = 0; i < 9; i++) {
            Ah[i] = 0;
            for (int j = 0; j < 9; j++) {
                Ah[i] += ATA[i][j] * h[j];
            }
        }

        /* Subtract projection onto current h (inverse iteration) */
        float dot = 0;
        for (int i = 0; i < 9; i++) dot += h[i] * Ah[i];

        /* Use ATA - (h^T ATA h) I to find smaller eigenvalue */
        for (int i = 0; i < 9; i++) {
            Ah[i] -= dot * h[i];
        }

        /* Normalize */
        float norm = 0;
        for (int i = 0; i < 9; i++) norm += Ah[i] * Ah[i];
        norm = sqrtf(norm);
        if (norm < 1e-10f) break;
        for (int i = 0; i < 9; i++) h[i] = Ah[i] / norm;
    }

    /* Extract E from h and denormalize */
    float En[3][3];
    En[0][0] = h[0]; En[0][1] = h[1]; En[0][2] = h[2];
    En[1][0] = h[3]; En[1][1] = h[4]; En[1][2] = h[5];
    En[2][0] = h[6]; En[2][1] = h[7]; En[2][2] = h[8];

    /* Denormalize: E = T2^T * En * T1 */
    float T2T[3][3];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            T2T[i][j] = T2[j][i];

    float tmp[3][3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            tmp[i][j] = 0;
            for (int k = 0; k < 3; k++) {
                tmp[i][j] += T2T[i][k] * En[k][j];
            }
        }
    }

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            E[i][j] = 0;
            for (int k = 0; k < 3; k++) {
                E[i][j] += tmp[i][k] * T1[k][j];
            }
        }
    }

    /* Enforce epipolar constraint: make E singular (SVD, set middle singular value to 0) */
    float U[3][3], S[3], V[3][3];
    svd3x3(E, U, S, V);
    S[1] = 0.0f;  /* enforce rank-2 */
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            E[i][j] = 0;
            for (int k = 0; k < 3; k++) {
                E[i][j] += U[i][k] * S[k] * V[j][k];
            }
        }
    }

    free(A);
    free(pn1);
    free(pn2);
    return true;
}

/* ================================================================== */
/*  Essential Matrix Decomposition → R, t                              */
/* ================================================================== */

bool vo_decompose_essential(float E[3][3], float R[3][3], float t[3]) {
    float U[3][3], S[3], V[3][3];
    svd3x3(E, U, S, V);

    /* Check that essential matrix is valid (two non-zero singular values) */
    if (S[0] < 1e-6f || S[1] < 1e-6f) return false;

    /* W matrix for rotation extraction */
    float W[3][3] = {
        { 0, -1, 0 },
        { 1,  0, 0 },
        { 0,  0, 1 }
    };

    /* R = U * W * V^T */
    float UW[3][3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            UW[i][j] = 0;
            for (int k = 0; k < 3; k++) {
                UW[i][j] += U[i][k] * W[k][j];
            }
        }
    }

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            R[i][j] = 0;
            for (int k = 0; k < 3; k++) {
                R[i][j] += UW[i][k] * V[j][k];
            }
        }
    }

    /* t = third column of U (or -third column) */
    t[0] = U[0][2];
    t[1] = U[1][2];
    t[2] = U[2][2];

    /* Normalize t to unit length */
    float tn = sqrtf(t[0]*t[0] + t[1]*t[1] + t[2]*t[2]);
    if (tn > 1e-6f) {
        t[0] /= tn;
        t[1] /= tn;
        t[2] /= tn;
    }

    return true;
}

/* ================================================================== */
/*  Epipolar Error                                                    */
/* ================================================================== */

float vo_epipolar_error(float E[3][3],
                        const float p1[2], const float p2[2]) {
    float p1h[3] = { p1[0], p1[1], 1.0f };
    float p2h[3] = { p2[0], p2[1], 1.0f };

    /* Compute p2^T * E * p1 */
    float Ep1[3] = {0, 0, 0};
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            Ep1[i] += E[i][j] * p1h[j];

    float err = 0;
    for (int i = 0; i < 3; i++)
        err += p2h[i] * Ep1[i];

    return fabsf(err);
}

/* ================================================================== */
/*  Outlier Rejection (RANSAC-like)                                    */
/* ================================================================== */

static int reject_outliers(float pts1[][2], float pts2[][2],
                           int n, float E[3][3], float threshold,
                           int *inlier_indices) {
    int num_inliers = 0;
    for (int i = 0; i < n; i++) {
        float err = vo_epipolar_error(E, pts1[i], pts2[i]);
        if (err < threshold) {
            inlier_indices[num_inliers++] = i;
        }
    }
    return num_inliers;
}

/* ================================================================== */
/*  Default Configuration                                              */
/* ================================================================== */

void vo_default_config(vo_config_t *cfg) {
    if (!cfg) return;
    memset(cfg, 0, sizeof(vo_config_t));

    cfg->fast_threshold    = 30;
    cfg->min_feature_dist  = 10;
    cfg->max_features      = 150;

    cfg->lk_pyramid_levels = 4;
    cfg->lk_window_size    = 5;
    cfg->lk_max_iterations = 15;

    cfg->ransac_threshold  = 3.0f;
    cfg->min_matches       = 20;

    cfg->known_height_m    = 0.0f;
    cfg->use_barometer     = true;
    cfg->use_gps           = false;

    /* AR.Drone 2.0 front camera: ~68° horizontal FOV */
    /* Normalized focal length: f / width */
    cfg->fx = 1.0f;  /* will be set from actual camera params */
    cfg->fy = 1.0f;
    cfg->cx = 0.5f;
    cfg->cy = 0.5f;
}

/* ================================================================== */
/*  Context Lifecycle                                                  */
/* ================================================================== */

vo_context_t* vo_create(const vo_config_t *cfg,
                        uint16_t image_width,
                        uint16_t image_height) {
    vo_context_t *ctx = calloc(1, sizeof(vo_context_t));
    if (!ctx) return NULL;

    ctx->cfg = *cfg;
    ctx->image_width = image_width;
    ctx->image_height = image_height;

    /* Create LK tracker */
    flow_stage2_config_t lk_cfg;
    memset(&lk_cfg, 0, sizeof(lk_cfg));
    lk_cfg.fast_threshold    = cfg->fast_threshold;
    lk_cfg.fast_min_distance = cfg->min_feature_dist;
    lk_cfg.max_corners       = cfg->max_features;
    lk_cfg.lk_window_size    = cfg->lk_window_size;
    lk_cfg.lk_pyramid_levels = cfg->lk_pyramid_levels;
    lk_cfg.lk_max_iterations = cfg->lk_max_iterations;
    lk_cfg.lk_subpixel_factor = 10;

    ctx->lk = lk_context_create(&lk_cfg, image_width, image_height);
    if (!ctx->lk) {
        free(ctx);
        return NULL;
    }

    /* Allocate previous frame buffer */
    ctx->prev_frame.buf = malloc(image_width * image_height);
    if (!ctx->prev_frame.buf) {
        lk_context_destroy(ctx->lk);
        free(ctx);
        return NULL;
    }
    ctx->prev_frame.w = image_width;
    ctx->prev_frame.h = image_height;
    ctx->prev_frame.stride = image_width;
    ctx->prev_frame.bytes_per_pixel = 1;
    ctx->prev_frame.type = VISION_IMAGE_GRAYSCALE;

    /* Initialize position */
    ctx->pos.valid = false;
    ctx->scale_factor = 1.0f;

    return ctx;
}

void vo_destroy(vo_context_t *ctx) {
    if (!ctx) return;
    if (ctx->lk) lk_context_destroy(ctx->lk);
    if (ctx->prev_frame.buf) free(ctx->prev_frame.buf);
    free(ctx);
}

void vo_reset(vo_context_t *ctx) {
    if (!ctx) return;
    memset(&ctx->fmap, 0, sizeof(vo_feature_map_t));
    ctx->fmap.next_id = 1;
    ctx->has_prev = false;
    memset(&ctx->pos, 0, sizeof(vo_position_t));
    ctx->scale_factor = 1.0f;
}

void vo_reconfigure(vo_context_t *ctx, const vo_config_t *cfg) {
    if (!ctx || !cfg) return;
    ctx->cfg = *cfg;
    lk_reconfigure(ctx->lk, NULL);  /* TODO: update LK config */
}

/* ================================================================== */
/*  Scale Recovery                                                     */
/* ================================================================== */

/* Recover scale from altitude change.
 * When the drone moves, the apparent motion in pixels relates to
 * real-world motion via: real_distance = pixel_flow * altitude / focal_length
 */
static float recover_scale_from_altitude(float altitude_m,
                                         float focal_length_px,
                                         float image_height) {
    if (altitude_m < 0.1f || focal_length_px < 1.0f) return 1.0f;
    /* Scale = altitude / focal_length (in image coordinates) */
    return altitude_m / focal_length_px;
}

/* Recover scale from known ground height */
static float recover_scale_from_height(float known_height_m,
                                        float pixel_flow,
                                        float dt) {
    if (known_height_m < 0.1f || fabsf(pixel_flow) < 0.1f || dt < 1.0f)
        return 1.0f;
    /* rough estimate: real_motion = known_height * pixel_flow / focal */
    return known_height_m;
}

/* ================================================================== */
/*  Main Processing                                                    */
/* ================================================================== */

int vo_process(vo_context_t *ctx,
               const vision_image_t *frame,
               float altitude_m,
               float gps_scale,
               uint32_t dt_ms,
               vo_motion_t *motion,
               vo_position_t *position) {
    if (!ctx || !frame || !frame->buf) return -1;

    /* Initialize motion output */
    if (motion) {
        memset(motion, 0, sizeof(vo_motion_t));
    }

    ctx->total_frames++;

    /* --- Step 1: Track features via LK --- */
    lk_track_t tracks[VO_MAX_FEATURES];
    uint16_t num_tracks = 0;

    if (ctx->has_prev && dt_ms > 0) {
        num_tracks = lk_track(ctx->lk, &ctx->prev_frame, frame,
                              tracks, VO_MAX_FEATURES);
    }

    /* --- Step 2: Filter tracked features (status == 1) --- */
    float pts1[VO_MAX_FEATURES][2];  /* previous positions */
    float pts2[VO_MAX_FEATURES][2];  /* current positions */
    int num_matched = 0;

    for (uint16_t i = 0; i < num_tracks && num_matched < VO_MAX_FEATURES; i++) {
        if (tracks[i].status == 1) {
            /* Convert subpixel to float */
            pts1[num_matched][0] = (float)tracks[i].x;
            pts1[num_matched][1] = (float)tracks[i].y;
            pts2[num_matched][0] = (float)tracks[i].x +
                                   (float)tracks[i].flow_x / LK_SUBPIXEL_FACTOR;
            pts2[num_matched][1] = (float)tracks[i].y +
                                   (float)tracks[i].flow_y / LK_SUBPIXEL_FACTOR;
            num_matched++;
        }
    }

    /* Save current frame as previous for next iteration */
    memcpy(ctx->prev_frame.buf, frame->buf, frame->w * frame->h);
    ctx->has_prev = true;

    /* Need at least 8 matches for essential matrix */
    if (num_matched < ctx->cfg.min_matches) {
        if (motion) motion->num_matches = num_matched;
        return 0;  /* not enough matches, skip frame */
    }

    /* --- Step 3: Compute essential matrix --- */
    float E[3][3];
    if (!vo_compute_essential(pts1, pts2, num_matched, E)) {
        return 0;
    }

    /* --- Step 4: Reject outliers --- */
    int inlier_indices[VO_MAX_FEATURES];
    int num_inliers = reject_outliers(pts1, pts2, num_matched, E,
                                       ctx->cfg.ransac_threshold,
                                       inlier_indices);

    if (num_inliers < ctx->cfg.min_matches) {
        if (motion) {
            motion->num_matches = num_matched;
            motion->num_inliers = num_inliers;
            motion->inlier_ratio = (num_matched > 0) ?
                                   (float)num_inliers / num_matched : 0;
        }
        return 0;
    }

    /* Recompute E with inliers only */
    float pts1_in[VO_MAX_FEATURES][2];
    float pts2_in[VO_MAX_FEATURES][2];
    for (int i = 0; i < num_inliers; i++) {
        pts1_in[i][0] = pts1[inlier_indices[i]][0];
        pts1_in[i][1] = pts1[inlier_indices[i]][1];
        pts2_in[i][0] = pts2[inlier_indices[i]][0];
        pts2_in[i][1] = pts2[inlier_indices[i]][1];
    }

    vo_compute_essential(pts1_in, pts2_in, num_inliers, E);

    /* --- Step 5: Decompose E → R, t --- */
    float R[3][3], t[3];
    if (!vo_decompose_essential(E, R, t)) {
        return 0;
    }

    /* --- Step 6: Recover scale --- */
    float scale = 1.0f;

    if (ctx->cfg.use_barometer && altitude_m > 0.1f) {
        /* Use barometer for absolute scale */
        float focal_px = ctx->cfg.fx * ctx->image_width;
        scale = recover_scale_from_altitude(altitude_m, focal_px,
                                            (float)ctx->image_height);
        ctx->scale_factor = scale;
    } else if (ctx->cfg.known_height_m > 0.1f) {
        scale = recover_scale_from_height(ctx->cfg.known_height_m,
                                           1.0f, (float)dt_ms);
    } else if (gps_scale > 0.01f) {
        scale = gps_scale;
        ctx->scale_factor = scale;
    } else {
        /* Use previous scale or default */
        scale = ctx->scale_factor;
    }

    float dt_s = (float)dt_ms / 1000.0f;

    /* --- Step 7: Integrate position --- */
    /* t is in camera frame. Transform to body frame (NED):
     * Camera forward = body forward (x)
     * Camera right   = body right (y)
     * Camera down    = body down (z)
     * AR.Drone camera is forward-facing, so no rotation needed.
     */
    float dx = t[0] * scale * dt_s * 10.0f;  /* scale factor for visible motion */
    float dy = t[1] * scale * dt_s * 10.0f;
    float dz = t[2] * scale * dt_s * 10.0f;

    ctx->pos.x += dx;
    ctx->pos.y += dy;
    ctx->pos.z += dz;

    /* Extract euler angles from R */
    ctx->pos.roll  = atan2f(R[2][1], R[2][2]);
    ctx->pos.pitch = asinf(-R[2][0]);
    ctx->pos.yaw   = atan2f(R[1][0], R[0][0]);

    /* Velocity (smoothed) */
    if (dt_s > 0.001f) {
        float alpha = 0.3f;  /* low-pass filter */
        ctx->pos.vx = alpha * (dx / dt_s) + (1.0f - alpha) * ctx->pos.vx;
        ctx->pos.vy = alpha * (dy / dt_s) + (1.0f - alpha) * ctx->pos.vy;
        ctx->pos.vz = alpha * (dz / dt_s) + (1.0f - alpha) * ctx->pos.vz;
    }

    ctx->pos.timestamp_ms = frame->timestamp_us / 1000;
    ctx->pos.valid = true;
    ctx->valid_frames++;
    ctx->total_matches += num_matched;
    ctx->total_inliers += num_inliers;

    /* Fill outputs */
    if (motion) {
        memcpy(motion->R, R, sizeof(R));
        memcpy(motion->t, t, sizeof(t));
        motion->num_matches = num_matched;
        motion->num_inliers = num_inliers;
        motion->inlier_ratio = (float)num_inliers / num_matched;
        motion->valid = true;
    }

    if (position) {
        *position = ctx->pos;
    }

    return 0;
}

/* ================================================================== */
/*  Query Functions                                                    */
/* ================================================================== */

void vo_get_position(const vo_context_t *ctx, vo_position_t *pos) {
    if (!ctx || !pos) return;
    *pos = ctx->pos;
}

uint16_t vo_feature_count(const vo_context_t *ctx) {
    if (!ctx) return 0;
    return ctx->fmap.count;
}
