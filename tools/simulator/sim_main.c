/*
 * Desktop simulation harness for vision pipeline.
 *
 * Usage: sim_main [options]
 *
 * Modes:
 *   --synthetic       Generate synthetic test patterns with known motion
 *   --pgm             Load PGM image sequence from directory
 *
 * Options:
 *   --frames N        Number of frames to process (default: 100)
 *   --flow-x DX       Known X flow for synthetic (default: 2.0)
 *   --flow-y DY       Known Y flow for synthetic (default: 1.0)
 *   --input DIR       Input directory with PGM frames
 *   --out DIR         Output directory for results CSV (default: results/)
 *   --visualize       Save visualization frames (requires PGM writing)
 *   --width W         Image width (default: 320)
 *   --height H        Image height (default: 240)
 *   --tile-size TS    SAD tile size (default: 8)
 *   --search-range SR Search range in pixels (default: 6)
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <sys/stat.h>

#include "types.h"
#include "image.h"
#include "flow_stage1.h"

/* ------------------------------------------------------------------ */
/*  Synthetic pattern generator                                       */
/* ------------------------------------------------------------------ */

static void generate_checkerboard(vision_image_t *img, int frame, int flow_x, int flow_y) {
  int phase_x = (frame * flow_x) % 32;
  int phase_y = (frame * flow_y) % 32;

  for (uint16_t y = 0; y < img->h; y++) {
    for (uint16_t x = 0; x < img->w; x++) {
      int px = x + phase_x;
      int py = y + phase_y;
      uint8_t v = ((px / 16) + (py / 16)) & 1 ? 200 : 50;
      img->buf[y * img->stride + x] = v;
    }
  }
}

static void generate_random_texture(vision_image_t *img, int frame, int flow_x, int flow_y) {
  static uint32_t seed = 12345;
  static bool seeded = false;

  if (!seeded) {
    srand(seed);
    seeded = true;
  }

  int shift_x = (frame * flow_x) % img->w;
  int shift_y = (frame * flow_y) % img->h;

  /* Generate new random pattern only on first frame, then shift */
  static uint8_t *pattern = NULL;
  static uint16_t pw = 0, ph = 0;

  if (!pattern || pw != img->w || ph != img->h) {
    if (pattern) free(pattern);
    pw = img->w;
    ph = img->h;
    pattern = (uint8_t*)malloc(pw * ph);
    for (uint32_t i = 0; i < (uint32_t)pw * ph; i++) {
      pattern[i] = (uint8_t)(rand() & 0xFF);
    }
  }

  /* Shift pattern with wrapping */
  for (uint16_t y = 0; y < img->h; y++) {
    for (uint16_t x = 0; x < img->w; x++) {
      int sx = ((int)x + shift_x) % img->w;
      int sy = ((int)y + shift_y) % img->h;
      img->buf[y * img->stride + x] = pattern[sy * pw + sx];
    }
  }

  (void)frame;
}

static void generate_gradient(vision_image_t *img, int frame, int flow_x, int flow_y) {
  int offset = frame * flow_x;

  for (uint16_t y = 0; y < img->h; y++) {
    for (uint16_t x = 0; x < img->w; x++) {
      img->buf[y * img->stride + x] = (uint8_t)((x + offset) & 0xFF);
    }
  }
  (void)flow_y;
}

/* ------------------------------------------------------------------ */
/*  PGM loader                                                        */
/* ------------------------------------------------------------------ */

static bool load_pgm(const char *filename, vision_image_t *img) {
  FILE *f = fopen(filename, "rb");
  if (!f) return false;

  char magic[3];
  int w, h, max_val;
  if (fscanf(f, "%2s %d %d %d\n", magic, &w, &h, &max_val) != 4) {
    fclose(f);
    return false;
  }

  if (magic[0] != 'P' || magic[1] != '5') {
    fclose(f);
    return false;
  }

  if (w != (int)img->w || h != (int)img->h) {
    fprintf(stderr, "Warning: image %s is %dx%d, expected %dx%d\n",
            filename, w, h, img->w, img->h);
  }

  fread(img->buf, 1, (size_t)(w * h), f);
  fclose(f);
  return true;
}

#if 0
static int pgm_filename_filter(const struct dirent *entry) {
  const char *ext = strrchr(entry->d_name, '.');
  return ext && (strcmp(ext, ".pgm") == 0);
}
#endif

/* ------------------------------------------------------------------ */
/*  Stats output                                                      */
/* ------------------------------------------------------------------ */

static void write_csv_header(FILE *f) {
  fprintf(f, "frame,flow_x_est,flow_y_est,quality,sad_score,"
             "flow_x_gt,flow_y_gt,error_x,error_y\n");
}

static void write_csv_row(FILE *f, int frame,
                           int32_t fx, int32_t fy, uint8_t q, uint32_t sad,
                           int32_t gx, int32_t gy) {
  double fx_d = fx / 10.0;
  double fy_d = fy / 10.0;
  double gx_d = gx / 10.0;
  double gy_d = gy / 10.0;
  fprintf(f, "%d,%.1f,%.1f,%u,%u,%.1f,%.1f,%.1f,%.1f\n",
          frame, fx_d, fy_d, q, sad, gx_d, gy_d, fx_d - gx_d, fy_d - gy_d);
}

/* ------------------------------------------------------------------ */
/*  main                                                              */
/* ------------------------------------------------------------------ */

typedef struct {
  enum { MODE_SYNTHETIC, MODE_PGM } mode;
  int     num_frames;
  double  flow_x;
  double  flow_y;
  const char *input_dir;
  const char *output_dir;
  int     visualize;
  int     width;
  int     height;
  int     tile_size;
  int     search_range;
} sim_config_t;

static void parse_args(sim_config_t *cfg, int argc, char **argv) {
  memset(cfg, 0, sizeof(*cfg));
  cfg->mode = MODE_SYNTHETIC;
  cfg->num_frames = 100;
  cfg->flow_x = 2.0;
  cfg->flow_y = 1.0;
  cfg->width = 320;
  cfg->height = 240;
  cfg->tile_size = 8;
  cfg->search_range = 6;
  cfg->output_dir = "results";
  cfg->input_dir = "test_sequences";

  for (int i = 1; i < argc; i++) {
    if (strcmp(argv[i], "--synthetic") == 0) {
      cfg->mode = MODE_SYNTHETIC;
    } else if (strcmp(argv[i], "--pgm") == 0) {
      cfg->mode = MODE_PGM;
    } else if (strcmp(argv[i], "--frames") == 0 && i + 1 < argc) {
      cfg->num_frames = atoi(argv[++i]);
    } else if (strcmp(argv[i], "--flow-x") == 0 && i + 1 < argc) {
      cfg->flow_x = atof(argv[++i]);
    } else if (strcmp(argv[i], "--flow-y") == 0 && i + 1 < argc) {
      cfg->flow_y = atof(argv[++i]);
    } else if (strcmp(argv[i], "--input") == 0 && i + 1 < argc) {
      cfg->input_dir = argv[++i];
    } else if (strcmp(argv[i], "--out") == 0 && i + 1 < argc) {
      cfg->output_dir = argv[++i];
    } else if (strcmp(argv[i], "--visualize") == 0) {
      cfg->visualize = 1;
    } else if (strcmp(argv[i], "--width") == 0 && i + 1 < argc) {
      cfg->width = atoi(argv[++i]);
    } else if (strcmp(argv[i], "--height") == 0 && i + 1 < argc) {
      cfg->height = atoi(argv[++i]);
    } else if (strcmp(argv[i], "--tile-size") == 0 && i + 1 < argc) {
      cfg->tile_size = atoi(argv[++i]);
    } else if (strcmp(argv[i], "--search-range") == 0 && i + 1 < argc) {
      cfg->search_range = atoi(argv[++i]);
    } else {
      fprintf(stderr, "Unknown option: %s\n", argv[i]);
      exit(1);
    }
  }
}

int main(int argc, char **argv) {
  sim_config_t cfg;
  parse_args(&cfg, argc, argv);

  printf("=== PARROT VISION SIMULATOR ===\n");
  printf("Mode:        %s\n", cfg.mode == MODE_SYNTHETIC ? "synthetic" : "pgm");
  printf("Frames:      %d\n", cfg.num_frames);
  printf("Resolution:  %dx%d\n", cfg.width, cfg.height);
  printf("Tile size:   %d\n", cfg.tile_size);
  printf("Search range: %d\n", cfg.search_range);
  if (cfg.mode == MODE_SYNTHETIC) {
    printf("Known flow:  (%.1f, %.1f) px/frame\n", cfg.flow_x, cfg.flow_y);
  }
  printf("\n");

  /* Create pipeline config */
  flow_stage1_config_t s1_cfg;
  s1_cfg.tile_size = (uint8_t)cfg.tile_size;
  s1_cfg.search_range = (uint8_t)cfg.search_range;
  s1_cfg.image_width = (uint16_t)cfg.width;
  s1_cfg.image_height = (uint16_t)cfg.height;
  s1_cfg.subsample = 1;
  s1_cfg.min_quality = 0;

  /* Create frames */
  vision_image_t frame, vis_frame;
  if (!image_create(&frame, cfg.width, cfg.height, VISION_IMAGE_GRAYSCALE)) {
    fprintf(stderr, "Failed to create frame buffer\n");
    return 1;
  }
  if (cfg.visualize) {
    if (!image_create(&vis_frame, cfg.width, cfg.height, VISION_IMAGE_GRAYSCALE)) {
      cfg.visualize = 0;
    }
  }

  /* Create flow processor */
  flow_stage1_t *flow_ctx = flow_stage1_create(&s1_cfg);
  if (!flow_ctx) {
    fprintf(stderr, "Failed to create flow context\n");
    image_destroy(&frame);
    if (cfg.visualize) image_destroy(&vis_frame);
    return 1;
  }

  /* Open CSV output */
  mkdir(cfg.output_dir, 0755);
  char csv_path[256];
  snprintf(csv_path, sizeof(csv_path), "%s/results.csv", cfg.output_dir);
  FILE *csv = fopen(csv_path, "w");
  if (csv) write_csv_header(csv);

  /* Processing loop */
  vision_result_t result;
  memset(&result, 0, sizeof(result));

  int32_t gt_flow_x = (int32_t)(cfg.flow_x * 10);
  int32_t gt_flow_y = (int32_t)(cfg.flow_y * 10);

  double total_error_x = 0, total_error_y = 0;
  int valid_frames = 0;

  for (int f = 0; f < cfg.num_frames; f++) {
    /* Get frame */
    if (cfg.mode == MODE_SYNTHETIC) {
      generate_random_texture(&frame, f, (int)cfg.flow_x, (int)cfg.flow_y);
    }

    /* Process */
    flow_stage1_process(flow_ctx, &frame, &result);

    /* Compute error against ground truth */
    if (f > 0) { /* skip first frame (no flow yet) */
      double err_x = (result.flow_x_fast - gt_flow_x) / 10.0;
      double err_y = (result.flow_y_fast - gt_flow_y) / 10.0;
      total_error_x += fabs(err_x);
      total_error_y += fabs(err_y);
      valid_frames++;

      if (csv) {
        write_csv_row(csv, f, result.flow_x_fast, result.flow_y_fast,
                       result.quality_fast, result.sad_score,
                       gt_flow_x, gt_flow_y);
      }

      /* Status every 10 frames */
      if (f % 10 == 0) {
        printf("Frame %4d | flow=(%5.1f, %5.1f) gt=(%5.1f, %5.1f) "
               "err=(%5.1f, %5.1f) qual=%3d sad=%5u\n",
               f,
               result.flow_x_fast / 10.0, result.flow_y_fast / 10.0,
               cfg.flow_x, cfg.flow_y,
               err_x, err_y,
               result.quality_fast, result.sad_score);
      }
    }

    /* Save visualization */
    if (cfg.visualize && f > 0) {
      image_copy(&frame, &vis_frame);
      image_draw_flow(&vis_frame, result.flow_x_fast, result.flow_y_fast, 10);

      char viz_path[256];
      snprintf(viz_path, sizeof(viz_path), "%s/frame_%04d.pgm", cfg.output_dir, f);
      FILE *vf = fopen(viz_path, "wb");
      if (vf) {
        fprintf(vf, "P5\n%d %d\n255\n", vis_frame.w, vis_frame.h);
        fwrite(vis_frame.buf, 1, (size_t)(vis_frame.w * vis_frame.h), vf);
        fclose(vf);
      }
    }
  }

  /* Summary */
  printf("\n=== RESULTS ===\n");
  if (valid_frames > 0) {
    printf("Mean absolute error: (%.3f, %.3f) px/frame\n",
           total_error_x / valid_frames, total_error_y / valid_frames);
    printf("RMSE: %.3f px/frame\n",
           sqrt((total_error_x * total_error_x + total_error_y * total_error_y) / valid_frames));
  }
  printf("Frames processed: %d\n", cfg.num_frames);
  uint32_t tiles_matched = 0;
  flow_stage1_stats(flow_ctx, NULL, &tiles_matched, NULL);
  printf("Stage 1 tiles: %u\n", tiles_matched);
  printf("\nResults written to: %s\n", csv_path);

  /* Cleanup */
  if (csv) fclose(csv);
  flow_stage1_destroy(flow_ctx);
  image_destroy(&frame);
  if (cfg.visualize) image_destroy(&vis_frame);

  return 0;
}
