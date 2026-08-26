#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <SDL2/SDL.h>

#include "quad_sim.h"

#define WIN_W 960
#define WIN_H 640

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

typedef struct {
    uint8_t r, g, b, a;
} color_t;

static const color_t COLOR_WHITE  = {255,255,255,255};
static const color_t COLOR_GREEN  = {0,255,0,255};
static const color_t COLOR_RED    = {255,50,50,255};
static const color_t COLOR_CYAN   = {0,255,255,255};
static const color_t COLOR_YELLOW = {255,255,0,255};
static const color_t COLOR_GRAY   = {128,128,128,255};
static const color_t COLOR_DKGRAY = {60,60,60,255};
static const color_t COLOR_ORANGE = {255,160,0,255};

/* 8x8 bitmap font (ASCII 32-126) */
static const uint8_t font8x8[95][8] = {
  {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
  {0x18,0x18,0x18,0x18,0x00,0x00,0x18,0x00},
  {0x6c,0x6c,0x6c,0x00,0x00,0x00,0x00,0x00},
  {0x6c,0x6c,0xfe,0x6c,0xfe,0x6c,0x6c,0x00},
  {0x18,0x3e,0x60,0x3c,0x06,0x7c,0x18,0x00},
  {0x00,0x66,0xac,0xd8,0x36,0x6a,0xcc,0x00},
  {0x38,0x6c,0x38,0x76,0xdc,0xcc,0x76,0x00},
  {0x18,0x18,0x18,0x00,0x00,0x00,0x00,0x00},
  {0x0c,0x18,0x30,0x30,0x30,0x18,0x0c,0x00},
  {0x30,0x18,0x0c,0x0c,0x0c,0x18,0x30,0x00},
  {0x00,0x66,0x3c,0xff,0x3c,0x66,0x00,0x00},
  {0x00,0x18,0x18,0x7e,0x18,0x18,0x00,0x00},
  {0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x30},
  {0x00,0x00,0x00,0x7e,0x00,0x00,0x00,0x00},
  {0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x00},
  {0x06,0x0c,0x18,0x30,0x60,0xc0,0x80,0x00},
  {0x3c,0x66,0x6e,0x7e,0x76,0x66,0x3c,0x00},
  {0x18,0x38,0x18,0x18,0x18,0x18,0x7e,0x00},
  {0x3c,0x66,0x06,0x1c,0x30,0x60,0x7e,0x00},
  {0x3c,0x66,0x06,0x1c,0x06,0x66,0x3c,0x00},
  {0x1c,0x3c,0x6c,0xcc,0xfe,0x0c,0x1e,0x00},
  {0x7e,0x60,0x7c,0x06,0x06,0x66,0x3c,0x00},
  {0x1c,0x30,0x60,0x7c,0x66,0x66,0x3c,0x00},
  {0x7e,0x06,0x0c,0x18,0x30,0x30,0x30,0x00},
  {0x3c,0x66,0x66,0x3c,0x66,0x66,0x3c,0x00},
  {0x3c,0x66,0x66,0x3e,0x06,0x0c,0x38,0x00},
  {0x00,0x18,0x18,0x00,0x00,0x18,0x18,0x00},
  {0x00,0x18,0x18,0x00,0x00,0x18,0x18,0x30},
  {0x0c,0x18,0x30,0x60,0x30,0x18,0x0c,0x00},
  {0x00,0x00,0x7e,0x00,0x7e,0x00,0x00,0x00},
  {0x30,0x18,0x0c,0x06,0x0c,0x18,0x30,0x00},
  {0x3c,0x66,0x06,0x1c,0x18,0x00,0x18,0x00},
  {0x3c,0x66,0x6e,0x6e,0x6e,0x60,0x3c,0x00},
  {0x18,0x3c,0x66,0x66,0x7e,0x66,0x66,0x00},
  {0x7c,0x66,0x66,0x7c,0x66,0x66,0x7c,0x00},
  {0x1c,0x30,0x60,0x60,0x60,0x30,0x1c,0x00},
  {0x78,0x6c,0x66,0x66,0x66,0x6c,0x78,0x00},
  {0x7e,0x60,0x60,0x7c,0x60,0x60,0x7e,0x00},
  {0x7e,0x60,0x60,0x7c,0x60,0x60,0x60,0x00},
  {0x3c,0x66,0x60,0x6e,0x66,0x66,0x3c,0x00},
  {0x66,0x66,0x66,0x7e,0x66,0x66,0x66,0x00},
  {0x7e,0x18,0x18,0x18,0x18,0x18,0x7e,0x00},
  {0x06,0x06,0x06,0x06,0x66,0x66,0x3c,0x00},
  {0x66,0x6c,0x78,0x70,0x78,0x6c,0x66,0x00},
  {0x60,0x60,0x60,0x60,0x60,0x60,0x7e,0x00},
  {0xc6,0xee,0xfe,0xd6,0xc6,0xc6,0xc6,0x00},
  {0x66,0x76,0x7e,0x7e,0x6e,0x66,0x66,0x00},
  {0x3c,0x66,0x66,0x66,0x66,0x66,0x3c,0x00},
  {0x7c,0x66,0x66,0x7c,0x60,0x60,0x60,0x00},
  {0x3c,0x66,0x66,0x66,0x6e,0x3c,0x07,0x00},
  {0x7c,0x66,0x66,0x7c,0x78,0x6c,0x66,0x00},
  {0x3c,0x66,0x60,0x3c,0x06,0x66,0x3c,0x00},
  {0x7e,0x18,0x18,0x18,0x18,0x18,0x18,0x00},
  {0x66,0x66,0x66,0x66,0x66,0x66,0x3c,0x00},
  {0x66,0x66,0x66,0x66,0x66,0x3c,0x18,0x00},
  {0xc6,0xc6,0xc6,0xd6,0xfe,0xee,0xc6,0x00},
  {0x66,0x66,0x3c,0x18,0x3c,0x66,0x66,0x00},
  {0x66,0x66,0x66,0x3c,0x18,0x18,0x18,0x00},
  {0x7e,0x06,0x0c,0x18,0x30,0x60,0x7e,0x00},
  {0x3c,0x30,0x30,0x30,0x30,0x30,0x3c,0x00},
  {0xc0,0x60,0x30,0x18,0x0c,0x06,0x02,0x00},
  {0x3c,0x0c,0x0c,0x0c,0x0c,0x0c,0x3c,0x00},
  {0x18,0x3c,0x66,0x00,0x00,0x00,0x00,0x00},
  {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xff},
  {0x18,0x18,0x0c,0x00,0x00,0x00,0x00,0x00},
  {0x00,0x00,0x3c,0x06,0x3e,0x66,0x3e,0x00},
  {0x60,0x60,0x7c,0x66,0x66,0x66,0x7c,0x00},
  {0x00,0x00,0x3c,0x60,0x60,0x60,0x3c,0x00},
  {0x06,0x06,0x3e,0x66,0x66,0x66,0x3e,0x00},
  {0x00,0x00,0x3c,0x66,0x7e,0x60,0x3c,0x00},
  {0x1c,0x30,0x7c,0x30,0x30,0x30,0x7e,0x00},
  {0x00,0x00,0x3e,0x66,0x66,0x3e,0x06,0x7c},
  {0x60,0x60,0x7c,0x66,0x66,0x66,0x66,0x00},
  {0x18,0x00,0x38,0x18,0x18,0x18,0x3c,0x00},
  {0x18,0x00,0x38,0x18,0x18,0x18,0x18,0x70},
  {0x60,0x60,0x66,0x6c,0x78,0x6c,0x66,0x00},
  {0x38,0x18,0x18,0x18,0x18,0x18,0x3c,0x00},
  {0x00,0x00,0xcc,0xfe,0xd6,0xc6,0xc6,0x00},
  {0x00,0x00,0x7c,0x66,0x66,0x66,0x66,0x00},
  {0x00,0x00,0x3c,0x66,0x66,0x66,0x3c,0x00},
  {0x00,0x00,0x7c,0x66,0x66,0x7c,0x60,0x60},
  {0x00,0x00,0x3e,0x66,0x66,0x3e,0x06,0x06},
  {0x00,0x00,0x7c,0x66,0x60,0x60,0x60,0x00},
  {0x00,0x00,0x3e,0x60,0x3c,0x06,0x7c,0x00},
  {0x30,0x30,0x7c,0x30,0x30,0x30,0x1c,0x00},
  {0x00,0x00,0x66,0x66,0x66,0x66,0x3e,0x00},
  {0x00,0x00,0x66,0x66,0x66,0x3c,0x18,0x00},
  {0x00,0x00,0xc6,0xc6,0xd6,0xfe,0x6c,0x00},
  {0x00,0x00,0x66,0x3c,0x18,0x3c,0x66,0x00},
  {0x00,0x00,0x66,0x66,0x66,0x3e,0x06,0x7c},
  {0x00,0x00,0x7e,0x0c,0x18,0x30,0x7e,0x00},
  {0x0e,0x18,0x18,0x70,0x18,0x18,0x0e,0x00},
  {0x18,0x18,0x18,0x00,0x18,0x18,0x18,0x00},
  {0x70,0x18,0x18,0x0e,0x18,0x18,0x70,0x00},
  {0x76,0xdc,0x00,0x00,0x00,0x00,0x00,0x00},
};

typedef struct {
    uint32_t pixels[WIN_W * WIN_H];
} canvas_t;

static void canvas_clear(canvas_t *c, uint8_t r, uint8_t g, uint8_t b) {
    uint32_t v = 0xFF000000 | (r << 16) | (g << 8) | b;
    for (int i = 0; i < WIN_W * WIN_H; i++) c->pixels[i] = v;
}

static void canvas_put(canvas_t *c, int x, int y, color_t col) {
    if (x < 0 || x >= WIN_W || y < 0 || y >= WIN_H) return;
    c->pixels[y * WIN_W + x] = (0xFF << 24) | (col.r << 16) | (col.g << 8) | col.b;
}

static void canvas_draw_char(canvas_t *c, int x, int y, char ch, color_t fg) {
    if (ch < 32 || ch > 126) ch = '?';
    int idx = ch - 32;
    for (int row = 0; row < 8; row++) {
        uint8_t bits = font8x8[idx][row];
        for (int col = 0; col < 8; col++)
            if (bits & (1 << (7 - col)))
                canvas_put(c, x + col, y + row, fg);
    }
}

static void canvas_str(canvas_t *c, int x, int y, const char *s, color_t fg) {
    while (*s) { canvas_draw_char(c, x, y, *s, fg); x += 8; s++; }
}

static void canvas_line(canvas_t *c, int x0, int y0, int x1, int y1, color_t col) {
    int dx = abs(x1-x0), sx = x0<x1?1:-1;
    int dy = -abs(y1-y0), sy = y0<y1?1:-1;
    int err = dx+dy;
    while (1) {
        canvas_put(c, x0, y0, col);
        if (x0==x1 && y0==y1) break;
        int e2 = 2*err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

static void canvas_rect(canvas_t *c, int x, int y, int w, int h, color_t col) {
    canvas_line(c, x, y, x+w, y, col);
    canvas_line(c, x+w, y, x+w, y+h, col);
    canvas_line(c, x+w, y+h, x, y+h, col);
    canvas_line(c, x, y+h, x, y, col);
}

static void canvas_fill_rect(canvas_t *c, int x0, int y0, int w, int h, color_t col) {
    for (int y = y0; y < y0+h; y++)
        for (int x = x0; x < x0+w; x++)
            canvas_put(c, x, y, col);
}

static void canvas_circle(canvas_t *c, int cx, int cy, int r, color_t col) {
    int x = r, y = 0, err = 0;
    while (x >= y) {
        canvas_put(c, cx+x, cy+y, col); canvas_put(c, cx+y, cy+x, col);
        canvas_put(c, cx-y, cy+x, col); canvas_put(c, cx-x, cy+y, col);
        canvas_put(c, cx-x, cy-y, col); canvas_put(c, cx-y, cy-x, col);
        canvas_put(c, cx+y, cy-x, col); canvas_put(c, cx+x, cy-y, col);
        if (err<=0) { y++; err+=2*y+1; }
        if (err>0)  { x--; err-=2*x+1; }
    }
}

/* ================================================================== */
/*  Flight controller mode                                             */
/* ================================================================== */

typedef enum {
    MODE_PILOT,    /* manual rate control */
    MODE_HOVER,    /* ADRC altitude hold */
    MODE_LAND,     /* descend and land */
} pilot_mode_t;

int main(int argc, char **argv) {
    bool use_adrc = true;  /* ADRC vs PID toggle */
    float target_alt = 2.0f;
    float wind_x = 0.0f, wind_y = 0.0f;
    pilot_mode_t mode = MODE_HOVER;

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--pid")) use_adrc = false;
        if (!strcmp(argv[i], "--adrc")) use_adrc = true;
        if (!strcmp(argv[i], "--alt") && i+1 < argc) target_alt = atof(argv[++i]);
        if (!strcmp(argv[i], "--wind-x") && i+1 < argc) wind_x = atof(argv[++i]);
        if (!strcmp(argv[i], "--wind-y") && i+1 < argc) wind_y = atof(argv[++i]);
        if (!strcmp(argv[i], "--help") || !strcmp(argv[i], "-h")) {
            printf("Usage: quad_sim_visual [options]\n");
            printf("  --adrc        Use ADRC controller (default)\n");
            printf("  --pid         Use PID controller\n");
            printf("  --alt <m>     Target altitude (default: 2.0)\n");
            printf("  --wind-x <v>  Wind speed X m/s (default: 0)\n");
            printf("  --wind-y <v>  Wind speed Y m/s (default: 0)\n");
            printf("\nControls:\n");
            printf("  W/S           Pitch forward/back\n");
            printf("  A/D           Roll left/right\n");
            printf("  Q/E           Yaw left/right\n");
            printf("  UP/DOWN       Throttle up/down\n");
            printf("  SPACE         Toggle hover mode\n");
            printf("  TAB           Switch ADRC/PID\n");
            printf("  R             Reset\n");
            printf("  +/-           Zoom in/out\n");
            printf("  ESC/Q         Quit\n");
            return 0;
        }
    }

    /* Init quadcopter */
    quad_state_t quad;
    quad_init(&quad);
    quad_set_hover(&quad, target_alt);

    quad_adrc_controllers_t adrc;
    quad_adrc_init(&adrc);
    quad_adrc_reset(&adrc);

    quad_pid_controllers_t pid;
    quad_pid_init(&pid);
    quad_pid_reset(&pid);

    /* SDL init */
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        fprintf(stderr, "SDL_Init: %s\n", SDL_GetError());
        return 1;
    }

    SDL_Window *win = SDL_CreateWindow("Quadcopter ADRC Simulator",
        SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, WIN_W, WIN_H,
        SDL_WINDOW_SHOWN);
    if (!win) { fprintf(stderr, "SDL_CreateWindow: %s\n", SDL_GetError()); return 1; }

    SDL_Renderer *ren = SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED);
    SDL_Texture *tex = SDL_CreateTexture(ren, SDL_PIXELFORMAT_ARGB8888,
        SDL_TEXTUREACCESS_STREAMING, WIN_W, WIN_H);

    canvas_t cv;
    bool running = true;
    bool paused = false;

    /* Input state */
    float inp_roll = 0, inp_pitch = 0, inp_yaw = 0, inp_thr = 0;

    /* History for graphs */
    #define HIST_LEN 300
    float hist_alt[HIST_LEN] = {0};
    float hist_roll[HIST_LEN] = {0};
    float hist_motors[4][HIST_LEN] = {{0}};
    int hist_idx = 0;

    quad_wind_t wind = { .velocity = {wind_x, wind_y, 0}, .turbulence = 0.05f };

    while (running) {
        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_QUIT) running = false;
            else if (e.type == SDL_KEYDOWN) {
                switch (e.key.keysym.sym) {
                    case SDLK_ESCAPE: case SDLK_q: running = false; break;
                    case SDLK_SPACE:
                        mode = (mode == MODE_HOVER) ? MODE_PILOT : MODE_HOVER;
                        break;
                    case SDLK_TAB: use_adrc = !use_adrc; break;
                    case SDLK_r:
                        quad_init(&quad);
                        quad_set_hover(&quad, target_alt);
                        quad_adrc_reset(&adrc);
                        quad_pid_reset(&pid);
                        memset(hist_alt, 0, sizeof(hist_alt));
                        memset(hist_roll, 0, sizeof(hist_roll));
                        memset(hist_motors, 0, sizeof(hist_motors));
                        hist_idx = 0;
                        break;
                    default: break;
                }
            }
        }

        /* Continuous key state for flight controls */
        const uint8_t *keys = SDL_GetKeyboardState(NULL);
        inp_pitch = 0; inp_roll = 0; inp_yaw = 0; inp_thr = 0;
        if (keys[SDL_SCANCODE_W]) inp_pitch = -0.5f;
        if (keys[SDL_SCANCODE_S]) inp_pitch = 0.5f;
        if (keys[SDL_SCANCODE_A]) inp_roll = -0.5f;
        if (keys[SDL_SCANCODE_D]) inp_roll = 0.5f;
        if (keys[SDL_SCANCODE_Q]) inp_yaw = -0.5f;
        if (keys[SDL_SCANCODE_E]) inp_yaw = 0.5f;
        if (keys[SDL_SCANCODE_UP]) inp_thr = 0.3f;
        if (keys[SDL_SCANCODE_DOWN]) inp_thr = -0.3f;

        /* Update wind from command line */
        wind.velocity.x = wind_x;
        wind.velocity.y = wind_y;

        /* === Physics + Control === */
        if (!paused) {
            float cmd_roll, cmd_pitch, cmd_yaw, cmd_thr;

            if (mode == MODE_HOVER) {
                if (use_adrc) {
                    quad_adrc_control(&adrc, &quad,
                        inp_roll, inp_pitch, inp_yaw, target_alt,
                        QUAD_SIM_DT, &cmd_roll, &cmd_pitch, &cmd_yaw, &cmd_thr);
                } else {
                    quad_pid_control(&pid, &quad,
                        inp_roll, inp_pitch, inp_yaw, target_alt,
                        QUAD_SIM_DT, &cmd_roll, &cmd_pitch, &cmd_yaw, &cmd_thr);
                }
            } else {
                /* Manual mode: direct rate control */
                cmd_roll = inp_roll;
                cmd_pitch = inp_pitch;
                cmd_yaw = inp_yaw;
                cmd_thr = 0.23f + inp_thr;  /* hover + manual throttle */
                if (cmd_thr < 0) cmd_thr = 0;
                if (cmd_thr > 1) cmd_thr = 1;
            }

            quad_step(&quad, cmd_roll, cmd_pitch, cmd_yaw, cmd_thr, &wind);

            /* Record history */
            hist_alt[hist_idx] = -quad.pos.z;
            hist_roll[hist_idx] = atan2f(2*(quad.quat.w*quad.quat.x+quad.quat.y*quad.quat.z),
                                         1-2*(quad.quat.x*quad.quat.x+quad.quat.y*quad.quat.y));
            for (int m = 0; m < 4; m++)
                hist_motors[m][hist_idx] = quad.motor_rpm[m];
            hist_idx = (hist_idx + 1) % HIST_LEN;
        }

        /* === RENDER === */
        canvas_clear(&cv, 16, 20, 28);

        /* Compute Euler angles */
        float roll_rad = atan2f(2*(quad.quat.w*quad.quat.x+quad.quat.y*quad.quat.z),
                                1-2*(quad.quat.x*quad.quat.x+quad.quat.y*quad.quat.y));
        float pitch_rad = asinf(2*(quad.quat.w*quad.quat.y-quad.quat.z*quad.quat.x));
        float yaw_rad = atan2f(2*(quad.quat.w*quad.quat.z+quad.quat.x*quad.quat.y),
                               1-2*(quad.quat.y*quad.quat.y+quad.quat.z*quad.quat.z));
        float roll_deg = roll_rad * 57.2958f;
        float pitch_deg = pitch_rad * 57.2958f;
        float yaw_deg = yaw_rad * 57.2958f;

        /* ============================================================ */
        /*  LEFT HALF: Artificial Horizon (attitude indicator)           */
        /* ============================================================ */
        int ah_cx = WIN_W / 4, ah_cy = WIN_H / 2 - 10;
        int ah_r = 120;

        /* Clip circle */
        canvas_circle(&cv, ah_cx, ah_cy, ah_r, COLOR_WHITE);
        canvas_circle(&cv, ah_cx, ah_cy, ah_r + 1, COLOR_WHITE);

        /* Sky/Ground split: pitch shifts the horizon line */
        int horizon_y = ah_cy + (int)(pitch_deg * 1.5f);

        for (int y = ah_cy - ah_r; y <= ah_cy + ah_r; y++) {
            for (int x = ah_cx - ah_r; x <= ah_cx + ah_r; x++) {
                int dx = x - ah_cx, dy = y - ah_cy;
                if (dx*dx + dy*dy > ah_r*ah_r) continue;

                /* Apply roll rotation to check if sky or ground */
                float ry = (float)(y - ah_cy);
                float rx = (float)(x - ah_cx);
                float rot_y = rx * sinf(roll_rad) + ry * cosf(roll_rad);

                if (rot_y < horizon_y - ah_cy) {
                    /* Sky (blue) */
                    canvas_put(&cv, x, y, (color_t){50, 100, 180, 255});
                } else {
                    /* Ground (brown) */
                    canvas_put(&cv, x, y, (color_t){120, 80, 40, 255});
                }
            }
        }

        /* Horizon line (white, rolled) */
        {
            float cy_f = (float)(horizon_y - ah_cy);
            float cos_r = cosf(roll_rad), sin_r = sinf(roll_rad);
            float hx = sqrtf((float)(ah_r*ah_r) - cy_f*cy_f);
            if (hx > 0) {
                int x0 = ah_cx + (int)(-hx * cos_r - 0 * sin_r);
                int y0 = ah_cy + (int)(-hx * sin_r + 0 * cos_r + cy_f);
                int x1 = ah_cx + (int)( hx * cos_r - 0 * sin_r);
                int y1 = ah_cy + (int)( hx * sin_r + 0 * cos_r + cy_f);
                canvas_line(&cv, x0, y0, x1, y1, COLOR_WHITE);
            }
        }

        /* Pitch ladder lines (every 10 deg) */
        for (int p = -30; p <= 30; p += 10) {
            if (p == 0) continue;
            float p_y = (float)(p * 1.5f);
            int half_w = 20;
            int lx0 = ah_cx - half_w, lx1 = ah_cx + half_w;
            /* Rotate by roll */
            float ry0 = -half_w * sinf(roll_rad) + 0;
            float ry1 = half_w * sinf(roll_rad) + 0;
            int py0 = ah_cy + (int)(p_y + ry0);
            int py1 = ah_cy + (int)(p_y + ry1);
            canvas_line(&cv, lx0, py0, lx1, py1, COLOR_WHITE);
        }

        /* Fixed aircraft reference (center cross) */
        canvas_line(&cv, ah_cx - 30, ah_cy, ah_cx - 10, ah_cy, COLOR_YELLOW);
        canvas_line(&cv, ah_cx + 10, ah_cy, ah_cx + 30, ah_cy, COLOR_YELLOW);
        canvas_line(&cv, ah_cx, ah_cy - 8, ah_cx, ah_cy - 2, COLOR_YELLOW);
        canvas_line(&cv, ah_cx, ah_cy + 2, ah_cx, ah_cy + 8, COLOR_YELLOW);

        /* Attitude text */
        {
            char buf[64];
            snprintf(buf, sizeof(buf), "ROLL %+6.1f", (double)roll_deg);
            canvas_str(&cv, ah_cx - 32, ah_cy + ah_r + 8, buf, COLOR_CYAN);
            snprintf(buf, sizeof(buf), "PITCH%+6.1f", (double)pitch_deg);
            canvas_str(&cv, ah_cx - 32, ah_cy + ah_r + 18, buf, COLOR_CYAN);
            snprintf(buf, sizeof(buf), "YAW  %+6.1f", (double)yaw_deg);
            canvas_str(&cv, ah_cx - 32, ah_cy + ah_r + 28, buf, COLOR_YELLOW);
        }

        /* ============================================================ */
        /*  RIGHT HALF: 3D Quadcopter (isometric, tilted with attitude)  */
        /* ============================================================ */
        int qcx = WIN_W * 3 / 4, qcy = WIN_H / 2 - 10;

        /* Camera: looking down at ~30 degrees, rotated by yaw */
        float cam_pitch = -0.5f;  /* radians, looking slightly down */
        float cx_ = cosf(yaw_rad), sy_ = sinf(yaw_rad);
        float cp = cosf(cam_pitch), sp = sinf(cam_pitch);

        /* Ground grid (3D → projected) */
        for (int i = -4; i <= 4; i++) {
            for (int j = -4; j <= 4; j++) {
                float gx = i * 30.0f, gy = j * 30.0f, gz = 0;
                /* Rotate by yaw */
                float rx = gx * cx_ - gy * sy_;
                float ry = gx * sy_ + gy * cx_;
                /* Camera transform (pitch + scale) */
                float depth = ry * sp + gz * cp;
                float scale = 400.0f / (400.0f + depth);
                int sx = qcx + (int)(rx * scale);
                int sy = qcy + (int)((ry * cp - gz * sp) * scale);
                canvas_put(&cv, sx, sy, COLOR_DKGRAY);
            }
        }

        /* Altitude reference lines on ground */
        float alt_m = -quad.pos.z;
        float alt_px = alt_m * 40.0f;

        /* Shadow on ground */
        {
            float depth = 0;
            float scale = 400.0f / (400.0f + depth);
            int sx = qcx + (int)(quad.pos.x * 40.0f * scale);
            int sy = qcy + (int)(quad.pos.y * 40.0f * scale);
            canvas_circle(&cv, sx, sy, 8, (color_t){40, 40, 40, 255});
        }

        /* Quad position in world (relative to origin) */
        float wx = quad.pos.x * 40.0f;
        float wy = quad.pos.y * 40.0f;
        float wz = -alt_px;  /* NED: up is negative z */

        /* Rotate world by camera yaw */
        float wcx = wx * cx_ - wy * sy_;
        float wcy = wx * sy_ + wy * cx_;
        float wcz = wz;

        /* Camera projection */
        float depth_q = wcy * sp + wcz * cp;
        float scale_q = 400.0f / (400.0f + depth_q);
        int qx_px = qcx + (int)(wcx * scale_q);
        int qy_px = qcy + (int)((wcy * cp - wcz * sp) * scale_q);

        /* Draw altitude line from ground to quad */
        {
            float depth_g = 0;
            float scale_g = 400.0f / (400.0f + depth_g);
            int gnd_x = qcx + (int)(wcx * scale_g);
            int gnd_y = qcy + (int)(wcy * scale_g);
            canvas_line(&cv, gnd_x, gnd_y, qx_px, qy_px, COLOR_DKGRAY);
            char buf[32];
            snprintf(buf, sizeof(buf), "%.1fm", (double)alt_m);
            canvas_str(&cv, gnd_x + 4, (gnd_y + qy_px) / 2, buf, COLOR_YELLOW);
        }

        /* Draw quad body (3D rotation by attitude) */
        float arm_len = 50.0f * scale_q;
        /* Motor positions in body frame: + config */
        float bx[4] = {0, arm_len, 0, -arm_len};
        float by[4] = {-arm_len, 0, arm_len, 0};

        /* Apply attitude rotation (roll, pitch, yaw) to body vectors */
        float cr = cosf(roll_rad), sr = sinf(roll_rad);
        float cp2 = cosf(pitch_rad), sp2 = sinf(pitch_rad);

        for (int m = 0; m < 4; m++) {
            float lx = bx[m], ly = by[m], lz = 0;

            /* Roll (around X) */
            float ry1 = ly * cr - lz * sr;
            float rz1 = ly * sr + lz * cr;
            ly = ry1; lz = rz1;

            /* Pitch (around Y) */
            float rx2 = lx * cp2 + lz * sp2;
            float rz2 = -lx * sp2 + lz * cp2;
            lx = rx2; lz = rz2;

            /* Yaw (around Z) */
            float rx3 = lx * cx_ - ly * sy_;
            float ry3 = lx * sy_ + ly * cx_;
            lx = rx3; ly = ry3;

            int mx = qx_px + (int)lx;
            int my = qy_px + (int)ly;

            /* Arm from center to motor */
            canvas_line(&cv, qx_px, qy_px, mx, my, COLOR_GRAY);

            /* Motor disc (size proportional to throttle) */
            int msize = 4 + (int)(quad.motor_rpm[m] * 12);
            canvas_circle(&cv, mx, my, msize, COLOR_WHITE);

            /* Thrust indicator (colored by RPM) */
            color_t mcol = quad.motor_rpm[m] > 0.8f ? COLOR_RED :
                           quad.motor_rpm[m] > 0.3f ? COLOR_GREEN : COLOR_GRAY;
            canvas_circle(&cv, mx, my, msize - 1, mcol);

            /* Motor label */
            char mbuf[8];
            snprintf(mbuf, sizeof(mbuf), "M%d", m);
            canvas_str(&cv, mx - 4, my + msize + 2, mbuf, COLOR_GRAY);
        }

        /* Body center */
        canvas_fill_rect(&cv, qx_px - 4, qy_px - 4, 8, 8, COLOR_WHITE);

        /* Heading arrow (nose direction) */
        float nose_len = 30.0f * scale_q;
        float nx = -sinf(yaw_rad) * nose_len;
        float ny = -cosf(yaw_rad) * nose_len;
        canvas_line(&cv, qx_px, qy_px, qx_px + (int)nx, qy_px + (int)ny, COLOR_GREEN);

        /* ============================================================ */
        /*  BOTTOM: Telemetry graphs                                     */
        /* ============================================================ */
        int gx = 4, gy = WIN_H - 100, gw = WIN_W - 8, gh = 90;
        canvas_rect(&cv, gx, gy, gw, gh, COLOR_DKGRAY);

        /* Grid lines */
        for (int i = 1; i < 4; i++) {
            int gy2 = gy + (gh * i) / 4;
            canvas_line(&cv, gx, gy2, gx+gw, gy2, COLOR_DKGRAY);
        }

        /* Altitude graph (green) */
        canvas_str(&cv, gx + 2, gy + 2, "ALT(m)", COLOR_GREEN);
        for (int i = 1; i < HIST_LEN; i++) {
            int idx0 = (hist_idx - i - 1 + HIST_LEN) % HIST_LEN;
            int idx1 = (hist_idx - i + HIST_LEN) % HIST_LEN;
            float a0 = hist_alt[idx0], a1 = hist_alt[idx1];
            int y0 = gy + gh - (int)(a0 / 6.0f * gh);
            int y1 = gy + gh - (int)(a1 / 6.0f * gh);
            int x0 = gx + gw - i * (gw-2) / HIST_LEN;
            int x1 = x0 + (gw-2) / HIST_LEN;
            canvas_line(&cv, x0, y0, x1, y1, COLOR_GREEN);
        }

        /* Target altitude line */
        int tgt_y = gy + gh - (int)(target_alt / 6.0f * gh);
        if (tgt_y >= gy && tgt_y <= gy+gh)
            for (int x = gx; x < gx+gw; x += 4)
                canvas_put(&cv, x, tgt_y, COLOR_YELLOW);

        /* Roll graph (cyan, scaled) */
        canvas_str(&cv, gx + 50, gy + 2, "ROLL(deg)", COLOR_CYAN);
        for (int i = 1; i < HIST_LEN; i++) {
            int idx0 = (hist_idx - i - 1 + HIST_LEN) % HIST_LEN;
            int idx1 = (hist_idx - i + HIST_LEN) % HIST_LEN;
            float r0 = hist_roll[idx0] * 57.3f, r1 = hist_roll[idx1] * 57.3f;
            int y0 = gy + gh/2 - (int)(r0 / 30.0f * gh/2);
            int y1 = gy + gh/2 - (int)(r1 / 30.0f * gh/2);
            int x0 = gx + gw - i * (gw-2) / HIST_LEN;
            int x1 = x0 + (gw-2) / HIST_LEN;
            canvas_line(&cv, x0, y0, x1, y1, COLOR_CYAN);
        }

        /* Zero line for roll */
        canvas_line(&cv, gx, gy + gh/2, gx+gw, gy + gh/2, COLOR_DKGRAY);

        /* Motor traces (dimmer) */
        color_t mcols[4] = {{255,100,100,255}, {100,255,100,255}, {100,100,255,255}, {255,255,100,255}};
        for (int m = 0; m < 4; m++) {
            for (int i = 1; i < HIST_LEN; i++) {
                int idx0 = (hist_idx - i - 1 + HIST_LEN) % HIST_LEN;
                int idx1 = (hist_idx - i + HIST_LEN) % HIST_LEN;
                float m0 = hist_motors[m][idx0], m1 = hist_motors[m][idx1];
                int y0 = gy + gh - (int)(m0 * gh);
                int y1 = gy + gh - (int)(m1 * gh);
                int x0 = gx + gw - i * (gw-2) / HIST_LEN;
                int x1 = x0 + (gw-2) / HIST_LEN;
                canvas_line(&cv, x0, y0, x1, y1, mcols[m]);
            }
        }

        /* ============================================================ */
        /*  RIGHT SIDE: Telemetry panel                                  */
        /* ============================================================ */
        int lx = WIN_W / 2 + 8, ly = 4;
        color_t mode_col = use_adrc ? COLOR_CYAN : COLOR_ORANGE;
        canvas_str(&cv, lx, ly, use_adrc ? "ADRC" : "PID", mode_col);
        canvas_str(&cv, lx + 40, ly, mode == MODE_HOVER ? "HOVER" : "MANUAL", COLOR_GREEN);
        ly += 12;

        char buf[128];
        snprintf(buf, sizeof(buf), "Alt:  %5.1fm / %.1fm", (double)alt_m, (double)target_alt);
        canvas_str(&cv, lx, ly, buf, COLOR_YELLOW); ly += 10;
        snprintf(buf, sizeof(buf), "Gyro: %+6.1f %+6.1f %+6.1f",
                 (double)(quad.omega.x*57.3), (double)(quad.omega.y*57.3), (double)(quad.omega.z*57.3));
        canvas_str(&cv, lx, ly, buf, COLOR_GRAY); ly += 10;
        snprintf(buf, sizeof(buf), "Vel:  %+5.2f %+5.2f %+5.2f",
                 (double)quad.vel.x, (double)quad.vel.y, (double)quad.vel.z);
        canvas_str(&cv, lx, ly, buf, COLOR_GRAY); ly += 12;

        /* Motors */
        canvas_str(&cv, lx, ly, "MOTORS", COLOR_WHITE); ly += 10;
        for (int m = 0; m < 4; m++) {
            int bar_w = 50;
            int filled = (int)(quad.motor_rpm[m] * bar_w);
            color_t mcol = quad.motor_rpm[m] > 0.8f ? COLOR_RED :
                           quad.motor_rpm[m] > 0.3f ? COLOR_GREEN : COLOR_GRAY;
            canvas_rect(&cv, lx, ly, bar_w, 5, COLOR_DKGRAY);
            canvas_fill_rect(&cv, lx, ly, filled, 5, mcol);
            char mbuf[16];
            snprintf(mbuf, sizeof(mbuf), "M%d %.0f%%", m, quad.motor_rpm[m]*100);
            canvas_str(&cv, lx + bar_w + 3, ly - 1, mbuf, mcol);
            ly += 9;
        }
        ly += 4;

        /* ESO states (ADRC only) */
        if (use_adrc) {
            canvas_str(&cv, lx, ly, "ESO", COLOR_CYAN); ly += 10;
            float z1, z2, z3;
            adrc_get_states(&adrc.roll_adrc, &z1, &z2, &z3);
            snprintf(buf, sizeof(buf), "R:%+6.1f %+5.1f %+5.1f", (double)z1, (double)z2, (double)z3);
            canvas_str(&cv, lx, ly, buf, COLOR_CYAN); ly += 9;
            adrc_get_states(&adrc.alt_adrc, &z1, &z2, &z3);
            snprintf(buf, sizeof(buf), "A:%+6.1f %+5.1f %+5.1f", (double)z1, (double)z2, (double)z3);
            canvas_str(&cv, lx, ly, buf, COLOR_CYAN); ly += 12;
        }

        /* Time + wind */
        snprintf(buf, sizeof(buf), "t=%.1fs  w=(%.1f,%.1f)", (double)quad.time_s, (double)wind_x, (double)wind_y);
        canvas_str(&cv, lx, ly, buf, COLOR_GRAY); ly += 10;

        /* Controls */
        canvas_str(&cv, lx, ly, "W/S Pitch A/D Roll", COLOR_GRAY); ly += 9;
        canvas_str(&cv, lx, ly + 9, "Q/E Yaw ^v Throttle", COLOR_GRAY); ly += 20;
        canvas_str(&cv, lx, ly, "TAB ADRC/PID SPACE Mode", COLOR_GRAY); ly += 9;
        canvas_str(&cv, lx, ly + 9, "+/- Zoom R Reset", COLOR_GRAY);

        /* Status bar */
        canvas_str(&cv, 4, WIN_H - 12, "ESC:Quit", COLOR_GRAY);

        /* Update texture */
        SDL_UpdateTexture(tex, NULL, cv.pixels, WIN_W * 4);
        SDL_RenderClear(ren);
        SDL_Rect dst = {0, 0, WIN_W, WIN_H};
        SDL_RenderCopy(ren, tex, NULL, &dst);
        SDL_RenderPresent(ren);

        SDL_Delay(16);  /* ~60 FPS */
    }

    SDL_DestroyTexture(tex);
    SDL_DestroyRenderer(ren);
    SDL_DestroyWindow(win);
    SDL_Quit();
    return 0;
}
