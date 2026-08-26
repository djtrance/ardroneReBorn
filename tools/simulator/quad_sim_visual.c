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

/* Simple 3D projection: isometric-like view */
typedef struct { float x, y, z; } vec3f_t;

static vec3f_t project(float x, float y, float z, float cam_dist) {
    float scale = cam_dist / (cam_dist + z);
    return (vec3f_t){ x * scale, y * scale, z };
}

static void draw_line3d(canvas_t *c, vec3f_t a, vec3f_t b, color_t col) {
    canvas_line(c, (int)a.x, (int)a.y, (int)b.x, (int)b.y, col);
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
    float cam_dist = 600.0f;
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
                    case SDLK_EQUALS: case SDLK_PLUS: cam_dist += 50; break;
                    case SDLK_MINUS: if (cam_dist > 200) cam_dist -= 50; break;
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

        /* Title */
        color_t mode_col = use_adrc ? COLOR_CYAN : COLOR_ORANGE;
        canvas_str(&cv, 4, 4, use_adrc ? "CONTROLLER: ADRC" : "CONTROLLER: PID", mode_col);
        canvas_str(&cv, 4, 14, mode == MODE_HOVER ? "MODE: HOVER (auto)" : "MODE: PILOT (manual)", COLOR_GREEN);

        /* === 3D Quadcopter View (right panel) === */
        int qcx = WIN_W * 2 / 3, qcy = WIN_H / 2;

        /* Draw ground grid */
        for (int i = -3; i <= 3; i++) {
            float gz = 0;
            vec3f_t a = project(qcx + i*50, qcy + 80, gz, cam_dist);
            vec3f_t b = project(qcx + i*50, qcy - 120, gz, cam_dist);
            draw_line3d(&cv, a, b, COLOR_DKGRAY);
        }
        for (int i = -3; i <= 3; i++) {
            float gz = 0;
            vec3f_t a = project(qcx - 150, qcy + 80 + i*40, gz, cam_dist);
            vec3f_t b = project(qcx + 150, qcy + 80 + i*40, gz, cam_dist);
            draw_line3d(&cv, a, b, COLOR_DKGRAY);
        }

        /* Altitude line */
        float alt_px = -quad.pos.z * 60;  /* scale: 1m = 60px */
        vec3f_t gnd = project(qcx, qcy + 80, 0, cam_dist);
        vec3f_t hov = project(qcx, qcy + 80 - alt_px, 0, cam_dist);
        draw_line3d(&cv, gnd, hov, COLOR_DKGRAY);
        char alt_buf[32];
        snprintf(alt_buf, sizeof(alt_buf), "%.1fm", -quad.pos.z);
        canvas_str(&cv, (int)hov.x + 4, (int)hov.y - 4, alt_buf, COLOR_YELLOW);

        /* Draw quadcopter body */
        float qx = qcx, qy = qcy + 80 - alt_px;

        /* Compute motor positions from quaternion */
        float qw = quad.quat.w, qxx = quad.quat.x, qy2 = quad.quat.y, qz = quad.quat.z;
        float arm_len = 60;
        /* Motor offsets in body frame (+ config): M0=front, M1=right, M2=rear, M3=left */
        float mx_body[4] = {0, arm_len, 0, -arm_len};
        float my_body[4] = {-arm_len, 0, arm_len, 0};

        /* Rotate by quaternion */
        for (int m = 0; m < 4; m++) {
            float bx = mx_body[m], by = my_body[m];
            /* Quaternion rotation: v' = q * v * q^-1 */
            float rx = (1-2*(qy2*qy2+qz*qz))*bx + 2*(qxx*qy2-qz*qw)*by;
            float ry = 2*(qxx*qy2+qz*qw)*bx + (1-2*(qxx*qxx+qz*qz))*by;

            vec3f_t p = project(qx + rx, qy + ry, 0, cam_dist);
            vec3f_t center = project(qx, qy, 0, cam_dist);

            /* Arm */
            color_t arm_col = COLOR_GRAY;
            draw_line3d(&cv, center, p, arm_col);

            /* Motor disc (size = throttle) */
            int msize = 4 + (int)(quad.motor_rpm[m] * 10);
            canvas_circle(&cv, (int)p.x, (int)p.y, msize, COLOR_WHITE);

            /* Motor label */
            char mbuf[8];
            snprintf(mbuf, sizeof(mbuf), "M%d", m);
            canvas_str(&cv, (int)p.x - 4, (int)p.y + msize + 2, mbuf, COLOR_GRAY);
        }

        /* Body center */
        canvas_fill_rect(&cv, (int)qx - 6, (int)qy - 3, 12, 6, COLOR_WHITE);

        /* Heading indicator (nose direction) */
        float nx = (1-2*(qy2*qy2+qz*qz))*0 + 2*(qxx*qy2-qz*qw)*(-20);
        float ny = 2*(qxx*qy2+qz*qw)*0 + (1-2*(qxx*qxx+qz*qz))*(-20);
        vec3f_t nose = project(qx + nx, qy + ny, 0, cam_dist);
        vec3f_t ctr = project(qx, qy, 0, cam_dist);
        draw_line3d(&cv, ctr, nose, COLOR_GREEN);

        /* === Left panel: Telemetry === */
        int lx = 4, ly = 30;
        canvas_str(&cv, lx, ly, "=== TELEMETRY ===", COLOR_WHITE); ly += 12;

        char buf[128];
        snprintf(buf, sizeof(buf), "Alt:  %6.2f m  (target: %.1f)", (double)(-quad.pos.z), (double)target_alt);
        canvas_str(&cv, lx, ly, buf, COLOR_YELLOW); ly += 10;

        float roll_deg = atan2f(2*(quad.quat.w*quad.quat.x+quad.quat.y*quad.quat.z),
                                1-2*(quad.quat.x*quad.quat.x+quad.quat.y*quad.quat.y)) * 57.2958f;
        float pitch_deg = asinf(2*(quad.quat.w*quad.quat.y-quad.quat.z*quad.quat.x)) * 57.2958f;
        float yaw_deg = atan2f(2*(quad.quat.w*quad.quat.z+quad.quat.x*quad.quat.y),
                               1-2*(quad.quat.y*quad.quat.y+quad.quat.z*quad.quat.z)) * 57.2958f;

        snprintf(buf, sizeof(buf), "Roll: %6.1f deg", (double)roll_deg);
        canvas_str(&cv, lx, ly, buf, COLOR_WHITE); ly += 10;
        snprintf(buf, sizeof(buf), "Pitch:%6.1f deg", (double)pitch_deg);
        canvas_str(&cv, lx, ly, buf, COLOR_WHITE); ly += 10;
        snprintf(buf, sizeof(buf), "Yaw:  %6.1f deg", (double)yaw_deg);
        canvas_str(&cv, lx, ly, buf, COLOR_WHITE); ly += 10;

        snprintf(buf, sizeof(buf), "Gyro: %+6.1f %+6.1f %+6.1f deg/s",
                 (double)(quad.omega.x*57.3), (double)(quad.omega.y*57.3), (double)(quad.omega.z*57.3));
        canvas_str(&cv, lx, ly, buf, COLOR_GRAY); ly += 10;

        snprintf(buf, sizeof(buf), "Pos:  %+6.2f %+6.2f m (NED)",
                 (double)quad.pos.x, (double)quad.pos.y);
        canvas_str(&cv, lx, ly, buf, COLOR_GRAY); ly += 10;

        snprintf(buf, sizeof(buf), "Vel:  %+5.2f %+5.2f %+5.2f m/s",
                 (double)quad.vel.x, (double)quad.vel.y, (double)quad.vel.z);
        canvas_str(&cv, lx, ly, buf, COLOR_GRAY); ly += 14;

        /* Motor output */
        canvas_str(&cv, lx, ly, "=== MOTORS ===", COLOR_WHITE); ly += 12;
        for (int m = 0; m < 4; m++) {
            int bar_w = 60;
            int filled = (int)(quad.motor_rpm[m] * bar_w);
            color_t mcol = quad.motor_rpm[m] > 0.8f ? COLOR_RED :
                           quad.motor_rpm[m] > 0.3f ? COLOR_GREEN : COLOR_GRAY;
            canvas_rect(&cv, lx, ly, bar_w, 6, COLOR_DKGRAY);
            canvas_fill_rect(&cv, lx, ly, filled, 6, mcol);
            char mbuf[16];
            snprintf(mbuf, sizeof(mbuf), "M%d %.0f%%", m, quad.motor_rpm[m]*100);
            canvas_str(&cv, lx + bar_w + 4, ly - 1, mbuf, mcol);
            ly += 10;
        }
        ly += 4;

        /* ESO states (ADRC only) */
        if (use_adrc) {
            canvas_str(&cv, lx, ly, "=== ESO (ADRC) ===", COLOR_CYAN); ly += 12;
            float z1, z2, z3;
            adrc_get_states(&adrc.roll_adrc, &z1, &z2, &z3);
            snprintf(buf, sizeof(buf), "Roll  z1=%+7.1f z2=%+6.1f z3=%+6.1f", (double)z1, (double)z2, (double)z3);
            canvas_str(&cv, lx, ly, buf, COLOR_CYAN); ly += 10;
            adrc_get_states(&adrc.alt_adrc, &z1, &z2, &z3);
            snprintf(buf, sizeof(buf), "Alt   z1=%+7.1f z2=%+6.1f z3=%+6.1f", (double)z1, (double)z2, (double)z3);
            canvas_str(&cv, lx, ly, buf, COLOR_CYAN); ly += 10;
        }

        ly += 4;

        /* Time */
        snprintf(buf, sizeof(buf), "Time: %.2fs  Steps: %u", (double)quad.time_s, quad.step_count);
        canvas_str(&cv, lx, ly, buf, COLOR_GRAY); ly += 10;

        /* Wind */
        snprintf(buf, sizeof(buf), "Wind: %+5.1f %+5.1f m/s", (double)wind_x, (double)wind_y);
        canvas_str(&cv, lx, ly, buf, COLOR_GRAY); ly += 14;

        /* Controls */
        canvas_str(&cv, lx, ly, "=== CONTROLS ===", COLOR_WHITE); ly += 12;
        snprintf(buf, sizeof(buf), "W/S:Pitch A/D:Roll Q/E:Yaw");
        canvas_str(&cv, lx, ly, buf, COLOR_GRAY); ly += 10;
        snprintf(buf, sizeof(buf), "UP/DN:Throttle SPACE:Mode");
        canvas_str(&cv, lx, ly, buf, COLOR_GRAY); ly += 10;
        snprintf(buf, sizeof(buf), "TAB:Switch +/-:Zoom R:Reset");
        canvas_str(&cv, lx, ly, buf, COLOR_GRAY); ly += 14;

        /* === Bottom: Telemetry graphs === */
        int gx = 4, gy = WIN_H - 90, gw = WIN_W - 8, gh = 80;
        canvas_rect(&cv, gx, gy, gw, gh, COLOR_DKGRAY);

        /* Grid lines */
        for (int i = 1; i < 4; i++) {
            int gy2 = gy + (gh * i) / 4;
            canvas_line(&cv, gx, gy2, gx+gw, gy2, COLOR_DKGRAY);
        }

        /* Altitude graph (green) */
        canvas_str(&cv, gx + 2, gy + 2, "ALT", COLOR_GREEN);
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
        if (tgt_y >= gy && tgt_y <= gy+gh) {
            for (int x = gx; x < gx+gw; x += 4)
                canvas_put(&cv, x, tgt_y, COLOR_YELLOW);
        }

        /* Roll graph (cyan, scaled) */
        canvas_str(&cv, gx + 40, gy + 2, "ROLL", COLOR_CYAN);
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

        /* Status bar */
        canvas_str(&cv, 4, WIN_H - 12, "ESC:Quit SPACE:Mode TAB:ADRC/PID +/-:Zoom R:Reset",
                   COLOR_GRAY);

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
