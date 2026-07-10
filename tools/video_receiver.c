#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <SDL2/SDL.h>

#define W 320
#define H 240
#define SCALE 2
#define WIN_W (W * SCALE)
#define WIN_H (H * SCALE)

#define HEADER_SIZE 64
#define FRAME_SIZE (W * H)
#define BUF_SIZE (HEADER_SIZE + FRAME_SIZE)

static volatile int g_running = 1;
static void handle_signal(int sig) { (void)sig; g_running = 0; }

static void put_pixel(uint32_t *pixels, int x, int y, uint32_t color) {
  if (x >= 0 && x < W && y >= 0 && y < H) pixels[y * W + x] = color;
}

int main(int argc, char **argv) {
  int port = argc > 1 ? atoi(argv[1]) : 9090;

  signal(SIGINT, handle_signal);
  signal(SIGTERM, handle_signal);

  int fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0) { perror("socket"); return 1; }
  struct sockaddr_in addr;
  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  addr.sin_addr.s_addr = INADDR_ANY;
  if (bind(fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
    perror("bind"); return 1;
  }
  struct timeval tv = { .tv_sec = 0, .tv_usec = 100000 };
  setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  if (SDL_Init(SDL_INIT_VIDEO) < 0) {
    fprintf(stderr, "SDL_Init: %s\n", SDL_GetError());
    return 1;
  }
  SDL_Window *win = SDL_CreateWindow("Drone Video Receiver",
    SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, WIN_W, WIN_H, 0);
  SDL_Renderer *ren = SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED);
  SDL_Texture *tex = SDL_CreateTexture(ren, SDL_PIXELFORMAT_ARGB8888,
    SDL_TEXTUREACCESS_STREAMING, W, H);

  uint8_t *buf = malloc(BUF_SIZE);
  uint32_t *pixels = malloc(W * H * sizeof(uint32_t));
  int frame_count = 0;

  printf("Listening on UDP port %d...\n", port);
  printf("Start test_stream on drone: /data/video/test_stream /dev/video0 <host-ip> 9090\n");

  while (g_running) {
    SDL_Event e;
    while (SDL_PollEvent(&e)) {
      if (e.type == SDL_QUIT) g_running = 0;
      if (e.type == SDL_KEYDOWN && e.key.keysym.sym == SDLK_ESCAPE) g_running = 0;
    }

    socklen_t from_len = sizeof(addr);
    ssize_t n = recvfrom(fd, buf, BUF_SIZE, 0,
                          (struct sockaddr*)&addr, &from_len);
    if (n < 0) continue;

    /* Parse header */
    int frame_num = ntohl(*(uint32_t*)buf);
    int flow_x = (int)ntohl(*(uint32_t*)(buf + 4)) / 10;
    int flow_y = (int)ntohl(*(uint32_t*)(buf + 8)) / 10;
    int quality = buf[12];
    int looming = buf[13];
    int asymmetry = (int8_t)buf[14];
    int confidence = buf[15];
    int line_found = buf[16];
    int line_x = buf[17] | (buf[18] << 8);
    uint8_t *gray = buf + HEADER_SIZE;

    /* Convert grayscale to ARGB */
    for (int i = 0; i < FRAME_SIZE; i++)
      pixels[i] = 0xFF000000 | (gray[i] << 16) | (gray[i] << 8) | gray[i];

    /* Looming bar (top center) */
    int bar_w = 80, bar_h = 8;
    int bar_x = (W - bar_w) / 2;
    uint32_t loom_col = looming < 80 ? 0xFF00FF00 :
                         looming < 160 ? 0xFFFFFF00 : 0xFFFF0000;
    int filled = (looming * bar_w) / 255;
    if (filled > bar_w) filled = bar_w;
    for (int y = 1; y <= bar_h; y++)
      for (int x = bar_x; x < bar_x + bar_w; x++)
        put_pixel(pixels, x, y, (x - bar_x) < filled ? loom_col : 0xFF333333);

    /* Asymmetry arrow (below bar) */
    int asym_mag = abs(asymmetry);
    if (asym_mag > 5) {
      int ax = W / 2, ay = bar_h + 8;
      int len = (asym_mag * 20) / 128;
      if (len < 3) len = 3;
      int dir = asymmetry < 0 ? -1 : 1;
      put_pixel(pixels, ax, ay, 0xFF00FFFF);
      for (int i = 0; i < len; i++) {
        put_pixel(pixels, ax + dir * i, ay, 0xFF00FFFF);
        put_pixel(pixels, ax + dir * i, ay - 1, 0xFF00FFFF);
        put_pixel(pixels, ax + dir * i, ay + 1, 0xFF00FFFF);
      }
    }

    /* Vertical line */
    if (line_found)
      for (int y = 0; y < H; y++)
        put_pixel(pixels, line_x, y, 0xFFFFFF00);

    /* Status text via window title */
    char title[128];
    snprintf(title, sizeof(title),
             "F%d FX%d FY%d Q%d L%d A%d C%d",
             frame_num, flow_x, flow_y, quality,
             looming, asymmetry, confidence);
    SDL_SetWindowTitle(win, title);

    SDL_UpdateTexture(tex, NULL, pixels, W * 4);
    SDL_RenderClear(ren);
    SDL_Rect dst = {0, 0, WIN_W, WIN_H};
    SDL_RenderCopy(ren, tex, NULL, &dst);
    SDL_RenderPresent(ren);

    frame_count++;
  }

  printf("\nReceived %d frames\n", frame_count);
  free(buf); free(pixels);
  SDL_DestroyTexture(tex);
  SDL_DestroyRenderer(ren);
  SDL_DestroyWindow(win);
  SDL_Quit();
  close(fd);
  return 0;
}
