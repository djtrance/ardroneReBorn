CC = gcc
CFLAGS = -Wall -Wextra -O2 -g -I../../src/vision
LDFLAGS = -lm

SDL_CFLAGS = $(shell pkg-config --cflags sdl2)
SDL_LDFLAGS = $(shell pkg-config --libs sdl2)

COMMON_SRC = \
    ../../src/vision/image.c \
    ../../src/vision/flow_stage1.c \
    ../../src/vision/flow_stage2.c \
    ../../src/vision/fast_detect.c \
    ../../src/vision/lk_flow.c \
    ../../src/vision/rotation.c \
    ../../src/vision/pattern.c \
    ../../src/vision/obstacle.c

VO_SRC = ../../src/vision/visual_odometry.c ../../src/vision/image.c ../../src/vision/fast_detect.c ../../src/vision/lk_flow.c ../../src/vision/rotation.c
LINE_SRC = ../../src/vision/line_detect.c ../../src/vision/image.c

SRC = sim_main.c $(COMMON_SRC)
STAGE2_SRC = test_stage2.c $(COMMON_SRC)
STAGE34_SRC = test_stage34.c $(COMMON_SRC)
OBSTACLE_SRC = test_obstacle.c $(COMMON_SRC)
VO_SRC_TEST = test_visual_odometry.c $(VO_SRC)
LINE_SRC_TEST = test_line_detect.c $(LINE_SRC)

sim_main: $(SRC:.c=.o)
	$(CC) -o $@ $(SRC:.c=.o) $(LDFLAGS)

test_stage2: $(STAGE2_SRC:.c=.o)
	$(CC) -o $@ $(STAGE2_SRC:.c=.o) $(LDFLAGS)

test_stage34: $(STAGE34_SRC:.c=.o)
	$(CC) -o $@ $(STAGE34_SRC:.c=.o) $(LDFLAGS)

test_obstacle: $(OBSTACLE_SRC:.c=.o)
	$(CC) -o $@ $(OBSTACLE_SRC:.c=.o) $(LDFLAGS)

test_visual_odometry: $(VO_SRC_TEST:.c=.o)
	$(CC) -o $@ $(VO_SRC_TEST:.c=.o) $(LDFLAGS)

test_line_detect: $(LINE_SRC_TEST:.c=.o)
	$(CC) -o $@ $(LINE_SRC_TEST:.c=.o) $(LDFLAGS)

VISUAL_SRC = visual_sim.c $(COMMON_SRC) ../../src/vision/line_detect.c

visual_sim: $(VISUAL_SRC:.c=.o)
	$(CC) -o $@ $(VISUAL_SRC:.c=.o) $(LDFLAGS) $(SDL_LDFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) $(SDL_CFLAGS) -c -o $@ $<

# Stage 1 tests
test_synthetic: sim_main
	./sim_main --synthetic --frames 50 --flow-x 2 --flow-y 1

test_hover: sim_main
	./sim_main --synthetic --frames 50 --flow-x 0 --flow-y 0

# Stage 2 tests
test_stage2_run: test_stage2
	./test_stage2

# Stage 3+4 tests
test_stage34_run: test_stage34
	./test_stage34

# Visual odometry tests
test_vo_run: test_visual_odometry
	./test_visual_odometry

# Line detection tests
test_line_run: test_line_detect
	./test_line_detect

# Generate synthetic test sequences
generate_tests:
	python3 test_sequences/generate_synthetic.py

# Run visual simulator
run_visual: visual_sim
	./visual_sim --flow-x 20 --flow-y 10

run_rot: visual_sim
	./visual_sim --yaw 1.0 --no-pad

run_descent: visual_sim
	./visual_sim --descent 0.005 --no-pad

run_obstacle: visual_sim
	./visual_sim --obstacle --scene wall --approach 0.003 --flow-x 0 --flow-y 0 --no-pad

# Video receiver for drone stream
VIDEO_RECV_SRC = ../video_receiver.c
VIDEO_RECV_OBJ = video_receiver.o

video_receiver: $(VIDEO_RECV_OBJ)
	$(CC) -o $@ $(VIDEO_RECV_OBJ) $(LDFLAGS) $(SDL_LDFLAGS)

$(VIDEO_RECV_OBJ): $(VIDEO_RECV_SRC)
	$(CC) $(CFLAGS) $(SDL_CFLAGS) -c -o $@ $<

run_obstacle_wall: visual_sim
	./visual_sim --obstacle --scene wall --approach 0.003 --flow-x 0 --flow-y 0 --no-pad

run_obstacle_corridor: visual_sim
	./visual_sim --obstacle --scene corridor --flow-x 0 --flow-y 0 --no-pad

run_obstacle_asymmetry: visual_sim
	./visual_sim --obstacle --scene asymmetry --approach 0.003 --flow-x 0 --flow-y 0 --no-pad

test_obstacle_run: test_obstacle
	./test_obstacle

# Run all tests
check: test_synthetic test_hover test_stage2_run test_stage34_run test_obstacle_run test_vo_run test_line_run visual_sim
	@echo "All tests passed"

clean:
	rm -f sim_main test_stage2 test_stage34 test_obstacle test_visual_odometry test_line_detect visual_sim video_receiver \
		sim_main.o test_stage2.o test_stage34.o test_obstacle.o test_visual_odometry.o test_line_detect.o visual_sim.o video_receiver.o \
		../../src/vision/*.o

.PHONY: test_synthetic test_hover test_stage2_run test_stage34_run test_obstacle_run test_vo_run test_line_run visual_sim run_visual run_rot check clean
