include Makefile.include

SRC_DIR := ../src
OBJ_DIR := obj
BIN_DIR := bin
TEST_DIR := $(SRC_DIR)/tests

VISION_DIR := $(SRC_DIR)/vision
VISION_SRCS := $(wildcard $(VISION_DIR)/*.c)
VISION_OBJS := $(patsubst $(VISION_DIR)/%.c, $(OBJ_DIR)/vision/%.o, $(VISION_SRCS))

NAV_DIR := $(SRC_DIR)/navigation
NAV_SRCS := $(wildcard $(NAV_DIR)/*.c)
NAV_OBJS := $(patsubst $(NAV_DIR)/%.c, $(OBJ_DIR)/navigation/%.o, $(NAV_SRCS))

NET_DIR := $(SRC_DIR)/network
NET_SRCS := $(filter-out $(NET_DIR)/skyproxy.c, $(wildcard $(NET_DIR)/*.c))
NET_OBJS := $(patsubst $(NET_DIR)/%.c, $(OBJ_DIR)/network/%.o, $(NET_SRCS))

DET_DIR := $(SRC_DIR)/detection
DET_SRCS := $(wildcard $(DET_DIR)/*.c)
DET_OBJS := $(patsubst $(DET_DIR)/%.c, $(OBJ_DIR)/detection/%.o, $(DET_SRCS))

FLIGHT_DIR := $(SRC_DIR)/flight
FLIGHT_SRCS := $(wildcard $(FLIGHT_DIR)/*.c)
FLIGHT_OBJS := $(patsubst $(FLIGHT_DIR)/%.c, $(OBJ_DIR)/flight/%.o, $(FLIGHT_SRCS))

TARGET := $(BIN_DIR)/drone_encoder

SRCS := $(filter-out $(SRC_DIR)/gst_plugin.c $(SRC_DIR)/dsp_init_main.c $(wildcard $(TEST_DIR)/*.c), $(wildcard $(SRC_DIR)/*.c))
OBJS := $(patsubst $(SRC_DIR)/%.c, $(OBJ_DIR)/%.o, $(SRCS))

.PHONY: all clean distclean install tests

all: $(TARGET)

$(OBJ_DIR):
	mkdir -p $(OBJ_DIR)

$(OBJ_DIR)/vision:
	mkdir -p $(OBJ_DIR)/vision

$(OBJ_DIR)/navigation:
	mkdir -p $(OBJ_DIR)/navigation

$(OBJ_DIR)/network:
	mkdir -p $(OBJ_DIR)/network

$(OBJ_DIR)/detection:
	mkdir -p $(OBJ_DIR)/detection

$(OBJ_DIR)/flight:
	mkdir -p $(OBJ_DIR)/flight

$(BIN_DIR):
	mkdir -p $(BIN_DIR)

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c | $(OBJ_DIR)
	$(CC) $(CFLAGS) -I$(VISION_DIR) -I$(NAV_DIR) -I$(NET_DIR) -I$(DET_DIR) -I$(SRC_DIR) -c $< -o $@

$(OBJ_DIR)/vision/%.o: $(VISION_DIR)/%.c | $(OBJ_DIR)/vision
	$(CC) $(CFLAGS) -I$(VISION_DIR) -I$(NAV_DIR) -I$(NET_DIR) -I$(DET_DIR) -I$(SRC_DIR) -c $< -o $@

$(OBJ_DIR)/navigation/%.o: $(NAV_DIR)/%.c | $(OBJ_DIR)/navigation
	$(CC) $(CFLAGS) -I$(VISION_DIR) -I$(NAV_DIR) -I$(NET_DIR) -I$(DET_DIR) -I$(SRC_DIR) -c $< -o $@

$(OBJ_DIR)/network/%.o: $(NET_DIR)/%.c | $(OBJ_DIR)/network
	$(CC) $(CFLAGS) -I$(VISION_DIR) -I$(NAV_DIR) -I$(NET_DIR) -I$(DET_DIR) -I$(SRC_DIR) -c $< -o $@

$(OBJ_DIR)/detection/%.o: $(DET_DIR)/%.c | $(OBJ_DIR)/detection
	$(CC) $(CFLAGS) -I$(VISION_DIR) -I$(NAV_DIR) -I$(NET_DIR) -I$(DET_DIR) -I$(SRC_DIR) -c $< -o $@

$(OBJ_DIR)/flight/%.o: $(FLIGHT_DIR)/%.c | $(OBJ_DIR)/flight
	$(CC) $(CFLAGS) -I$(VISION_DIR) -I$(NAV_DIR) -I$(NET_DIR) -I$(DET_DIR) -I$(FLIGHT_DIR) -I$(SRC_DIR) -c $< -o $@

$(TARGET): $(OBJS) $(VISION_OBJS) $(NAV_OBJS) $(NET_OBJS) $(DET_OBJS) $(FLIGHT_OBJS) | $(BIN_DIR)
	$(CC) $(CFLAGS) $(OBJS) $(VISION_OBJS) $(NAV_OBJS) $(NET_OBJS) $(DET_OBJS) $(FLIGHT_OBJS) -o $@ $(LDFLAGS) \
		-Wl,--unresolved-symbols=ignore-in-object-files -lm
	$(STRIP) $@
	@echo "=== Build complete: $(TARGET) ==="
	@file $(TARGET)

# Library targets (GStreamer plugin)
PLUGIN_TARGET := $(BIN_DIR)/libgstparrot_enc.so

PLUGIN_OBJS := $(OBJ_DIR)/gst_plugin.o $(OBJ_DIR)/h264_encode.o $(OBJ_DIR)/video_capture.o

$(PLUGIN_TARGET): $(PLUGIN_OBJS) | $(BIN_DIR)
	$(CC) $(CFLAGS) -shared -fPIC \
		-nodefaultlibs -nostartfiles \
		$(PLUGIN_OBJS) -o $@ \
		-Wl,--allow-shlib-undefined \
		-lgcc
	@echo "=== GStreamer plugin built: $(PLUGIN_TARGET) ==="

plugin: $(PLUGIN_TARGET)

# Hardware init standalone binary
H264_INIT_TARGET := $(BIN_DIR)/dsp_init
DSP_INIT_OBJS := $(OBJ_DIR)/dsp_init_main.o $(OBJ_DIR)/h264_init.o

$(H264_INIT_TARGET): $(DSP_INIT_OBJS) | $(BIN_DIR)
	$(CC) $(CFLAGS) $(DSP_INIT_OBJS) -o $@ $(LDFLAGS)
	@echo "=== DSP init binary: $(H264_INIT_TARGET) ==="

dsp_init: $(H264_INIT_TARGET)

# === On-drone test programs ===
# Soft-float toolchain + static linking matches drone's soft-float ABI.
# Each test compiles all its sources in one pass to avoid mixing hard/soft float .o files.

# test_connection: AT commands + navdata
$(BIN_DIR)/test_connection: $(TEST_DIR)/test_connection.c $(SRC_DIR)/connection.c | $(BIN_DIR)
	$(SOFT_CC) $(SOFT_CFLAGS) -I$(SRC_DIR) $^ -o $@ $(SOFT_LDFLAGS)
	@echo "=== Built: $@ ==="

# test_camera: V4L2 capture
$(BIN_DIR)/test_camera: $(TEST_DIR)/test_camera.c $(SRC_DIR)/video_capture.c | $(BIN_DIR)
	$(SOFT_CC) $(SOFT_CFLAGS) -I$(SRC_DIR) $^ -o $@ $(SOFT_LDFLAGS)
	@echo "=== Built: $@ ==="

# test_vision: camera + flow + obstacle
TEST_VISION_SRCS := $(TEST_DIR)/test_vision.c $(SRC_DIR)/video_capture.c \
                    $(VISION_DIR)/image.c $(VISION_DIR)/pattern.c \
                    $(VISION_DIR)/flow_stage1.c $(VISION_DIR)/obstacle.c
$(BIN_DIR)/test_vision: $(TEST_VISION_SRCS) | $(BIN_DIR)
	$(SOFT_CC) $(SOFT_CFLAGS) -I$(SRC_DIR) $^ -o $@ $(SOFT_LDFLAGS)
	@echo "=== Built: $@ ==="

# test_stream: camera + flow + obstacle + UDP stream
TEST_STREAM_SRCS := $(TEST_DIR)/test_stream.c $(SRC_DIR)/video_capture.c \
                    $(VISION_DIR)/image.c $(VISION_DIR)/pattern.c \
                    $(VISION_DIR)/flow_stage1.c $(VISION_DIR)/obstacle.c
$(BIN_DIR)/test_stream: $(TEST_STREAM_SRCS) | $(BIN_DIR)
	$(SOFT_CC) $(SOFT_CFLAGS) -I$(SRC_DIR) $^ -o $@ $(SOFT_LDFLAGS)
	@echo "=== Built: $@ ==="

# test_minimal: minimal hello world to verify toolchain works on drone
$(BIN_DIR)/test_minimal: $(TEST_DIR)/test_minimal.c | $(BIN_DIR)
	$(SOFT_CC) $(SOFT_CFLAGS) $^ -o $@ $(SOFT_LDFLAGS)
	@echo "=== Built: $@ ==="

# test_wifi_caps: WiFi capability test (monitor mode, raw sockets, channels)
$(BIN_DIR)/test_wifi_caps: $(TEST_DIR)/test_wifi_caps.c | $(BIN_DIR)
	$(SOFT_CC) $(SOFT_CFLAGS) -I$(SRC_DIR) $^ -o $@ $(SOFT_LDFLAGS)
	@echo "=== Built: $@ ==="

# test_uvc_camera: USB UVC camera via libusb
# Note: libusb symbols resolved at runtime (drone has libusb-1.0.so)
$(BIN_DIR)/test_uvc_camera: $(TEST_DIR)/test_uvc_camera.c | $(BIN_DIR)
	$(SOFT_CC) $(SOFT_CFLAGS) -I$(SRC_DIR) $^ -o $@ $(SOFT_LDFLAGS) \
		-Wl,--unresolved-symbols=ignore-in-object-files
	@echo "=== Built: $@ ==="

# test_gps: GPS NMEA parser test
$(BIN_DIR)/test_gps: $(TEST_DIR)/test_gps.c $(NAV_DIR)/gps.c | $(BIN_DIR)
	$(SOFT_CC) $(SOFT_CFLAGS) -I$(NAV_DIR) -I$(SRC_DIR) $^ -o $@ $(SOFT_LDFLAGS) -lm
	@echo "=== Built: $@ ==="

# test_modem: 4G modem control test
$(BIN_DIR)/test_modem: $(TEST_DIR)/test_modem.c $(NET_DIR)/modem.c | $(BIN_DIR)
	$(SOFT_CC) $(SOFT_CFLAGS) -I$(NET_DIR) -I$(SRC_DIR) $^ -o $@ $(SOFT_LDFLAGS) -lm
	@echo "=== Built: $@ ==="

# skyproxy: Skycontroller 2 <-> AR.Drone bridge
$(BIN_DIR)/skyproxy: $(NET_DIR)/skyproxy.c $(NET_DIR)/mdns.c $(NET_DIR)/recovery.c | $(BIN_DIR)
	$(SOFT_CC) $(SOFT_CFLAGS) -I$(NET_DIR) -I$(SRC_DIR) $^ -o $@ $(SOFT_LDFLAGS) -lpthread -lm
	@echo "=== Built: $@ ==="

# test_arsdk3_proto: ARNetworkAL + ARStream + ARSDK3 event protocol tests
$(BIN_DIR)/test_arsdk3_proto: $(TEST_DIR)/test_arsdk3_proto.c | $(BIN_DIR)
	$(SOFT_CC) $(SOFT_CFLAGS) -I$(SRC_DIR) -I$(NET_DIR) $^ -o $@ $(SOFT_LDFLAGS)
	@echo "=== Built: $@ ==="

# test_handshake: JSON handshake parser tests
$(BIN_DIR)/test_handshake: $(TEST_DIR)/test_handshake.c | $(BIN_DIR)
	$(SOFT_CC) $(SOFT_CFLAGS) $^ -o $@ $(SOFT_LDFLAGS)
	@echo "=== Built: $@ ==="

# test_flight: flight controller integration test
TEST_FLIGHT_SRCS := $(TEST_DIR)/test_flight.c $(SRC_DIR)/connection.c \
                    $(NAV_DIR)/gps.c $(FLIGHT_DIR)/flight_controller.c
$(BIN_DIR)/test_flight: $(TEST_FLIGHT_SRCS) | $(BIN_DIR)
	$(SOFT_CC) $(SOFT_CFLAGS) -I$(NAV_DIR) -I$(FLIGHT_DIR) -I$(SRC_DIR) $^ -o $@ $(SOFT_LDFLAGS) -lm
	@echo "=== Built: $@ ==="

# Build all test programs
tests: $(BIN_DIR)/test_minimal $(BIN_DIR)/test_connection $(BIN_DIR)/test_camera $(BIN_DIR)/test_vision $(BIN_DIR)/test_stream $(BIN_DIR)/test_wifi_caps $(BIN_DIR)/test_uvc_camera $(BIN_DIR)/test_gps $(BIN_DIR)/test_modem $(BIN_DIR)/skyproxy $(BIN_DIR)/test_arsdk3_proto $(BIN_DIR)/test_handshake $(BIN_DIR)/test_flight

# Size stats
size: $(TARGET)
	$(SIZE) $(TARGET)
	$(SIZE) $(PLUGIN_TARGET) 2>/dev/null || true

clean:
	rm -rf $(OBJ_DIR)

distclean: clean
	rm -rf $(BIN_DIR)

install: all plugin
	@echo "Upload to drone:"
	@echo "  cd ../tools && ./deploy.sh $(TARGET) 192.168.1.1"
	@echo "  cd ../tools && ./deploy.sh $(PLUGIN_TARGET) 192.168.1.1"
