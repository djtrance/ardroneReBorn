include Makefile.include

SRC_DIR := ../src
OBJ_DIR := obj
BIN_DIR := bin
TEST_DIR := $(SRC_DIR)/tests

VISION_DIR := $(SRC_DIR)/vision
VISION_SRCS := $(wildcard $(VISION_DIR)/*.c)
VISION_OBJS := $(patsubst $(VISION_DIR)/%.c, $(OBJ_DIR)/vision/%.o, $(VISION_SRCS))

TARGET := $(BIN_DIR)/drone_encoder

SRCS := $(filter-out $(SRC_DIR)/gst_plugin.c $(SRC_DIR)/dsp_init_main.c $(wildcard $(TEST_DIR)/*.c), $(wildcard $(SRC_DIR)/*.c))
OBJS := $(patsubst $(SRC_DIR)/%.c, $(OBJ_DIR)/%.o, $(SRCS))

.PHONY: all clean distclean install tests

all: $(TARGET)

$(OBJ_DIR):
	mkdir -p $(OBJ_DIR)

$(OBJ_DIR)/vision:
	mkdir -p $(OBJ_DIR)/vision

$(BIN_DIR):
	mkdir -p $(BIN_DIR)

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c | $(OBJ_DIR)
	$(CC) $(CFLAGS) -I$(VISION_DIR) -I$(SRC_DIR) -c $< -o $@

$(OBJ_DIR)/vision/%.o: $(VISION_DIR)/%.c | $(OBJ_DIR)/vision
	$(CC) $(CFLAGS) -I$(VISION_DIR) -I$(SRC_DIR) -c $< -o $@

$(TARGET): $(OBJS) $(VISION_OBJS) | $(BIN_DIR)
	$(CC) $(CFLAGS) $(OBJS) $(VISION_OBJS) -o $@ $(LDFLAGS) \
		-Wl,--unresolved-symbols=ignore-in-object-files
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
# Static linking avoids dynamic linker mismatch with drone's firmware
TEST_LDFLAGS := -static $(LDFLAGS)

TEST_LIBS := $(OBJ_DIR)/connection.o $(OBJ_DIR)/video_capture.o $(VISION_OBJS)

# test_connection: connection test (AT commands + navdata)
TEST_CONN_OBJS := $(OBJ_DIR)/tests/test_connection.o $(OBJ_DIR)/connection.o
$(OBJ_DIR)/tests/test_connection.o: $(TEST_DIR)/test_connection.c | $(OBJ_DIR)
	mkdir -p $(OBJ_DIR)/tests
	$(CC) $(CFLAGS) -I$(VISION_DIR) -I$(SRC_DIR) -c $< -o $@

$(BIN_DIR)/test_connection: $(TEST_CONN_OBJS) | $(BIN_DIR)
	$(CC) $(CFLAGS) $(TEST_CONN_OBJS) -o $@ $(TEST_LDFLAGS)
	$(STRIP) $@
	@echo "=== Built: $@ ==="

# test_camera: V4L2 capture test
TEST_CAM_OBJS := $(OBJ_DIR)/tests/test_camera.o $(OBJ_DIR)/video_capture.o
$(OBJ_DIR)/tests/test_camera.o: $(TEST_DIR)/test_camera.c | $(OBJ_DIR)
	mkdir -p $(OBJ_DIR)/tests
	$(CC) $(CFLAGS) -I$(VISION_DIR) -I$(SRC_DIR) -c $< -o $@

$(BIN_DIR)/test_camera: $(TEST_CAM_OBJS) | $(BIN_DIR)
	$(CC) $(CFLAGS) $(TEST_CAM_OBJS) -o $@ $(TEST_LDFLAGS)
	$(STRIP) $@
	@echo "=== Built: $@ ==="

# test_vision: camera + flow + obstacle
TEST_VISION_OBJS := $(OBJ_DIR)/tests/test_vision.o $(OBJ_DIR)/video_capture.o $(VISION_OBJS)
$(OBJ_DIR)/tests/test_vision.o: $(TEST_DIR)/test_vision.c | $(OBJ_DIR)
	mkdir -p $(OBJ_DIR)/tests
	$(CC) $(CFLAGS) -I$(VISION_DIR) -I$(SRC_DIR) -c $< -o $@

$(BIN_DIR)/test_vision: $(TEST_VISION_OBJS) | $(BIN_DIR)
	$(CC) $(CFLAGS) $(TEST_VISION_OBJS) -o $@ $(TEST_LDFLAGS)
	$(STRIP) $@
	@echo "=== Built: $@ ==="

# test_stream: camera + flow + obstacle + UDP stream
TEST_STREAM_OBJS := $(OBJ_DIR)/tests/test_stream.o $(OBJ_DIR)/video_capture.o $(VISION_OBJS)
$(OBJ_DIR)/tests/test_stream.o: $(TEST_DIR)/test_stream.c | $(OBJ_DIR)
	mkdir -p $(OBJ_DIR)/tests
	$(CC) $(CFLAGS) -I$(VISION_DIR) -I$(SRC_DIR) -c $< -o $@

$(BIN_DIR)/test_stream: $(TEST_STREAM_OBJS) | $(BIN_DIR)
	$(CC) $(CFLAGS) $(TEST_STREAM_OBJS) -o $@ $(TEST_LDFLAGS)
	$(STRIP) $@
	@echo "=== Built: $@ ==="

# Build all test programs
tests: $(BIN_DIR)/test_connection $(BIN_DIR)/test_camera $(BIN_DIR)/test_vision $(BIN_DIR)/test_stream

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
