include Makefile.include

SRC_DIR := ../src
OBJ_DIR := obj
BIN_DIR := bin

TARGET := $(BIN_DIR)/drone_encoder

SRCS := $(filter-out $(SRC_DIR)/gst_plugin.c $(SRC_DIR)/dsp_init_main.c, $(wildcard $(SRC_DIR)/*.c))
OBJS := $(patsubst $(SRC_DIR)/%.c, $(OBJ_DIR)/%.o, $(SRCS))

.PHONY: all clean distclean install

all: $(TARGET)

$(OBJ_DIR):
	mkdir -p $(OBJ_DIR)

$(BIN_DIR):
	mkdir -p $(BIN_DIR)

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c | $(OBJ_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(TARGET): $(OBJS) | $(BIN_DIR)
	$(CC) $(CFLAGS) $(OBJS) -o $@ $(LDFLAGS) \
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
