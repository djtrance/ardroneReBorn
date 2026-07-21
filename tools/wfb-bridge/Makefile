# Makefile for wifibroadcast tx bridge
# Compiles a minimal tx program that reads H.264 RTP from UDP
# and transmits it via raw WiFi (monitor mode)
#
# Uses the Parrot 2012.03 soft-float toolchain to match drone's ABI

include ../../build/Makefile.include

WFB_SRCS := wfb_tx.c
WFB_BIN  := ../../build/bin/wfb_tx

# Override SOFT flags for wfb (needs libsodium for encryption)
WFB_LIBS := -lsodium -lpthread -lm

$(WFB_BIN): $(WFB_SRCS)
	$(SOFT_CC) $(SOFT_CFLAGS) -I../../src $^ -o $@ $(SOFT_LDFLAGS) $(WFB_LIBS)
	@echo "=== Built: $@ ==="

all: $(WFB_BIN)

clean:
	rm -f $(WFB_BIN)

.PHONY: all clean
