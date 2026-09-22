# esp32-airplane — host-side build & tests
#
# The firmware itself is built by the Arduino IDE / PlatformIO for the ESP32
# target. This Makefile only builds the pure-logic modules on the host so the
# algorithms can be unit-tested (checklist J2), mirroring tools/simulator.
#
#   make test      run the host unit tests
#   make clean

CXX      ?= g++
CXXFLAGS ?= -std=gnu++17 -Wall -Wextra -O2
INC       = -Iesp32_airplane

# Pure-logic modules (no Arduino.h dependency) -> host-buildable
MODULES = \
	esp32_airplane/mixing.cpp \
	esp32_airplane/ahrs.cpp \
	esp32_airplane/control.cpp \
	esp32_airplane/gps_nav.cpp \
	esp32_airplane/guidance.cpp \
	esp32_airplane/failsafe.cpp \
	esp32_airplane/drivers/ld2450.cpp \
	esp32_airplane/drivers/imu.cpp \
	esp32_airplane/drivers/sensors.cpp

HDRS = $(wildcard esp32_airplane/*.h esp32_airplane/drivers/*.h)

BUILD   = build
TESTBIN = $(BUILD)/test_wing_core

.PHONY: all test clean
all: test

$(BUILD):
	@mkdir -p $(BUILD)

$(TESTBIN): tests/test_wing_core.cpp $(MODULES) $(HDRS) | $(BUILD)
	$(CXX) $(CXXFLAGS) $(INC) tests/test_wing_core.cpp $(MODULES) -o $@

test: $(TESTBIN)
	@$(TESTBIN)

clean:
	rm -rf $(BUILD)
