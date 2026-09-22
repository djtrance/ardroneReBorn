# esp32-airplane — host-side build & tests
#
# The firmware itself is built by the Arduino IDE / PlatformIO for the ESP32
# target. This Makefile only builds the pure-logic modules on the host so the
# algorithms can be unit-tested (checklist J2), mirroring tools/simulator.
#
#   make test          run all host unit tests
#   make test IMU=87   same, but compile the GY-87 board instead of the GY-91
#   make clean
#
# `IMU=` only affects which board driver is compiled active; both board files
# are always in the build (that is how the Arduino IDE sees them too).

CXX      ?= g++
CXXFLAGS ?= -std=gnu++17 -Wall -Wextra -O2
INC       = -Iesp32_airplane

# Compile-time IMU selection (config.h). Blank => GY-91 (the default there).
ifneq ($(strip $(IMU)),)
  ifeq ($(IMU),87)
    IMUDEF = -DIMU_GY87
  else ifeq ($(IMU),91)
    IMUDEF = -DIMU_GY91
  else
    $(error IMU must be 87 or 91)
  endif
endif

# Pure-logic modules (no Arduino.h dependency) -> host-buildable
MODULES = \
	esp32_airplane/mixing.cpp \
	esp32_airplane/ahrs.cpp \
	esp32_airplane/control.cpp \
	esp32_airplane/gps_nav.cpp \
	esp32_airplane/guidance.cpp \
	esp32_airplane/failsafe.cpp \
	esp32_airplane/drivers/ld2450.cpp

# Config + RC + sensor-translation modules (still Arduino-free)
HW_MODULES = \
	esp32_airplane/settings.cpp \
	esp32_airplane/rc_input.cpp \
	esp32_airplane/wifi_config.cpp \
	esp32_airplane/drivers/sensor_math.cpp \
	esp32_airplane/drivers/imu.cpp \
	esp32_airplane/drivers/imu_gy87.cpp \
	esp32_airplane/drivers/imu_gy91.cpp \
	esp32_airplane/drivers/sensors.cpp

HDRS = $(wildcard esp32_airplane/*.h esp32_airplane/drivers/*.h)

BUILD   = build
TESTBINS = $(BUILD)/test_wing_core $(BUILD)/test_rc_settings

.PHONY: all test clean
all: test

$(BUILD):
	@mkdir -p $(BUILD)

$(BUILD)/test_wing_core: tests/test_wing_core.cpp $(MODULES) $(HDRS) | $(BUILD)
	$(CXX) $(CXXFLAGS) $(IMUDEF) $(INC) tests/test_wing_core.cpp $(MODULES) -o $@

$(BUILD)/test_rc_settings: tests/test_rc_settings.cpp $(HW_MODULES) $(MODULES) $(HDRS) | $(BUILD)
	$(CXX) $(CXXFLAGS) $(IMUDEF) $(INC) tests/test_rc_settings.cpp $(HW_MODULES) $(MODULES) -o $@

test: $(TESTBINS)
	@$(BUILD)/test_wing_core
	@$(BUILD)/test_rc_settings

clean:
	rm -rf $(BUILD)
