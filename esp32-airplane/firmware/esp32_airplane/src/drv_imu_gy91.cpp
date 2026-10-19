// Arduino build shim: the builder only compiles the sketch root and src/,
// so drivers/imu_gy91.cpp is pulled in here. One TU per driver (same split as the
// host Makefile) to keep each file's statics private. Arduino-only file.
#ifdef ARDUINO
#include "../drivers/imu_gy91.cpp"
#endif
