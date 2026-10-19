// Arduino build shim: the builder only compiles the sketch root and src/,
// so drivers/ld2450.cpp is pulled in here. One TU per driver (same split as the
// host Makefile) to keep each file's statics private. Arduino-only file.
#ifdef ARDUINO
#include "../drivers/ld2450.cpp"
#endif
