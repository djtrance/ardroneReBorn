# Basic Connection Example

Connects to the AR.Drone 2.0, sends takeoff/hover/land commands, and reads navdata (battery, altitude, velocity, attitude).

## Build

```bash
cd build
make
```

## Deploy & Run

```bash
cd tools
./deploy.sh ../build/bin/drone_encoder 192.168.1.1
telnet 192.168.1.1
/data/video/drone_encoder
```

The drone will take off, hover for ~5 seconds displaying telemetry, then land.

## Key Files

- `main.c` — application entry point
- `../../src/connection.c` — connection and AT command library
- `../../src/connection.h` — header with API
