// AR.Drone 2.0 — Sensor Bridge for Arduino/ESP32
// Runs on an Arduino Nano (or ESP32) connected via USB serial to the drone.
// Provides I2C/GPIO access to sensors not directly supported by the drone.
//
// Protocol (115200 baud, \r\n terminated):
//   I2C:addr,reg     Read I2C register  (e.g. I2C:0x77,0xAA)
//   I2C:addr,reg,val Write I2C register (e.g. I2C:0x77,0xAA,0x42)
//   GPIO:pin,val     Set GPIO pin (e.g. GPIO:13,1)
//   GPIO:pin,R       Read GPIO pin (e.g. GPIO:12,R)
//   PING             Returns PONG
//
// Response format:
//   OK:<data>     Success
//   ERR:<reason>  Failure
//
// Example sensors:
//   - HC-SR04 ultrasonic via GPIO trigger/echo
//   - BMP280 barometer via I2C
//   - VL53L1X ToF ranging via I2C
//   - HMC5883L magnetometer via I2C
//   - RGB LED status via GPIO PWM

#include <Arduino.h>

// ===== Configuration =====
#define BAUD_RATE 115200
#define BUFFER_SIZE 64
#define I2C_TIMEOUT_MS 100

// HC-SR04 pins (if using ultrasonic)
#define TRIG_PIN 9
#define ECHO_PIN 10

// Status LED
#define STATUS_LED LED_BUILTIN

// ===== Globals =====
static char buffer[BUFFER_SIZE];
static int buf_pos = 0;

// ===== I2C Read =====
static void i2c_read(uint8_t addr, uint8_t reg) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) {
        Serial.print("ERR:I2C_NAK\r\n");
        return;
    }

    int n = Wire.requestFrom((int)addr, 1);
    if (n == 0) {
        Serial.print("ERR:I2C_NODATA\r\n");
        return;
    }

    uint8_t val = Wire.read();
    char resp[16];
    snprintf(resp, sizeof(resp), "OK:%02X\r\n", val);
    Serial.print(resp);
}

// ===== I2C Write =====
static void i2c_write(uint8_t addr, uint8_t reg, uint8_t val) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(val);
    uint8_t err = Wire.end Transmission(true);
    if (err == 0) {
        Serial.print("OK\r\n");
    } else {
        Serial.print("ERR:I2C_WRITE\r\n");
    }
}

// ===== GPIO Read =====
static void gpio_read(uint8_t pin) {
    int val = digitalRead(pin);
    char resp[16];
    snprintf(resp, sizeof(resp), "OK:%d\r\n", val);
    Serial.print(resp);
}

// ===== GPIO Write =====
static void gpio_write(uint8_t pin, uint8_t val) {
    digitalWrite(pin, val ? HIGH : LOW);
    Serial.print("OK\r\n");
}

// ===== HC-SR04 Ultrasonic Read =====
static void ultrasonic_read(void) {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH, 30000);  // 30ms timeout ≈ 5m
    if (duration == 0) {
        Serial.print("ERR:TIMEOUT\r\n");
        return;
    }

    // Speed of sound: 343 m/s = 0.0343 cm/µs
    // Distance = (duration / 2) * 0.0343
    float dist_cm = duration * 0.01715;
    char resp[32];
    snprintf(resp, sizeof(resp), "OK:%.1f\r\n", dist_cm);
    Serial.print(resp);
}

// ===== Command Parser =====
static void process_command(const char *cmd) {
    if (strcmp(cmd, "PING") == 0) {
        Serial.print("OK:PONG\r\n");
        digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
        return;
    }

    if (strcmp(cmd, "ULTRASONIC") == 0) {
        ultrasonic_read();
        return;
    }

    if (strncmp(cmd, "I2C:", 4) == 0) {
        uint8_t addr, reg, val;
        int n = sscanf(cmd + 4, "%hhx,%hhx,%hhx", &addr, &reg, &val);
        if (n >= 2 && n <= 3) {
            if (n == 2) i2c_read(addr, reg);
            else        i2c_write(addr, reg, val);
        } else {
            Serial.print("ERR:SYNTAX\r\n");
        }
        return;
    }

    if (strncmp(cmd, "GPIO:", 5) == 0) {
        uint8_t pin;
        char mode;
        int n = sscanf(cmd + 5, "%hhu,%c", &pin, &mode);
        if (n == 2) {
            if (mode == 'R') {
                pinMode(pin, INPUT);
                gpio_read(pin);
            } else if (mode == '0' || mode == '1') {
                pinMode(pin, OUTPUT);
                gpio_write(pin, mode == '1' ? 1 : 0);
            } else {
                Serial.print("ERR:SYNTAX\r\n");
            }
        } else {
            Serial.print("ERR:SYNTAX\r\n");
        }
        return;
    }

    Serial.print("ERR:UNKNOWN\r\n");
}

// ===== Arduino Lifecycle =====
void setup() {
    Serial.begin(BAUD_RATE);
    Wire.begin();
    Wire.setWireTimeout(I2C_TIMEOUT_MS * 1000, true);

    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    pinMode(STATUS_LED, OUTPUT);
    digitalWrite(TRIG_PIN, LOW);

    Serial.print("OK:BRIDGE_READY\r\n");
}

void loop() {
    while (Serial.available() > 0) {
        char c = Serial.read();
        if (c == '\n') {
            buffer[buf_pos] = 0;
            if (buf_pos > 0) {
                // Strip trailing \r
                if (buf_pos > 0 && buffer[buf_pos-1] == '\r')
                    buffer[buf_pos-1] = 0;
                process_command(buffer);
            }
            buf_pos = 0;
        } else if (buf_pos < BUFFER_SIZE - 1) {
            buffer[buf_pos++] = c;
        } else {
            // Buffer overflow, reset
            buf_pos = 0;
            Serial.print("ERR:OVERFLOW\r\n");
        }
    }
}
