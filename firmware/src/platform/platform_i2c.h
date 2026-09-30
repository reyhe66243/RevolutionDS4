// platform_i2c.h - Platform I2C Hardware Abstraction Layer
//
// Thin abstraction over platform-specific I2C APIs.
// Implementations: rp2040/platform_i2c_rp2040.c, esp32/platform_i2c_esp32.c

#ifndef PLATFORM_I2C_H
#define PLATFORM_I2C_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

// Opaque I2C bus handle
typedef struct platform_i2c* platform_i2c_t;

// I2C bus configuration
typedef struct {
    uint8_t bus;        // Bus index (0 or 1)
    uint8_t sda_pin;    // SDA GPIO pin
    uint8_t scl_pin;    // SCL GPIO pin
    uint32_t freq_hz;   // Clock frequency (e.g. 400000 for 400kHz)
} platform_i2c_config_t;

// Initialize an I2C bus. Returns handle, or NULL on failure.
platform_i2c_t platform_i2c_init(const platform_i2c_config_t* config);

// Write data to an I2C device. Returns 0 on success, negative on error.
int platform_i2c_write(platform_i2c_t bus, uint8_t addr, const uint8_t* data, size_t len);

// Read data from an I2C device. Returns 0 on success, negative on error.
int platform_i2c_read(platform_i2c_t bus, uint8_t addr, uint8_t* data, size_t len);

// Write then read (repeated start). Returns 0 on success, negative on error.
int platform_i2c_write_read(platform_i2c_t bus, uint8_t addr,
                            const uint8_t* wr, size_t wr_len,
                            uint8_t* rd, size_t rd_len);

#endif // PLATFORM_I2C_H
