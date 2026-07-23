#ifndef MPU6050_I2C_H
#define MPU6050_I2C_H

#include <stdint.h>
#include <stdbool.h>

void mpu6050_i2c_sda_unlock(void);
bool mpu_i2c_read_byte(uint8_t devAddr, uint8_t reg, uint8_t *val);
bool mpu_i2c_read_bytes(uint8_t devAddr, uint8_t reg, uint8_t *buf, uint32_t len);
bool mpu_i2c_write_byte(uint8_t devAddr, uint8_t reg, uint8_t val);

#endif