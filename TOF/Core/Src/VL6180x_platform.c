#include "VL6180x_platform.h"
#include "main.h"

extern I2C_HandleTypeDef hi2c1;

int VL6180x_I2CWrite(VL6180xDev_t dev, uint16_t index, uint8_t *data, uint16_t length) {
    uint8_t buffer[2] = {index >> 8, index & 0xFF};
    HAL_I2C_Mem_Write(&hi2c1, dev.I2cAddr << 1, index, I2C_MEMADD_SIZE_16BIT, data, length, HAL_MAX_DELAY);
    return 0;
}

int VL6180x_I2CRead(VL6180xDev_t dev, uint16_t index, uint8_t *data, uint16_t length) {
    HAL_I2C_Mem_Read(&hi2c1, dev.I2cAddr << 1, index, I2C_MEMADD_SIZE_16BIT, data, length, HAL_MAX_DELAY);
    return 0;
}

void VL6180x_Delay(int ms) {
    HAL_Delay(ms);
}