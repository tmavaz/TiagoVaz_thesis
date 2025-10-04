#ifndef VL6180X_PLATFORM_H
#define VL6180X_PLATFORM_H

#include "VL6180x_def.h"

int VL6180x_I2CWrite(VL6180xDev_t dev, uint16_t index, uint8_t *data, uint16_t length);
int VL6180x_I2CRead(VL6180xDev_t dev, uint16_t index, uint8_t *data, uint16_t length);
void VL6180x_Delay(int ms);

#endif