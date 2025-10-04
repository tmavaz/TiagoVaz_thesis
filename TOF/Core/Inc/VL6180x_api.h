#ifndef VL6180X_API_H
#define VL6180X_API_H

#include "VL6180x_def.h"

int VL6180x_Init(VL6180xDev_t *dev);
int VL6180x_Prepare(VL6180xDev_t *dev);
int VL6180x_RangePollMeasurement(VL6180xDev_t *dev, uint8_t *range);



#endif
