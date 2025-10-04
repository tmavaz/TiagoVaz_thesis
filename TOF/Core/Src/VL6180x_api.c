#include "VL6180x_api.h"
#include "VL6180x_platform.h"

int VL6180x_Init(VL6180xDev_t *dev) {
    uint8_t id;
    VL6180x_I2CRead(*dev, 0x0000, &id, 1);
    return (id == 0xB4) ? 0 : -1;
}

/*int VL6180x_Prepare(VL6180xDev_t *dev) {
    uint8_t config[] = {0x01};
    return VL6180x_I2CWrite(*dev, 0x0207, config, 1);
}*/

int VL6180x_Prepare(VL6180xDev_t *dev) {
    VL6180x_I2CWrite(*dev, 0x0207, (uint8_t[]){0x01}, 1);
    VL6180x_I2CWrite(*dev, 0x0208, (uint8_t[]){0x01}, 1);
    VL6180x_I2CWrite(*dev, 0x0096, (uint8_t[]){0x00}, 1);
    VL6180x_I2CWrite(*dev, 0x0097, (uint8_t[]){0xfd}, 1);
    VL6180x_I2CWrite(*dev, 0x00e3, (uint8_t[]){0x00}, 1);
    VL6180x_I2CWrite(*dev, 0x00e4, (uint8_t[]){0x04}, 1);
    VL6180x_I2CWrite(*dev, 0x00e5, (uint8_t[]){0x02}, 1);
    VL6180x_I2CWrite(*dev, 0x00e6, (uint8_t[]){0x01}, 1);
    VL6180x_I2CWrite(*dev, 0x00e7, (uint8_t[]){0x03}, 1);
    VL6180x_I2CWrite(*dev, 0x00f5, (uint8_t[]){0x02}, 1);
    VL6180x_I2CWrite(*dev, 0x00d9, (uint8_t[]){0x05}, 1);
    VL6180x_I2CWrite(*dev, 0x00db, (uint8_t[]){0xce}, 1);
    VL6180x_I2CWrite(*dev, 0x00dc, (uint8_t[]){0x03}, 1);
    VL6180x_I2CWrite(*dev, 0x00dd, (uint8_t[]){0xf8}, 1);
    VL6180x_I2CWrite(*dev, 0x009f, (uint8_t[]){0x00}, 1);
    VL6180x_I2CWrite(*dev, 0x00a3, (uint8_t[]){0x3c}, 1);
    VL6180x_I2CWrite(*dev, 0x00b7, (uint8_t[]){0x00}, 1);
    VL6180x_I2CWrite(*dev, 0x00bb, (uint8_t[]){0x3c}, 1);
    VL6180x_I2CWrite(*dev, 0x00b2, (uint8_t[]){0x09}, 1);
    VL6180x_I2CWrite(*dev, 0x00ca, (uint8_t[]){0x09}, 1);
    VL6180x_I2CWrite(*dev, 0x0198, (uint8_t[]){0x01}, 1);
    VL6180x_I2CWrite(*dev, 0x01b0, (uint8_t[]){0x17}, 1);
    VL6180x_I2CWrite(*dev, 0x01ad, (uint8_t[]){0x00}, 1);
    VL6180x_I2CWrite(*dev, 0x00ff, (uint8_t[]){0x05}, 1);
    VL6180x_I2CWrite(*dev, 0x0100, (uint8_t[]){0x05}, 1);
    VL6180x_I2CWrite(*dev, 0x0199, (uint8_t[]){0x05}, 1);
    VL6180x_I2CWrite(*dev, 0x01a6, (uint8_t[]){0x1b}, 1);
    VL6180x_I2CWrite(*dev, 0x01ac, (uint8_t[]){0x3e}, 1);
    VL6180x_I2CWrite(*dev, 0x01a7, (uint8_t[]){0x1f}, 1);
    VL6180x_I2CWrite(*dev, 0x0030, (uint8_t[]){0x00}, 1);

    return 0;
}




int VL6180x_RangePollMeasurement(VL6180xDev_t *dev, uint8_t *range) {
    uint8_t start = 0x01;
    VL6180x_I2CWrite(*dev, 0x0018, &start, 1);
    VL6180x_Delay(10);
    return VL6180x_I2CRead(*dev, 0x0062, range, 1);
}

int VL6180x_SetOffset(VL6180xDev_t *dev, int8_t offset_mm) {
    uint8_t val = (uint8_t)offset_mm; // 2's complement automático
    return VL6180x_I2CWrite(*dev, 0x0024, &val, 1);
}


