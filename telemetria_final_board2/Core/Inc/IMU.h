#include "stm32f4xx_hal.h"

#ifndef SENSOR_H
#define SENSOR_H
#include "sensor.h"
#endif

extern SPI_HandleTypeDef hspi1;
extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;
extern UART_HandleTypeDef huart2;

extern Sensor accel; // data saved as xyz
extern Sensor gyro; // data saved as xyz
extern Sensor mag; //

#define ACCEL_SENS 16384.0f  // ±2g (LSB/g)
#define GYRO_SENS  131.0f    // ±250°/s (LSB/(°/s))
#define MAG_SENS 6.6667f

extern uint8_t tx_buffer[128];
extern uint8_t accelData[6]; //buffer com os dados do accel
extern uint8_t gyroData[6];
extern uint8_t accelSPI[6+6+1]; //buffer de leitura do SPI em que o primeiro não interessa

extern uint8_t magData[6];
extern uint8_t magSPI[9+1];        // buffer for 9 bytes mag data (+1 dummy)


//static int flagreceive;
//static int flagreadregs;
//static int flagsetbank;

//static int flagSetMag;
//static int flagReadMag;
//static int flagMagDone;



void SPI1_DMA_Reset(void);
void IMU_TxRxCpltCallback(void);
void write_IMU_data();
void ICM20948_ISR(int flagSetMag_input);
void ICM20948_SetBank(uint8_t bank);
void ICM20948_WriteReg(uint8_t bank, uint8_t reg, uint8_t value);
void ICM20948_Setup();
