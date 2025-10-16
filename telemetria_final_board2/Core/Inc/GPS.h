#include "stm32f4xx_hal.h"

#ifndef SENSOR_H
#define SENSOR_H
#include "sensor.h"
#endif



#define NMEA_BUF_SIZE  512

extern Sensor gps;
extern int flagGPS;
extern uint8_t rx_dma_buffer[NMEA_BUF_SIZE];  // Buffer DMA para recepção
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart2;
//extern DMA_HandleTypeDef hdma_uart4_rx;


void write_gps_data();

void gps_RxEventCallback(uint16_t Size);



extern LoRaMessage loraGpsMsg;
