#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <string.h>

#define sensor_length 1024U

typedef struct {
    uint8_t data[sensor_length];   // Ponteiro para os dados do sensor
    uint16_t index;       			// Indice atual
    uint8_t sent_low;     			// Flag para a parte baixa
    uint8_t send_high;    			// Flag para a parte alta
    uint32_t timestamp;				// timestamp para guardar na memória
    uint32_t memBlock;					//bloco de memória onde vai ser guardado;
    uint8_t sensorType;				//1-accel 2-gyro 3-mag
} Sensor;

extern uint32_t timestamp;
extern uint32_t counter_ms;
extern char timestamp_str[20];  // "YYYYMMDD_HHMMSS\r\n\0" = 18+1 máx
extern char timestamp_msg[50];

extern RTC_HandleTypeDef hrtc;
extern RTC_TimeTypeDef sTime;
extern RTC_DateTypeDef sDate;
extern UART_HandleTypeDef huart2;


void save_data_to_sensor(Sensor *sensor, uint8_t data);


typedef struct {
    char sentence[512];    // buffer da sentença NMEA terminada em '\0'
    uint16_t len;                    // comprimento real em bytes (incluindo '\0')
} LoRaGpsMessage;


