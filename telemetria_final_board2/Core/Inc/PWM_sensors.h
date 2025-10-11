#include "stm32f4xx_hal.h"

#ifndef SENSOR_H
#define SENSOR_H
#include "sensor.h"
#endif


#define steering_angle_TIM &htim2
#define rear_susp_TIM &htim3
#define speed_pwm_TIM &htim4
#define front_susp_TIM &htim8

#define steering_angle_timer TIM2
#define rear_susp_timer TIM3
#define speed_pwm_timer TIM4
#define front_susp_timer TIM8

//#define MEDIA_N 4
//#define ESCALA 100
//#define ESCALA_TOTAL 360000
//#define PULSOS_POR_ROT 48
//#define MM_POR_ROT 1815
//#define FREQ_BASE 1000000UL
//#define DMA_POS() (CAPTURE_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(htim3.hdma[TIM_DMA_ID_CC2]))
#define timer_peripheral_freq 16000000.0f


extern Sensor speed_sensor;
extern uint32_t capture_value_speed, capture_value2_speed, frequency_speed;
extern Sensor front_susp_sensor;
extern Sensor rear_susp_sensor;
extern Sensor steering_angle_sensor;

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim8;


extern uint8_t tx_buffer[128];

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
void write_pwm_data();

void Speed_ISR();
void Steering_angle_ISR();
void Rear_suspension_ISR();
void Front_suspension_magnet_ISR();
