/*
 * sensors.c
 *
 *  Created on: Jul 2, 2025
 *      Author: tmava
 */

#include "sensors.h"

//----------------------------------------
//ISR functions
//----------------------------------------
void Strain_gauge_ISR(){};
void IMU_Attitude_ISR(){};
void Oil_pressure_ISR(){
	HAL_ADC_Start_IT(&hadc1);
};

void write_brake_pressure_data(){

	uint32_t adc_val=oil_sensor.data[oil_sensor.index-1];
	float tensao = adc_val * 3.3f / 4095.0f;
	float corrente_mA = (-6.81f * tensao) + 22.39f;
	float pressao_bar = (corrente_mA - 4.0f) * (70.0f / 16.0f);

	char msg[64];
	sprintf(msg, "ADC = %lu | I = %.2f mA | P = %.2f bar\r\n",
			adc_val, corrente_mA, pressao_bar);
	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

void Front_suspension_vision_ISR(){};

//----------------------------------------
//Init functions
//----------------------------------------
void Oil_pressure_Init(){};
void Front_suspension_magnet_Init(){};
void Front_suspension_vision_Init(){};
void Rear_suspension_Init(){};
void Steering_angle_Init(){};
void Strain_gauge_Init(){};

void GPS_Init(){};
void Speed_Init(){};

void Mem_Init(){};


//----------------------------------------
//Setup functions
//----------------------------------------
void Oil_pressure_Setup(){};
void Front_suspension_magnet_Setup(){};
void Front_suspension_vision_Setup(){};
void Rear_suspension_Setup(){};
void Steering_angle_Setup(){};
void Strain_gauge_Setup(){};



void GPS_Setup(){};
void Speed_Setup(){};

void Mem_Setup(){};



