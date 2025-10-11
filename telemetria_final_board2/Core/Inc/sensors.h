/*
 * sensors.h
 *
 *  Created on: Jul 2, 2025
 *      Author: tmava
 */

#ifndef INC_SENSORS_H_
#define INC_SENSORS_H_

#include "stm32f4xx_hal.h"
#include "IMU.h"
#include "GPS.h"
#include "eMMC.h"
#include "PWM_sensors.h"
#include "rfm95.h"

extern ADC_HandleTypeDef hadc1;

extern Sensor oil_sensor;

	void Strain_gauge_ISR();
	void GPS_ISR();

	void IMU_Attitude_ISR();
	void Oil_pressure_ISR();
	void write_brake_pressure_data();

	void Front_suspension_vision_ISR();





	void Oil_pressure_Init();
	void Front_suspension_magnet_Init();
	void Front_suspension_vision_Init();
	void Rear_suspension_Init();
	void Steering_angle_Init();
	void Strain_gauge_Init();

	void GPS_Init();
	void Speed_Init();

	void Mem_Init();


	void Oil_pressure_Setup();
	void Front_suspension_magnet_Setup();
	void Front_suspension_vision_Setup();
	void Rear_suspension_Setup();
	void Steering_angle_Setup();
	void Strain_gauge_Setup();
	void GPS_Setup();
	void Speed_Setup();

	void Mem_Setup();

#endif /* INC_SENSORS_H_ */
