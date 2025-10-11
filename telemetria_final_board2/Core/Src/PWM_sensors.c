#include "PWM_sensors.h"


int flagPWM=0;
int counter_pwm_front_susp_timer=0;
int counter_pwm_rear_susp_timer=0;
int counter_pwm_steering_angle_timer=0;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10,1);
	if(htim -> Channel == HAL_TIM_ACTIVE_CHANNEL_1 && htim->Instance == speed_pwm_timer)
    {
		capture_value_speed = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		capture_value2_speed = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
		if(capture_value_speed)
		{
			HAL_TIM_IC_Stop(htim,TIM_CHANNEL_1);
			frequency_speed = timer_peripheral_freq / capture_value_speed;
			save_data_to_sensor(&speed_sensor,frequency_speed);
			//snprintf((char*)tx_buffer, sizeof(tx_buffer),"PWM in speed entered freq=%d\r\n", frequency_speed);
			//HAL_UART_Transmit(&huart2, tx_buffer, strlen((char*)tx_buffer), HAL_MAX_DELAY);
		}
	}
    if(htim -> Channel == HAL_TIM_ACTIVE_CHANNEL_1 && htim->Instance == front_susp_timer)
    {

    	uint32_t capture_value_f_susp = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    	uint32_t capture_value2_f_susp = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

		if(capture_value_f_susp)
		{
			//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4);
			HAL_TIM_IC_Stop(htim,TIM_CHANNEL_1);
			HAL_TIM_IC_Stop(htim,TIM_CHANNEL_2);
			//__HAL_TIM_SET_COUNTER(htim, 0); // reset para próxima medição
			if (counter_pwm_front_susp_timer==1){
				uint32_t duty_cycle_f_susp = 100.0 * capture_value2_f_susp / capture_value_f_susp;
				save_data_to_sensor(&front_susp_sensor,duty_cycle_f_susp);
				//snprintf((char*)tx_buffer, sizeof(tx_buffer),"PWM in f susp entered %d\r\n", duty_cycle_f_susp);
				//HAL_UART_Transmit(&huart2, tx_buffer, strlen((char*)tx_buffer), HAL_MAX_DELAY);
			}
		}
		counter_pwm_front_susp_timer+=1;
	}
    if(htim -> Channel == HAL_TIM_ACTIVE_CHANNEL_1 && htim->Instance == rear_susp_timer)
    {

    	uint32_t capture_value_r_susp = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
    	uint32_t capture_value2_r_susp = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

		if(capture_value_r_susp)
		{
			//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4);
			HAL_TIM_IC_Stop(htim,TIM_CHANNEL_1);
			HAL_TIM_IC_Stop(htim,TIM_CHANNEL_2);
			//__HAL_TIM_SET_COUNTER(htim, 0); // reset para próxima medição
			//if (counter_pwm_rear_susp_timer==1){
				uint32_t duty_cycle_r_susp = 100 - (100.0 * capture_value2_r_susp / capture_value_r_susp);
				save_data_to_sensor(&rear_susp_sensor,duty_cycle_r_susp);
				//snprintf((char*)tx_buffer, sizeof(tx_buffer),"PWM in r susp entered %d\r\n", duty_cycle_r_susp);
				//HAL_UART_Transmit(&huart2, tx_buffer, strlen((char*)tx_buffer), HAL_MAX_DELAY);
			//}
		}
		counter_pwm_rear_susp_timer+=1;
	}
    if(htim -> Channel == HAL_TIM_ACTIVE_CHANNEL_1 && htim->Instance == steering_angle_timer)
    {

    	uint32_t capture_value_steer = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    	uint32_t capture_value2_steer = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

		if(capture_value_steer)
		{
			//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4);
			HAL_TIM_IC_Stop(htim,TIM_CHANNEL_1);
			HAL_TIM_IC_Stop(htim,TIM_CHANNEL_2);
			//__HAL_TIM_SET_COUNTER(htim, 0); // reset para próxima medição
			if (counter_pwm_steering_angle_timer==1){
				uint32_t duty_cycle_steer = 100.0 * capture_value2_steer / capture_value_steer;
				save_data_to_sensor(&steering_angle_sensor,duty_cycle_steer);
				//snprintf((char*)tx_buffer, sizeof(tx_buffer),"PWM in steering entered %d\r\n", duty_cycle_steer);
				//HAL_UART_Transmit(&huart2, tx_buffer, strlen((char*)tx_buffer), HAL_MAX_DELAY);
			}
		}
		counter_pwm_steering_angle_timer+=1;
	}
    flagPWM=1;
    //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10,0);
}

void write_pwm_data(){
	if (flagPWM == 1){
		snprintf((char*)tx_buffer, sizeof(tx_buffer),"PWM entered\r\n");
		HAL_UART_Transmit(&huart2, tx_buffer, strlen((char*)tx_buffer), HAL_MAX_DELAY);
		flagPWM=0;
	}

}

void Speed_ISR(){
	HAL_TIM_IC_Start_IT(speed_pwm_TIM, TIM_CHANNEL_1);
	HAL_TIM_IC_Start(speed_pwm_TIM, TIM_CHANNEL_2);
}

void Front_suspension_magnet_ISR(){
	counter_pwm_front_susp_timer=0;
	HAL_TIM_IC_Start_IT(front_susp_TIM,TIM_CHANNEL_1);
	HAL_TIM_IC_Start(front_susp_TIM,TIM_CHANNEL_2);
}

void Rear_suspension_ISR(){
	counter_pwm_rear_susp_timer=0;
	HAL_TIM_IC_Start_IT(rear_susp_TIM,TIM_CHANNEL_1);
	HAL_TIM_IC_Start(rear_susp_TIM,TIM_CHANNEL_2);
}

void Steering_angle_ISR(){
	counter_pwm_steering_angle_timer=0;
	HAL_TIM_IC_Start_IT(steering_angle_TIM,TIM_CHANNEL_1);
	HAL_TIM_IC_Start(steering_angle_TIM,TIM_CHANNEL_2);
};
