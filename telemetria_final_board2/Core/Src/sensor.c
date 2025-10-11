#include "sensor.h"

void update_timestamp(void)
{
    RTC_TimeTypeDef sTime;
    RTC_DateTypeDef sDate;

    HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

    timestamp = sTime.Hours * 3600 + sTime.Minutes * 60 + sTime.Seconds;

    /*snprintf(timestamp_str, sizeof(timestamp_str), "20%02d%02d%02d_%02d%02d%02d\r\n",
                 sDate.Year, sDate.Month, sDate.Date,
                 sTime.Hours, sTime.Minutes, sTime.Seconds);*/
    //HAL_UART_Transmit(&huart2, timestamp_str, strlen((char *)timestamp_str), HAL_MAX_DELAY);
    snprintf(timestamp_msg, sizeof(timestamp_msg), "Timestamp = %lu\r\n", timestamp);
    HAL_UART_Transmit(&huart2, (uint8_t*)timestamp_msg, strlen(timestamp_msg), HAL_MAX_DELAY);
}


void update_sensor_timestamp(Sensor *sensor) //atualizar para ficar em ms
{
    HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

    sensor->timestamp = (sTime.Hours * 3600 + sTime.Minutes * 60 + sTime.Seconds)*1000+counter_ms;
    snprintf(timestamp_msg, sizeof(timestamp_msg), "Timestamp=%lu, sensor=%d\r\n", sensor->timestamp, sensor->sensorType);
    HAL_UART_Transmit(&huart2, (uint8_t*)timestamp_msg, strlen(timestamp_msg), HAL_MAX_DELAY);
}

void save_data_to_sensor(Sensor *sensor, uint8_t data){
	if (sensor->index == 0){
		sensor->data[0]=sensor->sensorType;
		update_sensor_timestamp(sensor);
		memcpy(&sensor->data[1], &(sensor->timestamp), sizeof(sensor->timestamp));  // guarda timestamp em binário
		sensor->index = 5;  // avança o índice após 1 byte de tipo + 4 bytes de timestamp
		sensor->sent_low=0;
	}
	if (sensor->index == (sensor_length/2)){
		sensor->data[sensor_length/2]=sensor->sensorType;
		update_sensor_timestamp(sensor);
		memcpy(&sensor->data[(sensor_length/2)+1], &(sensor->timestamp), sizeof(sensor->timestamp));  // guarda timestamp em binário
		sensor->index = (sensor_length/2) + 5;  // avança o índice após 1 byte de tipo + 4 bytes de timestamp
	}
	sensor->data[sensor->index]=data;
	if (sensor->index == (sensor_length-1))
		sensor->send_high=1;
	sensor->index = (sensor->index + 1) % sensor_length;
}
