#include "GPS.h"

void write_gps_data(){
	if(flagGPS == 1){
		HAL_UART_Transmit(&huart2, rx_dma_buffer, NMEA_BUF_SIZE, HAL_MAX_DELAY);
		flagGPS=0;
	}
};


void gps_RxEventCallback(uint16_t Size){
	uint16_t size_data=0;
	// Garantir terminação da string
	if (Size < NMEA_BUF_SIZE) {
		rx_dma_buffer[Size] = '\0';
		size_data=Size+1;
	} else {
		rx_dma_buffer[NMEA_BUF_SIZE-1] = '\0';
		size_data=NMEA_BUF_SIZE;
	}
	flagGPS=1;

	//salvar dados no sensor
	for (uint16_t i = 0; i < size_data; ++i) {
		uint8_t byte = rx_dma_buffer[i];
		save_data_to_sensor(&gps, byte);
	}
    // Transmitir sentenças completas via UART2 (terminal)
    //HAL_UART_Transmit(&huart2, rx_dma_buffer, Size, HAL_MAX_DELAY);

    // (Opcional) armazenar sentenças em lista ou buffer circular aqui
    //processReceivedData((char *)rx_dma_buffer);
    // Reiniciar recepção DMA para próxima sentença


	//para teste de LoRa em pista reitrar dps
	memcpy(loraGpsMsg.sentence, rx_dma_buffer, size_data);
	loraGpsMsg.len = size_data;


    HAL_UARTEx_ReceiveToIdle_DMA(&huart4, rx_dma_buffer, NMEA_BUF_SIZE);
}
