#include "IMU.h"

static int flagreceive=1;
static int flagreadregs=1;
static int flagsetbank=1;

static int flagSetMag = 1;
static int flagReadMag = 1;
static int flagMagDone = 1;


void SPI1_DMA_Reset(void) {
    // Aborta transferências DMA em curso (só as do SPI1)
    if (hspi1.hdmarx) {
        HAL_DMA_Abort(hspi1.hdmarx);
        __HAL_DMA_CLEAR_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx));
    }
    if (hspi1.hdmatx) {
        HAL_DMA_Abort(hspi1.hdmatx);
        __HAL_DMA_CLEAR_FLAG(hspi1.hdmatx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmatx));
    }

    // Reset ao periférico SPI1
    __HAL_RCC_SPI1_FORCE_RESET();
    __HAL_RCC_SPI1_RELEASE_RESET();

    // Re-inicializa SPI1
    HAL_SPI_DeInit(&hspi1);
    if (HAL_SPI_Init(&hspi1) != HAL_OK) {
        // aqui podes sinalizar erro (ex: LED ou debug UART)
    }

    // Religar SPI ao DMA (caso tenha sido perdido)
    if (hspi1.hdmarx) __HAL_LINKDMA(&hspi1, hdmarx, *hspi1.hdmarx);
    if (hspi1.hdmatx) __HAL_LINKDMA(&hspi1, hdmatx, *hspi1.hdmatx);
}

void IMU_TxRxCpltCallback(void){

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);  // CS high (end of SPI transaction)

    // If a magnetometer read was in progress:
    if (flagSetMag == 1) {
        // Bank 0 selected, now initiate magnetometer data read (9 bytes)
        flagSetMag = 0;
        uint8_t addrMag[10];
        addrMag[0] = 0x3B | 0x80;      // starting at EXT_SLV_SENS_DATA_00 (0x3B) with read bit
        for(int i = 1; i < 10; ++i) addrMag[i] = 0x00;
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET); // CS low
        flagReadMag = 1;
        int error = HAL_SPI_TransmitReceive_DMA(&hspi1, addrMag, magSPI, 10);  // read 9 bytes + dummy
    	if (error != HAL_OK){
    		SPI1_DMA_Reset();
    		error =HAL_SPI_TransmitReceive_DMA(&hspi1, addrMag, magSPI, 10);
    		if (error != HAL_OK){
    			snprintf((char *)tx_buffer, sizeof(tx_buffer),"ERRO NO SPI DO IMU!!!\r\n");
				HAL_UART_Transmit(&huart2, tx_buffer, strlen((char *)tx_buffer), HAL_MAX_DELAY);
    		}
    	}
        //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9,0);
        return;  // wait for magnetometer data DMA to complete
    }
    if (flagReadMag == 1) {
        // Magnetometer data received
        flagReadMag = 0;
        // Copy raw magnetometer bytes (skip magSPI[0] dummy and [1] ST1, take [2..7] = HXL..HZH)
        for(int i = 0; i < 6; ++i) {
            magData[i] = magSPI[2 + i];
            save_data_to_sensor(&mag,magData[i]);//guardar dados em memória
        }
        flagMagDone = 1;   // mark new magnetometer data ready
        // Note: ST2 is magSPI[9] if needed to check overflow (HOFL) or clear DRDY:contentReference[oaicite:11]{index=11}
    }

    // Existing accel/gyro handling:
    if (flagsetbank == 1) {
        // Bank selected for accel/gyro, now start reading 12 bytes (accel+gyro)
        flagsetbank = 0;
        uint8_t addr[13];
        addr[0] = 0x2D | 0x80;        // ACCEL_XOUT_H | 0x80
        for(int i = 1; i < 13; ++i) addr[i] = 0x00;
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
        flagreadregs = 1;
        int error = HAL_SPI_TransmitReceive_DMA(&hspi1, addr, accelSPI, 13);
    	if (error != HAL_OK){
    		SPI1_DMA_Reset();
    		error = HAL_SPI_TransmitReceive_DMA(&hspi1, addr, accelSPI, 13);
    		if (error != HAL_OK){
    			snprintf((char *)tx_buffer, sizeof(tx_buffer),"ERRO NO SPI DO IMU!!!\r\n");
				HAL_UART_Transmit(&huart2, tx_buffer, strlen((char *)tx_buffer), HAL_MAX_DELAY);
    		}
    	}
        //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9,0);
        return;
    }
    if (flagreadregs == 1) {
        // Accel/Gyro data received
        flagreadregs = 0;
        // Demultiplex 12 bytes into accelData and gyroData
        for (int i = 0; i < 6; ++i) {
            accelData[i] = accelSPI[1 + i];      // bytes 1-6: accel X,Y,Z
            save_data_to_sensor(&accel,accelData[i]);//guardar dados em memória
            gyroData[i]  = accelSPI[7 + i];      // bytes 7-12: gyro X,Y,Z
            save_data_to_sensor(&gyro,gyroData[i]);//guardar dados em memória
        }
        flagreceive = 1;  // mark new accel/gyro data ready
    }
}

void write_IMU_data(){
	if (flagreceive == 1){
		// Conversão de bytes brutos para valores de 16 bits
		int16_t rawAx = (int16_t)(accelData[0] << 8 | accelData[1]);
		int16_t rawAy = (int16_t)(accelData[2] << 8 | accelData[3]);
		int16_t rawAz = (int16_t)(accelData[4] << 8 | accelData[5]);
		int16_t rawGx = (int16_t)(gyroData[0] << 8 | gyroData[1]);
		int16_t rawGy = (int16_t)(gyroData[2] << 8 | gyroData[3]);
		int16_t rawGz = (int16_t)(gyroData[4] << 8 | gyroData[5]);

		// Conversão para unidades físicas
		float ax = (float)rawAx / ACCEL_SENS;
		float ay = (float)rawAy / ACCEL_SENS;
		float az = (float)rawAz / ACCEL_SENS;
		float gx = (float)rawGx / GYRO_SENS;
		float gy = (float)rawGy / GYRO_SENS;
		float gz = (float)rawGz / GYRO_SENS;

		// Envia dados convertidos via UART2
		snprintf((char *)tx_buffer, sizeof(tx_buffer),
			   "Accel (g): X=%.2f, Y=%.2f, Z=%.2f | Gyro (dps): X=%.2f, Y=%.2f, Z=%.2f\r\n",
			   ax, ay, az, gx, gy, gz);
		HAL_UART_Transmit(&huart2, tx_buffer, strlen((char *)tx_buffer), HAL_MAX_DELAY);
		flagreceive=0;
		//HAL_Delay(100);

	}
    if (flagMagDone == 1) {
        // Convert magnetometer (±4900 µT range)
        int16_t rawMx = (int16_t)(((uint16_t)magData[1] << 8) | magData[0]);  // HXH<<8 | HXL
        int16_t rawMy = (int16_t)(((uint16_t)magData[3] << 8) | magData[2]);  // HYH<<8 | HYL
        int16_t rawMz = (int16_t)(((uint16_t)magData[5] << 8) | magData[4]);  // HZH<<8 | HZL
        float mx = rawMx / MAG_SENS;
        float my = rawMy / MAG_SENS;
        float mz = rawMz / MAG_SENS;
        snprintf((char*)tx_buffer, sizeof(tx_buffer),
                 "Mag (uT): X=%.2f, Y=%.2f, Z=%.2f\r\n", mx, my, mz);
        HAL_UART_Transmit(&huart2, tx_buffer, strlen((char*)tx_buffer), HAL_MAX_DELAY);
        flagMagDone = 0;
    }
};

void ICM20948_SetBank(uint8_t bank) {
    uint8_t data[2];
    uint8_t rxData[2];
    data[0] = 0x7F;           // Endereço REG_BANK_SEL (0x7F), MSB=0 (escrita)
    data[1] = (bank << 4);    // Valor com banco (bits 5:4)
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET); // CS baixo
    int error = HAL_SPI_TransmitReceive_DMA(&hspi1, data, rxData, 2);
	if (error != HAL_OK){
		SPI1_DMA_Reset();
		error = HAL_SPI_TransmitReceive_DMA(&hspi1, data, rxData, 2);
		if (error != HAL_OK){
			snprintf((char *)tx_buffer, sizeof(tx_buffer),"ERRO NO SPI DO IMU!!!\r\n");
			HAL_UART_Transmit(&huart2, tx_buffer, strlen((char *)tx_buffer), HAL_MAX_DELAY);
		}
	}
};



void ICM20948_WriteReg(uint8_t bank, uint8_t reg, uint8_t value) {
    // Seleciona banco apropriado

    uint8_t data[2];
    uint8_t rxData[2];
    uint8_t txData[2];

    data[0] = 0x7F;           // Endereço REG_BANK_SEL (0x7F), MSB=0 (escrita)
    data[1] = (bank << 4);    // Valor com banco (bits 5:4)
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET); // CS baixo
    int error = HAL_SPI_TransmitReceive(&hspi1, data, rxData, 2, HAL_MAX_DELAY);
    if (error != HAL_OK){
    	return;
    }
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);   // CS alto
    txData[0] = reg & 0x7F;  // Endereço do registo (MSB=0 para escrita)
    txData[1] = value;
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET); // CS baixo
    HAL_SPI_TransmitReceive(&hspi1, txData,rxData, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);   // CS alto
};


void ICM20948_Setup(){
	// Desabilita interface I2C (USER_CTRL I2C_IF_DIS)
	ICM20948_WriteReg(0, 0x03, 0x10);  // USER_CTRL (0x03) = 0x10 (I2C_IF_DIS = 1)
	HAL_Delay(10);

	// Sair de modo sleep, configurar clock interno (PWR_MGMT_1)
	ICM20948_WriteReg(0, 0x06, 0x01);  // PWR_MGMT_1 (0x06) = 0x01 (auto select CLK)
	HAL_Delay(10);
	// Habilita acelerômetro e giroscópio (PWR_MGMT_2)
	ICM20948_WriteReg(0, 0x07, 0x00);  // PWR_MGMT_2 (0x07) = 0x00 (todos eixos ativados)
	HAL_Delay(10);

	// Configura faixa de aceleração e giroscópio (banco 2)
	ICM20948_WriteReg(2, 0x14, 0x00);  // ACCEL_CONFIG: ±2g, DLPF off
	ICM20948_WriteReg(2, 0x01, 0x00);  // GYRO_CONFIG1: ±250°/s, DLPF off
	HAL_Delay(10);

	//---------------------Magnetómetro--------------------------//
	// Enable I2C master and disable I2C slave interface on ICM-20948
	ICM20948_WriteReg(0, 0x03, 0x30);  // USER_CTRL = 0x30 (I2C_MST_EN=1, I2C_IF_DIS=1)
	HAL_Delay(10);

	// Configure AK09916 magnetometer (onboard ICM-20948) for continuous 100Hz mode
	ICM20948_WriteReg(3, 0x13, 0x0C);  // I2C_SLV4_ADDR = 0x0C (mag address, write mode)
	ICM20948_WriteReg(3, 0x14, 0x31);  // I2C_SLV4_REG  = 0x31 (mag CNTL2 register)
	ICM20948_WriteReg(3, 0x16, 0x08);  // I2C_SLV4_DO   = 0x08 (value to write: continuous 100Hz mode)
	ICM20948_WriteReg(3, 0x15, 0x80);  // I2C_SLV4_CTRL = 0x80 (trigger Slave4 one-byte write)
	HAL_Delay(10);  // wait for write to complete :contentReference[oaicite:1]{index=1}

	// Set up I2C Slave0 to read 9 bytes from magnetometer (ST1 through ST2)
	ICM20948_WriteReg(3, 0x03, 0x8C);  // I2C_SLV0_ADDR = 0x8C (mag address 0x0C, read mode MSB=1)
	ICM20948_WriteReg(3, 0x04, 0x10);  // I2C_SLV0_REG  = 0x10 (start at mag ST1 register)
	ICM20948_WriteReg(3, 0x05, 0x88);  // I2C_SLV0_CTRL = 0x88 (enable, read 9 bytes: ST1->ST2)
	HAL_Delay(10);
};


void ICM20948_ISR(int flagSetMag_input){
	flagSetMag=flagSetMag_input;
    flagsetbank=1;
    ICM20948_SetBank(0);
};
