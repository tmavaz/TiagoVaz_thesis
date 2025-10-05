#ifndef RFM95_H
#define RFM95_H

#include "stm32f4xx_hal.h"  // ajuste o include de acordo com sua família STM32

// Definições de pinos de controle do RFM95 (ajuste se necessário)
#define RFM95_CS_GPIO       GPIOB
#define RFM95_CS_PIN        GPIO_PIN_12
#define RFM95_RESET_GPIO    GPIOB
#define RFM95_RESET_PIN     GPIO_PIN_0

// Registradores importantes do SX1276 (LoRa mode)
#define REG_FIFO                0x00
#define REG_OP_MODE             0x01
#define REG_FRF_MSB             0x06
#define REG_FRF_MID             0x07
#define REG_FRF_LSB             0x08
#define REG_PA_CONFIG           0x09
#define REG_FIFO_ADDR_PTR       0x0D
#define REG_FIFO_TX_BASE_ADDR   0x0E
#define REG_FIFO_RX_BASE_ADDR   0x0F
#define REG_IRQ_FLAGS           0x12
#define REG_MODEM_CONFIG1       0x1D
#define REG_MODEM_CONFIG2       0x1E
#define REG_MODEM_CONFIG3       0x26
#define REG_PAYLOAD_LENGTH      0x22
#define REG_DIO_MAPPING1        0x40
#define REG_VERSION             0x42
#define REG_PA_DAC              0x4D

// Modo LoRa (bit 7 do RegOpMode) e modos de operação (bits 2:0 do RegOpMode)
#define LONG_RANGE_MODE         0x80    // LoRa mode (bit7=1)
#define MODE_SLEEP              0x00
#define MODE_STDBY              0x01
#define MODE_TX                 0x03

// Códigos de retorno
#define RFM95_OK    0
#define RFM95_ERR   (-1)

// Protótipos de funções do driver
int RFM95_Init(void);
int RFM95_SendPacket(uint8_t *data, uint8_t length);

#endif // RFM95_H
