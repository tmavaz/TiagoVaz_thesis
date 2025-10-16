#include "rfm95.h"
#include <string.h>  // para memset, etc.



// Funções auxiliares estáticas para SPI
static void RFM95_Select(void) {
    HAL_GPIO_WritePin(RFM95_CS_GPIO, RFM95_CS_PIN, GPIO_PIN_RESET); // CS baixo
}
static void RFM95_Unselect(void) {
    HAL_GPIO_WritePin(RFM95_CS_GPIO, RFM95_CS_PIN, GPIO_PIN_SET);   // CS alto
}
static void RFM95_WriteReg(uint8_t addr, uint8_t data) {
    uint8_t buf[2];
    buf[0] = addr | 0x80;   // MSB=1 indica escrita&#8203;:contentReference[oaicite:11]{index=11}
    buf[1] = data;
    RFM95_Select();
    HAL_SPI_Transmit_DMA(&hspi2, buf, 2);
    RFM95_Unselect();
}
static uint8_t RFM95_ReadReg(uint8_t addr) {
    uint8_t tx = addr & 0x7F;  // MSB=0 indica leitura
    uint8_t rx = 0;
    RFM95_Select();
    HAL_SPI_Transmit_DMA(&hspi2, &tx, 1);
    HAL_SPI_Receive_DMA(&hspi2, &rx, 1);
    RFM95_Unselect();
    return rx;
}

// Implementação da inicialização do RFM95
int RFM95_Init(void) {
    // 1. Resetar o módulo LoRa
    HAL_GPIO_WritePin(RFM95_RESET_GPIO, RFM95_RESET_PIN, GPIO_PIN_RESET);
    HAL_Delay(5);
    HAL_GPIO_WritePin(RFM95_RESET_GPIO, RFM95_RESET_PIN, GPIO_PIN_SET);
    HAL_Delay(5);

    // 2. Verificar o registro de versão para assegurar comunicação SPI
    uint8_t version = RFM95_ReadReg(REG_VERSION);
    if (version != 0x12) {
        // Versão esperada do SX1276 é 0x12&#8203;:contentReference[oaicite:12]{index=12}
        char msg[] = "Erro: RFM95 nao encontrado!\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        return RFM95_ERR;
    }

    // 3. Colocar em modo Sleep e habilitar LoRa (Long Range Mode).
    RFM95_WriteReg(REG_OP_MODE, MODE_SLEEP);            // primeiro Sleep em FSK
    RFM95_WriteReg(REG_OP_MODE, LONG_RANGE_MODE | MODE_SLEEP); // LoRa Sleep&#8203;:contentReference[oaicite:13]{index=13}
    HAL_Delay(10);

    // 4. Configurar frequência de operação (868 MHz)
    // Fórmula: reg_freq = (Frequência / Fstep), com Fstep = 32e6/2^19 ≈ 61 Hz&#8203;:contentReference[oaicite:14]{index=14}.
    // Para 868000000 Hz, reg_freq = 0xD90000&#8203;:contentReference[oaicite:15]{index=15}.
    RFM95_WriteReg(REG_FRF_MSB, 0xD9);
    RFM95_WriteReg(REG_FRF_MID, 0x00);
    RFM95_WriteReg(REG_FRF_LSB, 0x00);

    // 5. Configurar potência de transmissão no PA_BOOST
    // PaSelect=1 (PA_BOOST), MaxPower=111 (0x7) e OutputPower=1111 (0xF)&#8203;:contentReference[oaicite:16]{index=16}.
    // 0x8F já ativa PA_BOOST e OutputPower=15 (~17 dBm)&#8203;:contentReference[oaicite:17]{index=17}.
    RFM95_WriteReg(REG_PA_CONFIG, 0x8F);
    // (Opcional: habilitar +20dBm, se necessário, via REG_PA_DAC. Não habilitado aqui.)

    // 6. Configurar parâmetros LoRa: BW=125kHz, CR=4/5, explicit header, SF=7, CRC on.
    RFM95_WriteReg(REG_MODEM_CONFIG1, 0x72); // 0x72 = 0b01110010: BW125k, CR4/5, header explícito&#8203;:contentReference[oaicite:18]{index=18}
    RFM95_WriteReg(REG_MODEM_CONFIG2, 0x74); // 0x74 = 0b01110100: SF7, CRC habilitado&#8203;:contentReference[oaicite:19]{index=19}
    RFM95_WriteReg(REG_MODEM_CONFIG3, 0x04); // 0x04: LowDataRateOptimize off (SF7 não precisa), AGC on&#8203;:contentReference[oaicite:20]{index=20}

    // 7. Configurar base dos endereços FIFO (opcional, usar defaults)
    RFM95_WriteReg(REG_FIFO_TX_BASE_ADDR, 0x80); // Início TX FIFO = 0x80 (default)&#8203;:contentReference[oaicite:21]{index=21}
    RFM95_WriteReg(REG_FIFO_RX_BASE_ADDR, 0x00); // Início RX FIFO = 0x00 (default)

    // 8. Colocar em modo standby para aguardar envios
    RFM95_WriteReg(REG_OP_MODE, LONG_RANGE_MODE | MODE_STDBY);
    HAL_Delay(5);

    // 9. Indicar sucesso na UART2
    char okmsg[] = "RFM95 iniciado com sucesso\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t*)okmsg, strlen(okmsg), HAL_MAX_DELAY);
    return RFM95_OK;
}

// Envia um pacote LoRa (até 64 bytes) e aguarda conclusão
int RFM95_SendPacket(uint8_t *data, uint8_t length) {
    // Limitar tamanho ao máximo (64 bytes conforme requisitado)
    if (length > 64) length = 64;

    // 1. Entrar em modo de espera (Standby) para preparar FIFO
    RFM95_WriteReg(REG_OP_MODE, LONG_RANGE_MODE | MODE_STDBY);
    //HAL_Delay(1);

    // 2. Ponteiro FIFO aponta para base de TX
    RFM95_WriteReg(REG_FIFO_ADDR_PTR, 0x80);

    // 3. Escrever o tamanho do payload no registrador correspondente
    RFM95_WriteReg(REG_PAYLOAD_LENGTH, length);

    // 4. Escrever os dados do payload no FIFO
    for (uint8_t i = 0; i < length; i++) {
        RFM95_WriteReg(REG_FIFO, data[i]);
    }

    // 5. Iniciar transmissão LoRa (modo TX)
    RFM95_WriteReg(REG_OP_MODE, LONG_RANGE_MODE | MODE_TX);

    // 6. Aguardar até que a transmissão complete (TxDone setado)&#8203;:contentReference[oaicite:22]{index=22}
    // Verifica o bit TxDone (bit3) em REG_IRQ_FLAGS (0x12).
    uint8_t irqFlags;
    do {
        irqFlags = RFM95_ReadReg(REG_IRQ_FLAGS);
    } while ((irqFlags & 0x08) == 0);  // 0x08: máscara do bit TxDone

    // 7. Limpar a flag TxDone escrevendo 1 nesse bit&#8203;:contentReference[oaicite:23]{index=23}
    RFM95_WriteReg(REG_IRQ_FLAGS, 0x08);

    // 8. Retornar ao modo standby automaticamente ocorre após TX (por design)
    // Envio concluído
    return RFM95_OK;
}

char buffer[20];
char dbgMsg[50];
uint16_t counterHello = 0;


void SendLoRa(){
	if (flagLoRaGPS==1){
		RFM95_SendPacket((uint8_t *)loraGpsMsg.sentence, loraGpsMsg.len);
		 flagLoRaGPS=0;
	}
	if (flagLoRaIMUAttitude==1){
		//RFM95_SendPacket((uint8_t *)loraIMUMsg.sentence, loraIMUMsg.len);
		flagLoRaIMUAttitude=0;
	}
	if (flagLoRaCAN1Hz==1){
		//RFM95_SendPacket((uint8_t *)loraCAN1HzMsg.sentence, loraCAN1HzMsg.len);
		flagLoRaCAN1Hz=0;
	}
	if (flagLoRaCAN2Hz==1){
		//RFM95_SendPacket((uint8_t *)loraCAN2HzMsg.sentence, loraCAN2HzMsg.len);
		flagLoRaCAN2Hz=0;
	}
	if (flagLoRaSpeed==1){
		//RFM95_SendPacket((uint8_t *)loraSpeedMsg.sentence, loraSpeedMsg.len);
		flagLoRaSpeed=0;
	}
	if (flagLoRaOil==1){
		//RFM95_SendPacket((uint8_t *)loraOilMsg.sentence, loraOilMsg.len);
		flagLoRaOil=0;
	}
}


