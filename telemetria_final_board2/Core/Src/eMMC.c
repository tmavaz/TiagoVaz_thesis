#include "eMMC.h"


// --- escreve memBlock no bloco 0 ---
void update_memBlock_nr(void) {
    memset(sector0, 0x00, sizeof(sector0));          // opcional
    memcpy(sector0, &memBlock, sizeof(memBlock));    // guarda nos 4 primeiros bytes

    if (MMC_WriteBlocks(0, 1, sector0) == MMC_OK) {
        //snprintf(uart_msg, sizeof(uart_msg), "Bloco mem atualizado OK (%lu)\r\n", (unsigned long)memBlock);
    } else {
        snprintf(uart_msg, sizeof(uart_msg), "Erro bloco mem não atualizado\r\n");
    }
    //HAL_UART_Transmit(&huart2, (uint8_t*)uart_msg, strlen(uart_msg), HAL_MAX_DELAY);
}

// --- lê memBlock do bloco 0 ---
void get_memBlock_nr(void) {
    if (MMC_ReadBlocks(0, 1, sector0read) != MMC_OK) {
        snprintf(uart_msg, sizeof(uart_msg), "Erro na leitura do ultimo bloco escrito\r\n");
        HAL_UART_Transmit(&huart2, (uint8_t*)uart_msg, strlen(uart_msg), HAL_MAX_DELAY);
        return;
    }

    // Espera a transferência finalizar
    uint32_t t0 = HAL_GetTick();
    while (HAL_MMC_GetCardState(&hmmc) != HAL_MMC_CARD_TRANSFER) {
        if ((HAL_GetTick() - t0) > MMC_TIMEOUT) {     // usa o mesmo MMC_TIMEOUT=10000U
            snprintf(uart_msg, sizeof(uart_msg), "Timeout na leitura do ultimo bloco escrito\r\n");
            HAL_UART_Transmit(&huart2, (uint8_t*)uart_msg, strlen(uart_msg), HAL_MAX_DELAY);
            return;
        }
    }

    // Recupera o valor
    memcpy(&memBlock, sector0read, sizeof(memBlock));
    snprintf(uart_msg, sizeof(uart_msg), "Leitura OK. memBlock=%lu\r\n", (unsigned long)memBlock);
    HAL_UART_Transmit(&huart2, (uint8_t*)uart_msg, strlen(uart_msg), HAL_MAX_DELAY);
}


void HAL_MMC_TxCpltCallback(MMC_HandleTypeDef *hmmc) {
    //HAL_UART_Transmit(&huart2, (uint8_t*)"Transferência de escrita completa (DMA).\r\n", 42, HAL_MAX_DELAY);
}

void HAL_MMC_RxCpltCallback(MMC_HandleTypeDef *hmmc) {
    //HAL_UART_Transmit(&huart2, (uint8_t*)"Transferência de leitura completa (DMA).\r\n", 42, HAL_MAX_DELAY);
}

void HAL_MMC_ErrorCallback(MMC_HandleTypeDef *hmmc) {
    char err_msg[80];
    snprintf(err_msg, sizeof(err_msg), "Erro na transferência MMC: 0x%08lX\r\n", hmmc->ErrorCode);
    HAL_UART_Transmit(&huart2, (uint8_t*)err_msg, strlen(err_msg), HAL_MAX_DELAY);
}


void Verificar_DMA_Status(void) {
    // Verificar instâncias DMA
    if (hdma_sdio_rx.Instance != DMA2_Stream3 || hdma_sdio_tx.Instance != DMA2_Stream6) {
        HAL_UART_Transmit(&huart2, (uint8_t*)"[ERRO] Streams DMA incorretas!\r\n", 34, HAL_MAX_DELAY);
        Error_Handler();
    } else {
        HAL_UART_Transmit(&huart2, (uint8_t*)"[OK] Streams DMA corretas.\r\n", 30, HAL_MAX_DELAY);
    }

    // Verificar estado DMA
    if (hdma_sdio_rx.State == HAL_DMA_STATE_RESET || hdma_sdio_tx.State == HAL_DMA_STATE_RESET) {
        HAL_UART_Transmit(&huart2, (uint8_t*)"[ERRO] DMA não inicializado!\r\n", 32, HAL_MAX_DELAY);
        Error_Handler();
    } else {
        HAL_UART_Transmit(&huart2, (uint8_t*)"[OK] DMA inicializado.\r\n", 25, HAL_MAX_DELAY);
    }

    // Verificar ligação entre MMC e DMA
    if (hmmc.hdmarx != &hdma_sdio_rx || hmmc.hdmatx != &hdma_sdio_tx) {
        HAL_UART_Transmit(&huart2, (uint8_t*)"[ERRO] __HAL_LINKDMA não aplicado corretamente!\r\n", 51, HAL_MAX_DELAY);
        Error_Handler();
    } else {
        HAL_UART_Transmit(&huart2, (uint8_t*)"[OK] __HAL_LINKDMA corretamente aplicado.\r\n", 43, HAL_MAX_DELAY);
    }

    // Verificar configurações críticas do DMA RX
    if ((hdma_sdio_rx.Init.Direction != DMA_PERIPH_TO_MEMORY) ||
        (hdma_sdio_rx.Init.MemInc != DMA_MINC_ENABLE) ||
        (hdma_sdio_rx.Init.PeriphDataAlignment != DMA_PDATAALIGN_WORD) ||
        (hdma_sdio_rx.Init.Mode != DMA_PFCTRL)) {
        HAL_UART_Transmit(&huart2, (uint8_t*)"[ERRO] Configuração incorreta em DMA RX!\r\n", 43, HAL_MAX_DELAY);
        Error_Handler();
    } else {
        HAL_UART_Transmit(&huart2, (uint8_t*)"[OK] Configuração DMA RX válida.\r\n", 35, HAL_MAX_DELAY);
    }

    // Verificar configurações críticas do DMA TX
    if ((hdma_sdio_tx.Init.Direction != DMA_MEMORY_TO_PERIPH) ||
        (hdma_sdio_tx.Init.MemInc != DMA_MINC_ENABLE) ||
        (hdma_sdio_tx.Init.PeriphDataAlignment != DMA_PDATAALIGN_WORD) ||
        (hdma_sdio_tx.Init.Mode != DMA_PFCTRL)) {
        HAL_UART_Transmit(&huart2, (uint8_t*)"[ERRO] Configuração incorreta em DMA TX!\r\n", 43, HAL_MAX_DELAY);
        Error_Handler();
    } else {
        HAL_UART_Transmit(&huart2, (uint8_t*)"[OK] Configuração DMA TX válida.\r\n", 35, HAL_MAX_DELAY);
    }
}


void SDIO_DMA_Reset(void) {
    // Aborta transferências DMA em curso (só as do SDIO/MMC)
    if (hmmc.hdmarx) {
        HAL_DMA_Abort(hmmc.hdmarx);
        __HAL_DMA_CLEAR_FLAG(hmmc.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hmmc.hdmarx));
    }
    if (hmmc.hdmatx) {
        HAL_DMA_Abort(hmmc.hdmatx);
        __HAL_DMA_CLEAR_FLAG(hmmc.hdmatx, __HAL_DMA_GET_TC_FLAG_INDEX(hmmc.hdmatx));
    }

    // Reset ao periférico SDIO
    __HAL_RCC_SDIO_FORCE_RESET();
    __HAL_RCC_SDIO_RELEASE_RESET();

    // Re-inicializa SDIO/MMC
    HAL_MMC_DeInit(&hmmc);
    if (HAL_MMC_Init(&hmmc) != HAL_OK) {
        char msg[] = "Falha ao reinicializar SDIO/MMC!\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    }

    // Religar SDIO ao DMA (caso tenha sido perdido)
    if (hmmc.hdmarx) __HAL_LINKDMA(&hmmc, hdmarx, *hmmc.hdmarx);
    if (hmmc.hdmatx) __HAL_LINKDMA(&hmmc, hdmatx, *hmmc.hdmatx);
}


/* Escreve `numBlocks` blocos de 512B a partir de pData no endereço blockStart. */
MMC_Status MMC_WriteBlocks(uint32_t blockStart, uint32_t numBlocks, uint8_t *pData) {
    if (HAL_MMC_WriteBlocks_DMA(&hmmc, pData, blockStart, numBlocks) != HAL_OK) {
    	SDIO_DMA_Reset();
    	if (HAL_MMC_WriteBlocks_DMA(&hmmc, pData, blockStart, numBlocks) != HAL_OK){
        	snprintf(uart_msg, sizeof(uart_msg), "Erro: HAL_MMC_WriteBlocks_DMA falhou (0x%08lX)\r\n", hmmc.ErrorCode);
        	HAL_UART_Transmit(&huart2, (uint8_t*)uart_msg, strlen(uart_msg), HAL_MAX_DELAY);
        	return MMC_ERROR;
    	}
    }
    /* Espera até o fim da transferência */
    uint32_t tick = HAL_GetTick();
    while (HAL_MMC_GetCardState(&hmmc) != HAL_MMC_CARD_TRANSFER) {
        if ((HAL_GetTick() - tick) > MMC_TIMEOUT) {
        	snprintf(uart_msg, sizeof(uart_msg), "Erro: Timeout no HAL_MMC_GetCardState durante escrita\r\n");
        	HAL_UART_Transmit(&huart2, (uint8_t*)uart_msg, strlen(uart_msg), HAL_MAX_DELAY);
            return MMC_TIMEOUT_;
        }
    }
    return MMC_OK;
}

/* Lê `numBlocks` blocos de 512B para pData a partir do endereço blockStart. */
MMC_Status MMC_ReadBlocks(uint32_t blockStart, uint32_t numBlocks, uint8_t *pData) {
    if (HAL_MMC_ReadBlocks_DMA(&hmmc, pData, blockStart, numBlocks) != HAL_OK) {
    	SDIO_DMA_Reset();
    	if (HAL_MMC_ReadBlocks_DMA(&hmmc, pData, blockStart, numBlocks) != HAL_OK) {
			snprintf(uart_msg, sizeof(uart_msg), "Erro: HAL_MMC_ReadBlocks_DMA falhou (0x%08lX)\r\n", hmmc.ErrorCode);
			HAL_UART_Transmit(&huart2, (uint8_t*)uart_msg, strlen(uart_msg), HAL_MAX_DELAY);
			return MMC_ERROR;
    	}
    }
    uint32_t tick = HAL_GetTick();
    while (HAL_MMC_GetCardState(&hmmc) != HAL_MMC_CARD_TRANSFER) {
        if ((HAL_GetTick() - tick) > MMC_TIMEOUT) {
        	snprintf(uart_msg, sizeof(uart_msg), "Erro: Timeout no HAL_MMC_GetCardState durante leitura\r\n");
        	HAL_UART_Transmit(&huart2, (uint8_t*)uart_msg, strlen(uart_msg), HAL_MAX_DELAY);
            return MMC_TIMEOUT_;
        }
    }

    snprintf(uart_msg, sizeof(uart_msg), "Dados lidos (hex): ");
    /*for (int i = 0; i < 32; i++) {
        char temp[5];
        snprintf(temp, sizeof(temp), "%02X ", pData[i]);
        strncat(uart_msg, temp, sizeof(uart_msg) - strlen(uart_msg) - 1);
    }*/
    strncat(uart_msg, "\r\n", sizeof(uart_msg) - strlen(uart_msg) - 1);
    HAL_UART_Transmit(&huart2, (uint8_t*)uart_msg, strlen(uart_msg), HAL_MAX_DELAY);


    return MMC_OK;
}


/* Escreve e lê de volta para verificar. Usa MMC_WriteBlocks e MMC_ReadBlocks. */
MMC_Status MMC_WriteVerify(uint32_t blockStart, uint32_t numBlocks, uint8_t *pTxData, uint8_t *pRxData) {
    if (MMC_WriteBlocks(blockStart, numBlocks, pTxData) != MMC_OK) {
        return MMC_ERROR;
    }
    if (MMC_ReadBlocks(blockStart, numBlocks, pRxData) != MMC_OK) {
        return MMC_ERROR;
    }
    /* Comparação buffer a buffer */
    if (memcmp(pTxData, pRxData, numBlocks * 512U) != 0) {
        snprintf(uart_msg, sizeof(uart_msg), "Erro: Dados lidos != dados escritos\r\n");
        HAL_UART_Transmit(&huart2, (uint8_t*)uart_msg, strlen(uart_msg), HAL_MAX_DELAY);
        return MMC_ERROR;
    }
    return MMC_OK;
}


void SDIO_WriteFirstBlock(void) {
    uint8_t txBuf[512];
    char msg[64];

    // Preencher buffer com 70 (0x46)
    memset(txBuf, 70, sizeof(txBuf));

    // Escrever no bloco 0
    if (MMC_WriteBlocks(0,1,txBuf) != MMC_OK) {
        snprintf(msg, sizeof(msg), "Erro a escrever bloco 0! Código: 0x%08lX\r\n", hmmc.ErrorCode);
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        return;
    }

    // Espera o cartão acabar a escrita
    while (HAL_MMC_GetCardState(&hmmc) != HAL_MMC_CARD_TRANSFER) {}

    snprintf(msg, sizeof(msg), "Bloco 0 escrito com valor 70 em todos os bytes.\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}


void SDIO_ReadFirstBlock(void) {
    uint8_t rxBuf[512];   // buffer de 512 bytes para o bloco
    char msg[128];

    // Ler bloco 0 (1 bloco de 512 bytes)
    if (MMC_ReadBlocks(0, 1, rxBuf) != MMC_OK) {
        snprintf(msg, sizeof(msg), "Erro a ler first block! Código: 0x%08lX\r\n", hmmc.ErrorCode);
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        return;
    }

    // Espera até o cartão ficar pronto
    while (HAL_MMC_GetCardState(&hmmc) != HAL_MMC_CARD_TRANSFER) {}

    // Imprimir primeiros 64 bytes (para não encher demasiado o terminal)
    snprintf(msg, sizeof(msg), "Bloco 0 (primeiros 64 bytes):\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    for (int i = 0; i < 64; i++) {
        snprintf(msg, sizeof(msg), "%02X ", rxBuf[i]);
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

        if ((i + 1) % 16 == 0) {
            snprintf(msg, sizeof(msg), "\r\n");
            HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        }
    }
    snprintf(msg, sizeof(msg), "\r\nFim do bloco 0\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

void save_data_to_mem(Sensor *sensor) {
    if (sensor->index >= (sensor_length / 2) && sensor->sent_low == 0) {
        sensor->memBlock = memBlock++;
        update_memBlock_nr();
		if (MMC_WriteBlocks(sensor->memBlock, (sensor_length/2) / 512, (uint8_t*)sensor->data) == MMC_OK) {
        	//snprintf(uart_msg, sizeof(uart_msg), "[LOW]  Sensor %d OK bloco %d\r\n", sensor->sensorType, sensor->memBlock);
        } else {
            snprintf(uart_msg, sizeof(uart_msg), "[LOW] Sensor %d ERRO escrita MMC\r\n", sensor->sensorType);
        }
        //HAL_UART_Transmit(&huart2, (uint8_t*)uart_msg, strlen(uart_msg), HAL_MAX_DELAY);
        sensor->sent_low = 1;
        //HAL_GPIO_TogglePin (GPIOA, GPIO_PIN_4);
    } else if (sensor->send_high == 1){
        sensor->memBlock = memBlock++;
        update_memBlock_nr();
        if (MMC_WriteBlocks(sensor->memBlock, (sensor_length/2) / 512, (uint8_t*)&sensor->data[sensor_length / 2]) == MMC_OK) {
			//snprintf(uart_msg, sizeof(uart_msg), "[HIGH] Sensor %d OK bloco %d\r\n", sensor->sensorType, sensor->memBlock);
        } else {
            snprintf(uart_msg, sizeof(uart_msg), "[HIGH] Sensor %d ERRO escrita MMC\r\n", sensor->sensorType);
        }
        //HAL_UART_Transmit(&huart2, (uint8_t*)uart_msg, strlen(uart_msg), HAL_MAX_DELAY);
        sensor->send_high = 0;
        //HAL_GPIO_TogglePin (GPIOA, GPIO_PIN_4);
    }
}




#define BLOCKS_TO_SEND   5000U           // número de blocos a enviar
#define UART_TIMEOUT_MS  10000U         // timeout para transmissão UART (ms)

/**
 * @brief  Envia via UART os últimos 500 blocos de dados gravados no eMMC.
 *         O número do último bloco é lido a partir do bloco 0.
 */
void Send_Last500_Blocks_UART(void)
{
    uint32_t lastBlock = 0;

    // 1. Ler bloco 0 para descobrir o último bloco escrito
    uint8_t block0Buf[512];
    if (MMC_ReadBlocks(0U, 1U, block0Buf) != MMC_OK) {
        snprintf(uart_msg, sizeof(uart_msg),
                 "[ERRO] Falha a ler o bloco 0.\r\n");
        HAL_UART_Transmit(&huart2, (uint8_t*)uart_msg,
                          strlen(uart_msg), UART_TIMEOUT_MS);
        return;
    }
    // Aguardar conclusão conforme recomendado pela HAL:contentReference[oaicite:4]{index=4}
    while (HAL_MMC_GetCardState(&hmmc) != HAL_MMC_CARD_TRANSFER) {
        // opcional: adicionar timeout aqui para evitar loop infinito
    }

    // Copiar os 4 bytes do número do bloco; assume-se little‑endian
    memcpy(&lastBlock, block0Buf, sizeof(lastBlock));

    // 2. Determinar quantos blocos transmitir e o bloco inicial
    uint32_t blocksAvailable = (lastBlock >= BLOCKS_TO_SEND) ? BLOCKS_TO_SEND : lastBlock;
    uint32_t startBlock = (lastBlock >= BLOCKS_TO_SEND) ? (lastBlock - blocksAvailable + 1U) : 1U;

    // Buffer para ler cada bloco (512 bytes)
    uint8_t rxBuf[512];

    // 3. Ler e transmitir cada bloco
    for (uint32_t i = 0U; i < blocksAvailable; i++) {
        uint32_t blockIndex = startBlock + i;

        // Ler um bloco (512 bytes)
        if (MMC_ReadBlocks(blockIndex, 1U, rxBuf) != MMC_OK) {
            snprintf(uart_msg, sizeof(uart_msg),
                     "[ERRO] Falha na leitura do bloco %lu\r\n", (unsigned long)blockIndex);
            HAL_UART_Transmit(&huart2, (uint8_t*)uart_msg,
                              strlen(uart_msg), UART_TIMEOUT_MS);
            return;
        }
        // Aguardar que a transferência termine:contentReference[oaicite:5]{index=5}
        uint32_t t0 = HAL_GetTick();
        while (HAL_MMC_GetCardState(&hmmc) != HAL_MMC_CARD_TRANSFER) {
            if ((HAL_GetTick() - t0) > MMC_TIMEOUT) {
                snprintf(uart_msg, sizeof(uart_msg),
                         "[ERRO] Timeout a aguardar MMC no bloco %lu\r\n",
                         (unsigned long)blockIndex);
                HAL_UART_Transmit(&huart2, (uint8_t*)uart_msg,
                                  strlen(uart_msg), UART_TIMEOUT_MS);
                return;
            }
        }

        // 4. Transmitir o bloco lido via UART
        // HAL_UART_Transmit bloqueia até enviar todos os bytes ou até expirar
        // o timeout:contentReference[oaicite:6]{index=6}
        if (HAL_UART_Transmit(&huart2, rxBuf, sizeof(rxBuf), UART_TIMEOUT_MS) != HAL_OK) {
            snprintf(uart_msg, sizeof(uart_msg),
                     "[ERRO] Falha ao transmitir bloco %lu via UART\r\n",
                     (unsigned long)blockIndex);
            HAL_UART_Transmit(&huart2, (uint8_t*)uart_msg,
                              strlen(uart_msg), UART_TIMEOUT_MS);
            return;
        }
    }

    // 5. Mensagem final de sucesso
    snprintf(uart_msg, sizeof(uart_msg),
             "[OK] %lu blocos enviados via UART.\r\n", (unsigned long)blocksAvailable);
    HAL_UART_Transmit(&huart2, (uint8_t*)uart_msg,
                      strlen(uart_msg), UART_TIMEOUT_MS);
}

/*
 * Lê todos os blocos de dados gravados na eMMC até ao memBlock
 * e envia os dados pela UART em formato hexadecimal. Assume‑se
 * que o bloco 0 contém o valor de memBlock (próximo bloco livre)
 * e que os dados são armazenados a partir do bloco 1.
 */

//#define block_nr_to_start 56374U
#define block_nr_to_start 1U


void SDIO_ReadAllData(void) {
    char msg[64];

    /* 1. Actualizar memBlock com o valor guardado no bloco 0 */
    get_memBlock_nr();

    /* Se memBlock for zero, não há dados para ler */
    if (memBlock == 0U) {
        snprintf(msg, sizeof(msg), "Nenhum dado gravado. memBlock = 0\r\n");
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        return;
    }

    /* 2. Calcular último bloco válido.
       Na aplicação original, memBlock aponta para o próximo bloco livre;
       portanto, o último bloco de dados é memBlock - 1.
       Ajuste conforme o seu esquema de gravação (ex.: se usar bloco 0 para dados). */
    uint32_t lastBlock = (memBlock > 0U) ? (memBlock - 1U) : 0U;

    /* 3. Informar início da transferência */
    snprintf(msg, sizeof(msg), "Enviando dados dos blocos 1 a %lu\r\n",
             (unsigned long)lastBlock);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    /* 4. Buffer para um bloco de 512 bytes */
    uint8_t rxBuf[512];

    /* 5. Percorrer todos os blocos de dados */
    for (uint32_t blk = block_nr_to_start; blk <= lastBlock; blk++) {
        /* Mensagem de início do bloco */
        snprintf(msg, sizeof(msg), "Inicio do bloco %lu\r\n",
                 (unsigned long)blk);
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

        /* Ler bloco: um bloco (512 bytes) por vez */
        if (MMC_ReadBlocks(blk, 1U, rxBuf) != MMC_OK) {
            snprintf(msg, sizeof(msg), "Erro ao ler bloco %lu\r\n",
                     (unsigned long)blk);
            HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
            return;
        }

        /* Esperar o cartão ficar pronto.
           A ST recomenda verificar o estado da transferência após cada
           leitura com HAL_MMC_GetCardState():contentReference[oaicite:0]{index=0}.
         */
        uint32_t t0 = HAL_GetTick();
        while (HAL_MMC_GetCardState(&hmmc) != HAL_MMC_CARD_TRANSFER) {
            if ((HAL_GetTick() - t0) > MMC_TIMEOUT) {
                snprintf(msg, sizeof(msg), "Timeout ao ler bloco %lu\r\n",
                         (unsigned long)blk);
                HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg),
                                  HAL_MAX_DELAY);
                return;
            }
        }

        /* Transmitir 512 bytes em hexadecimal pela UART */
        for (size_t i = 0U; i < sizeof(rxBuf); i++) {
            char hexByte[4];
            snprintf(hexByte, sizeof(hexByte), "%02X ", rxBuf[i]);
            HAL_UART_Transmit(&huart2, (uint8_t*)hexByte,
                              strlen(hexByte), HAL_MAX_DELAY);
        }

        /* Mensagem de fim do bloco */
        snprintf(msg, sizeof(msg), "\r\nFim do bloco %lu\r\n",
                 (unsigned long)blk);
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    }

    /* 6. Indicar conclusão */
    snprintf(msg, sizeof(msg), "\r\nFim da transferência de dados\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}


