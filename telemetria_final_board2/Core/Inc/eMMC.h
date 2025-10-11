#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <string.h>

#ifndef SENSOR_H
#define SENSOR_H
#include "sensor.h"
#endif

#define MMC_TIMEOUT  10000U  /* timeout em ms */

typedef enum {
    MMC_OK = 0,
    MMC_ERROR,
    MMC_TIMEOUT_,
} MMC_Status;

extern char uart_msg[1050];
extern uint32_t memBlock;
// Buffer de sector (alinhado p/ SDIO DMA)
static uint8_t sector0[512];
static uint8_t sector0read[512];

extern MMC_HandleTypeDef hmmc;
extern DMA_HandleTypeDef hdma_sdio_rx;
extern DMA_HandleTypeDef hdma_sdio_tx;
extern UART_HandleTypeDef huart2;

void get_memBlock_nr(void);
void update_memBlock_nr(void);

void HAL_MMC_TxCpltCallback(MMC_HandleTypeDef *hmmc);
void HAL_MMC_RxCpltCallback(MMC_HandleTypeDef *hmmc);
void HAL_MMC_ErrorCallback(MMC_HandleTypeDef *hmmc);

void SDIO_DMA_Reset(void);
void Verificar_DMA_Status(void);
void SDIO_ReadFirstBlock(void);
void SDIO_WriteFirstBlock(void);

MMC_Status MMC_WriteVerify(uint32_t blockStart, uint32_t numBlocks, uint8_t *pTxData, uint8_t *pRxData);
MMC_Status MMC_ReadBlocks(uint32_t blockStart, uint32_t numBlocks, uint8_t *pData);
MMC_Status MMC_WriteBlocks(uint32_t blockStart, uint32_t numBlocks, uint8_t *pData);

void save_data_to_mem(Sensor *sensor);
void Send_Last500_Blocks_UART(void);
void SDIO_ReadAllData(void);

