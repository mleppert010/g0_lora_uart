/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "lwrb.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/**
 * \brief           Structure of UART associated buffers and indexes
 */
typedef struct {
    lwrb_t tx_rb;                       /**< TX ring buffer for DMA to read out */
    uint8_t tx_rb_data[384];            /**< TX ring buffer data */
    volatile size_t tx_dma_current_len; /**< Current TX DMA transfer length */
    lwrb_t rx_process_rb;               /**< RX data processing ring buffer */
    uint8_t rx_process_rb_data[384];    /**< RX data processing ring buffer data */
    uint8_t rx_dma_buff[64];            /**< RX circular buffer for DMA to write to */
    size_t old_pos;                     /**< Previous DMA write to index */
} uart_buff_t;

/**
 * \brief           Structure of TX DMA controller, channel, and associated flag clearing functions for a UART
 */
typedef struct {
    DMA_TypeDef* controller;             /**< DMA controller (DMA1/DMA2) */
    uint32_t channel;                    /**< DMA Channel */
    void (*clear_flag_TC)(DMA_TypeDef*); /**< Channel specific function pointer to clear TC flag */
    void (*clear_flag_HT)(DMA_TypeDef*); /**< Channel specific function pointer to clear HT flag */
    void (*clear_flag_GI)(DMA_TypeDef*); /**< Channel specific function pointer to clear GI flag */
    void (*clear_flag_TE)(DMA_TypeDef*); /**< Channel specific function pointer to clear TE flag */
} dma_tx_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/**
 * \brief           Should tx be looped back
 */
#define LOOPBACK 1

/**
 * \brief           Calculate length of statically allocated array
 */
#define ARRAY_LEN(x) (sizeof(x) / sizeof((x)[0]))

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uart_buff_t usart1_buff, usart2_buff;

dma_tx_t usart1_dma, usart2_buff;

/**
 * \brief           Ring buffer instance for TX data
 */
// lwrb_t lpuart1_tx_rb, usart1_tx_rb;

/**
 * \brief           Ring buffer data array for TX DMA
 */
// uint8_t lpuart1_tx_rb_data[384], usart1_tx_rb_data[384];

/**
 * \brief           Length of currently active TX DMA transfer
 */
// volatile size_t lpuart1_tx_dma_current_len, usart1_tx_dma_current_len;

/**
 * \brief           USART RX buffer for DMA to transfer every received byte
 * \note            Contains raw data that are about to be processed by different events
 */
// uint8_t lpuart1_rx_dma_buff[64], usart1_rx_dma_buff[64];

/**
 * \brief           DMA RX buffer previous position counter
 */
// size_t lpuart1_old_pos, usart1_old_pos;

/**
 * \brief           Ring buffer instance for processing buffer
 */
// lwrb_t lpuart1_rx_process_rb, usart1_rx_process_rb;

/**
 * \brief           Ring buffer data array for processing
 */
// uint8_t lpuart1_rx_process_rb_data[384], usart1_rx_process_rb_data[384];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void usart1_init(void);
void usart2_init(void);
void uart_rx_check(uart_buff_t* uart_buff);
void uart_process_data(uart_buff_t* uart_buff, const void* data, size_t len);
void uart_send_string(uart_buff_t* uart_buff, dma_tx_t* dma_tx, const char* str);
void uart_send_data(uart_buff_t* uart_buff, dma_tx_t* dma_tx, const void* data, size_t len);
uint8_t uart_start_tx_dma_transfer(uart_buff_t* uart_buff, dma_tx_t* dma_tx);
uint8_t find_crlf(uart_buff_t* uart_buff, size_t peekahead, uint8_t* old_char);
void process_char_loop(uart_buff_t* uart_buff, size_t peekahead, uint8_t* old_char);
uint8_t rylr_send_string(uart_buff_t* uart_buff, dma_tx_t* dma_tx, uint16_t address, char* str);
uint8_t rylr_send_data(uart_buff_t* uart_buff, dma_tx_t* dma_tx, uint16_t add, void* data, size_t len);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

    /* SysTick_IRQn interrupt configuration */
    NVIC_SetPriority(SysTick_IRQn, 3);

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    /* USER CODE BEGIN 2 */

    /* Initialize lpuart1 tx ringbuff */
    lwrb_init(&usart1_buff.tx_rb, usart1_buff.tx_rb_data, sizeof(usart1_buff.tx_rb_data));

    /* Initialize lpuart1 rx processing ringbuffer */
    lwrb_init(&usart1_buff.rx_process_rb, usart1_buff.rx_process_rb_data, sizeof(usart1_buff.rx_process_rb_data));

    /* Initialize usart1 tx ringbuff */
    lwrb_init(&usart2_buff.tx_rb, usart2_buff.tx_rb_data, sizeof(usart2_buff.tx_rb_data));

    /* Initialize usart1 rx processing ringbuffer */
    lwrb_init(&usart2_buff.rx_process_rb, usart2_buff.rx_process_rb_data, sizeof(usart2_buff.rx_process_rb_data));

    /* Initialize all configured peripherals */
    usart1_init();
    usart2_init();

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
    while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_2) {
    }

    /* HSI configuration and activation */
    LL_RCC_HSI_Enable();
    while (LL_RCC_HSI_IsReady() != 1) {
    }

    /* Main PLL configuration and activation */
    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_1, 8, LL_RCC_PLLR_DIV_2);
    LL_RCC_PLL_Enable();
    LL_RCC_PLL_EnableDomain_SYS();
    while (LL_RCC_PLL_IsReady() != 1) {
    }

    /* Set AHB prescaler*/
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

    /* Sysclk activation on the main PLL */
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) {
    }

    /* Set APB1 prescaler*/
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);

    LL_Init1msTick(64000000);

    /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
    LL_SetSystemCoreClock(64000000);
}

/* USER CODE BEGIN 4 */

void usart1_init(void) {
    LL_USART_InitTypeDef USART_InitStruct = {0};
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Peripheral clock enable */
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

    /*
     * LPUART1 GPIO Configuration
     *
     * PA9   ------> USART1_TX
     * PA10   ------> USART1_RX
     */
    GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 RX DMA init */
    LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_1, LL_DMAMUX_REQ_USART1_RX);
    LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PRIORITY_LOW);
    LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MODE_CIRCULAR);
    LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_BYTE);
    LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_BYTE);
    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_1, LL_USART_DMA_GetRegAddr(USART1, LL_USART_DMA_REG_DATA_RECEIVE));
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)usart1_buff.rx_dma_buff);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, ARRAY_LEN(usart1_buff.rx_dma_buff));

    /* USART1 TX DMA init */
    LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_2, LL_DMAMUX_REQ_USART1_TX);
    LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_2, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
    LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PRIORITY_LOW);
    LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MODE_NORMAL);
    LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PDATAALIGN_BYTE);
    LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MDATAALIGN_BYTE);
    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_2, LL_USART_DMA_GetRegAddr(USART1, LL_USART_DMA_REG_DATA_TRANSMIT));

    /* Enable HT & TC interrupts for RX */
    LL_DMA_EnableIT_HT(DMA1, LL_DMA_CHANNEL_1);
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);

    /* Enable HT & TC interrupts for TX */
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_2);

    /* DMA interrupt init for RX & TX */
    NVIC_SetPriority(DMA1_Channel3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
    NVIC_EnableIRQ(DMA1_Channel3_IRQn);
    NVIC_SetPriority(DMA1_Channel4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
    NVIC_EnableIRQ(DMA1_Channel4_IRQn);

    /* Initialize USART1 */
    USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
    USART_InitStruct.BaudRate = 115200;
    USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
    USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
    USART_InitStruct.Parity = LL_USART_PARITY_NONE;
    USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
    USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
    LL_USART_Init(USART1, &USART_InitStruct);
    LL_USART_SetTXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
    LL_USART_SetRXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
    LL_USART_DisableFIFO(USART1);
    LL_USART_EnableDMAReq_RX(USART1);
    LL_USART_EnableDMAReq_TX(USART1);
    LL_USART_EnableIT_IDLE(USART1);

    /* UDART1 interrupt */
    NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
    NVIC_EnableIRQ(USART1_IRQn);

    /* Associated DMA channel struct for USART1 */
    usart1_dma.controller = DMA1;
    usart1_dma.channel = LL_DMA_CHANNEL_2;
    usart1_dma.clear_flag_TC = &LL_DMA_ClearFlag_TC4;
    usart1_dma.clear_flag_GI = &LL_DMA_ClearFlag_GI4;
    usart1_dma.clear_flag_HT = &LL_DMA_ClearFlag_HT4;
    usart1_dma.clear_flag_TE = &LL_DMA_ClearFlag_TE4;

    /* Enable USART and DMA */
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
    LL_USART_Enable(USART1);
    while (!LL_USART_IsActiveFlag_TEACK(USART1) || !LL_USART_IsActiveFlag_REACK(USART1)) {
    }
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line) {
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
