/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    UART/UART_Printf/Src/main.c
  * @author  MCD Application Team
  * @brief   This example shows how to retarget the C library printf function
  *          to the UART.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "DAP.h"
#include "USB.h"

#include <stm32c0xx_hal_def.h>
#include <stm32c0xx_hal_gpio.h>
#include <unistd.h>
#include <errno.h>
#include <stm32c0xx_hal_rcc.h>

#include <stm32c0xx_hal_pcd.h>
#include <stm32c0xx_hal_uart.h>

#include "class/cdc/cdc_device.h"
#include "class/vendor/vendor_device.h"
#include "device/usbd.h"
#include "projdefs.h"
  #include "stm32c0xx_hal_rcc_ex.h"

#include "FreeRTOS.h"
#include "task.h"
#include "ddmi.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void USART1_Init();
static void MX_DMA_Init(void);
/* USER CODE BEGIN PFP */
#if defined(__ICCARM__)
__ATTRIBUTES size_t __write(int, const unsigned char *, size_t);
#endif /* __ICCARM__ */

#if defined(__ICCARM__)
/* New definition from EWARM V9, compatible with EWARM8 */
int iar_fputc(int ch);
#define PUTCHAR_PROTOTYPE int iar_fputc(int ch)
#elif defined ( __CC_ARM ) || defined(__ARMCC_VERSION)
/* ARM Compiler 5/6*/
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#elif defined(__GNUC__)
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#endif /* __ICCARM__ */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */


void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    /* USER CODE BEGIN vApplicationStackOverflowHook */
    /* This function is called when a stack overflow is detected */
    (void)xTask;  // Avoid unused parameter warning
    (void)pcTaskName;
    /* Add your error handling: e.g., log error, blink LED, or halt */
    while (1) {
        /* Infinite loop to halt execution for debugging */
    }
    /* USER CODE END vApplicationStackOverflowHook */
}


uint32_t HAL_GetTick(void)
{
    return xTaskGetTickCount(); // Returns FreeRTOS tick count in ms
}

static uint8_t dap_packet_buffer[512];
static volatile uint32_t dap_packet_len = 0;
static uint8_t response_buffer[512];
static uint32_t response_len = 0;
static TaskHandle_t dap_task_handle = NULL;
static TaskHandle_t ddmi_task_handle = NULL;
static TaskHandle_t usb_dap_tx_task_handle = NULL;
// Task Handles
TaskHandle_t bridgeTaskHandle;

// Buffers (64 Bytes as requested)
#define BRIDGE_BUF_SIZE 64
uint8_t uart_rx_buf[BRIDGE_BUF_SIZE];
uint8_t usb_tx_buf[BRIDGE_BUF_SIZE];

// Logic Flags
volatile uint8_t uart_tx_busy = 0;


uint8_t uart_host_2_target_buf[64];

int _write(int file, char *ptr, int len)
{
    if (file == STDOUT_FILENO || file == STDERR_FILENO) {
        HAL_StatusTypeDef status = HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
        if (status == HAL_OK) {
            return len; // Success, return number of bytes written
        }
        errno = EIO; // I/O error if transmission fails
        return -1;
    }
    errno = EBADF; // Bad file descriptor for non-stdout/stderr
    return -1;
}

bool ddmi_init_complete = 0;


void tud_vendor_rx_cb(uint8_t idx, const uint8_t *buffer, uint32_t bufsize) {
  // Wake up the DAP processing task
  if(idx == 0)
  {
    xTaskNotifyGive(dap_task_handle);
    taskYIELD();
  }
  if(idx == 1)
  {
    if(!ddmi_init_complete)
    {
      ddmi_init();
      ddmi_init_complete = 1;
    }
    xTaskNotifyGive(ddmi_task_handle);
    taskYIELD();
  }
}


void dap_worker_task(void *pvParameters) {

  while (1) {
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
      dap_packet_len = tud_vendor_n_read(0, dap_packet_buffer, sizeof(dap_packet_buffer));
    
      response_len= DAP_ExecuteCommand(dap_packet_buffer, 
        response_buffer) & 0xFFFF;
      tud_vendor_n_write(0, response_buffer, response_len);
      tud_vendor_n_write_flush(0);
  }
}


// Invoked when cdc when line state changed e.g connected/disconnected
// Use to reset to DFU when disconnect with 1200 bps
void tud_cdc_line_state_cb(uint8_t instance, bool dtr, bool rts) 
{
    if (dtr) 
    {
        cdc_line_coding_t coding;
        tud_cdc_get_line_coding(&coding);
    }
}
  


// Move the UART configuration to the LINE CODING callback instead
void tud_cdc_line_coding_cb(uint8_t itf, cdc_line_coding_t const* p_line_coding)
{
    __HAL_UART_DISABLE(&huart1);
    huart1.Init.BaudRate = p_line_coding->bit_rate;
    HAL_UART_Init(&huart1);
}


/* Private variables ---------------------------------------------------------*/
// We keep track of where we last stopped reading in the circular buffer
static uint32_t last_uart_rx_pos = 0; 

/* ... (other variables remain the same) ... */

void StartBridgeTask(void *argument)
{
  /* 1. Start UART RX DMA in CIRCULAR mode */
  /* The DMA will now loop back to index 0 automatically when it hits 64 */
  HAL_UART_Receive_DMA(&huart1, uart_rx_buf, BRIDGE_BUF_SIZE);

  uint32_t ulNotificationValue;
  const TickType_t xMaxBlockTime = pdMS_TO_TICKS(1); 

  for(;;)
  {
    // Wait 10ms or until notified of a full buffer
    ulNotificationValue = ulTaskNotifyTake(pdTRUE, xMaxBlockTime);

    // ============================================================
    // DIRECTION 1: UART -> USB (Circular logic)
    // ============================================================
    
    // CNDTR counts down from 64 to 0.
    uint32_t current_dma_pos = BRIDGE_BUF_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
    
    if (current_dma_pos != last_uart_rx_pos)
    {
        if (current_dma_pos > last_uart_rx_pos) 
        {
            // Case A: Data is linear in the buffer
            uint32_t len = current_dma_pos - last_uart_rx_pos;
            if (tud_cdc_connected()) {
                tud_cdc_write(&uart_rx_buf[last_uart_rx_pos], len);
            }
        }
        else 
        {
            // Case B: DMA has wrapped around (Circular)
            // Send from last position to end of buffer
            uint32_t len1 = BRIDGE_BUF_SIZE - last_uart_rx_pos;
            // Send from start of buffer to current position
            uint32_t len2 = current_dma_pos;

            if (tud_cdc_connected()) {
                tud_cdc_write(&uart_rx_buf[last_uart_rx_pos], len1);
                tud_cdc_write(&uart_rx_buf[0], len2);
            }
        }
        
        tud_cdc_write_flush();
        last_uart_rx_pos = current_dma_pos;
    }

    // ============================================================
    // DIRECTION 2: USB -> UART (Best Effort)
    // ============================================================
    
    if (uart_tx_busy == 0) 
    {
        uint32_t usb_avail = tud_cdc_available();
        if (usb_avail > 0) 
        {
            // To respect "best effort", we only pull what fits in our 64B block
            uint32_t read_len = tud_cdc_read(usb_tx_buf, (usb_avail > 64) ? 64 : usb_avail);
            
            if (read_len > 0) {
                uart_tx_busy = 1;
                HAL_UART_Transmit_DMA(&huart1, usb_tx_buf, read_len);
            }
        }
    }
  }
}

/* ---------------------------------------------------------------------------
   INTERRUPT CALLBACKS
--------------------------------------------------------------------------- */

// 1. UART RX Complete (DMA Buffer Overflow case)
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1) {
    // Notify the task immediately! This overrides the 10ms wait.
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(bridgeTaskHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}

// 2. UART TX Complete
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1) {
    uart_tx_busy = 0; // Clear busy flag, ready for next batch
  }
}

// 3. TinyUSB RX Callback (USB Buffer Overflow case)
// Called by TinyUSB interrupt context when data arrives
void tud_cdc_rx_cb(uint8_t itf)
{
    // Check if we have enough data to trigger an "Overflow" reaction immediately
    if (tud_cdc_available() >= BRIDGE_BUF_SIZE) {
        xTaskNotifyGive(bridgeTaskHandle);
        portYIELD();
    }
}
/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* STM32C0xx HAL library initialization:
       - Configure the Flash prefetch
       - Systick timer is configured by default as source of time base, but user 
         can eventually implement his proper time base source (a general purpose 
         timer for example or other time source), keeping in mind that Time base 
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
         handled in milliseconds basis.
       - Low Level Initialization
     */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* Initialize BSP Led for LED1 */
  //BSP_LED_Init(LED1);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  USART1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  DAP_Setup();
  //ddmi_init();

  //HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6, 1);
  xTaskCreate(StartBridgeTask, "CDC_Bridge_Task", 512, NULL, 1, &bridgeTaskHandle);

  xTaskCreate(usb_task, "USB", 256, NULL, 3, NULL);
  xTaskCreate(dap_worker_task, "DAP worker", 512, NULL, 4, &dap_task_handle);

  xTaskCreate(ddmi_worker_task, "DDMI worker", 512, NULL, 4, &ddmi_task_handle);
  vTaskStartScheduler();
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
  void SystemClock_Config(void)
  {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_CRSInitTypeDef RCC_CRSInitStruct = {0};
  
    __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_1);
  
    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
      Error_Handler();
    }
  
    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
    {
      Error_Handler();
    }
  
    /** Enable the CRS APB clock
    */
    __HAL_RCC_CRS_CLK_ENABLE();
  
    /** Configures CRS
    */
    RCC_CRSInitStruct.Prescaler = RCC_CRS_SYNC_DIV1;
    RCC_CRSInitStruct.Source = RCC_CRS_SYNC_SOURCE_USB;
    RCC_CRSInitStruct.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
    RCC_CRSInitStruct.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000,1000);
    RCC_CRSInitStruct.ErrorLimitValue = 34;
    RCC_CRSInitStruct.HSI48CalibrationValue = 32;
  
    HAL_RCCEx_CRSConfig(&RCC_CRSInitStruct);
  }

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

static void USART1_Init()
{
  huart1.Instance = USART1;

  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;

  if(HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER DMA CODE INSERTION BEGIN */
    
    /* USART1_RX Init */
    hdma_usart1_rx.Instance = DMA1_Channel1;
    hdma_usart1_rx.Init.Request = DMA_REQUEST_USART1_RX;
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode = DMA_CIRCULAR; // Kept as requested
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(&huart1, hdmarx, hdma_usart1_rx);

    /* USART1_TX Init */
    hdma_usart1_tx.Instance = DMA1_Channel2;
    hdma_usart1_tx.Init.Request = DMA_REQUEST_USART1_TX;
    hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx.Init.Mode = DMA_NORMAL;
    hdma_usart1_tx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(&huart1, hdmatx, hdma_usart1_tx);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 9, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    
  /* USER DMA CODE INSERTION END */
}

static void MX_DMA_Init(void)
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 9, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 9, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
}
/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART2 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}

#if defined(__ICCARM__)
size_t __write(int file, unsigned char const *ptr, size_t len)
{
  size_t idx;
  unsigned char const *pdata = ptr;

  for (idx = 0; idx < len; idx++)
  {
    iar_fputc((int)*pdata);
    pdata++;
  }
  return len;
}
#endif /* __ICCARM__ */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while (1)
  {
    /* Toggle LED1 for error */
    //BSP_LED_Toggle(LED1);
    HAL_Delay(500);
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
