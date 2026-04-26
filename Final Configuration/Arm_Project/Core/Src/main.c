/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : STM32F103C8T6 — NRF24 Receiver — Pair #1 (address 00001)
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "NRF24.h"
#include <string.h>
#include <stdio.h>
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
SPI_HandleTypeDef  hspi1;
TIM_HandleTypeDef  htim1;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t nrfAddress[5] = {'0', '0', '0', '0', '2'};  // PAIR 2 ADDRESS
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
void UART_Print(const char *msg);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void UART_Print(const char *msg) {
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
}
/* USER CODE END 0 */

/* ============================================================================
 * MAIN
 * ============================================================================ */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */

  HAL_Delay(1000);
  UART_Print("=== STM32F103 NRF24 RECEIVER — PAIR 2 ===\r\n");

  // ── NRF24 Init ─────────────────────────────────────────────────────────────
  nrf24_init();
  HAL_Delay(100);

  nrf24_set_channel(76);
  nrf24_data_rate(_250kbps);
  nrf24_set_addr_width(5);
  nrf24_tx_pwr(_0dbm);

  nrf24_flush_tx();
  nrf24_flush_rx();
  nrf24_clear_rx_dr();
  nrf24_clear_tx_ds();
  nrf24_clear_max_rt();

  // Open pipe 0 with pair-1 address
  nrf24_open_rx_pipe(0, nrfAddress);
  nrf24_pipe_pld_size(0, 1);
  nrf24_auto_ack(0, enable);

  // ── SPI sanity check ───────────────────────────────────────────────────────
  uint8_t status = nrf24_r_status();
  char dbg[64];
  snprintf(dbg, sizeof(dbg), "[NRF] STATUS = 0x%02X\r\n", status);
  UART_Print(dbg);

  if      (status == 0xFF) UART_Print("[NRF] ERROR: SPI stuck HIGH — check wiring!\r\n");
  else if (status == 0x00) UART_Print("[NRF] ERROR: SPI stuck LOW  — check wiring!\r\n");
  else                     UART_Print("[NRF] SPI OK — nRF24 responding\r\n");

  nrf24_listen();
  UART_Print("[NRF] Listening on address 00001...\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* ── NRF24 RECEIVE ──────────────────────────────────────────────────── */
    if (nrf24_data_available())
    {
      uint8_t rxByte[1] = {0};
      nrf24_receive(rxByte, 1);
      nrf24_clear_rx_dr();

      char cmd = (char)rxByte[0];

      const char* cmdName = "UNKNOWN";
      if      (cmd == 'f') cmdName = "FORWARD";
      else if (cmd == 'b') cmdName = "BACKWARD";
      else if (cmd == 'l') cmdName = "STRAFE LEFT";
      else if (cmd == 'r') cmdName = "STRAFE RIGHT";
      else if (cmd == 'q') cmdName = "ROTATE LEFT";
      else if (cmd == 'e') cmdName = "ROTATE RIGHT";
      else if (cmd == 's') cmdName = "STOP";

      char log[64];
      snprintf(log, sizeof(log), "[NRF] Received: '%c' --> %s\r\n", cmd, cmdName);
      UART_Print(log);

      // Toggle LED on each received packet (PC13, active LOW)
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    }

    HAL_Delay(10);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/* ============================================================================
 * SYSTEM CLOCK
 * STM32F103C8T6 @ 72 MHz — HSE 8 MHz crystal × PLL 9
 * If your blue pill has no crystal, switch to HSI:
 *   OscillatorType = RCC_OSCILLATORTYPE_HSI
 *   HSIState       = RCC_HSI_ON
 *   PLLSource      = RCC_PLLSOURCE_HSI_DIV2   (gives 64 MHz, change MUL to 16)
 * ============================================================================ */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL     = RCC_PLL_MUL9;        // 8 MHz × 9 = 72 MHz
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK  | RCC_CLOCKTYPE_SYSCLK
                                   | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;       // APB1 max 36 MHz
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) { Error_Handler(); }
}

/* ============================================================================
 * SPI1
 *   SCK  → PA5
 *   MISO → PA6
 *   MOSI → PA7
 *   CSN  → PA4  (GPIO output, software NSS)
 *   CE   → PA3  (GPIO output)
 * ============================================================================ */
static void MX_SPI1_Init(void)
{
  hspi1.Instance               = SPI1;
  hspi1.Init.Mode              = SPI_MODE_MASTER;
  hspi1.Init.Direction         = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize          = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity       = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase          = SPI_PHASE_1EDGE;
  hspi1.Init.NSS               = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16; // 72/16 = 4.5 MHz
  hspi1.Init.FirstBit          = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode            = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial     = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK) { Error_Handler(); }
}

/* ============================================================================
 * TIM1 — used by delay_us() inside NRF24 library
 *   Prescaler = 71 → 72 MHz / 72 = 1 MHz → 1 tick = 1 µs
 * ============================================================================ */
static void MX_TIM1_Init(void)
{
  TIM_ClockConfigTypeDef  sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig      = {0};

  htim1.Instance               = TIM1;
  htim1.Init.Prescaler         = 71;
  htim1.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim1.Init.Period            = 65535;
  htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK) { Error_Handler(); }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) { Error_Handler(); }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) { Error_Handler(); }
}

/* ============================================================================
 * USART1 — debug output (replaces USB CDC which F103C8 lacks by default)
 *   TX → PA9   (connect to RX of your USB-UART adapter)
 *   RX → PA10
 *   Baud: 115200
 * ============================================================================ */
static void MX_USART1_UART_Init(void)
{
  huart1.Instance          = USART1;
  huart1.Init.BaudRate     = 115200;
  huart1.Init.WordLength   = UART_WORDLENGTH_8B;
  huart1.Init.StopBits     = UART_STOPBITS_1;
  huart1.Init.Parity       = UART_PARITY_NONE;
  huart1.Init.Mode         = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK) { Error_Handler(); }
}

/* ============================================================================
 * GPIO
 *   PA3  → NRF CE  (output PP)
 *   PA4  → NRF CSN (output PP)
 *   PA5  → SPI1 SCK  (configured by HAL_SPI_MspInit)
 *   PA6  → SPI1 MISO (configured by HAL_SPI_MspInit)
 *   PA7  → SPI1 MOSI (configured by HAL_SPI_MspInit)
 *   PA9  → USART1 TX (configured by HAL_UART_MspInit)
 *   PA10 → USART1 RX (configured by HAL_UART_MspInit)
 *   PC13 → onboard LED (active LOW)
 * ============================================================================ */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Enable the clocks for Port A, Port B, Port C, and AFIO */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_AFIO_CLK_ENABLE();   // ← Required for remapping

  /* Free up PA15 from JTAG so it can be used as a normal GPIO for CSN */
  __HAL_AFIO_REMAP_SWJ_NOJTAG(); // ← ADD THIS LINE

  /* CE and CSN start LOW */
  HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_RESET);

  /* Configure CSN Pin */
  GPIO_InitStruct.Pin   = NRF_CSN_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(NRF_CSN_GPIO_Port, &GPIO_InitStruct);

  /* Configure CE Pin */
  GPIO_InitStruct.Pin   = NRF_CE_Pin;
  HAL_GPIO_Init(NRF_CE_GPIO_Port, &GPIO_InitStruct);

  /* PC13 onboard LED */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  GPIO_InitStruct.Pin   = GPIO_PIN_13;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

/* ============================================================================
 * ERROR HANDLER — blinks PC13 LED rapidly
 * ============================================================================ */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    for (volatile uint32_t i = 0; i < 500000; i++);
  }
}
