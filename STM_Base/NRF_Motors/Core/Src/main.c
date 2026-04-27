/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include "Hiwonder_Motor.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "usbd_cdc_if.h"
#include "NRF24.h"
#include "robot_control.h"
#include "motor_test.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PD */
/*
 * OPERATING MODE SELECT
 * Uncomment exactly ONE of the two lines below, or leave both commented.
 *
 * MODE_NRF_TEST_ONLY — radio only. No motors initialised, no PID.
 *   Use this first to confirm NRF is receiving commands correctly
 *   before connecting motors. Prints every received command to USB.
 *
 * MODE_RAW_PWM — motors run open loop. No PID, no kinematics.
 *   Use this as a sanity check that motors physically respond to
 *   commands before trusting the PID system. Same as your original
 *   working code.
 *
 * Both commented out — full PID mode. NRF and USB commands go through
 *   inverse kinematics and closed-loop PID. This is normal operation.
 */
// #define MODE_NRF_TEST_ONLY
// #define MODE_RAW_PWM
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef  hadc1;
I2C_HandleTypeDef  hi2c1;
SPI_HandleTypeDef  hspi2;
SPI_HandleTypeDef  hspi3;
TIM_HandleTypeDef  htim1;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
int32_t  encoders[4];
uint16_t battery_voltage;
float    distances[4];
float    rpms[4];

/*
 * controller_input is only used in full PID mode.
 * Declared here for all modes to keep compilation clean.
 */
ControllerInput controller_input;

uint8_t nrfAddress[5] = {'0','0','0','0','1'};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM1_Init(void);

/* USER CODE BEGIN 0 */
void USB_Print(const char *msg) {
    CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
    HAL_Delay(10);
}
/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USB_DEVICE_Init();
  MX_SPI3_Init();
  MX_TIM1_Init();

  /* USER CODE BEGIN 2 */
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOB_CLK_ENABLE();
  GPIO_InitStruct.Pin       = GPIO_PIN_8 | GPIO_PIN_9;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* ── MOTOR INIT ─────────────────────────────────────────────────────────
   * Skipped entirely in NRF_TEST_ONLY mode — no I2C traffic to motor board.
   * In RAW_PWM and full PID mode, motors are always initialised.
   * ----------------------------------------------------------------------- */
#ifndef MODE_NRF_TEST_ONLY
  HAL_Delay(2000);
  Motor_Init(&hi2c1);

  HAL_StatusTypeDef i2c_check = Motor_SetType(0);
  if (i2c_check != HAL_OK) {
      while (1) {
          HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
          HAL_Delay(200);
      }
  }
  HAL_Delay(100);
  Motor_SetPolarity(0);
  HAL_Delay(100);
  Motor_ResetEncoders();
  HAL_Delay(100);
#endif

  /* ── CALIBRATION / TEST MODE ─────────────────────────────────────────────
   * Only available when motors are initialised (not in NRF_TEST_ONLY).
   * Uncomment Motor_Test_Run() to enter test mode on startup.
   * Send 'x' over USB to exit and continue to normal operation.
   * Comment back out for deployment.
   * ----------------------------------------------------------------------- */
#ifndef MODE_NRF_TEST_ONLY
 // Motor_Test_Run();
#endif

  /* ── PID INIT ────────────────────────────────────────────────────────────
   * Only initialised in full PID mode.
   * RAW_PWM mode does not use PID structs.
   * ----------------------------------------------------------------------- */
#if !defined(MODE_NRF_TEST_ONLY) && !defined(MODE_RAW_PWM)
  Robot_Init();
#endif

  /* ── NRF INIT ────────────────────────────────────────────────────────────
   * Identical setup across all three modes.
   * ----------------------------------------------------------------------- */
  HAL_Delay(3000);

#if defined(MODE_NRF_TEST_ONLY)
  USB_Print("=== MODE: NRF TEST ONLY (no motors) ===\r\n");
#elif defined(MODE_RAW_PWM)
  USB_Print("=== MODE: RAW PWM (open loop, no PID) ===\r\n");
#else
  USB_Print("=== MODE: FULL PID ===\r\n");
#endif

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

  nrf24_open_rx_pipe(0, nrfAddress);
  nrf24_pipe_pld_size(0, 1);
  nrf24_auto_ack(0, enable);

  uint8_t nrf_status = nrf24_r_status();
  char dbg[60];
  snprintf(dbg, sizeof(dbg), "[NRF] STATUS = 0x%02X\r\n", nrf_status);
  USB_Print(dbg);

  if      (nrf_status == 0xFF) USB_Print("[NRF] ERROR: SPI stuck high\r\n");
  else if (nrf_status == 0x00) USB_Print("[NRF] ERROR: SPI stuck low\r\n");
  else                         USB_Print("[NRF] SPI OK\r\n");

  nrf24_listen();
  USB_Print("[NRF] Listening...\r\n");
  /* USER CODE END 2 */

  /* USER CODE BEGIN WHILE */
  while (1)
  {
      /* ── 1. NRF RECEIVE ──────────────────────────────────────────────── */
      if (nrf24_data_available())
      {
          uint8_t rxByte[1] = {0};
          nrf24_receive(rxByte, 1);
          nrf24_clear_rx_dr();
          char cmd = (char)rxByte[0];

#if defined(MODE_NRF_TEST_ONLY)
          /* ── NRF TEST ONLY: just print, no motors ─────────────────────
           * This is your original NRF test block, kept exactly as it was.
           * Use this to verify radio is receiving before touching motors.
           * ------------------------------------------------------------ */
          const char *cmdName = "UNKNOWN";
          if      (cmd == 'f') cmdName = "FORWARD";
          else if (cmd == 'b') cmdName = "BACKWARD";
          else if (cmd == 'l') cmdName = "STRAFE LEFT";
          else if (cmd == 'r') cmdName = "STRAFE RIGHT";
          else if (cmd == 'q') cmdName = "ROTATE LEFT";
          else if (cmd == 'e') cmdName = "ROTATE RIGHT";
          else if (cmd == 's') cmdName = "STOP";
          char log_nrf[60];
          snprintf(log_nrf, sizeof(log_nrf),
              "[NRF] Received: '%c' --> %s\r\n", cmd, cmdName);
          USB_Print(log_nrf);

#elif defined(MODE_RAW_PWM)
          /* ── RAW PWM: open loop, no PID ───────────────────────────────
           * This is your original working motor drive block, kept exactly
           * as it was. Use this to confirm motors physically respond to
           * commands before trusting the PID system.
           * ------------------------------------------------------------ */
          s8 pwm[4] = {0, 0, 0, 0};
          if      (cmd == 'f') { s8 p[4] = { 50,  50,  50,  50}; memcpy(pwm, p, 4); }
          else if (cmd == 'b') { s8 p[4] = {-50, -50, -50, -50}; memcpy(pwm, p, 4); }
          else if (cmd == 'l') { s8 p[4] = {-50,  50,  50, -50}; memcpy(pwm, p, 4); }
          else if (cmd == 'r') { s8 p[4] = { 50, -50, -50,  50}; memcpy(pwm, p, 4); }
          else if (cmd == 'q') { s8 p[4] = {-50,  50, -50,  50}; memcpy(pwm, p, 4); }
          else if (cmd == 'e') { s8 p[4] = { 50, -50,  50, -50}; memcpy(pwm, p, 4); }
          else if (cmd == 's') { s8 p[4] = {  0,   0,   0,   0}; memcpy(pwm, p, 4); }
          Motor_SetPWM(pwm[0], pwm[1], pwm[2], pwm[3]);
          char log_pwm[60];
          snprintf(log_pwm, sizeof(log_pwm),
              "[NRF] CMD:'%c' | PWM:%d %d %d %d\r\n",
              cmd, pwm[0], pwm[1], pwm[2], pwm[3]);
          USB_Print(log_pwm);

#else
          /* ── FULL PID: kinematics + closed loop ───────────────────────
           * NRF command fills the ControllerInput struct.
           * Robot_UpdateControl() converts it to RPM targets via kinematics.
           * Robot_PIDUpdate() closes the loop using encoder feedback.
           * ------------------------------------------------------------ */
          controller_input = (ControllerInput){0};
          if      (cmd == 'f') controller_input.forward      = 1;
          else if (cmd == 'b') controller_input.backward     = 1;
          else if (cmd == 'l') controller_input.strafe_left  = 1;
          else if (cmd == 'r') controller_input.strafe_right = 1;
          else if (cmd == 'q') controller_input.rotate_left  = 1;
          else if (cmd == 'e') controller_input.rotate_right = 1;
          else if (cmd == 's') controller_input.stop         = 1;
          char log_pid[40];
          snprintf(log_pid, sizeof(log_pid), "[NRF] CMD:'%c'\r\n", cmd);
          USB_Print(log_pid);
#endif
      }

      /* ── 2. USB FALLBACK COMMANDS ────────────────────────────────────── */
      extern uint8_t UserRxBufferFS[];
      char usb_cmd = UserRxBufferFS[0];

      if (usb_cmd != 0)
      {
#if defined(MODE_NRF_TEST_ONLY)
          /* ── NRF TEST ONLY: just print USB command ────────────────────
           * Your original USB print block, kept exactly as it was.
           * ------------------------------------------------------------ */
          char usblog[40];
          snprintf(usblog, sizeof(usblog),
              "[USB] Received: '%c'\r\n", usb_cmd);
          USB_Print(usblog);
          UserRxBufferFS[0] = 0;

#elif defined(MODE_RAW_PWM)
          /* ── RAW PWM: open loop USB commands ──────────────────────────
           * Your original USB motor drive block, kept exactly as it was.
           * ------------------------------------------------------------ */
          s8 pwm[4] = {0, 0, 0, 0};
          if      (usb_cmd == 'f') { s8 p[4] = { 50,  50,  50,  50}; memcpy(pwm,p,4); USB_Print("Motors: FORWARD\r\n");   }
          else if (usb_cmd == 'b') { s8 p[4] = {-50, -50, -50, -50}; memcpy(pwm,p,4); USB_Print("Motors: BACKWARD\r\n");  }
          else if (usb_cmd == 'l') { s8 p[4] = {-50,  50,  50, -50}; memcpy(pwm,p,4); USB_Print("Motors: STRAFE L\r\n");  }
          else if (usb_cmd == 'r') { s8 p[4] = { 50, -50, -50,  50}; memcpy(pwm,p,4); USB_Print("Motors: STRAFE R\r\n");  }
          else if (usb_cmd == 'q') { s8 p[4] = {-50,  50, -50,  50}; memcpy(pwm,p,4); USB_Print("Motors: ROTATE L\r\n");  }
          else if (usb_cmd == 'e') { s8 p[4] = { 50, -50,  50, -50}; memcpy(pwm,p,4); USB_Print("Motors: ROTATE R\r\n");  }
          else if (usb_cmd == 's') { s8 p[4] = {  0,   0,   0,   0}; memcpy(pwm,p,4); USB_Print("Motors: STOP\r\n");      }
          Motor_SetPWM(pwm[0], pwm[1], pwm[2], pwm[3]);
          UserRxBufferFS[0] = 0;

#else
          /* ── FULL PID: USB commands + live gain tuning ────────────────
           * Motor commands fill the struct same as NRF.
           * P/I/D commands let you tune gains live without reflashing.
           * Type "P1.5" to set Kp=1.5, "I0.3" to set Ki=0.3, etc.
           * ------------------------------------------------------------ */
          UserRxBufferFS[0] = 0;
          controller_input = (ControllerInput){0};

          if      (usb_cmd == 'f') { controller_input.forward      = 1; USB_Print("[USB] FORWARD\r\n");  }
          else if (usb_cmd == 'b') { controller_input.backward     = 1; USB_Print("[USB] BACKWARD\r\n"); }
          else if (usb_cmd == 'l') { controller_input.strafe_left  = 1; USB_Print("[USB] STRAFE L\r\n"); }
          else if (usb_cmd == 'r') { controller_input.strafe_right = 1; USB_Print("[USB] STRAFE R\r\n"); }
          else if (usb_cmd == 'q') { controller_input.rotate_left  = 1; USB_Print("[USB] ROTATE L\r\n"); }
          else if (usb_cmd == 'e') { controller_input.rotate_right = 1; USB_Print("[USB] ROTATE R\r\n"); }
          else if (usb_cmd == 's') { controller_input.stop         = 1; USB_Print("[USB] STOP\r\n");     }
          else if (usb_cmd == 'P' || usb_cmd == 'I' || usb_cmd == 'D') {
              float val = atof((char*)&UserRxBufferFS[1]);
              Robot_SetGains(usb_cmd, val);
              char ack[50];
              snprintf(ack, sizeof(ack), "[USB] SET %c=%.4f\r\n", usb_cmd, val);
              USB_Print(ack);
          }
#endif
      }

      /* ── 3. CONTROL LOOP AND TELEMETRY ───────────────────────────────── */
#if defined(MODE_NRF_TEST_ONLY)
      /* ── NRF TEST ONLY: no motors, short delay ────────────────────── */
      HAL_Delay(10);

#elif defined(MODE_RAW_PWM)
      /* ── RAW PWM: read telemetry, no PID ─────────────────────────── */
      Motor_ReadVoltage(&battery_voltage);
      Motor_ReadEncoders(encoders);
      Motor_GetKinematics(distances, rpms);

      char msg_raw[200];
      snprintf(msg_raw, sizeof(msg_raw),
          "VOLT:%u mV | ENC:%ld,%ld,%ld,%ld | RPM:%.1f,%.1f,%.1f,%.1f | DIST:%.3f,%.3f,%.3f,%.3f\r\n",
          battery_voltage,
          encoders[0], encoders[1], encoders[2], encoders[3],
          rpms[0],     rpms[1],     rpms[2],     rpms[3],
          distances[0],distances[1],distances[2],distances[3]);
      USB_Print(msg_raw);
      HAL_Delay(500);

#else
      /* ── FULL PID: kinematics → PID → telemetry ───────────────────
       * Robot_UpdateControl : maps controller_input to (Vx,Vy,omega),
       *   runs inverse kinematics, sets RPM targets in PID structs.
       * Motor_GetKinematics : reads encoders, computes actual RPM.
       *   Must come before PIDUpdate so values are fresh.
       * Robot_PIDUpdate     : compares target vs actual RPM, computes
       *   P+I+D correction, sends PWM to Hiwonder board.
       * ------------------------------------------------------------ */
      Robot_UpdateControl(&controller_input);
      Motor_GetKinematics(distances, rpms);
      Robot_PIDUpdate(rpms);

      Motor_ReadVoltage(&battery_voltage);
      Motor_ReadEncoders(encoders);

      float targets[4];
      Robot_GetTargets(targets);

      char msg_pid[250];
      snprintf(msg_pid, sizeof(msg_pid),
          "VOLT:%u | ENC:%ld,%ld,%ld,%ld | RPM:%.1f,%.1f,%.1f,%.1f | TGT:%.1f,%.1f,%.1f,%.1f\r\n",
          battery_voltage,
          encoders[0], encoders[1], encoders[2], encoders[3],
          rpms[0],     rpms[1],     rpms[2],     rpms[3],
          targets[0],  targets[1],  targets[2],  targets[3]);
      USB_Print(msg_pid);
      HAL_Delay(500);
#endif

  }
  /* USER CODE END WHILE */
}

/* ── Peripheral init — all unchanged from your original ── */

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM       = 25;
  RCC_OscInitStruct.PLL.PLLN       = 336;
  RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ       = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();
  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                   |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) Error_Handler();
}

static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  hadc1.Instance                   = ADC1;
  hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode          = DISABLE;
  hadc1.Init.ContinuousConvMode    = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion       = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();
  sConfig.Channel      = ADC_CHANNEL_4;
  sConfig.Rank         = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();
}

static void MX_I2C1_Init(void)
{
  hi2c1.Instance             = I2C1;
  hi2c1.Init.ClockSpeed      = 100000;
  hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1     = 0;
  hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2     = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) Error_Handler();
}

static void MX_SPI2_Init(void)
{
  hspi2.Instance               = SPI2;
  hspi2.Init.Mode              = SPI_MODE_MASTER;
  hspi2.Init.Direction         = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize          = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity       = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase          = SPI_PHASE_1EDGE;
  hspi2.Init.NSS               = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi2.Init.FirstBit          = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode            = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial     = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK) Error_Handler();
}

static void MX_SPI3_Init(void)
{
  hspi3.Instance               = SPI3;
  hspi3.Init.Mode              = SPI_MODE_MASTER;
  hspi3.Init.Direction         = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize          = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity       = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase          = SPI_PHASE_1EDGE;
  hspi3.Init.NSS               = SPI_NSS_HARD_INPUT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit          = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode            = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial     = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK) Error_Handler();
}

static void MX_TIM1_Init(void)
{
  TIM_ClockConfigTypeDef  sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig      = {0};
  htim1.Instance               = TIM1;
  htim1.Init.Prescaler         = 83;
  htim1.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim1.Init.Period            = 65535;
  htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK) Error_Handler();
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) Error_Handler();
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) Error_Handler();
}

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
  if (HAL_UART_Init(&huart1) != HAL_OK) Error_Handler();
}

static void MX_USART2_UART_Init(void)
{
  huart2.Instance          = USART2;
  huart2.Init.BaudRate     = 115200;
  huart2.Init.WordLength   = UART_WORDLENGTH_8B;
  huart2.Init.StopBits     = UART_STOPBITS_1;
  huart2.Init.Parity       = UART_PARITY_NONE;
  huart2.Init.Mode         = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOA, NRF_CE_Pin|NRF_CSN_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin  = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin  = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin   = NRF_CE_Pin|NRF_CSN_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin  = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin       = GPIO_PIN_10;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin       = GPIO_PIN_3;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_I2C2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif
