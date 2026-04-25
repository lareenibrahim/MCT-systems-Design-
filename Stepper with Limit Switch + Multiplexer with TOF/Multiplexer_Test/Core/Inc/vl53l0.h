/**
 ******************************************************************************
 * @file    vl53l0x.h
 * @brief   VL53L0X Time-of-Flight sensor driver header
 *          Target: STM32 Black Pill (STM32F411CEU6) via STM32 HAL I2C
 *
 * Wiring (Black Pill):
 *   VL53L0X VDD   -> 3.3V
 *   VL53L0X GND   -> GND
 *   VL53L0X SDA   -> PB7  (I2C1_SDA) — 4.7kΩ pull-up to 3.3V
 *   VL53L0X SCL   -> PB6  (I2C1_SCL) — 4.7kΩ pull-up to 3.3V
 *   VL53L0X XSHUT -> PB0  (GPIO output, active LOW reset)
 *   VL53L0X GPIO1 -> PB1  (optional interrupt input)
 *
 * Usage:
 *   1. Configure I2C1 at 400 kHz in STM32CubeIDE (Fast Mode)
 *   2. Configure PB0 as GPIO_Output (XSHUT)
 *   3. Call VL53L0X_Init(&dev, &hi2c1, GPIOB, GPIO_PIN_0)
 *   4. Call VL53L0X_PerformSingleRangingMeasurement(&dev, &data)
 ******************************************************************************
 */

#ifndef VL53L0X_H
#define VL53L0X_H

#include "stm32f4xx_hal.h"   /* Change to match your STM32 family if needed */
#include <stdint.h>
#include <stdbool.h>
#include "multiplexer.h"

/* ─── I2C Address ──────────────────────────────────────────────────────────── */
/* VL53L0X default 7-bit address = 0x29; HAL expects it shifted left by 1     */
#define VL53L0X_DEFAULT_ADDRESS     0x52U   /* 0x29 << 1 */

/* ─── HAL Timeout ──────────────────────────────────────────────────────────── */
#define VL53L0X_I2C_TIMEOUT_MS      100U

/* ─── Sensor Timeouts ──────────────────────────────────────────────────────── */
#define VL53L0X_BOOT_TIMEOUT_MS     500U    /* Max wait for device boot        */
#define VL53L0X_MEAS_TIMEOUT_MS    1000U    /* Max wait for single measurement */

/* ─── Register Map ─────────────────────────────────────────────────────────── */
#define REG_SYSRANGE_START                              0x00U
#define REG_SYSTEM_SEQUENCE_CONFIG                      0x01U
#define REG_SYSTEM_RANGE_CONFIG                         0x09U
#define REG_SYSTEM_INTERRUPT_CONFIG_GPIO                0x0AU
#define REG_SYSTEM_INTERRUPT_CLEAR                      0x0BU
#define REG_SYSTEM_THRESH_HIGH                          0x0CU
#define REG_SYSTEM_THRESH_LOW                           0x0EU
#define REG_SYSTEM_INTERMEASUREMENT_PERIOD              0x04U
#define REG_GPIO_HV_MUX_ACTIVE_HIGH                     0x84U
#define REG_RESULT_INTERRUPT_STATUS                     0x13U
#define REG_RESULT_RANGE_STATUS                         0x14U  /* +10 = range_mm */
#define REG_MSRC_CONFIG_CONTROL                         0x60U
#define REG_PRE_RANGE_CONFIG_MIN_SNR                    0x27U
#define REG_PRE_RANGE_CONFIG_VALID_PHASE_LOW            0x56U
#define REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH           0x57U
#define REG_PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT          0x64U
#define REG_FINAL_RANGE_CONFIG_MIN_SNR                  0x67U
#define REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW          0x47U
#define REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH         0x48U
#define REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT 0x44U
#define REG_PRE_RANGE_CONFIG_SIGMA_THRESH_HI            0x61U
#define REG_PRE_RANGE_CONFIG_SIGMA_THRESH_LO            0x62U
#define REG_PRE_RANGE_CONFIG_VCSEL_PERIOD               0x50U
#define REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI          0x51U
#define REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO          0x52U
#define REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD             0x70U
#define REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI        0x71U
#define REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO        0x72U
#define REG_CROSSTALK_COMPENSATION_PEAK_RATE_MCPS       0x20U
#define REG_MSRC_CONFIG_TIMEOUT_MACROP                  0x46U
#define REG_ALGO_PART_TO_PART_RANGE_OFFSET_MM           0x28U
#define REG_GLOBAL_CONFIG_VCSEL_WIDTH                   0x32U
#define REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0            0xB0U
#define REG_GLOBAL_CONFIG_REF_EN_START_SELECT           0xB6U
#define REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD         0x4EU
#define REG_DYNAMIC_SPAD_REF_EN_START_OFFSET            0x4FU
#define REG_POWER_MANAGEMENT_GO1_POWER_FORCE            0x80U
#define REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV            0x89U
#define REG_ALGO_PHASECAL_CONFIG_TIMEOUT                0x30U
#define REG_IDENTIFICATION_MODEL_ID                     0xC0U  /* Should read 0xEE */
#define REG_IDENTIFICATION_REVISION_ID                  0xC2U
#define REG_I2C_SLAVE_DEVICE_ADDRESS                    0x8AU
#define REG_OSC_CALIBRATE_VAL                           0xF8U

/* ─── Return Status Codes ──────────────────────────────────────────────────── */
typedef enum {
	VL53L0X_OK                  =  0,
	VL53L0X_ERR_INVALID_PARAMS  = -1,
	VL53L0X_ERR_I2C             = -2,
	VL53L0X_ERR_WRONG_DEVICE    = -3,
	VL53L0X_ERR_TIMEOUT         = -4,
	VL53L0X_ERR_NOT_INIT        = -5,
} VL53L0X_Status_t;

/* ─── Range Status (mirrors ST API values) ─────────────────────────────────── */
typedef enum {
	VL53L0X_RANGE_STATUS_VALID         = 0,   /* Measurement valid                 */
	VL53L0X_RANGE_STATUS_SIGMA_FAIL    = 1,   /* Sigma estimator check failed      */
	VL53L0X_RANGE_STATUS_SIGNAL_FAIL   = 2,   /* Signal rate check failed          */
	VL53L0X_RANGE_STATUS_MIN_RANGE     = 3,   /* Target below minimum detection    */
	VL53L0X_RANGE_STATUS_PHASE_FAIL    = 4,   /* Phase out of valid limits         */
	VL53L0X_RANGE_STATUS_HARDWARE_FAIL = 5,   /* Hardware failure                  */
	VL53L0X_RANGE_STATUS_NO_UPDATE     = 255, /* No updated measurement available  */
} VL53L0X_RangeStatus_t;

/* ─── Device Mode ──────────────────────────────────────────────────────────── */
typedef enum {
	VL53L0X_DEVICEMODE_SINGLE_RANGING    = 0,  /* One measurement per trigger       */
	VL53L0X_DEVICEMODE_CONTINUOUS_RANGING = 1, /* Continuous back-to-back ranging   */
} VL53L0X_DeviceMode_t;

/* ─── Limit Check IDs (matching ST API names) ──────────────────────────────── */
#define VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE        0U
#define VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE  1U

/* ─── Ranging Measurement Data ─────────────────────────────────────────────── */
typedef struct {
	uint16_t             RangeMilliMeter;     /* Measured distance in mm           */
	uint8_t              RangeStatus;         /* 0 = valid, see VL53L0X_RangeStatus_t */
	uint32_t             SignalRateRtnMegaCps;/* Signal rate (9.7 fixed-point)     */
} VL53L0X_RangingMeasurementData_t;

/* ─── Device Handle ────────────────────────────────────────────────────────── */
typedef struct {
	I2C_HandleTypeDef   *hi2c;               /* Pointer to HAL I2C handle         */
	uint8_t              i2c_addr;           /* 8-bit I2C address (default 0x52)  */
	GPIO_TypeDef        *xshut_port;         /* XSHUT GPIO port (NULL = not used) */
	uint16_t             xshut_pin;          /* XSHUT GPIO pin                    */
	bool                 is_initialized;     /* Driver init flag                  */
	uint8_t              stop_variable;      /* Internal calibration variable     */
	uint32_t             measurement_timing_budget_us; /* Timing budget in µs     */
	VL53L0X_DeviceMode_t device_mode;        /* Current ranging mode              */
	int8_t               id;                 /* User-assigned sensor ID (optional)*/
	struct {
		bool        mux_enabled;     /* True if sensor is behind multiplexer */
		void       *mux_dev;        /* Pointer to TCA9548A_Dev_t (void* to avoid circular dependency) */
		uint8_t     mux_channel;    /* Multiplexer channel (0-7) */
	} mux_config;
} VL53L0X_Dev_t;

/* ─── Public API ───────────────────────────────────────────────────────────── */

/**
 * @brief  Initialize and calibrate the VL53L0X sensor.
 * @note   Must be called before any other VL53L0X function.
 * @param  dev        Pointer to device handle (caller-allocated)
 * @param  hi2c       HAL I2C handle (configured and started by CubeIDE)
 * @param  xshut_port GPIO port for XSHUT pin (NULL if tied high / not used)
 * @param  xshut_pin  GPIO pin for XSHUT
 * @param  id         User-assigned sensor ID (0, 1, 2 …)
 * @retval VL53L0X_OK on success, error code otherwise
 */
VL53L0X_Status_t VL53L0X_Init(VL53L0X_Dev_t   *dev,
		I2C_HandleTypeDef *hi2c,
		GPIO_TypeDef      *xshut_port,
		uint16_t           xshut_pin,
		int8_t             id);

/**
 * @brief  Perform a single ranging measurement and return the result.
 * @param  dev   Pointer to initialised device handle
 * @param  data  Output: range, status and signal rate
 * @retval VL53L0X_OK on success, error code otherwise
 */
VL53L0X_Status_t VL53L0X_PerformSingleRangingMeasurement(
		VL53L0X_Dev_t                  *dev,
		VL53L0X_RangingMeasurementData_t *data);

/**
 * @brief  Start continuous back-to-back ranging.
 * @param  dev                    Pointer to initialised device handle
 * @param  period_ms              Inter-measurement period in ms (0 = fastest)
 * @retval VL53L0X_OK on success
 */
VL53L0X_Status_t VL53L0X_StartContinuous(VL53L0X_Dev_t *dev,
		uint32_t        period_ms);

/**
 * @brief  Read the latest range value in continuous mode (blocks until ready).
 * @param  dev   Pointer to initialised device handle
 * @param  data  Output: range, status and signal rate
 * @retval VL53L0X_OK on success, VL53L0X_ERR_TIMEOUT if no data within limit
 */
VL53L0X_Status_t VL53L0X_ReadRangeContinuous(
		VL53L0X_Dev_t                  *dev,
		VL53L0X_RangingMeasurementData_t *data);

/**
 * @brief  Stop continuous ranging.
 * @param  dev Pointer to initialised device handle
 * @retval VL53L0X_OK on success
 */
VL53L0X_Status_t VL53L0X_StopContinuous(VL53L0X_Dev_t *dev);

/**
 * @brief  Set the measurement timing budget (accuracy vs. speed trade-off).
 * @param  dev          Pointer to initialised device handle
 * @param  budget_us    Timing budget in microseconds (min ~20000, default 33000)
 * @retval VL53L0X_OK on success
 */
VL53L0X_Status_t VL53L0X_SetMeasurementTimingBudget(VL53L0X_Dev_t *dev,
		uint32_t        budget_us);

/**
 * @brief  Enable or disable a signal quality limit check.
 * @param  dev         Pointer to initialised device handle
 * @param  limit_id    VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE or
 *                     VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE
 * @param  enable      1 = enable, 0 = disable
 * @retval VL53L0X_OK on success
 */
VL53L0X_Status_t VL53L0X_SetLimitCheckEnable(VL53L0X_Dev_t *dev,
		uint8_t         limit_id,
		uint8_t         enable);

/**
 * @brief  Change the I2C address (useful for multi-sensor setups).
 *         After calling this, update dev->i2c_addr before further communication.
 * @param  dev       Pointer to initialised device handle
 * @param  new_addr  New 7-bit address (HAL form: new_addr << 1)
 * @retval VL53L0X_OK on success
 */
VL53L0X_Status_t VL53L0X_SetI2CAddress(VL53L0X_Dev_t *dev, uint8_t new_addr);

/* ─── Low-level I2C helpers (can be used for debugging) ───────────────────── */
VL53L0X_Status_t VL53L0X_WriteReg8 (VL53L0X_Dev_t *dev, uint8_t reg, uint8_t  val);
VL53L0X_Status_t VL53L0X_WriteReg16(VL53L0X_Dev_t *dev, uint8_t reg, uint16_t val);
VL53L0X_Status_t VL53L0X_ReadReg8  (VL53L0X_Dev_t *dev, uint8_t reg, uint8_t  *val);
VL53L0X_Status_t VL53L0X_ReadReg16 (VL53L0X_Dev_t *dev, uint8_t reg, uint16_t *val);
VL53L0X_Status_t VL53L0X_SetMuxChannel(VL53L0X_Dev_t *dev,void *mux_dev,uint8_t mux_channel);

#endif /* VL53L0X_H */
