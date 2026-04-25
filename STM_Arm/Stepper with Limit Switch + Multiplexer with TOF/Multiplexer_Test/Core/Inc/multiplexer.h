/*
 * multiplexer.h
 *
 *  Created on: Apr 24, 2026
 *      Author: Mohamed
 */

#ifndef INC_MULTIPLEXER_H_
#define INC_MULTIPLEXER_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ─── I2C Address ──────────────────────────────────────────────────────────── */
#define TCA9548A_DEFAULT_ADDRESS    0xE0U   /* 0x70 << 1 (HAL format) */

/* ─── Channel Definitions ──────────────────────────────────────────────────── */
#define TCA9548A_CHANNEL_0          0x01U
#define TCA9548A_CHANNEL_1          0x02U
#define TCA9548A_CHANNEL_2          0x04U
#define TCA9548A_CHANNEL_3          0x08U
#define TCA9548A_CHANNEL_4          0x10U
#define TCA9548A_CHANNEL_5          0x20U
#define TCA9548A_CHANNEL_6          0x40U
#define TCA9548A_CHANNEL_7          0x80U
#define TCA9548A_ALL_CHANNELS       0xFFU
#define TCA9548A_NO_CHANNELS        0x00U

/* ─── HAL Timeout ──────────────────────────────────────────────────────────── */
#define TCA9548A_I2C_TIMEOUT_MS     100U

/* ─── Return Status Codes ──────────────────────────────────────────────────── */
typedef enum {
    TCA9548A_OK                 =  0,
    TCA9548A_ERR_INVALID_PARAMS = -1,
    TCA9548A_ERR_I2C            = -2,
    TCA9548A_ERR_NOT_INIT       = -3,
    TCA9548A_ERR_INVALID_CHANNEL = -4,
} TCA9548A_Status_t;

/* ─── Device Handle ────────────────────────────────────────────────────────── */
typedef struct {
    I2C_HandleTypeDef   *hi2c;           /* Pointer to HAL I2C handle         */
    uint8_t              i2c_addr;       /* 8-bit I2C address (default 0xE0)  */
    GPIO_TypeDef        *reset_port;     /* RESET GPIO port (NULL = not used) */
    uint16_t             reset_pin;      /* RESET GPIO pin                    */
    uint8_t              current_channel;/* Currently active channel mask     */
    bool                 is_initialized; /* Driver init flag                  */
} TCA9548A_Dev_t;

/* ─── Public API ───────────────────────────────────────────────────────────── */

/**
 * @brief  Initialize the TCA9548A multiplexer.
 * @param  dev        Pointer to device handle (caller-allocated)
 * @param  hi2c       HAL I2C handle (configured and started by CubeIDE)
 * @param  i2c_addr   8-bit I2C address (default: TCA9548A_DEFAULT_ADDRESS)
 * @param  reset_port GPIO port for RESET pin (NULL if not used)
 * @param  reset_pin  GPIO pin for RESET
 * @retval TCA9548A_OK on success, error code otherwise
 */
TCA9548A_Status_t TCA9548A_Init(TCA9548A_Dev_t  *dev,
                                 I2C_HandleTypeDef *hi2c,
                                 uint8_t            i2c_addr,
                                 GPIO_TypeDef      *reset_port,
                                 uint16_t           reset_pin);

/**
 * @brief  Select a single channel (disables all others).
 * @param  dev       Pointer to initialised device handle
 * @param  channel   Channel number (0-7)
 * @retval TCA9548A_OK on success, error code otherwise
 */
TCA9548A_Status_t TCA9548A_SelectChannel(TCA9548A_Dev_t *dev, uint8_t channel);

/**
 * @brief  Enable multiple channels simultaneously.
 * @param  dev         Pointer to initialised device handle
 * @param  channel_mask Bitmask of channels to enable (use TCA9548A_CHANNEL_X)
 * @retval TCA9548A_OK on success, error code otherwise
 */
TCA9548A_Status_t TCA9548A_SetChannelMask(TCA9548A_Dev_t *dev, uint8_t channel_mask);

/**
 * @brief  Disable all channels (power-up default state).
 * @param  dev Pointer to initialised device handle
 * @retval TCA9548A_OK on success, error code otherwise
 */
TCA9548A_Status_t TCA9548A_DisableAll(TCA9548A_Dev_t *dev);

/**
 * @brief  Read the current channel selection state.
 * @param  dev           Pointer to initialised device handle
 * @param  channel_mask  Output: current channel selection bitmask
 * @retval TCA9548A_OK on success, error code otherwise
 */
TCA9548A_Status_t TCA9548A_ReadChannelMask(TCA9548A_Dev_t *dev, uint8_t *channel_mask);

/**
 * @brief  Hardware reset the multiplexer via RESET pin.
 * @param  dev Pointer to initialised device handle
 * @retval TCA9548A_OK on success, error code otherwise
 */
TCA9548A_Status_t TCA9548A_Reset(TCA9548A_Dev_t *dev);

/**
 * @brief  Scan for devices on a specific channel (debug helper).
 * @param  dev      Pointer to initialised device handle
 * @param  channel  Channel number (0-7) to scan
 * @note   Prints found addresses via printf (requires retargeting)
 */
void TCA9548A_ScanChannel(TCA9548A_Dev_t *dev, uint8_t channel);


#endif /* INC_MULTIPLEXER_H_ */
