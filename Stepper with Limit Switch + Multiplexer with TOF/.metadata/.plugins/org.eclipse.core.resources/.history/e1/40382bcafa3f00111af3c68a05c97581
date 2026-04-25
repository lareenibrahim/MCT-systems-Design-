/*
 * multiplexer.c
 *
 *  Created on: Apr 24, 2026
 *      Author: Mohamed
 */


/**
 ******************************************************************************
 * @file    tca9548a.c
 * @brief   TCA9548A I2C 1-to-8 Channel Multiplexer Driver Implementation
 *
 * The TCA9548A is controlled by writing a single byte to its control register.
 * Each bit in the control byte corresponds to one of the 8 channels:
 *   Bit 0 = Channel 0, Bit 1 = Channel 1, ..., Bit 7 = Channel 7
 * Setting a bit to 1 enables the corresponding channel.
 *
 * The channel becomes active after a STOP condition on the I2C bus.
 ******************************************************************************
 */

#include "multiplexer.h"
#include <string.h>
#include <stdio.h>  /* For TCA9548A_ScanChannel debug output */

/* ─── Private Macros ───────────────────────────────────────────────────────── */
#define RETURN_IF_ERR(x)   do { TCA9548A_Status_t _s = (x); \
                                if (_s != TCA9548A_OK) return _s; } while(0)

/* ══════════════════════════════════════════════════════════════════════════════
 *  CORE I2C COMMUNICATION
 * ══════════════════════════════════════════════════════════════════════════════ */

/**
 * @brief  Write the control register (channel selection byte).
 * @param  dev     Pointer to device handle
 * @param  channel_mask  Channel selection bitmask
 * @retval Status
 */
static TCA9548A_Status_t TCA9548A_WriteControl(TCA9548A_Dev_t *dev, uint8_t channel_mask)
{
    if (dev == NULL || dev->hi2c == NULL)
        return TCA9548A_ERR_INVALID_PARAMS;

    HAL_StatusTypeDef rc = HAL_I2C_Master_Transmit(dev->hi2c,
                                                      dev->i2c_addr,
                                                      &channel_mask,
                                                      1,
                                                      TCA9548A_I2C_TIMEOUT_MS);
    if (rc != HAL_OK)
        return TCA9548A_ERR_I2C;

    dev->current_channel = channel_mask;
    return TCA9548A_OK;
}

/**
 * @brief  Read the control register (current channel selection).
 * @param  dev           Pointer to device handle
 * @param  channel_mask  Output: current channel selection
 * @retval Status
 */
static TCA9548A_Status_t TCA9548A_ReadControl(TCA9548A_Dev_t *dev, uint8_t *channel_mask)
{
    if (dev == NULL || dev->hi2c == NULL || channel_mask == NULL)
        return TCA9548A_ERR_INVALID_PARAMS;

    HAL_StatusTypeDef rc = HAL_I2C_Master_Receive(dev->hi2c,
                                                     dev->i2c_addr,
                                                     channel_mask,
                                                     1,
                                                     TCA9548A_I2C_TIMEOUT_MS);
    if (rc != HAL_OK)
        return TCA9548A_ERR_I2C;

    dev->current_channel = *channel_mask;
    return TCA9548A_OK;
}

/* ══════════════════════════════════════════════════════════════════════════════
 *  INITIALIZATION
 * ══════════════════════════════════════════════════════════════════════════════ */

TCA9548A_Status_t TCA9548A_Init(TCA9548A_Dev_t   *dev,
                                 I2C_HandleTypeDef *hi2c,
                                 uint8_t            i2c_addr,
                                 GPIO_TypeDef      *reset_port,
                                 uint16_t           reset_pin)
{
    if (dev == NULL || hi2c == NULL)
        return TCA9548A_ERR_INVALID_PARAMS;

    /* Populate handle */
    memset(dev, 0, sizeof(TCA9548A_Dev_t));
    dev->hi2c          = hi2c;
    dev->i2c_addr      = i2c_addr;
    dev->reset_port    = reset_port;
    dev->reset_pin     = reset_pin;
    dev->current_channel = TCA9548A_NO_CHANNELS;

    /* ── Hardware reset via RESET pin (if available) ───────────────────────── */
    if (reset_port != NULL)
    {
        /* Ensure RESET is high (inactive) */
        HAL_GPIO_WritePin(reset_port, reset_pin, GPIO_PIN_SET);

        /* Pulse RESET low for at least 6ns (we use 1ms for safety) */
        HAL_GPIO_WritePin(reset_port, reset_pin, GPIO_PIN_RESET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(reset_port, reset_pin, GPIO_PIN_SET);
        HAL_Delay(1);  /* Recovery time before start condition */
    }

    /* ── Verify device is responding ───────────────────────────────────────── */
    uint8_t dummy;
    HAL_StatusTypeDef rc = HAL_I2C_Master_Receive(dev->hi2c,
                                                     dev->i2c_addr,
                                                     &dummy,
                                                     1,
                                                     TCA9548A_I2C_TIMEOUT_MS);
    if (rc != HAL_OK)
    {
        /* Device not found at specified address */
        return TCA9548A_ERR_I2C;
    }

    /* ── Ensure all channels are disabled (power-up default) ──────────────── */
    TCA9548A_WriteControl(dev, TCA9548A_NO_CHANNELS);

    dev->is_initialized = true;
    return TCA9548A_OK;
}

/* ══════════════════════════════════════════════════════════════════════════════
 *  CHANNEL SELECTION
 * ══════════════════════════════════════════════════════════════════════════════ */

TCA9548A_Status_t TCA9548A_SelectChannel(TCA9548A_Dev_t *dev, uint8_t channel)
{
    if (!dev->is_initialized)
        return TCA9548A_ERR_NOT_INIT;

    if (channel > 7)
        return TCA9548A_ERR_INVALID_CHANNEL;

    /* Convert channel number to bitmask */
    uint8_t mask = (1U << channel);

    /* Write control register to select only this channel */
    return TCA9548A_WriteControl(dev, mask);
}

TCA9548A_Status_t TCA9548A_SetChannelMask(TCA9548A_Dev_t *dev, uint8_t channel_mask)
{
    if (!dev->is_initialized)
        return TCA9548A_ERR_NOT_INIT;

    /* Write the raw bitmask directly */
    return TCA9548A_WriteControl(dev, channel_mask);
}

TCA9548A_Status_t TCA9548A_DisableAll(TCA9548A_Dev_t *dev)
{
    if (!dev->is_initialized)
        return TCA9548A_ERR_NOT_INIT;

    return TCA9548A_WriteControl(dev, TCA9548A_NO_CHANNELS);
}

TCA9548A_Status_t TCA9548A_ReadChannelMask(TCA9548A_Dev_t *dev, uint8_t *channel_mask)
{
    if (!dev->is_initialized)
        return TCA9548A_ERR_NOT_INIT;

    return TCA9548A_ReadControl(dev, channel_mask);
}

/* ══════════════════════════════════════════════════════════════════════════════
 *  RESET
 * ══════════════════════════════════════════════════════════════════════════════ */

TCA9548A_Status_t TCA9548A_Reset(TCA9548A_Dev_t *dev)
{
    if (!dev->is_initialized)
        return TCA9548A_ERR_NOT_INIT;

    if (dev->reset_port == NULL)
    {
        /* No reset pin available - do software reset by disabling all */
        return TCA9548A_DisableAll(dev);
    }

    /* Hardware reset via RESET pin */
    HAL_GPIO_WritePin(dev->reset_port, dev->reset_pin, GPIO_PIN_RESET);
    HAL_Delay(1);  /* Minimum 6ns, 1ms for safety */
    HAL_GPIO_WritePin(dev->reset_port, dev->reset_pin, GPIO_PIN_SET);
    HAL_Delay(1);  /* Recovery time */

    dev->current_channel = TCA9548A_NO_CHANNELS;
    return TCA9548A_OK;
}

/* ══════════════════════════════════════════════════════════════════════════════
 *  CHANNEL SCAN (DEBUG)
 * ══════════════════════════════════════════════════════════════════════════════ */

void TCA9548A_ScanChannel(TCA9548A_Dev_t *dev, uint8_t channel)
{
    if (!dev->is_initialized || channel > 7)
        return;

    printf("Scanning I2C bus on channel %d...\r\n", channel);

    /* Select the channel */
    if (TCA9548A_SelectChannel(dev, channel) != TCA9548A_OK)
    {
        printf("  Failed to select channel %d\r\n", channel);
        return;
    }

    /* Scan all 7-bit addresses (1-127) */
    uint8_t found = 0;
    for (uint8_t addr = 1; addr < 128; addr++)
    {
        uint8_t hal_addr = addr << 1;

        /* Quick I2C probe: send address with write bit, check for ACK */
        HAL_StatusTypeDef rc = HAL_I2C_IsDeviceReady(dev->hi2c,
                                                        hal_addr,
                                                        1,
                                                        TCA9548A_I2C_TIMEOUT_MS);
        if (rc == HAL_OK)
        {
            printf("  Device found at 0x%02X (7-bit: 0x%02X)\r\n",
                   hal_addr, addr);
            found++;
        }
    }

    if (found == 0)
    {
        printf("  No devices found on channel %d\r\n", channel);
    }
    else
    {
        printf("  Found %d device(s) on channel %d\r\n", found, channel);
    }
}
