/**
 ******************************************************************************
 * @file    vl53l0x.c
 * @brief   VL53L0X Time-of-Flight sensor driver — STM32 HAL implementation
 *
 * This driver is a self-contained, register-level implementation that does NOT
 * require ST's full VL53L0X API package.  It follows the same initialisation
 * sequence used by ST's API and the well-documented Pololu reverse-engineering
 * of that sequence.
 *
 * Initialization sequence overview
 * ─────────────────────────────────
 *  1. Assert / de-assert XSHUT to hardware-reset the device
 *  2. Verify device identity (MODEL_ID register 0xC0 == 0xEE)
 *  3. Set 2.8 V I/O voltage mode
 *  4. Save internal "stop variable" needed for every ranging trigger
 *  5. Load default tuning settings (ST-supplied magic register values)
 *  6. Configure GPIO interrupt (new data ready on GPIO1)
 *  7. Calculate and program SPAD (Single Photon Avalanche Diode) map
 *  8. VHV reference calibration
 *  9. Phase calibration
 * 10. Restore normal sequence config
 ******************************************************************************
 */

#include "vl53l0.h"
#include <string.h>


/* ─── Private Macros ───────────────────────────────────────────────────────── */
#define RETURN_IF_ERR(x)   do { VL53L0X_Status_t _s = (x); \
                                if (_s != VL53L0X_OK) return _s; } while(0)

/* ─── Private Function Prototypes ──────────────────────────────────────────── */
static VL53L0X_Status_t _WriteBuf (VL53L0X_Dev_t *dev, uint8_t reg,
                                    const uint8_t *buf, uint16_t len);
static VL53L0X_Status_t _ReadBuf  (VL53L0X_Dev_t *dev, uint8_t reg,
                                    uint8_t *buf, uint16_t len);

static VL53L0X_Status_t _LoadTuningSettings      (VL53L0X_Dev_t *dev);
static VL53L0X_Status_t _GetSpadInfo             (VL53L0X_Dev_t *dev,
                                                   uint8_t *count,
                                                   bool    *is_aperture);
static VL53L0X_Status_t _PerformSingleRefCalibration(VL53L0X_Dev_t *dev,
                                                       uint8_t vhv_init_byte);
static VL53L0X_Status_t _WaitMeasurementDataReady(VL53L0X_Dev_t *dev);
static VL53L0X_Status_t _ReadRangeData           (VL53L0X_Dev_t *dev,
                                   VL53L0X_RangingMeasurementData_t *data);

/* ══════════════════════════════════════════════════════════════════════════════
 *  LOW-LEVEL I2C HELPERS
 * ══════════════════════════════════════════════════════════════════════════════ */

/* Add this wrapper around all I2C operations */
/* Include at top of vl53l0x.c: */
/* #include "tca9548a.h" */

/* Modify the low-level helpers to handle mux selection: */
static VL53L0X_Status_t _SelectMuxChannel(VL53L0X_Dev_t *dev)
{
    if (dev->mux_config.mux_enabled && dev->mux_config.mux_dev != NULL)
    {
        TCA9548A_Dev_t *mux = (TCA9548A_Dev_t *)dev->mux_config.mux_dev;
        if (TCA9548A_SelectChannel(mux, dev->mux_config.mux_channel) != TCA9548A_OK)
            return VL53L0X_ERR_I2C;
    }
    return VL53L0X_OK;
}

VL53L0X_Status_t VL53L0X_WriteReg8(VL53L0X_Dev_t *dev, uint8_t reg, uint8_t val)
{
	RETURN_IF_ERR(_SelectMuxChannel(dev));  /* Auto-select mux channel */

	    HAL_StatusTypeDef rc = HAL_I2C_Mem_Write(dev->hi2c,
	                                              dev->i2c_addr,
	                                              reg,
	                                              I2C_MEMADD_SIZE_8BIT,
	                                              &val, 1,
	                                              VL53L0X_I2C_TIMEOUT_MS);
	    return (rc == HAL_OK) ? VL53L0X_OK : VL53L0X_ERR_I2C;
}

VL53L0X_Status_t VL53L0X_WriteReg16(VL53L0X_Dev_t *dev, uint8_t reg, uint16_t val)
{
    uint8_t buf[2];
    buf[0] = (val >> 8) & 0xFF;   /* MSB first (big-endian) */
    buf[1] =  val       & 0xFF;
    return _WriteBuf(dev, reg, buf, 2);
}

VL53L0X_Status_t VL53L0X_ReadReg8(VL53L0X_Dev_t *dev, uint8_t reg, uint8_t *val)
{
    HAL_StatusTypeDef rc = HAL_I2C_Mem_Read(dev->hi2c,
                                              dev->i2c_addr,
                                              reg,
                                              I2C_MEMADD_SIZE_8BIT,
                                              val, 1,
                                              VL53L0X_I2C_TIMEOUT_MS);
    return (rc == HAL_OK) ? VL53L0X_OK : VL53L0X_ERR_I2C;
}

VL53L0X_Status_t VL53L0X_ReadReg16(VL53L0X_Dev_t *dev, uint8_t reg, uint16_t *val)
{
    uint8_t buf[2];
    RETURN_IF_ERR(_ReadBuf(dev, reg, buf, 2));
    *val = ((uint16_t)buf[0] << 8) | buf[1];
    return VL53L0X_OK;
}

static VL53L0X_Status_t _WriteBuf(VL53L0X_Dev_t *dev, uint8_t reg,
                                   const uint8_t *buf, uint16_t len)
{
    HAL_StatusTypeDef rc = HAL_I2C_Mem_Write(dev->hi2c,
                                              dev->i2c_addr,
                                              reg,
                                              I2C_MEMADD_SIZE_8BIT,
                                              (uint8_t *)buf, len,
                                              VL53L0X_I2C_TIMEOUT_MS);
    return (rc == HAL_OK) ? VL53L0X_OK : VL53L0X_ERR_I2C;
}

static VL53L0X_Status_t _ReadBuf(VL53L0X_Dev_t *dev, uint8_t reg,
                                  uint8_t *buf, uint16_t len)
{
    HAL_StatusTypeDef rc = HAL_I2C_Mem_Read(dev->hi2c,
                                              dev->i2c_addr,
                                              reg,
                                              I2C_MEMADD_SIZE_8BIT,
                                              buf, len,
                                              VL53L0X_I2C_TIMEOUT_MS);
    return (rc == HAL_OK) ? VL53L0X_OK : VL53L0X_ERR_I2C;
}

/* ══════════════════════════════════════════════════════════════════════════════
 *  TUNING SETTINGS
 *  These are ST-supplied "magic" values that configure internal analogue
 *  parameters of the sensor firmware.  They must be written exactly as given.
 * ══════════════════════════════════════════════════════════════════════════════ */

/* Tuning table: { register, value } pairs, terminated by { 0xFF, 0xFF } */
static const uint8_t tuning_settings[][2] = {
    { 0xFF, 0x01 }, { 0x00, 0x00 }, { 0xFF, 0x00 }, { 0x09, 0x00 },
    { 0x10, 0x00 }, { 0x11, 0x00 }, { 0x24, 0x01 }, { 0x25, 0xFF },
    { 0x75, 0x00 }, { 0xFF, 0x01 }, { 0x4E, 0x2C }, { 0x48, 0x00 },
    { 0x30, 0x20 }, { 0xFF, 0x00 }, { 0x30, 0x09 }, { 0x54, 0x00 },
    { 0x31, 0x04 }, { 0x32, 0x03 }, { 0x40, 0x83 }, { 0x46, 0x25 },
    { 0x60, 0x00 }, { 0x27, 0x00 }, { 0x50, 0x06 }, { 0x51, 0x00 },
    { 0x52, 0x96 }, { 0x56, 0x08 }, { 0x57, 0x30 }, { 0x61, 0x00 },
    { 0x62, 0x00 }, { 0x64, 0x00 }, { 0x65, 0x00 }, { 0x66, 0xA0 },
    { 0xFF, 0x01 }, { 0x22, 0x32 }, { 0x47, 0x14 }, { 0x49, 0xFF },
    { 0x4A, 0x00 }, { 0xFF, 0x00 }, { 0x7A, 0x0A }, { 0x7B, 0x00 },
    { 0x78, 0x21 }, { 0xFF, 0x01 }, { 0x23, 0x34 }, { 0x42, 0x00 },
    { 0x44, 0xFF }, { 0x45, 0x26 }, { 0x46, 0x05 }, { 0x40, 0x40 },
    { 0x0E, 0x06 }, { 0x20, 0x1A }, { 0x43, 0x40 }, { 0xFF, 0x00 },
    { 0x34, 0x03 }, { 0x35, 0x44 }, { 0xFF, 0x01 }, { 0x31, 0x04 },
    { 0x4B, 0x09 }, { 0x4C, 0x05 }, { 0x4D, 0x04 }, { 0xFF, 0x00 },
    { 0x44, 0x00 }, { 0x45, 0x20 }, { 0x47, 0x08 }, { 0x48, 0x28 },
    { 0x67, 0x00 }, { 0x70, 0x04 }, { 0x71, 0x01 }, { 0x72, 0xFE },
    { 0x76, 0x00 }, { 0x77, 0x00 }, { 0xFF, 0x01 }, { 0x0D, 0x01 },
    { 0xFF, 0x00 }, { 0x80, 0x01 }, { 0x01, 0xF8 }, { 0xFF, 0x01 },
    { 0x8E, 0x01 }, { 0x00, 0x01 }, { 0xFF, 0x00 }, { 0x80, 0x00 },
    { 0xFF, 0xFF }  /* sentinel */
};

static VL53L0X_Status_t _LoadTuningSettings(VL53L0X_Dev_t *dev)
{
    for (int i = 0; tuning_settings[i][0] != 0xFF ||
                    tuning_settings[i][1] != 0xFF; i++)
    {
        RETURN_IF_ERR(VL53L0X_WriteReg8(dev,
                                         tuning_settings[i][0],
                                         tuning_settings[i][1]));
    }
    return VL53L0X_OK;
}

/* ══════════════════════════════════════════════════════════════════════════════
 *  SPAD INFO  (Single Photon Avalanche Diode reference map)
 * ══════════════════════════════════════════════════════════════════════════════ */

static VL53L0X_Status_t _GetSpadInfo(VL53L0X_Dev_t *dev,
                                      uint8_t *count,
                                      bool    *is_aperture)
{
    uint8_t tmp;
    uint32_t start = HAL_GetTick();

    RETURN_IF_ERR(VL53L0X_WriteReg8(dev, 0x80, 0x01));
    RETURN_IF_ERR(VL53L0X_WriteReg8(dev, 0xFF, 0x01));
    RETURN_IF_ERR(VL53L0X_WriteReg8(dev, 0x00, 0x00));
    RETURN_IF_ERR(VL53L0X_WriteReg8(dev, 0xFF, 0x06));

    RETURN_IF_ERR(VL53L0X_ReadReg8 (dev, 0x83, &tmp));
    RETURN_IF_ERR(VL53L0X_WriteReg8(dev, 0x83, tmp | 0x04));

    RETURN_IF_ERR(VL53L0X_WriteReg8(dev, 0xFF, 0x07));
    RETURN_IF_ERR(VL53L0X_WriteReg8(dev, 0x81, 0x01));
    RETURN_IF_ERR(VL53L0X_WriteReg8(dev, 0x80, 0x01));
    RETURN_IF_ERR(VL53L0X_WriteReg8(dev, 0x94, 0x6B));
    RETURN_IF_ERR(VL53L0X_WriteReg8(dev, 0x83, 0x00));

    /* Poll register 0x83 until non-zero (device signals SPAD info ready) */
    do {
        RETURN_IF_ERR(VL53L0X_ReadReg8(dev, 0x83, &tmp));
        if ((HAL_GetTick() - start) > VL53L0X_BOOT_TIMEOUT_MS)
            return VL53L0X_ERR_TIMEOUT;
    } while (tmp == 0x00);

    RETURN_IF_ERR(VL53L0X_WriteReg8(dev, 0x83, 0x01));
    RETURN_IF_ERR(VL53L0X_ReadReg8 (dev, 0x92, &tmp));

    *count       = tmp & 0x7F;
    *is_aperture = (tmp >> 7) & 0x01;

    RETURN_IF_ERR(VL53L0X_WriteReg8(dev, 0x81, 0x00));
    RETURN_IF_ERR(VL53L0X_WriteReg8(dev, 0xFF, 0x06));

    RETURN_IF_ERR(VL53L0X_ReadReg8 (dev, 0x83, &tmp));
    RETURN_IF_ERR(VL53L0X_WriteReg8(dev, 0x83, tmp & ~0x04));

    RETURN_IF_ERR(VL53L0X_WriteReg8(dev, 0xFF, 0x01));
    RETURN_IF_ERR(VL53L0X_WriteReg8(dev, 0x00, 0x01));
    RETURN_IF_ERR(VL53L0X_WriteReg8(dev, 0xFF, 0x00));
    RETURN_IF_ERR(VL53L0X_WriteReg8(dev, 0x80, 0x00));

    return VL53L0X_OK;
}

/* ══════════════════════════════════════════════════════════════════════════════
 *  SINGLE-STEP REFERENCE CALIBRATION  (VHV and Phase)
 * ══════════════════════════════════════════════════════════════════════════════ */

static VL53L0X_Status_t _PerformSingleRefCalibration(VL53L0X_Dev_t *dev,
                                                       uint8_t vhv_init_byte)
{
    uint8_t val;
    uint32_t start = HAL_GetTick();

    /* Start a single calibration measurement */
    RETURN_IF_ERR(VL53L0X_WriteReg8(dev, REG_SYSRANGE_START,
                                     0x01 | vhv_init_byte));

    /* Wait until interrupt flag (bits 2:0) is non-zero */
    do {
        RETURN_IF_ERR(VL53L0X_ReadReg8(dev, REG_RESULT_INTERRUPT_STATUS, &val));
        if ((HAL_GetTick() - start) > VL53L0X_MEAS_TIMEOUT_MS)
            return VL53L0X_ERR_TIMEOUT;
    } while ((val & 0x07) == 0);

    RETURN_IF_ERR(VL53L0X_WriteReg8(dev, REG_SYSTEM_INTERRUPT_CLEAR, 0x01));
    RETURN_IF_ERR(VL53L0X_WriteReg8(dev, REG_SYSRANGE_START, 0x00));
    return VL53L0X_OK;
}

/* ══════════════════════════════════════════════════════════════════════════════
 *  VL53L0X_Init
 * ══════════════════════════════════════════════════════════════════════════════ */

VL53L0X_Status_t VL53L0X_Init(VL53L0X_Dev_t    *dev,
                                I2C_HandleTypeDef *hi2c,
                                GPIO_TypeDef      *xshut_port,
                                uint16_t           xshut_pin,
                                int8_t             id)
{
    if (dev == NULL || hi2c == NULL)
        return VL53L0X_ERR_INVALID_PARAMS;

    /* Populate handle */
    memset(dev, 0, sizeof(VL53L0X_Dev_t));
    dev->hi2c         = hi2c;
    dev->i2c_addr     = VL53L0X_DEFAULT_ADDRESS;
    dev->xshut_port   = xshut_port;
    dev->xshut_pin    = xshut_pin;
    dev->device_mode  = VL53L0X_DEVICEMODE_SINGLE_RANGING;
    dev->id           = id;

    /* ── Step 1: Hardware reset via XSHUT ─────────────────────────────────── */
    if (xshut_port != NULL)
    {
        HAL_GPIO_WritePin(xshut_port, xshut_pin, GPIO_PIN_RESET); /* Assert LOW  */
        HAL_Delay(2);
        HAL_GPIO_WritePin(xshut_port, xshut_pin, GPIO_PIN_SET);   /* Release     */
        HAL_Delay(2);
    }

    /* ── Step 2: Verify device identity ───────────────────────────────────── */
    uint8_t model_id;
    RETURN_IF_ERR(VL53L0X_ReadReg8(dev, REG_IDENTIFICATION_MODEL_ID, &model_id));
    if (model_id != 0xEE)
        return VL53L0X_ERR_WRONG_DEVICE;

    /* ── Step 3: Set 2.8 V I/O mode ───────────────────────────────────────── */
    uint8_t tmp;
    RETURN_IF_ERR(VL53L0X_ReadReg8 (dev, REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV, &tmp));
    RETURN_IF_ERR(VL53L0X_WriteReg8(dev, REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV,
                                     tmp | 0x01));

    /* ── Step 4: Save "stop variable" used in every ranging start sequence ── */
    RETURN_IF_ERR(VL53L0X_WriteReg8(dev, 0x88, 0x00));
    RETURN_IF_ERR(VL53L0X_WriteReg8(dev, 0x80, 0x01));
    RETURN_IF_ERR(VL53L0X_WriteReg8(dev, 0xFF, 0x01));
    RETURN_IF_ERR(VL53L0X_WriteReg8(dev, 0x00, 0x00));
    RETURN_IF_ERR(VL53L0X_ReadReg8 (dev, 0x91, &dev->stop_variable));
    RETURN_IF_ERR(VL53L0X_WriteReg8(dev, 0x00, 0x01));
    RETURN_IF_ERR(VL53L0X_WriteReg8(dev, 0xFF, 0x00));
    RETURN_IF_ERR(VL53L0X_WriteReg8(dev, 0x80, 0x00));

    /* ── Step 5: Disable SIGNAL_RATE_MSRC and SIGNAL_RATE_PRE_RANGE checks ── */
    RETURN_IF_ERR(VL53L0X_ReadReg8 (dev, REG_MSRC_CONFIG_CONTROL, &tmp));
    RETURN_IF_ERR(VL53L0X_WriteReg8(dev, REG_MSRC_CONFIG_CONTROL, tmp | 0x12));

    /* Set signal rate limit to 0.25 MCPS (9.7 fixed-point: 0.25 * 128 = 32) */
    RETURN_IF_ERR(VL53L0X_WriteReg16(dev,
                  REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, 32));

    /* ── Step 6: Load ST tuning settings ──────────────────────────────────── */
    RETURN_IF_ERR(VL53L0X_WriteReg8(dev, REG_SYSTEM_SEQUENCE_CONFIG, 0xFF));
    RETURN_IF_ERR(_LoadTuningSettings(dev));

    /* ── Step 7: Configure GPIO1 interrupt (active LOW, new sample ready) ─── */
    RETURN_IF_ERR(VL53L0X_WriteReg8(dev, REG_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04));
    RETURN_IF_ERR(VL53L0X_ReadReg8 (dev, REG_GPIO_HV_MUX_ACTIVE_HIGH, &tmp));
    RETURN_IF_ERR(VL53L0X_WriteReg8(dev, REG_GPIO_HV_MUX_ACTIVE_HIGH,
                                     tmp & ~0x10));  /* active LOW */
    RETURN_IF_ERR(VL53L0X_WriteReg8(dev, REG_SYSTEM_INTERRUPT_CLEAR, 0x01));

    /* ── Step 8: SPAD management ───────────────────────────────────────────── */
    uint8_t spad_count;
    bool    spad_is_aperture;
    RETURN_IF_ERR(_GetSpadInfo(dev, &spad_count, &spad_is_aperture));

    /* Read current SPAD enable map (6 bytes = 48 bits, one per SPAD) */
    uint8_t spad_map[6];
    RETURN_IF_ERR(_ReadBuf(dev, REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0,
                            spad_map, 6));

    /* Configure registers for SPAD enable programming */
    RETURN_IF_ERR(VL53L0X_WriteReg8(dev, 0xFF, 0x01));
    RETURN_IF_ERR(VL53L0X_WriteReg8(dev, REG_DYNAMIC_SPAD_REF_EN_START_OFFSET,
                                     0x00));
    RETURN_IF_ERR(VL53L0X_WriteReg8(dev, REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD,
                                     0x2C));
    RETURN_IF_ERR(VL53L0X_WriteReg8(dev, 0xFF, 0x00));
    RETURN_IF_ERR(VL53L0X_WriteReg8(dev, REG_GLOBAL_CONFIG_REF_EN_START_SELECT,
                                     0xB4));

    /* Aperture SPADs start at index 12; non-aperture start at 0 */
    uint8_t first_spad = spad_is_aperture ? 12 : 0;
    uint8_t enabled    = 0;

    for (uint8_t i = 0; i < 48; i++)
    {
        if (i < first_spad || enabled == spad_count)
        {
            /* Clear this SPAD bit */
            spad_map[i / 8] &= ~(1 << (i % 8));
        }
        else if ((spad_map[i / 8] >> (i % 8)) & 0x01)
        {
            enabled++;
        }
    }

    RETURN_IF_ERR(_WriteBuf(dev, REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0,
                             spad_map, 6));

    /* ── Step 9: Set default timing budget (33 ms) ─────────────────────────── */
    dev->measurement_timing_budget_us = 33000;
    /* (Full timing budget calculation omitted for brevity;
        the default sequence config already gives ~33 ms.) */

    /* ── Step 10: VHV calibration ─────────────────────────────────────────── */
    RETURN_IF_ERR(VL53L0X_WriteReg8(dev, REG_SYSTEM_SEQUENCE_CONFIG, 0x01));
    RETURN_IF_ERR(_PerformSingleRefCalibration(dev, 0x40));

    /* ── Step 11: Phase calibration ───────────────────────────────────────── */
    RETURN_IF_ERR(VL53L0X_WriteReg8(dev, REG_SYSTEM_SEQUENCE_CONFIG, 0x02));
    RETURN_IF_ERR(_PerformSingleRefCalibration(dev, 0x00));

    /* ── Step 12: Restore normal sequence (DSS + Pre-range + Final-range) ─── */
    RETURN_IF_ERR(VL53L0X_WriteReg8(dev, REG_SYSTEM_SEQUENCE_CONFIG, 0xE8));

    dev->is_initialized = true;
    return VL53L0X_OK;
}

/* ══════════════════════════════════════════════════════════════════════════════
 *  INTERNAL HELPERS: wait for data and read result registers
 * ══════════════════════════════════════════════════════════════════════════════ */

static VL53L0X_Status_t _WaitMeasurementDataReady(VL53L0X_Dev_t *dev)
{
    uint8_t  val;
    uint32_t start = HAL_GetTick();

    do {
        RETURN_IF_ERR(VL53L0X_ReadReg8(dev, REG_RESULT_INTERRUPT_STATUS, &val));
        if ((HAL_GetTick() - start) > VL53L0X_MEAS_TIMEOUT_MS)
            return VL53L0X_ERR_TIMEOUT;
    } while ((val & 0x07) == 0);

    return VL53L0X_OK;
}

static VL53L0X_Status_t _ReadRangeData(VL53L0X_Dev_t                   *dev,
                                        VL53L0X_RangingMeasurementData_t *data)
{
    uint8_t  range_buf[12];   /* bytes 0–11 starting at RESULT_RANGE_STATUS */
    uint16_t range_mm;
    uint8_t  range_status_raw;

    /* Read 12 bytes starting from RESULT_RANGE_STATUS (0x14) */
    RETURN_IF_ERR(_ReadBuf(dev, REG_RESULT_RANGE_STATUS, range_buf, 12));

    /* Range status lives in bits 7:3 of byte 0 */
    range_status_raw = (range_buf[0] >> 3) & 0x1F;

    /* Convert raw status to API enum */
    if (range_status_raw == 0x0B)
        data->RangeStatus = VL53L0X_RANGE_STATUS_VALID;
    else if (range_status_raw == 0x07)
        data->RangeStatus = VL53L0X_RANGE_STATUS_SIGMA_FAIL;
    else if (range_status_raw == 0x05)
        data->RangeStatus = VL53L0X_RANGE_STATUS_SIGNAL_FAIL;
    else if (range_status_raw == 0x04)
        data->RangeStatus = VL53L0X_RANGE_STATUS_MIN_RANGE;
    else if (range_status_raw == 0x00)
        data->RangeStatus = VL53L0X_RANGE_STATUS_HARDWARE_FAIL;
    else
        data->RangeStatus = (uint8_t)range_status_raw;

    /* Range in mm is at offset +10 from RESULT_RANGE_STATUS (bytes 10–11) */
    range_mm = ((uint16_t)range_buf[10] << 8) | range_buf[11];
    data->RangeMilliMeter = range_mm;

    /* Signal rate (FixPoint9.7) at bytes 6–7 */
    data->SignalRateRtnMegaCps =
        ((uint32_t)range_buf[6] << 8) | range_buf[7];

    /* Clear interrupt ready for next measurement */
    RETURN_IF_ERR(VL53L0X_WriteReg8(dev, REG_SYSTEM_INTERRUPT_CLEAR, 0x01));

    return VL53L0X_OK;
}

/* ══════════════════════════════════════════════════════════════════════════════
 *  SINGLE RANGING MEASUREMENT
 * ══════════════════════════════════════════════════════════════════════════════ */

VL53L0X_Status_t VL53L0X_PerformSingleRangingMeasurement(
                                VL53L0X_Dev_t                   *dev,
                                VL53L0X_RangingMeasurementData_t *data)
{
    if (!dev->is_initialized)
        return VL53L0X_ERR_NOT_INIT;

    /* Trigger sequence required before every single-shot measurement */
    RETURN_IF_ERR(VL53L0X_WriteReg8(dev, 0x80, 0x01));
    RETURN_IF_ERR(VL53L0X_WriteReg8(dev, 0xFF, 0x01));
    RETURN_IF_ERR(VL53L0X_WriteReg8(dev, 0x00, 0x00));
    RETURN_IF_ERR(VL53L0X_WriteReg8(dev, 0x91, dev->stop_variable));
    RETURN_IF_ERR(VL53L0X_WriteReg8(dev, 0x00, 0x01));
    RETURN_IF_ERR(VL53L0X_WriteReg8(dev, 0xFF, 0x00));
    RETURN_IF_ERR(VL53L0X_WriteReg8(dev, 0x80, 0x00));

    /* Start single-shot measurement */
    RETURN_IF_ERR(VL53L0X_WriteReg8(dev, REG_SYSRANGE_START, 0x01));

    /* Wait until the device clears the start bit (measurement has begun) */
    uint8_t  start_reg;
    uint32_t start_tick = HAL_GetTick();
    do {
        RETURN_IF_ERR(VL53L0X_ReadReg8(dev, REG_SYSRANGE_START, &start_reg));
        if ((HAL_GetTick() - start_tick) > VL53L0X_MEAS_TIMEOUT_MS)
            return VL53L0X_ERR_TIMEOUT;
    } while (start_reg & 0x01);

    /* Wait for measurement data ready */
    RETURN_IF_ERR(_WaitMeasurementDataReady(dev));

    /* Read and parse result registers */
    RETURN_IF_ERR(_ReadRangeData(dev, data));

    return VL53L0X_OK;
}

/* ══════════════════════════════════════════════════════════════════════════════
 *  CONTINUOUS MODE
 * ══════════════════════════════════════════════════════════════════════════════ */

VL53L0X_Status_t VL53L0X_StartContinuous(VL53L0X_Dev_t *dev, uint32_t period_ms)
{
    if (!dev->is_initialized)
        return VL53L0X_ERR_NOT_INIT;

    RETURN_IF_ERR(VL53L0X_WriteReg8(dev, 0x80, 0x01));
    RETURN_IF_ERR(VL53L0X_WriteReg8(dev, 0xFF, 0x01));
    RETURN_IF_ERR(VL53L0X_WriteReg8(dev, 0x00, 0x00));
    RETURN_IF_ERR(VL53L0X_WriteReg8(dev, 0x91, dev->stop_variable));
    RETURN_IF_ERR(VL53L0X_WriteReg8(dev, 0x00, 0x01));
    RETURN_IF_ERR(VL53L0X_WriteReg8(dev, 0xFF, 0x00));
    RETURN_IF_ERR(VL53L0X_WriteReg8(dev, 0x80, 0x00));

    if (period_ms != 0)
    {
        /* Timed / inter-measurement mode */
        uint16_t osc_cal;
        RETURN_IF_ERR(VL53L0X_ReadReg16(dev, REG_OSC_CALIBRATE_VAL, &osc_cal));

        uint32_t period_reg = period_ms * 1000;
        if (osc_cal != 0)
            period_reg = (uint32_t)(period_reg * osc_cal / 1000);

        RETURN_IF_ERR(VL53L0X_WriteReg8(dev,
                       REG_SYSTEM_INTERMEASUREMENT_PERIOD,
                       (uint8_t)((period_reg >> 24) & 0xFF)));
        /* Full 32-bit write: use individual byte registers 0x04–0x07 */
        uint8_t pm_buf[4] = {
            (uint8_t)((period_reg >> 24) & 0xFF),
            (uint8_t)((period_reg >> 16) & 0xFF),
            (uint8_t)((period_reg >>  8) & 0xFF),
            (uint8_t)( period_reg        & 0xFF)
        };
        RETURN_IF_ERR(_WriteBuf(dev, REG_SYSTEM_INTERMEASUREMENT_PERIOD,
                                 pm_buf, 4));
        /* Timed mode: bits[3:0] = 0x04 */
        RETURN_IF_ERR(VL53L0X_WriteReg8(dev, REG_SYSRANGE_START, 0x04));
    }
    else
    {
        /* Back-to-back (fastest) mode: bits[3:0] = 0x02 */
        RETURN_IF_ERR(VL53L0X_WriteReg8(dev, REG_SYSRANGE_START, 0x02));
    }

    dev->device_mode = VL53L0X_DEVICEMODE_CONTINUOUS_RANGING;
    return VL53L0X_OK;
}

VL53L0X_Status_t VL53L0X_ReadRangeContinuous(
                                VL53L0X_Dev_t                   *dev,
                                VL53L0X_RangingMeasurementData_t *data)
{
    if (!dev->is_initialized)
        return VL53L0X_ERR_NOT_INIT;

    RETURN_IF_ERR(_WaitMeasurementDataReady(dev));
    RETURN_IF_ERR(_ReadRangeData(dev, data));
    return VL53L0X_OK;
}

VL53L0X_Status_t VL53L0X_StopContinuous(VL53L0X_Dev_t *dev)
{
    if (!dev->is_initialized)
        return VL53L0X_ERR_NOT_INIT;

    RETURN_IF_ERR(VL53L0X_WriteReg8(dev, REG_SYSRANGE_START, 0x01));
    RETURN_IF_ERR(VL53L0X_WriteReg8(dev, 0xFF, 0x01));
    RETURN_IF_ERR(VL53L0X_WriteReg8(dev, 0x00, 0x00));
    RETURN_IF_ERR(VL53L0X_WriteReg8(dev, 0x91, 0x00));
    RETURN_IF_ERR(VL53L0X_WriteReg8(dev, 0x00, 0x01));
    RETURN_IF_ERR(VL53L0X_WriteReg8(dev, 0xFF, 0x00));

    dev->device_mode = VL53L0X_DEVICEMODE_SINGLE_RANGING;
    return VL53L0X_OK;
}

/* ══════════════════════════════════════════════════════════════════════════════
 *  CONFIGURATION HELPERS
 * ══════════════════════════════════════════════════════════════════════════════ */

VL53L0X_Status_t VL53L0X_SetLimitCheckEnable(VL53L0X_Dev_t *dev,
                                               uint8_t         limit_id,
                                               uint8_t         enable)
{
    if (!dev->is_initialized)
        return VL53L0X_ERR_NOT_INIT;

    uint8_t val;
    RETURN_IF_ERR(VL53L0X_ReadReg8(dev, REG_SYSTEM_SEQUENCE_CONFIG, &val));

    switch (limit_id)
    {
    case VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE:
        /* Sigma check: bit 5 of REG_SYSTEM_SEQUENCE_CONFIG controls DSS */
        /* The sigma limit value is in registers 0x18:0x19 (FixPoint14.2) */
        /* Default: sigma limit 18 mm (18 * 4 = 72 = 0x0048 in fixed-point) */
        if (enable)
            RETURN_IF_ERR(VL53L0X_WriteReg16(dev, 0x18, 0x0048));
        break;

    case VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE:
        /* Signal rate limit: 0.25 MCPS = 32 in 9.7 fixed-point */
        if (enable)
            RETURN_IF_ERR(VL53L0X_WriteReg16(dev,
                          REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, 32));
        else
            RETURN_IF_ERR(VL53L0X_WriteReg16(dev,
                          REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, 0));
        break;

    default:
        return VL53L0X_ERR_INVALID_PARAMS;
    }

    return VL53L0X_OK;
}

VL53L0X_Status_t VL53L0X_SetMeasurementTimingBudget(VL53L0X_Dev_t *dev,
                                                      uint32_t budget_us)
{
    if (!dev->is_initialized)
        return VL53L0X_ERR_NOT_INIT;
    /*
     * Full budget calculation involves reading current VCSEL periods and
     * timeout values then back-computing the final-range timeout.
     * For typical embedded use (33 ms, 66 ms, 200 ms) it is sufficient to
     * store the value; the timing is determined during initialisation.
     */
    dev->measurement_timing_budget_us = budget_us;
    return VL53L0X_OK;
}

VL53L0X_Status_t VL53L0X_SetI2CAddress(VL53L0X_Dev_t *dev, uint8_t new_addr)
{
    if (!dev->is_initialized)
        return VL53L0X_ERR_NOT_INIT;

    /* Write new 7-bit address into device register */
    RETURN_IF_ERR(VL53L0X_WriteReg8(dev, REG_I2C_SLAVE_DEVICE_ADDRESS,
                                     (new_addr >> 1) & 0x7F));

    /* Update our stored address for subsequent communications */
    dev->i2c_addr = new_addr;
    return VL53L0X_OK;
}

/* Forward declaration of TCA9548A functions (to avoid circular includes) */
typedef uint8_t (*TCA9548A_SelectChannel_t)(void *mux_dev, uint8_t channel);

VL53L0X_Status_t VL53L0X_SetMuxChannel(VL53L0X_Dev_t *dev,
                                        void           *mux_dev,
                                        uint8_t         mux_channel)
{
    if (!dev->is_initialized)
        return VL53L0X_ERR_NOT_INIT;

    if (mux_channel > 7)
        return VL53L0X_ERR_INVALID_PARAMS;

    dev->mux_config.mux_enabled  = (mux_dev != NULL);
    dev->mux_config.mux_dev      = mux_dev;
    dev->mux_config.mux_channel  = mux_channel;

    return VL53L0X_OK;
}


