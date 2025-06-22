/**
  ******************************************************************************
  * @file    lis2dh12drv.c
  * @author  Thomas Reisnecker
  * @brief   tbd.
  ******************************************************************************
  */

/* logger configuration ----------------------------------------------_-------*/
#define ACC_CONFIG_LOG_ENABLED 1 /// TODO: just temporary until project specific sdk_config
#define ACC_CONFIG_LOG_LEVEL   3

#define NRF_LOG_MODULE_NAME imu
#if ACC_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL ACC_CONFIG_LOG_LEVEL
#else //BASE_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL 0
#endif //BASE_CONFIG_LOG_ENABLED
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

/* Includes ------------------------------------------------------------------*/
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrfx_gpiote.h"
#include "i2c.h"
#include "lis2dh12_reg.h"
#include "lis2dh12drv.h"

/* Private typedef -----------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/
#define VERIFY_RSLT(x)              \
do                                  \
{                                   \
    if (x != NRF_SUCCESS)           \
        return x;                   \
} while (0)

/* Private defines -----------------------------------------------------------*/
#define PIN_NOT_USED    0xFFFFFFFF

/* Private variables ---------------------------------------------------------*/
static uint8_t                    deviceAddr;
static stmdev_ctx_t               lis2dh12dev;
static uint32_t                   intPinNo = PIN_NOT_USED;
static lis2dh12drv_eventHandler_t evtHandler;
static lis2dh12drv_powerMode_t    powerMode = LIS2DH12_PWR_OFF;
static bool volatile              dataReady;

/* Private read only variables -----------------------------------------------*/
static uint8_t const ctrl_defaults[] =
{
    0x10,   // CTRL_REG0
    0x00,   // TEMP_CFG_REG
    0x07,   // CTRL_REG1
    0x00,   // CTRL_REG2
    0x00,   // CTRL_REG3
    0x00,   // CTRL_REG4
    0x00,   // CTRL_REG5
    0x00,   // CTRL_REG6
    0x00,   // REFERENCE
};

/* Private functions ---------------------------------------------------------*/
static int32_t write(void* pCtx, uint8_t reg, uint8_t const *pData, uint16_t len)
{
    (void) pCtx;

    uint8_t regAddr = reg | 0x80;   // enable auto increment

    return i2c_Write(deviceAddr, regAddr, pData, len);
}

static int32_t read(void* pCtx, uint8_t reg, uint8_t *pData, uint16_t len)
{
    (void) pCtx;

    uint8_t regAddr = reg | 0x80;   // enable auto increment

    return i2c_Read(deviceAddr, regAddr, pData, len);
}

ret_code_t resetSensor()
{
    // reset device
    // since there is no way to initiate a reset through software,
    // reset is done by manually writing all control and config registers to their default values.
    VERIFY_RSLT(lis2dh12_write_reg(&lis2dh12dev, LIS2DH12_CTRL_REG0, (uint8_t*)ctrl_defaults, sizeof(ctrl_defaults)));
    VERIFY_RSLT(lis2dh12_write_reg(&lis2dh12dev, LIS2DH12_FIFO_CTRL_REG, (uint8_t*)&ctrl_defaults[3], 1));  // ctrl_defaults[3] = 0
    VERIFY_RSLT(lis2dh12_write_reg(&lis2dh12dev, LIS2DH12_INT1_CFG, (uint8_t*)&ctrl_defaults[3], 1));  // ctrl_defaults[3] = 0
    VERIFY_RSLT(lis2dh12_write_reg(&lis2dh12dev, LIS2DH12_INT1_THS, (uint8_t*)&ctrl_defaults[3], 3));  // ctrl_defaults[3..5] = 0
    VERIFY_RSLT(lis2dh12_write_reg(&lis2dh12dev, LIS2DH12_INT2_THS, (uint8_t*)&ctrl_defaults[3], 3));  // ctrl_defaults[3..5] = 0
    VERIFY_RSLT(lis2dh12_write_reg(&lis2dh12dev, LIS2DH12_CLICK_THS, (uint8_t*)&ctrl_defaults[3], 6));  // ctrl_defaults[3..8] = 0

    return NRF_SUCCESS;
}

ret_code_t initSensor(uint8_t addr)
{
    if (addr != LIS2DH12_ADDR0 && addr != LIS2DH12_ADDR1)
        return NRF_ERROR_INVALID_ADDR;

    deviceAddr = addr;

    lis2dh12dev.write_reg = write;
    lis2dh12dev.read_reg = read;
    lis2dh12dev.mdelay = nrf_delay_ms;

    // read device id to ensure device is present
    uint8_t deviceId;
    VERIFY_RSLT(lis2dh12_device_id_get(&lis2dh12dev, &deviceId));   /// TODO: will this fail if the device isn't fully operational yet?
    if (deviceId != LIS2DH12_ID)
        return NRF_ERROR_NOT_FOUND;

    VERIFY_RSLT(resetSensor());  /// TODO: really necessary only after firmware update

    /// TODO: self test?

    // enable block data update to prevent overwriting data while reading
    VERIFY_RSLT(lis2dh12_block_data_update_set(&lis2dh12dev, PROPERTY_ENABLE));

    // set full scale to 8g
    VERIFY_RSLT(lis2dh12_full_scale_set(&lis2dh12dev, LIS2DH12_8g));

    // configure filter
    VERIFY_RSLT(lis2dh12_high_pass_bandwidth_set(&lis2dh12dev, LIS2DH12_AGGRESSIVE));   /// TODO: find optimal solution
    VERIFY_RSLT(lis2dh12_high_pass_mode_set(&lis2dh12dev, LIS2DH12_NORMAL_WITH_RST));

    // configure interrupt 1 functions to generate motion interrupt
    VERIFY_RSLT(lis2dh12_high_pass_int_conf_set(&lis2dh12dev, LIS2DH12_ON_INT1_GEN));   // enable high pass filter for interrupt generation
    VERIFY_RSLT(lis2dh12_int1_gen_threshold_set(&lis2dh12dev, 3));                      // threshold to 168 mg (in low power mode)
    VERIFY_RSLT(lis2dh12_int1_gen_duration_set(&lis2dh12dev, 0));                       // 0 duration (=40ms because of 25Hz ODR) (in low power mode)
    lis2dh12_int1_cfg_t intcfg =
    {
        .xhie = 1,
        .yhie = 1,
        .zhie = 1,
    };
    VERIFY_RSLT(lis2dh12_int1_gen_conf_set(&lis2dh12dev, &intcfg));                     // enable detection on all axis

    return NRF_SUCCESS;
}

static void pinChangeHandler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    (void)action;

    if (pin != intPinNo)
        return;     // not my business

    dataReady = true;
}

ret_code_t configLocalInts(uint32_t pin)
{
    if (!nrf_gpio_pin_present_check(pin))
        return NRF_ERROR_INVALID_PARAM;

    ret_code_t errCode;

    // initialize gpiote if not already done
    if (!nrfx_gpiote_is_init())
    {
        errCode = nrfx_gpiote_init();
        if (errCode != NRF_SUCCESS)
            return errCode;
    }

    nrfx_gpiote_in_config_t intConfig = NRFX_GPIOTE_CONFIG_IN_SENSE_LOTOHI(false);
    errCode = nrfx_gpiote_in_init(pin, &intConfig, &pinChangeHandler);
    if (errCode != NRF_SUCCESS)
        return errCode;

    nrfx_gpiote_in_event_enable(pin, true);

    intPinNo = pin;

    return NRF_SUCCESS;
}

static ret_code_t changeSensorConfig(lis2dh12drv_powerMode_t newMode)
{
    uint8_t dummy;

    /// TODO: maybe change int threshold and duration due to different resolution and data rate in motion and data mode?

    switch (newMode)
    {
    case LIS2DH12_PWR_OFF:
        // set output rate to off
        VERIFY_RSLT(lis2dh12_data_rate_set(&lis2dh12dev, LIS2DH12_POWER_DOWN));
        break;
    case LIS2DH12_PWR_MOTION:
        // set device to low power mode
        VERIFY_RSLT(lis2dh12_operating_mode_set(&lis2dh12dev, LIS2DH12_LP_8bit));
        // set output rate to 25Hz
        VERIFY_RSLT(lis2dh12_data_rate_set(&lis2dh12dev, LIS2DH12_ODR_25Hz));
        // read reference register to reset filtering block
        VERIFY_RSLT(lis2dh12_filter_reference_get(&lis2dh12dev, &dummy));
        break;
    case LIS2DH12_PWR_DATA:
        // set device to high resolution mode
        VERIFY_RSLT(lis2dh12_operating_mode_set(&lis2dh12dev, LIS2DH12_HR_12bit));
        // set output rate to 100Hz
        VERIFY_RSLT(lis2dh12_data_rate_set(&lis2dh12dev, LIS2DH12_ODR_100Hz));
        // read reference register to reset filtering block
        VERIFY_RSLT(lis2dh12_filter_reference_get(&lis2dh12dev, &dummy));
        break;
    }

    return NRF_SUCCESS;
}

static ret_code_t configRemoteInt(lis2dh12drv_powerMode_t newMode)
{
    lis2dh12_ctrl_reg3_t reg3 = {0};

    switch (newMode)
    {
    case LIS2DH12_PWR_OFF:
        // don't activate any interrupt
        break;

    case LIS2DH12_PWR_MOTION:
        // route interrupt 1 functions to int1 pin
        reg3.i1_ia1 = 1;
        break;

    case LIS2DH12_PWR_DATA:
        // route data ready function to int1 pin
        reg3.i1_zyxda = 1;
        break;
    }

    VERIFY_RSLT(lis2dh12_pin_int1_config_set(&lis2dh12dev, &reg3));

    return NRF_SUCCESS;
}

/* Public functions ----------------------------------------------------------*/
ret_code_t lis2dh12drv_Init(lis2dh12drv_init_t const* pInit)
{
    ret_code_t errCode;

    if (pInit == NULL || pInit->handler == NULL)
        return NRF_ERROR_NULL;

    errCode = initSensor(pInit->addr);
    if (errCode != NRF_SUCCESS)
        return errCode;

    errCode = configLocalInts(pInit->int1Pin);
    if (errCode != NRF_SUCCESS)
        return errCode;

    evtHandler = pInit->handler;

    return NRF_SUCCESS;
}

ret_code_t lis2dh12drv_SetPowerMode(lis2dh12drv_powerMode_t newMode)
{
    ret_code_t errCode;

    if (evtHandler == NULL || intPinNo == PIN_NOT_USED)
        return NRF_ERROR_INVALID_STATE;

    if (newMode == powerMode)
        return NRF_SUCCESS;

    errCode = changeSensorConfig(newMode);
    if (errCode != NRF_SUCCESS)
        return errCode;

    errCode = configRemoteInt(newMode);
    if (errCode != NRF_SUCCESS)
        return errCode;

    powerMode = newMode;

    return NRF_SUCCESS;
}

void lis2dh12drv_Execute()
{
    ret_code_t errCode;
    lis2dh12drv_event_t evt;
    lis2dh12_int1_src_t intSrc;

    if (dataReady)
    {
        if (powerMode == LIS2DH12_PWR_DATA && evtHandler != NULL)
        {
            errCode = lis2dh12_acceleration_raw_get(&lis2dh12dev, evt.data.accel);
            if (errCode != NRF_SUCCESS)
            {
                NRF_LOG_ERROR("error %d reading accel data", errCode);
            }
            else
            {
                evt.type = LIS2DH12_EVT_DATA;
                evtHandler(&evt);
            }

            errCode = lis2dh12_int1_gen_source_get(&lis2dh12dev, &intSrc);
            if (errCode != NRF_SUCCESS)
            {
                NRF_LOG_ERROR("error %d reading int source", errCode)
            }
            else if (intSrc.ia)
            {
                evt.type = LIS2DH12_EVT_MOTION;
                evtHandler(&evt);
            }
        }
        else if (powerMode == LIS2DH12_PWR_MOTION && evtHandler != NULL)
        {
            evt.type = LIS2DH12_EVT_MOTION;
            evtHandler(&evt);
        }

        dataReady = false;
    }
}

/**END OF FILE*****************************************************************/
