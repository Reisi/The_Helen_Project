/**
  ******************************************************************************
  * @file    bmi160drv.c
  * @author  Thomas Reisnecker
  * @brief   tbd.
  ******************************************************************************
  */

/* logger configuration ----------------------------------------------_-------*/
#define IMU_CONFIG_LOG_ENABLED 1 /// TODO: just temporary until project specific sdk_config
#define IMU_CONFIG_LOG_LEVEL   3

#define NRF_LOG_MODULE_NAME imu
#if IMU_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL IMU_CONFIG_LOG_LEVEL
#else //BASE_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL 0
#endif //BASE_CONFIG_LOG_ENABLED
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

/* Includes ------------------------------------------------------------------*/
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrfx_gpiote.h"
#include "app_timer.h"

#include "bmi160.h"
#include "bsxlite_interface.h"

#include "i2c.h"
#include "bmi160drv.h"
#include "main.h"

/* bmi160 driver extension ---------------------------------------------------*/
#define BMI160_STATUS_GYRO_SELF_TEST_OK     UINT8_C(0x01)
#define BMI160_STATUS_MAG_MAN_OP            UINT8_C(0x02)
#define BMI160_STATUS_FOC_RDY               UINT8_C(0x03)
#define BMI160_STATUS_NVM_RDY               UINT8_C(0x04)
#define BMI160_STATUS_DRDY_AMG              UINT8_C(0x05)
#define BMI160_STATUS_DRDY_GYR              UINT8_C(0x06)
#define BMI160_STATUS_DRDY_ACC              UINT8_C(0x07)

struct bmi160_status_bits
{
    uint8_t reserved         : 1;
    uint8_t gyr_self_test_ok : 1;
    uint8_t mag_man_op       : 1;
    uint8_t foc_rdy          : 1;
    uint8_t nvm_rdy          : 1;
    uint8_t drdy_mag         : 1;
    uint8_t drdy_gyr         : 1;
    uint8_t drdy_acc         : 1;
};

union bmi160_status
{
    uint8_t data;
    struct bmi160_status_bits bit;
};

static int8_t bmi160_get_status(union bmi160_status* status, struct bmi160_dev const* dev)
{
    return bmi160_get_regs(BMI160_STATUS_ADDR, &status->data, 1, dev);
}

/* Private typedef -----------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/
#define VERIFY_RSLT(x)              \
do                                  \
{                                   \
    if (x != BMI160_OK)             \
        return NRF_ERROR_INTERNAL;  \
} while (0)

#define IS_ONBOARD_SENSOR()     (intPinNo != BMI160DRV_NOINT)
#define IS_SENSOR_PRESENT()     (bmi160dev.id != 0)

/* Private defines -----------------------------------------------------------*/
#define TB_STDBY        APP_TIMER_TICKS(1000)
#define TB_ON           APP_TIMER_TICKS(10)

#define M_PI            3.14159265358979323846f
#define M_2PI           (2.f * M_PI)

/* Private variables ---------------------------------------------------------*/
struct bmi160_dev               bmi160dev;
bsxlite_instance_t              bsxInst;
static volatile uint8_t         dataReady;
static bmi160drv_powerMode_t    powerMode;

static uint32_t                 intPinNo = BMI160DRV_NOINT;
static bmi160drv_eventHandler_t evtHandler;
static bmi160drv_reorientate_t  reorient;

APP_TIMER_DEF(pollingTimer);

/* Private read only variables -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
/** @brief function to initialize the sensor
 *  @note This function tries to detect the sensor by using both device
 *        addresses. Check bmi160dev.id to differentiate between onboard
 *        (BMI160_I2C_ADDR) sensor and external module (BMI160_I2C_ADDR + 1).
 *
 * @return NRF_SUCCESS, NRF_ERROR_NOT_FOUND if no sensor is responding,
 *         NRF_ERROR_INTERNAL for communication erros
 */
static ret_code_t initSensor(uint8_t addr)
{
    int8_t retVal;

    bmi160dev.write    = i2c_Write;
    bmi160dev.read     = i2c_Read;
    bmi160dev.delay_ms = nrf_delay_ms;
    bmi160dev.id       = addr;
    bmi160dev.intf     = BMI160_I2C_INTF;   // use I2C interface

    retVal = bmi160_init(&bmi160dev);
    if (retVal == BMI160_E_DEV_NOT_FOUND)
    {
        bmi160dev.id = 0;
        return NRF_ERROR_NOT_FOUND;
    }

    return retVal == BMI160_OK ? NRF_SUCCESS : NRF_ERROR_INTERNAL;
}

/** @brief function to configure the interrupts on the sensor side
 *
 * @param[in] mode     the power mode
 * @param[in] onboard  true for onboard sensor, false for external module
 * @return NRF_SUCCESS, NRF_ERROR_INTERNAL for communication errors
 */
static ret_code_t configRemoteInts(bmi160drv_powerMode_t mode, bool onboard)
{
    struct bmi160_int_settg intSettings = {0};
    int8_t retVal;

    if (mode >= BMI160_PWR_ON)
    {
        intSettings.int_pin_settg.latch_dur = BMI160_LATCH_DUR_40_MILLI_SEC;

        // any motion
        intSettings.int_type      = BMI160_ACC_ANY_MOTION_INT;
        intSettings.int_type_cfg.acc_any_motion_int.anymotion_en = BMI160_ENABLE;
        intSettings.int_type_cfg.acc_any_motion_int.anymotion_x = BMI160_ENABLE;
        intSettings.int_type_cfg.acc_any_motion_int.anymotion_y = BMI160_ENABLE;
        intSettings.int_type_cfg.acc_any_motion_int.anymotion_z = BMI160_ENABLE;
        intSettings.int_type_cfg.acc_any_motion_int.anymotion_thr = 10;
        intSettings.int_channel   = BMI160_INT_CHANNEL_2;
        intSettings.int_pin_settg.output_en = BMI160_DISABLE;

        retVal = bmi160_set_int_config(&intSettings, &bmi160dev);
        VERIFY_RSLT(retVal);

        // data ready
        intSettings.int_type      = BMI160_ACC_GYRO_DATA_RDY_INT;
        if (onboard) // if no pins available reuse the settings from any motion int
        {
            intSettings.int_channel   = BMI160_INT_CHANNEL_1;
            intSettings.int_pin_settg.output_en = BMI160_ENABLE;    // push-pull
            intSettings.int_pin_settg.output_type = BMI160_ENABLE;  // active-high
        }
        retVal = bmi160_set_int_config(&intSettings, &bmi160dev);
        VERIFY_RSLT(retVal);
    }
    else if (mode == BMI160_PWR_STANDBY)
    {
        // only any motion irq use, deactivate data ready first
        intSettings.int_channel = BMI160_INT_CHANNEL_NONE;
        intSettings.int_type = BMI160_ACC_GYRO_DATA_RDY_INT;
        retVal = bmi160_set_int_config(&intSettings, &bmi160dev);
        VERIFY_RSLT(retVal);

        // config for any motion in standby mode
        intSettings.int_type      = BMI160_ACC_ANY_MOTION_INT;
        intSettings.int_type_cfg.acc_any_motion_int.anymotion_en = BMI160_ENABLE;
        intSettings.int_type_cfg.acc_any_motion_int.anymotion_x = BMI160_ENABLE;
        intSettings.int_type_cfg.acc_any_motion_int.anymotion_y = BMI160_ENABLE;
        intSettings.int_type_cfg.acc_any_motion_int.anymotion_z = BMI160_ENABLE;
        intSettings.int_type_cfg.acc_any_motion_int.anymotion_thr = 20;
        intSettings.int_pin_settg.latch_dur = BMI160_LATCH_DUR_1_28_SEC;    // readout once per second, should be enough
        if (onboard)
        {
            intSettings.int_channel   = BMI160_INT_CHANNEL_1;
            intSettings.int_pin_settg.output_en = BMI160_ENABLE;    // push-pull
            intSettings.int_pin_settg.output_type = BMI160_ENABLE;  // active-high
        }
        else
        {
            intSettings.int_channel   = BMI160_INT_CHANNEL_2;
            intSettings.int_pin_settg.output_en = BMI160_DISABLE;
        }
        retVal = bmi160_set_int_config(&intSettings, &bmi160dev);
        VERIFY_RSLT(retVal);
    }
    else if (mode == BMI160_PWR_OFF)
    {
        // deactivate all used interrupts
        intSettings.int_channel = BMI160_INT_CHANNEL_NONE;

        intSettings.int_type = BMI160_ACC_GYRO_DATA_RDY_INT;
        retVal = bmi160_set_int_config(&intSettings, &bmi160dev);
        VERIFY_RSLT(retVal);

        intSettings.int_type = BMI160_ACC_ANY_MOTION_INT;
        retVal = bmi160_set_int_config(&intSettings, &bmi160dev);
        VERIFY_RSLT(retVal);
    }

    return NRF_SUCCESS;
}

static void pinChangeHandler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    // changing from on to standby mode often generates an undesired event, idk
    // if this is still a delayed drdy int or if the setup change causes the pin
    // to toggle. Until I know the exact reason I just ignore the first event
    // when switching form on to standby mode.

    (void)action;

    static bmi160drv_powerMode_t oldMode;

    if (pin != intPinNo)
        return;     // not my business

    if (oldMode >= BMI160_PWR_ON && powerMode == BMI160_PWR_STANDBY)
        dataReady = 0;  // ignore transition from idle to standby mode
    else
        dataReady++;

    oldMode = powerMode;
}

static ret_code_t configLocalInts(uint32_t pin)
{
    ret_code_t errCode;

    nrfx_gpiote_in_config_t intConfig = NRFX_GPIOTE_CONFIG_IN_SENSE_HITOLO(false);
    errCode = nrfx_gpiote_in_init(pin, &intConfig, &pinChangeHandler);
    if (errCode != NRF_SUCCESS)
        return errCode;

    nrfx_gpiote_in_event_enable(pin, true);

    intPinNo = pin;

    return NRF_SUCCESS;
}

static ret_code_t changeTimer(bmi160drv_powerMode_t mode)
{
    (void)app_timer_stop(pollingTimer);

    if (mode >= BMI160_PWR_ON)
        return app_timer_start(pollingTimer, TB_ON, NULL);
    else if (mode == BMI160_PWR_STANDBY)
        return app_timer_start(pollingTimer, TB_STDBY, NULL);

    return NRF_SUCCESS;
}

static void timerCallback(void *p_context)
{
    (void)p_context;

    dataReady++;
}

static ret_code_t configTimer()
{
    return app_timer_create(&pollingTimer, APP_TIMER_MODE_REPEATED, &timerCallback);
}

static ret_code_t changeSensor(bmi160drv_powerMode_t mode)
{
    int8_t retVal;

    if (mode >= BMI160_PWR_ON)
    {
        bmi160dev.accel_cfg.odr   = BMI160_ACCEL_ODR_100HZ;
        bmi160dev.accel_cfg.range = BMI160_ACCEL_RANGE_8G;
        bmi160dev.accel_cfg.bw    = BMI160_ACCEL_BW_OSR4_AVG1;
        bmi160dev.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

        bmi160dev.gyro_cfg.odr    = BMI160_GYRO_ODR_100HZ;
        bmi160dev.gyro_cfg.range  = BMI160_GYRO_RANGE_2000_DPS;
        bmi160dev.gyro_cfg.bw     = BMI160_GYRO_BW_NORMAL_MODE;
        bmi160dev.gyro_cfg.power  = BMI160_GYRO_NORMAL_MODE;
    }
    else if (mode == BMI160_PWR_STANDBY)
    {
        bmi160dev.accel_cfg.odr   = BMI160_ACCEL_ODR_50HZ;
        bmi160dev.accel_cfg.range = BMI160_ACCEL_RANGE_8G;
        bmi160dev.accel_cfg.bw    = BMI160_ACCEL_BW_OSR4_AVG1;
        bmi160dev.accel_cfg.power = BMI160_ACCEL_LOWPOWER_MODE;

        bmi160dev.gyro_cfg.power  = BMI160_GYRO_SUSPEND_MODE;
    }
    else // if (mode == BMI160_PWR_OFF)
    {
        bmi160dev.accel_cfg.power = BMI160_ACCEL_SUSPEND_MODE;
        bmi160dev.gyro_cfg.power  = BMI160_GYRO_SUSPEND_MODE;
    }

    /// TODO: sometimes this function fails with communication error, maybe to soon after soft reset?
    retVal = bmi160_set_sens_conf(&bmi160dev);
    if (retVal == BMI160_E_COM_FAIL)    // if communication failed, try again
        retVal = bmi160_set_sens_conf(&bmi160dev);
    if (retVal != BMI160_OK)
    {
        NRF_LOG_WARNING("bmi set sensor error %d", retVal);
    }

    return retVal == BMI160_OK ? NRF_SUCCESS : NRF_ERROR_INTERNAL;
}

static ret_code_t initBSXlite()
{
    bsxlite_return_t retVal = BSXLITE_OK;

    retVal = bsxlite_init(&bsxInst);

    return retVal == BSXLITE_OK ? NRF_SUCCESS : NRF_ERROR_INTERNAL;
}

static ret_code_t modeChangeBSXlite(bmi160drv_powerMode_t newMode)
{
    bsxlite_return_t retVal = BSXLITE_OK;

    if (newMode >= BMI160_PWR_ON)
        retVal = bsxlite_set_to_default(&bsxInst);

    return retVal == BSXLITE_OK ? NRF_SUCCESS : NRF_ERROR_INTERNAL;
}

static void processDataBSXlite(int16_t const* pAccel, int16_t const *pGyro, int32_t timestamp)
{
    vector_3d_t accel, gyro;
    bsxlite_out_t out;
    bsxlite_return_t retVal;
    bmi160drv_event_t event;

    accel.x = pAccel[0] * ACC_SCALING_ACC_LSB_TO_MPS2;
    accel.y = pAccel[1] * ACC_SCALING_ACC_LSB_TO_MPS2;
    accel.z = pAccel[2] * ACC_SCALING_ACC_LSB_TO_MPS2;

    gyro.x = pGyro[0] * GYROSCOPE_SCALING_GYRO_LSB_TO_RADPS;
    gyro.y = pGyro[1] * GYROSCOPE_SCALING_GYRO_LSB_TO_RADPS;
    gyro.z = pGyro[2] * GYROSCOPE_SCALING_GYRO_LSB_TO_RADPS; /// watch !

    timestamp = timestamp * 39;    // this is just a 24bit timer, problem?

    retVal = bsxlite_do_step(&bsxInst, timestamp, &accel, &gyro, &out);
    if (retVal != BSXLITE_OK)
    {
        /// TODO: reset for critical error?
        return;
    }

    out.orientation.pitch *= -1.f;
    out.orientation.roll *= -1.f;
    out.orientation.yaw *= -1.f;

    if (out.orientation.pitch < 0)
        out.orientation.pitch += M_2PI;
    out.orientation.pitch *= (1 << 15);
    out.orientation.pitch /= M_2PI;

    if (out.orientation.roll < 0)
        out.orientation.roll += M_2PI;
    out.orientation.roll *= (1 << 15);
    out.orientation.roll /= M_2PI;

    if (out.orientation.yaw < 0)
        out.orientation.yaw += M_2PI;
    out.orientation.yaw *= (1 << 15);
    out.orientation.yaw /= M_2PI;

    event.type = BMI160_EVT_TYPE_DATA;
    event.data.euler[0] = out.orientation.pitch;
    event.data.euler[1] = out.orientation.roll;
    event.data.euler[2] = out.orientation.yaw;

    if (evtHandler)
        evtHandler(&event);
}

/* Public functions ----------------------------------------------------------*/
ret_code_t bmi_Init(bmi160drv_init_t const* pInit)
{
    ret_code_t errCode;

    if (pInit == NULL || pInit->handler == NULL)
        return NRF_ERROR_NULL;

    errCode = initSensor(pInit->addr);
    if (errCode != NRF_SUCCESS)
        return errCode;

    if (nrf_gpio_pin_present_check(pInit->int1Pin))
        errCode = configLocalInts(pInit->int1Pin);
    else
        errCode = configTimer();
    if (errCode != NRF_SUCCESS)
        return errCode;

    errCode = initBSXlite();
    if (errCode != NRF_SUCCESS)
        return errCode;

    reorient = pInit->reorient;
    evtHandler = pInit->handler;

    return NRF_SUCCESS;
}

ret_code_t bmi_SetPowerMode(bmi160drv_powerMode_t newMode)
{
    ret_code_t errCode;
    bool onboard = IS_ONBOARD_SENSOR();

    if (!IS_SENSOR_PRESENT())
        return NRF_ERROR_NOT_FOUND;

    if (newMode == powerMode)
        return NRF_SUCCESS;

    if (onboard)
        nrfx_gpiote_in_event_disable(intPinNo);

    errCode = modeChangeBSXlite(newMode);
    if (errCode != NRF_SUCCESS)
        return errCode;

    errCode = changeSensor(newMode);
    if (errCode != NRF_SUCCESS)
        return errCode;

    errCode = configRemoteInts(newMode, onboard);
    if (errCode != NRF_SUCCESS)
        return errCode;

    if (onboard)
    {
        powerMode = newMode;
        nrfx_gpiote_in_event_enable(intPinNo, true);
    }
    else
    {
        errCode = changeTimer(newMode);
        if (errCode != NRF_SUCCESS)
            return errCode;
        powerMode = newMode;
    }

    return NRF_SUCCESS;
}

/** @brief function to read and process imu data
 *
 * @note typical CPU usage ~51000-53000 clock cycles
 */
static void readData(bool justMotion)
{
    int8_t retVal;
    union bmi160_int_status irq = {0};
    union bmi160_status status = {0};
    struct bmi160_sensor_data rawAccel;
    struct bmi160_sensor_data rawGyro;
    int16_t accel[3], gyro[3];
    bmi160drv_event_t event;

    if (evtHandler == NULL)
        return;

    retVal = bmi160_get_int_status(BMI160_INT_STATUS_0, &irq, &bmi160dev);
    if (retVal == BMI160_OK && irq.bit.anym)
    {
        event.type = BMI160_EVT_TYPE_MOTION;
        evtHandler(&event);
        //NRF_LOG_INFO("motion from read");
    }

    if (justMotion)
        return;

    retVal = bmi160_get_status(&status, &bmi160dev);
    if (retVal != BMI160_OK || !(status.bit.drdy_acc || status.bit.drdy_gyr))
        return;

    retVal = bmi160_get_sensor_data((BMI160_ACCEL_SEL | BMI160_GYRO_SEL | BMI160_TIME_SEL), &rawAccel, &rawGyro, &bmi160dev);
    if (retVal != BMI160_OK)
        return;

    accel[0] = rawAccel.x;
    accel[1] = rawAccel.y;
    accel[2] = rawAccel.z;
    gyro[0]  = rawGyro.x;
    gyro[1]  = rawGyro.y;
    gyro[2]  = rawGyro.z;

    if (reorient)
        reorient(accel, gyro);

    processDataBSXlite(accel, gyro, rawAccel.sensortime);
}

void bmi_Execute()
{
    if (!IS_SENSOR_PRESENT())
        return;

    /// TODO: If GPIOTE int priority is higher than TWI int priority an change from on to standby triggers a motion event

    while (dataReady)
    {
        switch (powerMode)
        {
        case BMI160_PWR_ON:
            readData(false);
            break;
        case BMI160_PWR_STANDBY:
            if (IS_ONBOARD_SENSOR() && evtHandler)
            {
                bmi160drv_event_t event = {.type = BMI160_EVT_TYPE_MOTION};
                evtHandler(&event);
                //NRF_LOG_INFO("motion from int, %d", dataReady);
            }
            else
                readData(true);
            break;
        default:
            break;
        }

        dataReady = 0;
    }
}

/**END OF FILE*****************************************************************/
