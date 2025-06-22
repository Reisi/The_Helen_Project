/**
  ******************************************************************************
  * @file    vl53l0xdrv.c
  * @author  Thomas Reisnecker
  * @brief   tbd.
  ******************************************************************************
  */

/* logger configuration ----------------------------------------------_-------*/
#define TOF_CONFIG_LOG_ENABLED 1 /// TODO: just temporary until project specific sdk_config
#define TOF_CONFIG_LOG_LEVEL   3

#define NRF_LOG_MODULE_NAME imu
#if TOF_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL TOF_CONFIG_LOG_LEVEL
#else //TOF_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL 0
#endif //TOF_CONFIG_LOG_ENABLED
#include "nrf_log.h"
//NRF_LOG_MODULE_REGISTER();

/* Includes ------------------------------------------------------------------*/
#include "vl53l0xdrv.h"
#include "vl53l0x_api.h"
#include "app_timer.h"
#include "stdlib.h"

/* Private typedef -----------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/
#define VERIFY_RESULT(x)            \
do                                  \
{                                   \
    if (x != VL53L0X_ERROR_NONE)    \
        return NRF_ERROR_INTERNAL;  \
} while(0)

/* Private defines -----------------------------------------------------------*/
#define POLLING_PERIOD      APP_TIMER_TICKS(10)
#define TEMPERATURE_STEP    (8 << 7)

/* Private variables ---------------------------------------------------------*/
static VL53L0X_Dev_t vlDev;
APP_TIMER_DEF(pollingTimer);
static vl53l0xdrv_powerMode_t currentPwrMode;
static bool pollingFlag;
static vl53l0xdrv_eventHandler_t handler;
static q9_7_t lastTemperature;
static int32_t defaultOffset;

/* Private read only variables -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
static ret_code_t initSensor(int32_t offset)
{
    VL53L0X_Error errCode;

    /// TODO: can we check the presence of device?

    // reset device, since we don't know if this was a power on or system off start
    /// TODO: maybe check if already initialized and skip?
    errCode = VL53L0X_ResetDevice(&vlDev);
    if (errCode == VL53L0X_ERROR_CONTROL_INTERFACE - 1) /// TODO: define No Ack Error Code instead
        return NRF_ERROR_NOT_FOUND;
    VERIFY_RESULT(errCode);

    // delay necessary?

    // initialization
    errCode = VL53L0X_DataInit(&vlDev);
    VERIFY_RESULT(errCode);

    errCode = VL53L0X_StaticInit(&vlDev);
    VERIFY_RESULT(errCode);

    // perform calibrations
    uint8_t isApertureSpads;
    uint32_t refSpadCnt;
    errCode = VL53L0X_PerformRefSpadManagement(&vlDev, &refSpadCnt, &isApertureSpads);
    VERIFY_RESULT(errCode);

    // no temp calibration here, because adc isn't initialized yet, instead do it in Execute()

    // load default offset calibration data, to be able to set to default later
    errCode = VL53L0X_GetOffsetCalibrationDataMicroMeter(&vlDev, &defaultOffset);
    VERIFY_RESULT(errCode);

    // set offset calibration data if available
    if (offset)
    {
        errCode = VL53L0X_SetOffsetCalibrationDataMicroMeter(&vlDev, offset);
        VERIFY_RESULT(errCode);
    }

    /// TODO: load calibration data for xtalk (if necessary)

    // setting device to fast mode
    errCode = VL53L0X_SetLimitCheckValue(&vlDev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.25*65536));
    VERIFY_RESULT(errCode);

    errCode = VL53L0X_SetLimitCheckValue(&vlDev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(32*65536));
    VERIFY_RESULT(errCode);

    errCode = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(&vlDev, 20000);
    VERIFY_RESULT(errCode);

    // enable continuous ranging
    errCode = VL53L0X_SetDeviceMode(&vlDev, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
    VERIFY_RESULT(errCode);

    return NRF_SUCCESS;
}

static void pollingHandler(void* pContext)
{
    (void)pContext;

    pollingFlag = true;
}

/* Public functions ----------------------------------------------------------*/
ret_code_t vl53l0xdrv_Init(vl53l0xdrv_init_t const* pInit)
{
    ret_code_t errCode;

    if (pInit == NULL || pInit->handler == NULL)
        return NRF_ERROR_NULL;

    vlDev.I2cDevAddr = VL53L0X_ADDR;

    errCode = initSensor(pInit->offsetInMicrom);
    if (errCode != NRF_SUCCESS)
        return errCode;

    handler = pInit->handler;

    errCode = app_timer_create(&pollingTimer, APP_TIMER_MODE_REPEATED, pollingHandler);
    if (errCode != NRF_SUCCESS)
        return errCode;

    return NRF_SUCCESS;
}

ret_code_t vl53l0xdrv_SetPowerMode(vl53l0xdrv_powerMode_t powerMode)
{
    if (handler == NULL)
        return NRF_ERROR_INVALID_STATE;

    if (currentPwrMode == powerMode)
        return NRF_SUCCESS;

    VL53L0X_Error errCode;

    if (powerMode == VL53L0X_PWR_DATA)
    {
        // enable ranging
        errCode = VL53L0X_StartMeasurement(&vlDev);
        VERIFY_RESULT(errCode);

        currentPwrMode = VL53L0X_PWR_DATA;

        // enable polling timer
        return app_timer_start(pollingTimer, POLLING_PERIOD, NULL);
    }
    else if (powerMode == VL53L0X_PWR_OFF)
    {
        // disable ranging
        errCode = VL53L0X_StopMeasurement(&vlDev);
        VERIFY_RESULT(errCode);

        currentPwrMode = VL53L0X_PWR_OFF;

        // polling timer is disabled after reading final result
        return NRF_SUCCESS;
    }
    else
        return NRF_ERROR_INVALID_PARAM;
}

//volatile VL53L0X_RangingMeasurementData_t data;

void vl53l0xdrv_Execute(q9_7_t temperature)
{
    VL53L0X_Error errCode;
    uint8_t dataRdy = 0;
    VL53L0X_RangingMeasurementData_t data;

    if (handler == NULL)
        return;

    if (abs((int32_t)lastTemperature - temperature) >= TEMPERATURE_STEP) // this will also trigger a calibration after start when lastTemperature is 0
    {
        uint8_t vhvSet, phaseCal;
        errCode = VL53L0X_PerformRefCalibration(&vlDev, &vhvSet, &phaseCal);
        if (errCode != VL53L0X_ERROR_NONE)
            NRF_LOG_WARNING("error %d performing Temp calibration", errCode);

        lastTemperature = temperature;

        // temp calibration stops measurements, so restart is required
        if (currentPwrMode == VL53L0X_PWR_DATA)
        {
            errCode = VL53L0X_StartMeasurement(&vlDev);
            if (errCode != VL53L0X_ERROR_NONE)
                NRF_LOG_ERROR("error %d restarting after Temp calibration", errCode);
        }
    }

    if (!pollingFlag)
        return;

    if (currentPwrMode == VL53L0X_PWR_DATA)
    {
        errCode = VL53L0X_GetMeasurementDataReady(&vlDev, &dataRdy);
        if (errCode != VL53L0X_ERROR_NONE)
        {
            NRF_LOG_WARNING("error %d getting data ready", errCode);
            return;
        }
    }
    else // if (currentPwrMode == VL53L0X_PWR_OFF)
    {
        uint32_t stopStatus = 0;

        errCode = VL53L0X_GetStopCompletedStatus(&vlDev, &stopStatus);   /// TODO: what is this function actually returning?
        if (errCode != VL53L0X_ERROR_NONE)
        {
            NRF_LOG_WARNING("error getting stop status");
            return;
        }

        dataRdy = stopStatus ? 1 : 0;

        (void)app_timer_stop(pollingTimer);
    }

    if (!dataRdy)
        return;

    errCode = VL53L0X_GetRangingMeasurementData(&vlDev, (VL53L0X_RangingMeasurementData_t*)&data);
    if (errCode != VL53L0X_ERROR_NONE)
    {
        NRF_LOG_WARNING("error %d getting data", errCode);
        return;
    }

    errCode = VL53L0X_ClearInterruptMask(&vlDev, VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);
    if (errCode != VL53L0X_ERROR_NONE)
    {
        NRF_LOG_WARNING("error %d clearing interrupt mask", errCode);
        return;
    }

    if (handler)
        handler(data.RangeMilliMeter);
}

ret_code_t vl53l0xdrv_CalibrateOffset(uint16_t offsetInmm, int32_t* pNewOffsetInMikrom)
{
    if (handler == NULL)
        return NRF_ERROR_INVALID_STATE;

    VL53L0X_Error errCode;

    errCode = VL53L0X_PerformOffsetCalibration(&vlDev, offsetInmm << 16, pNewOffsetInMikrom);
    VERIFY_RESULT(errCode);

    return NRF_SUCCESS;
}

ret_code_t cl53l0xdrv_ClearOffsetCalibration()
{
    if (handler == NULL)
        return NRF_ERROR_INVALID_STATE;

    VL53L0X_Error errCode;

    errCode = VL53L0X_SetOffsetCalibrationDataMicroMeter(&vlDev, defaultOffset);
    VERIFY_RESULT(errCode);

    return NRF_SUCCESS;
}

/**END OF FILE*****************************************************************/
