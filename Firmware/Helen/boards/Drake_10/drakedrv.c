/**
  ******************************************************************************
  * @file    drakedrv.c
  * @author  Thomas Reisnecker
  * @brief   driver module for controlling drake via i2c
  ******************************************************************************
  */

/* logger configuration ----------------------------------------------_-------*/
#define DRD_CONFIG_LOG_ENABLED 1 /// TODO: just temporary until project specific sdk_config
#define DRD_CONFIG_LOG_LEVEL   3

#define NRF_LOG_MODULE_NAME drakedrv
#if DRD_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL DRD_CONFIG_LOG_LEVEL
#else //BASE_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL 0
#endif //BASE_CONFIG_LOG_ENABLED
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

/* Includes ------------------------------------------------------------------*/
#include "i2c.h"
#include "drakedrv.h"
#include "drake_regs.h"

/* Private typedef -----------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/
#define VERIFY_PRESENT()            \
do                                  \
{                                   \
    if (!isPresent)                 \
        return NRF_ERROR_NOT_FOUND; \
} while(0)

/* Private defines -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static bool isPresent;

/* Private read only variables -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
static ret_code_t read(uint8_t reg, uint8_t* pBuffer, uint16_t cnt)
{
    return i2c_Read(DRK_ADDR, reg, pBuffer, cnt);
}

static ret_code_t write(uint8_t reg, uint8_t const* pData, uint16_t cnt)
{
    return i2c_Write(DRK_ADDR, reg, pData, cnt);
}

/* Public functions ----------------------------------------------------------*/
ret_code_t drakedrv_Init(bool* pIsMotorSensorPresent, bool* pIsTravelSensorPresent)
{
    if (pIsMotorSensorPresent == NULL || pIsTravelSensorPresent == NULL)
        return NRF_ERROR_NULL;

    ret_code_t errCode;
    uint8_t buf;

    errCode = read(DRK_REG_WHOAMI, &buf, 1);
    if (errCode == NRFX_ERROR_DRV_TWI_ERR_ANACK)
        return NRF_ERROR_NOT_FOUND;
    else if (errCode != NRF_SUCCESS)
        return errCode;

    if (buf != DRK_WHO_AM_I)
        return NRF_ERROR_NOT_FOUND;

    isPresent = true;

    errCode = read(DRK_REG_STATUS0, &buf, 1);
    VERIFY_SUCCESS(errCode);

    *pIsMotorSensorPresent = buf & DRK_STATUS_MPR_MASK;
    *pIsTravelSensorPresent = buf & DRK_STATUS_TPR_MASK;

    return NRF_SUCCESS;
}

ret_code_t drakedrv_ReqSysOff()
{
    VERIFY_PRESENT();

    uint8_t buf = DRK_PWR_SYS_OFF_MASK;

    return write(DRK_REG_PWR, &buf, 1);
}

ret_code_t drakedrv_GetMotorPosition(q8_t* pPosition)
{
    VERIFY_PRESENT();
    VERIFY_PARAM_NOT_NULL(pPosition);

    return read(DRK_REG_MDATA, pPosition, 1);
}

ret_code_t drakedrv_SetMotorPosition(q8_t position)
{
    VERIFY_PRESENT();

    return write(DRK_REG_MDATA, &position, 1);
}

ret_code_t drakedrv_GetTravel(q8_t* pTravel)
{
    VERIFY_PRESENT();
    VERIFY_PARAM_NOT_NULL(pTravel);

    return read(DRK_REG_TDATA, pTravel, 1);
}

ret_code_t drakedrv_GetMotorConfig(drakedrv_motorConfig_t* pMotorConfig)
{
    VERIFY_PRESENT();
    VERIFY_PARAM_NOT_NULL(pMotorConfig);

    uint8_t buf[4];
    ret_code_t errCode;

    errCode = read(DRK_REG_MTCFG0, buf, 4);
    if (errCode == NRF_SUCCESS)
    {
        pMotorConfig->dutyCycleOpen = buf[0];
        pMotorConfig->dutyCycleClose = buf[1];
        pMotorConfig->timeoutOpen = buf[2];
        pMotorConfig->timeoutClose = buf[3];
    }

    return errCode;
}

ret_code_t drakedrv_SetMotorConfig(drakedrv_motorConfig_t const* pMotorConfig)
{
    VERIFY_PRESENT();
    VERIFY_PARAM_NOT_NULL(pMotorConfig);

    uint8_t buf[4];

    buf[0] = pMotorConfig->dutyCycleOpen;
    buf[1] = pMotorConfig->dutyCycleClose;
    buf[2] = pMotorConfig->timeoutOpen;
    buf[3] = pMotorConfig->timeoutClose;

    return write(DRK_REG_MTCFG0, buf, 4);
}

ret_code_t drakedrv_GetMotorSensorData(drakedrv_motorSensorData_t* pMotorSensorData)
{
    VERIFY_PRESENT();
    VERIFY_PARAM_NOT_NULL(pMotorSensorData);

    uint8_t buf[4];
    ret_code_t errCode;

    errCode = read(DRK_REG_MTCAL1, buf, 4);
    if (errCode == NRF_SUCCESS)
    {
        pMotorSensorData->threshLow = buf[0];
        pMotorSensorData->threshUp = buf[1];
        pMotorSensorData->fullOpenCnt = buf[2] + (buf[3] << 8);
    }

    return errCode;
}

ret_code_t drakedrv_ClearThresholdCalibrationData(void)
{
    VERIFY_PRESENT();

    uint8_t buf = DRK_CALIB_THC_MASK;

    return write(DRK_REG_MTCAL0, &buf, 1);
}

ret_code_t drakedrv_StartThresholdCalibration(void)
{
    VERIFY_PRESENT();

    uint8_t buf = DRK_CALIB_THS_MASK;

    return write(DRK_REG_MTCAL0, &buf, 1);
}

ret_code_t drakedrv_GetFullOpenCalbrationData(uint16_t* pFullOpen)
{
    VERIFY_PRESENT();
    VERIFY_PARAM_NOT_NULL(pFullOpen);

    uint8_t buf[2];
    ret_code_t errCode;

    errCode = read(DRK_REG_MTCAL3, buf, 2);
    if (errCode == NRF_SUCCESS)
        *pFullOpen = buf[0] + (buf[1] << 8);

    return errCode;
}

ret_code_t drakedrv_ClearFullOpenThresholdData(void)
{
    VERIFY_PRESENT();

    uint8_t buf = DRK_CALIB_FOC_MASK;

    return write(DRK_REG_MTCAL0, &buf, 1);
}

ret_code_t drakedrv_StartFullOpenCalibration(void)
{
    VERIFY_PRESENT();

    uint8_t buf = DRK_CALIB_FOS_MASK;

    return write(DRK_REG_MTCAL0, &buf, 1);
}

ret_code_t drakedrv_GetTravelSensorData(drakedrv_travelSensorData_t* pTravelSensor)
{
    VERIFY_PRESENT();
    VERIFY_PARAM_NOT_NULL(pTravelSensor);

    uint8_t buf[4];
    ret_code_t errCode;

    errCode = read(DRK_REG_TRVCAL1, buf, 4);
    if (errCode == NRF_SUCCESS)
    {
        pTravelSensor->travel = buf[0] + (buf[1] << 8);
        pTravelSensor->offset = buf[2] + (buf[3] << 8);
    }

    return errCode;
}

ret_code_t drakedrv_StartTravelSensorOffsetCalibration(uint16_t travel)
{
    VERIFY_PRESENT();

    uint8_t buf[2];

    buf[0] = travel & 0xFF;
    buf[1] = travel >> 8;

    return write(DRK_REG_TRVCAL1, buf, 2);
}

ret_code_t drakedrv_ClearTravelSensorOffsetCalibrationData(void)
{
    VERIFY_PRESENT();

    uint8_t buf = DRK_CALIB_TOC_MASK;

    return write(DRK_REG_TRVCAL0, &buf, 1);
}

ret_code_t drakedrv_GetIndexedTravelConfig(drakedrv_indexedConfig_t* pIndexedConfig)
{
    VERIFY_PRESENT();
    VERIFY_PARAM_NOT_NULL(pIndexedConfig);

    uint8_t buf[2 + DRK_NUM_OF_IND_POS];
    ret_code_t errCode;

    errCode = read(DRK_REG_TRVCAL1, buf, sizeof(buf));
    if (errCode == NRF_SUCCESS)
    {
        pIndexedConfig->openBlocking = buf[0];
        pIndexedConfig->closeTimeout = buf[1];
        for (uint8_t i = 0; i < DRK_NUM_OF_IND_POS; i++)
        {
            pIndexedConfig->positions[i] = buf[2 + i];
        }
    }

    return errCode;
}

ret_code_t drakedrv_SetIndexedTravelConfig(drakedrv_indexedConfig_t const* pIndexedConfig)
{
    VERIFY_PRESENT();
    VERIFY_PARAM_NOT_NULL(pIndexedConfig);

    uint8_t buf[2 + DRK_NUM_OF_IND_POS];

    buf[0] = pIndexedConfig->openBlocking;
    buf[1] = pIndexedConfig->closeTimeout;
    for (uint8_t i = 0; i < DRK_NUM_OF_IND_POS; i++)
    {
        buf[2 + 0] = pIndexedConfig->positions[i];
    }

    return write(DRK_REG_TRVCAL1, buf, 2);
}

/**END OF FILE*****************************************************************/
