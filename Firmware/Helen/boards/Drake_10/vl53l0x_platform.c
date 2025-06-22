/**
  ******************************************************************************
  * @file    vl53l0x_plattform.c
  * @author  Thomas Reisnecker
  * @brief   tbd.
  ******************************************************************************
  */

/* logger configuration ----------------------------------------------_-------*/
#define TOF_CONFIG_LOG_ENABLED 1 /// TODO: just temporary until project specific sdk_config
#define TOF_CONFIG_LOG_LEVEL   3

#define NRF_LOG_MODULE_NAME tof
#if TOF_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL TOF_CONFIG_LOG_LEVEL
#else //BASE_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL 0
#endif //BASE_CONFIG_LOG_ENABLED
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

/* Includes ------------------------------------------------------------------*/
#include "i2c.h"
#include "vl53l0x_platform.h"

/* Private typedef -----------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/
#define VERIFY_RSLT(x)              \
do                                  \
{                                   \
    if (x != NRF_SUCCESS)           \
        return x;                   \
} while (0)

/* Private defines -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private read only variables -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/* Public functions ----------------------------------------------------------*/
VL53L0X_Error VL53L0X_LockSequenceAccess(VL53L0X_DEV Dev)
{
    return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_UnlockSequenceAccess(VL53L0X_DEV Dev)
{
    return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_WriteMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pData, uint32_t count)
{
    ret_code_t errCode;

    errCode = i2c_Write(Dev->I2cDevAddr, index, pData, count);

    if (errCode == NRFX_ERROR_DRV_TWI_ERR_ANACK)
        return VL53L0X_ERROR_CONTROL_INTERFACE - 1;
    else if (errCode != NRF_SUCCESS)
        return VL53L0X_ERROR_CONTROL_INTERFACE;
    else
        return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_ReadMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pData, uint32_t count)
{
    ret_code_t errCode;

    errCode = i2c_Read(Dev->I2cDevAddr, index, pData, count);

    return errCode == NRF_SUCCESS ? VL53L0X_ERROR_NONE : VL53L0X_ERROR_UNDEFINED;
}

VL53L0X_Error VL53L0X_WrByte(VL53L0X_DEV Dev, uint8_t index, uint8_t data)
{
    return VL53L0X_WriteMulti(Dev, index, &data, 1);
}

VL53L0X_Error VL53L0X_WrWord(VL53L0X_DEV Dev, uint8_t index, uint16_t data)
{
    uint8_t buffer[2] =
    {
        (data >> 8) & 0xFF,
        (data >> 0) & 0xFF
    };

    return VL53L0X_WriteMulti(Dev, index, buffer, 2);
}

VL53L0X_Error VL53L0X_WrDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t data)
{
    uint8_t buffer[4] =
    {
        (data >> 24) & 0xFF,
        (data >> 16) & 0xFF,
        (data >> 8) & 0xFF,
        (data >> 0) & 0xFF
    };

    return VL53L0X_WriteMulti(Dev, index, buffer, 4);
}

VL53L0X_Error VL53L0X_RdByte(VL53L0X_DEV Dev, uint8_t index, uint8_t *data)
{
    return VL53L0X_ReadMulti(Dev, index, data, 1);
}

VL53L0X_Error VL53L0X_RdWord(VL53L0X_DEV Dev, uint8_t index, uint16_t *data)
{
    uint8_t buffer[2];
    VL53L0X_Error errCode;

    errCode = VL53L0X_ReadMulti(Dev, index, buffer, 2);

    if (errCode == VL53L0X_ERROR_NONE)
        *data = ((uint16_t)buffer[0] << 8) + ((uint16_t)buffer[1] << 0);

    return errCode;
}

VL53L0X_Error VL53L0X_RdDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t *data)
{
    uint8_t buffer[4];
    VL53L0X_Error errCode;

    errCode = VL53L0X_ReadMulti(Dev, index, buffer, 4);

    if (errCode == VL53L0X_ERROR_NONE)
        *data = ((uint32_t)buffer[0] << 24) + ((uint32_t)buffer[1] << 16) +
                ((uint32_t)buffer[2] << 8) + ((uint32_t)buffer[3] << 0);

    return errCode;
}

VL53L0X_Error VL53L0X_UpdateByte(VL53L0X_DEV Dev, uint8_t index, uint8_t AndData, uint8_t OrData)
{
    uint8_t buffer;
    VL53L0X_Error errCode;

    errCode = VL53L0X_RdByte(Dev, index, &buffer);
    if (errCode != VL53L0X_ERROR_NONE)
        return errCode;

    buffer = (buffer & AndData) | OrData;

    return VL53L0X_WrByte(Dev, index, buffer);
}

VL53L0X_Error VL53L0X_PollingDelay(VL53L0X_DEV Dev)
{
    return VL53L0X_ERROR_NONE;
}

/**END OF FILE*****************************************************************/
