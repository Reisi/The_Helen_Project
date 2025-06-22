/**
  ******************************************************************************
  * @file    drakedrv.h
  * @author  Thomas Reisnecker
  * @brief   driver for drake over i2c
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DRAKEDRV_H_INCLUDED
#define DRAKEDRV_H_INCLUDED

/* Includes ------------------------------------------------------------------*/
#include "sdk_errors.h"
#include "stdint.h"
#include "drake_regs.h"

/* Exported types ------------------------------------------------------------*/
typedef uint8_t q8_t;

typedef struct
{
    uint8_t dutyCycleOpen;  // DC 0..255
    uint8_t dutyCycleClose; // DC 0..255
    uint8_t timeoutOpen;    // timeout in 1/128s
    uint8_t timeoutClose;   // timeout in 1/128s
} drakedrv_motorConfig_t;

typedef struct
{
    uint8_t threshLow;
    uint8_t threshUp;
    uint16_t fullOpenCnt;
} drakedrv_motorSensorData_t;

typedef struct
{
    uint16_t travel;        // total seatpost travel in mm
    int16_t offset;         // sensor offset in µm
} drakedrv_travelSensorData_t;

typedef struct
{
    uint8_t openBlocking;
    uint8_t closeTimeout;
    q8_t positions[DRK_NUM_OF_IND_POS];
} drakedrv_indexedConfig_t;

/* Exported constants --------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
/** @brief function to initialize the drake driver
 *
 * @return NRF_SUCCESS, tbd
 */
ret_code_t drakedrv_Init(bool* pIsMotorSensorPresent, bool* pIsTravelSensorPresent);

ret_code_t drakedrv_ReqSysOff();

ret_code_t drakedrv_GetMotorPosition(q8_t* pPosition);

ret_code_t drakedrv_SetMotorPosition(q8_t position);

ret_code_t drakedrv_GetTravel(q8_t* pTravel);

ret_code_t drakedrv_GetMotorConfig(drakedrv_motorConfig_t* pMotorConfig);

ret_code_t drakedrv_SetMotorConfig(drakedrv_motorConfig_t const* pMotorConfig);

ret_code_t drakedrv_GetMotorSensorData(drakedrv_motorSensorData_t* pMotorSensorData);

ret_code_t drakedrv_ClearThresholdCalibrationData(void);

ret_code_t drakedrv_StartThresholdCalibration(void);

ret_code_t drakedrv_ClearFullOpenThresholdData(void);

ret_code_t drakedrv_StartFullOpenCalibration(void);

ret_code_t drakedrv_GetTravelSensorData(drakedrv_travelSensorData_t* pTravelSensor);

ret_code_t drakedrv_StartTravelSensorOffsetCalibration(uint16_t travel);

ret_code_t drakedrv_ClearTravelSensorOffsetCalibrationData(void);

ret_code_t drakedrv_GetIndexedTravelConfig(drakedrv_indexedConfig_t* pIndexedConfig);

ret_code_t drakedrv_SetIndexedTravelConfig(drakedrv_indexedConfig_t const* pIndexedConfig);

#endif // PL_CTRL_REM_H_INCLUDED

/**END OF FILE*****************************************************************/

