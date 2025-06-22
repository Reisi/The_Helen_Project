/**
  ******************************************************************************
  * @file    Drake_i2cs.h
  * @author  Thomas Reisnecker
  * @brief   plunger control over i2c slave for Helen Drake Hardware.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DRAKE_I2CS_H_INCLUDED
#define DRAKE_I2CS_H_INCLUDED

/* Includes ------------------------------------------------------------------*/
#include "drake_regs.h"

/* Exported constants --------------------------------------------------------*/
//#define DRAKE_I2CS_NUM_OF_IND_POS   4

/* Exported types ------------------------------------------------------------*/
typedef enum
{
    DRAKE_I2CS_READ_OPERATION,      // a read operation is ongoing
    DRAKE_I2CS_SYS_OFF_REQ,         // sys off was requested
    DRAKE_I2CS_MOT_POS_RCVD,        // new motor position received
    DRAKE_I2CS_MOT_CFG_RCVD,        // new motor config received
    DRAKE_I2CS_MOT_CAL_THR_CLEAR,   // request to clear the threshold calibration data
    DRAKE_I2CS_MOT_CAL_THR_START,   // request to start the threshold calibration
    DRAKE_I2CS_MOT_CAL_FO_CLEAR,    // request to clear the full open calibration data
    DRAKE_I2CS_MOT_CAL_FO_START,    // request to start the full open calibration
    DRAKE_I2CS_TRV_CAL_OFF_CLEAR,   // request to clear the travel sensor offset calibration data
    DRAKE_I2CS_TRV_CAL_OFF_START,   // request to start the travel sensor offset calibration
    DRAKE_I2CS_IND_CFG_RCVD         // new indexed configuration received
} Drake_I2cs_event_type_t;

typedef struct
{
    uint8_t dutyCycleOpen;  // DC 0..255
    uint8_t dutyCycleClose; // DC 0..255
    uint8_t timeoutOpen;    // timeout in 1/128s
    uint8_t timeoutClose;   // timeout in 1/128s
} Drake_I2cs_motorConfig_t;

typedef uint8_t q8_t;

typedef struct
{
    uint8_t threshLow;
    uint8_t threshUp;
    uint16_t fullOpenCnt;
} Drake_I2cs_sensorConfig_t;

typedef struct
{
    uint16_t travel;        // total seatpost travel in mm
    int16_t offset;         // sensor offset in µm
} Drake_I2cs_travelSensor_t;

typedef struct
{
    uint8_t openBlocking;
    uint8_t closeTimeout;
    q8_t positions[DRK_NUM_OF_IND_POS];
} Drake_I2cs_indexedConfig_t;

typedef struct
{
    Drake_I2cs_event_type_t type;
    union
    {
        q8_t motorPosition;
        Drake_I2cs_motorConfig_t const* pMotorCfg;
        uint16_t travel;
        Drake_I2cs_indexedConfig_t const* pIndexedCfg;
    };
} Drake_I2cs_event_t;

typedef void(*Drake_I2cs_eventHandler_t)(Drake_I2cs_event_t const* pEvt);

typedef struct
{
    Drake_I2cs_eventHandler_t handler;
    bool isMotorSensorPresent;
    bool isTravelSensorPresent;
    q8_t motorPosition;
    q8_t travel;
    Drake_I2cs_motorConfig_t const* pMotorCfg;
    Drake_I2cs_sensorConfig_t const* pSensorCfg;
    Drake_I2cs_travelSensor_t const* pTravelSens;
    Drake_I2cs_indexedConfig_t const* pIndexedCfg;
} Drake_I2cs_init_t;

/* Exported macros -----------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
ret_code_t Drake_I2cs_Init(Drake_I2cs_init_t const* pInit);

void Drake_I2cs_UpdateMotPos(q8_t motorPosition);

void Drake_I2cs_UpdateTravel(q8_t travel);

void Drake_I2cs_UpdateMotorConfig(Drake_I2cs_motorConfig_t const* pMotorCfg);

void Drake_I2cs_UpdateSensorConfig(Drake_I2cs_sensorConfig_t const* pSensorCfg);

void Drake_I2cs_UpdateTravelSensor(Drake_I2cs_travelSensor_t const* pTravelSens);

void Drake_I2cs_UpdateIndexedConfig(Drake_I2cs_indexedConfig_t const* pIndexedCfg);

#endif // DRAKE_I2CS_H_INCLUDED

/**END OF FILE*****************************************************************/

