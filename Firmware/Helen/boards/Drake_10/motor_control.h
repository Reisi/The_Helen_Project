/**
  ******************************************************************************
  * @file    motor_control.h
  * @author  Thomas Reisnecker
  * @brief   local motor control for Helen Drake Hardware.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MTCTRL_H_INCLUDED
#define MTCTRL_H_INCLUDED

/* Includes ------------------------------------------------------------------*/
#include "sdk_errors.h"
#include "motor_sensor.h"

/* Exported types ------------------------------------------------------------*/
typedef uint8_t q8_t;

typedef enum
{
    MTCTRL_EVT_TYPE_POWER_FAIL,                // event sent when supply voltage dropped below 2.8V and motor was shut off early
    MTCTRL_EVT_TYPE_NEW_POSITION,              // new motor position value
    MTCTRL_EVT_TYPE_CALIB_SENSOR,              // new calibration values for motor sensor
    MTCTRL_EVT_TYPE_CALIB_FULL_OPEN            // new value for full open count
} mtCtrl_eventType_t;

typedef struct
{
    mtCtrl_eventType_t type;
    union
    {
        q8_t position;                          // data for event @REF MTCTRL_EVT_TYPE_NEW_POSITION
        mtSens_thresholds_t const* pThresholds; // data for event @REF MTCTRL_EVT_TYPE_CALIB_SENSOR
        uint16_t fullyOpenCount;                // data for event @REF MTCTRL_EVT_TYPE_CALIB_POSITION
    };
} mtCtrl_event_t;

typedef void(*mtCtrl_eventHandler_t)(mtCtrl_event_t const* pEvt);

typedef struct
{
    uint32_t openTimeout;
    uint32_t closeTimeout;
    uint16_t openDutyCycle;     // 0..256
    uint16_t closeDutyCycle;    // 0..256
} mtCtrl_motorConfig_t;

typedef struct
{
    mtSens_thresholds_t compThresholds;
    uint16_t            fullOpenCount;
} mtCtrl_sensorConfig_t;

typedef struct
{
    mtCtrl_motorConfig_t const*  pMotorConfig;
    mtCtrl_sensorConfig_t const* pSensorConfig;
    mtCtrl_eventHandler_t eventHandler;
} mtCtrl_init_t;

/* Exported constants --------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
/** @brief function to initialize the local motor control module
 *
 * @param[in] pInit
 * @return NRF_SUCCESS, NRF_ERROR_NULL
 */
ret_code_t mtCtrl_Init(mtCtrl_init_t const* pInit, bool* pSensorAvailable);

/** @brief function to set the plunger position
 *
 * @param[in] position  the desired position
 * @note currently no analog mode is supported, so only valid values are 0 for
 *       close and 255 for fully open
 */
ret_code_t mtCtrl_SetPosition(q8_t position);

/** @brief function to retrieve the current plunger position
 * @return the current plunger position
 */
q8_t mtCtrl_GetCurrentPosition(void);

/** @brief function to initialize the sensor calibration routine
 *
 * @param[out] pTh  the new threshold values
 * @return tbd.
 */
ret_code_t mtCtrl_CalibrateSensor(mtSens_thresholds_t* pTh);

/** @brief function to initialize the position calibration routine
 *
 * @note the result will be delivered through the event handler
 *
 * @return tbd.
 */
ret_code_t mtCtrl_CalibrateFullOpen(void);

/** @brief function to update the motor configuration
 *
 * @param[in] pMotorConfig  the new motor configuration
 * @return NRF_SUCCESS, NRF_ERROR_NULL, NRF_ERROR_INVALID_PARAM
 */
ret_code_t mtCtrl_UpdateMotorConfig(mtCtrl_motorConfig_t const* pMotorConfig);

/** @brief function to indicate the battery state of charge in morse code
 *
 * @note this drives the motor (closing) with audible PWM and indicating the state of charge from 0-9 in morse code
 *
 * @param[in] soc       battery state of charge
 * @param[in] timebase  the lenght of a dot
 * @return tbd.
 */
ret_code_t mtCtrl_MorseSOC(q8_t soc, uint32_t timebase);

#endif // MTCTRL_H_INCLUDED

/**END OF FILE*****************************************************************/

