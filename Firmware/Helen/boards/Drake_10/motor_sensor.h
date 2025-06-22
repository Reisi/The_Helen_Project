/**
  ******************************************************************************
  * @file    motor_sensor.h
  * @author  Thomas Reisnecker
  * @brief   motor sensor sensor for Helen Drake Hardware.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MOT_SENS_H_INCLUDED
#define MOT_SENS_H_INCLUDED

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

#include "sdk_errors.h"

/* Exported types ------------------------------------------------------------*/
typedef uint8_t q8_t;

typedef enum
{
    //MTSENS_EVTTYPE_CALIB_DATA,
    MTSENS_EVTTYPE_POS_CHANGE
} mtSens_evtType_t;

typedef struct
{
    mtSens_evtType_t type;
    union
    {
        int16_t position;
        //nrf_comp_th_t thresholds;
    };
} mtSens_evt_t;

typedef void (*mtSens_eventHandler_t)(mtSens_evt_t const* pEvt);

typedef struct
{
    q8_t upper;
    q8_t lower;
} mtSens_thresholds_t;

typedef struct
{
    mtSens_thresholds_t const* pThresholds;   // the upper and lower thresholds for the reflex coupler
    mtSens_eventHandler_t evtHandler;         // the event handler (must not be NULL)
} mtSens_Init_t;

typedef enum
{
    MTSENS_DIR_FORWARD,                  // counter is counting up
    MTSENS_DIR_REVERSE                   // counter is counting down
} mtSens_direction_t;

/* Exported constants --------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
/** @brief function to initialize the motor sensor
 *
 * @param[in] pInit
 * @return NRF_SUCCESS, NRF_ERROR_NULL, NRF_ERROR_NOT_FOUND
 */
ret_code_t mtSens_Init(mtSens_Init_t const* pInit);

/** @brief function to en-/disable the sensor interface
 *
 * @param[in] enable     true to enable, false to disable
 * @param[in] direction  motor direction (will be ignored if enable is false)
 * @return NRF_SUCCESS, NRF_ERROR_NULL, NRF_ERROR_NOT_FOUND
 */
ret_code_t mtSens_Enable(bool enable);

/** @brief function to set the counter direction
 *
 * @param[in] direction  motor direction
 * @return NRF_SUCCESS, NRF_ERROR_NULL, NRF_ERROR_NOT_FOUND
 */
 ret_code_t mtSens_SetDirection(mtSens_direction_t direction);

/** @brief function to reset the sensor counter value
 */
void mtSens_ResetCounter(void);

/** @brief function to calibrate the comparator threshold for the optical sensor
 *
 * @param[out] pTh  the new thresholds
 * @return NRF_SUCCESS, NRF_ERROR_NULL, NRF_ERROR_NOT_FOUND
 */
ret_code_t mtSens_Calibrate(mtSens_thresholds_t* pTh);

#endif // MOT_SENS_H_INCLUDED

/**END OF FILE*****************************************************************/

