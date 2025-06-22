/**
  ******************************************************************************
  * @file    charge_control.h
  * @author  Thomas Reisnecker
  * @brief   charge control module for Helen Drake Hardware.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CHARGE_CONTROL_H_INCLUDED
#define CHARGE_CONTROL_H_INCLUDED

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

#include "sdk_errors.h"

/* Exported types ------------------------------------------------------------*/
typedef uint16_t q1_15_t;
typedef uint16_t q16_t;
typedef uint16_t q3_13_t;

typedef enum
{
    CHCTRL_CURRENT_200MA,
    CHCTRL_CURRENT_500MA
} chctrl_current_t;

typedef enum
{
    CHCTRL_STATE_DISCONNECTED,
    CHCTRL_STATE_NOT_CHARGING,
    CHCTRL_STATE_CHARGING,
    CHCTRL_STATE_FULL
} chctrl_state_t;

/*typedef enum
{
    CHCTRL_EVT_STATE_CHANGE
} chctrl_evtType_t;

typedef struct
{
    chctrl_evtType_t type;
    union
    {
        chctrl_state_t newState;
    };
} chctrl_evt_t;

typedef void (*chctrl_eventHandler_t)(chctrl_evt_t const* pEvt);

typedef struct
{
    chctrl_current_t      maxChargeCurrent;
    chctrl_eventHandler_t eventHandler;
} chctrl_init_t;*/

/* Exported constants --------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
/** @brief function to initialize the charge control module
 *
 * @param[in]  pInit
 * @return NRF_SUCCESS, NRF_ERROR_NULL
 */
//ret_code_t chctrl_Init(chctrl_init_t const* pInit);
ret_code_t chctrl_Init(chctrl_current_t maxChargeCurrent);

/** @brief function to enable the charge module
 *
 * @param[in] enable  true to enable, false to disable (default charging with 200mA, without checking)
 */
void chctrl_Enable(bool enable);

/** @brief execute function of the charge module
 *
 * @param[in] cc1Voltage     voltage on cc1 pin
 * @param[in] cc2Voltage     voltage on cc2 pin
 * @param[in] chargeCurrent  actual charge current
 * @param[in] batteryVoltage actual battery voltage
 */
//void chctrl_Execute(q1_15_t cc1Voltage, q1_15_t cc2Voltage, q16_t chargeCurrent, q3_13_t batteryVoltage);
chctrl_state_t chctrl_Feed(q1_15_t cc1Voltage, q1_15_t cc2Voltage, q16_t chargeCurrent, q3_13_t batteryVoltage);

/** @brief function to release the charge control pin for measuring the charging current
 *
 * @note The state of the charge control pin determines the charge current. An
 *       active high disabled charging, active low enables charging with 500mA
 *       and a passive pin enables charging with 200mA. Measurements are only
 *       possible in this mode. Therefore callings this function with enable
 *       = true disabled fast charging and the measurable charging current is
 *       therefore limited to 200mA.
 *
 * @param{in] enable  true to enable measurement mode, false to disable
 * @return NRF_SUCCESS or NRF_ERROR_INVALID_STATE if not charging
 */
ret_code_t chctrl_MeasurementMode(bool enable);

#endif // CHARGE_CONTROL_H_INCLUDED

/**END OF FILE*****************************************************************/

