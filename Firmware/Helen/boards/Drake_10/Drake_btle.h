/**
  ******************************************************************************
  * @file    Drake_btle.h
  * @author  Thomas Reisnecker
  * @brief   tbd.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DRAKE_BTLE_H_INCLUDED
#define DRAKE_BTLE_H_INCLUDED

/* Includes ------------------------------------------------------------------*/
#include "sdk_errors.h"

/* Exported types ------------------------------------------------------------*/
typedef uint8_t q8_t;

/* Exported constants --------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
/** @brief function to initialize the drake bleutooth module. This initializes
 *         the drake control service and the battery service
 *
 * @param[in] pFeatures
 * @param[in] batterySoc               initial state of charge of battery
 * @param[in] isPlungerControlPresent
 * @param[in] isPlungerSensorPresent
 * @return NRF_SUCCESS, NRF_ERROR_NULL
 */
ret_code_t Drake_btle_Init(brd_features_t const* pFeatures, q8_t batterySoc, bool isActuatorPresent, bool isMotorSensorPresent, bool isTravelSensorPresent);

/** @brief function to update the battery level
 *
 * @param[in] batterySoc  state of charge of battery
 * @return propagated error code of ble_bas_battery_level_update() *
 */
ret_code_t Drake_btle_BatteryLevelUpdate(q8_t batterySoc);

#endif // KD2_BTLE_H_INCLUDED

/**END OF FILE*****************************************************************/
