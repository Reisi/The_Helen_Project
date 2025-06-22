/**
  ******************************************************************************
  * @file    KD2_btle.h
  * @author  Thomas Reisnecker
  * @brief   tbd.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef KD2_BTLE_H_INCLUDED
#define KD2_BTLE_H_INCLUDED

/* Includes ------------------------------------------------------------------*/
#include "sdk_errors.h"

/* Exported types ------------------------------------------------------------*/
typedef uint8_t q8_t;

/* Exported constants --------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
/** @brief Function to initialize the KD2 bluetooth module. This implements the
 *         KD2 Control Service and if com is used also the battery service
 *
 * @param[in] pFeatures
 * @param[in] isImuPresent
 * @return NRF_SUCCESS, NRF_ERROR_NULL or NRF_ERROR_INVALID_PARAM *
 */
ret_code_t KD2_btle_Init(brd_features_t const* pFeatures, bool isImuPresent);

/** @brief function to uptade the battery level
 *
 * @param[in] batterySoc  battery state of charge
 * @return NRF_SUCCESS, NRF_ERROR_NOT_SUPPORTED if com isn't used or
 *         propagated error from ble_bas_battery_level_update()
 */
ret_code_t KD2_btle_BatteryLevelUpdate(q8_t batterySoc);

#endif // KD2_BTLE_H_INCLUDED

/**END OF FILE*****************************************************************/
