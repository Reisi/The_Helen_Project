/**
  ******************************************************************************
  * @file    Drake_hmi.h
  * @author  Thomas Reisnecker
  * @brief   Header for Drake hmi module
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DRAKE_HMI_H_INCLUDED
#define DRAKE_HMI_H_INCLUDED

/* Includes ------------------------------------------------------------------*/
#include "sdk_errors.h"

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
#define BUTTON_NOT_USED 0xFFFFFFFF

/* Exported macros -----------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
/** @brief function to initialize the hmi module
 *
 * @param[in] mainButton       pin number of onboard button
 * @param[in] secondaryButton  pin number of secondary button (BUTTON_NOT_USED if not used)
 * @param[in] timeout          time in APP_TIMER_TICKS until status leds are
 *                             switched off for reduced current consumption,
 *                             reloaded on butten events. Use 0 for no switching
 *                             off.
 * @return NRF_SUCCESS or a propagated error from but_DebounceInit
 */
ret_code_t Drake_Hmi_Init(uint32_t mainButton, uint32_t secondaryButton, uint32_t ledsTimeout);

/** @brief The execute function of the hmi module
 */
void Drake_Hmi_Execute(bool resetLedTimeout);

#endif // DRAKE_HMI_H_INCLUDED

/**END OF FILE*****************************************************************/

