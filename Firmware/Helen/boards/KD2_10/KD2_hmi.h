/**
  ******************************************************************************
  * @file    KD2_hmi.h
  * @author  Thomas Reisnecker
  * @brief   Header for KD2 hmi module
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef KD2_HMI_H_INCLUDED
#define KD2_HMI_H_INCLUDED

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
 * @return NRF_SUCCESS or a propagated error from but_DebounceInit
 */
ret_code_t KD2_Hmi_Init(uint32_t mainButton, uint32_t secondaryButton);

/** @brief The execute function of the hmi module
 */
void KD2_Hmi_Execute(void);

#endif // KD2_HMI_H_INCLUDED

/**END OF FILE*****************************************************************/

