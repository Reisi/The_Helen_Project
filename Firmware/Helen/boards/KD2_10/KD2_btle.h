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

/* Exported constants --------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
ret_code_t KD2_btle_Init(brd_features_t const* pFeatures, bool isImuPresent);

#endif // KD2_BTLE_H_INCLUDED

/**END OF FILE*****************************************************************/
