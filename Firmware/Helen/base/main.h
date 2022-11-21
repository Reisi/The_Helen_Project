/**
  ******************************************************************************
  * @file    main.h
  * @author  Thomas Reisnecker
  * @brief   Header for main module
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MAIN_H_INCLUDED
#define MAIN_H_INCLUDED

/* Includes ------------------------------------------------------------------*/
#include "sdk_errors.h"

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
/** @brief
 *
 * @param void
 * @return ret_code_t
 *
 */
ret_code_t main_ResetIdleTimer(void);

#endif // MAIN_H_INCLUDED

/**END OF FILE*****************************************************************/
