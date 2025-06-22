/**
  ******************************************************************************
  * @file    brake2d.h
  * @author  Thomas Reisnecker
  * @brief   2D brake detection algo module.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BRAKE2D_H_INCLUDED
#define BRAKE2D_H_INCLUDED

/* Includes ------------------------------------------------------------------*/
#include "sdk_errors.h"
#include "stdbool.h"
#include "stdint.h"

/* Exported types ------------------------------------------------------------*/
typedef int16_t q3_12_t;
typedef int16_t q15_t;

/* Exported constants --------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
ret_code_t brake2d_Init(void);

ret_code_t brake2d_Feed(q3_12_t const* pAccel, q3_12_t* pDecelleration, q15_t* pInlcination);


#endif // BRAKE2D_H_INCLUDED

/**END OF FILE*****************************************************************/

