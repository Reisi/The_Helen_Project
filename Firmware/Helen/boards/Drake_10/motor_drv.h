/**
  ******************************************************************************
  * @file    motor_drv.h
  * @author  Thomas Reisnecker
  * @brief   motor driver Helen Drake Hardware.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MOTOR_DRV_H_INCLUDED
#define MOTOR_DRV_H_INCLUDED

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

#include "sdk_errors.h"

/* Exported types ------------------------------------------------------------*/
typedef enum
{
    MTDRV_OFF,
    MTDRV_REVERSE,
    MTDRV_FORWARD,
    MTDRV_BRAKE
} mtdrv_mode_t;

/* Exported constants --------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
/** @brief function to initialize the motor driver
 *
 * @return NRF_SUCCESS or tbd.
 */
ret_code_t mtdrv_Init(void);

/** @brief function to enable the motor
 *
 * @param[in] mode       motor mode
 * @param[in] dutyCycle  intensity (= PMW duty cycle)
 */
void mtdrv_Enable(mtdrv_mode_t mode, uint16_t dutyCycle);

#endif // MOTOR_DRV_H_INCLUDED

/**END OF FILE*****************************************************************/

