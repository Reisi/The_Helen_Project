/**
  ******************************************************************************
  * @file    board.h
  * @author  Thomas Reisnecker
  * @brief   Header for board module
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BOARD_H_INCLUDED
#define BOARD_H_INCLUDED

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>

/* Exported types ------------------------------------------------------------*/
typedef enum
{
    BRD_LT_RED = 0,
    BRD_LT_GREEN,
    BRD_LT_BLUE,
    BRD_LT_CNT
} brd_ledType_t;

/* Exported constants --------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
/** @brief function to en-, or disable the status leds
 *
 * @param[in] color  the led type
 * @param     enable
 */
void brd_EnableLed(brd_ledType_t color, bool enable);

#endif // BOARD_H_INCLUDED

/**END OF FILE*****************************************************************/
