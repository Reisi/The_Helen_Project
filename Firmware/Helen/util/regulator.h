/**
  ******************************************************************************
  * @file    regulator.h
  * @author  Thomas Reisnecker
  * @brief   tbd.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef REGULATOR_H_INCLUDED
#define REGULATOR_H_INCLUDED

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/
typedef int32_t q31_t;

typedef struct
{
    q31_t gainProp;         // Kp
    q31_t gainInt;          // Ki
    int32_t integrator;     // the integrator sum
    int32_t intUpperLimit;  // upper limit for the integrator
    int32_t intLowerLimit;  // lower limit for the integrator
} reg_piInstance_t;

typedef struct
{
    q31_t gainProp;
    q31_t gainInt;
    int32_t intUpperLimit;
    int32_t intLowerLimit;
} reg_piInit_t;

/* Exported constants --------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
/** @brief initialization function for the PI regulator
 *
 */
void reg_PiInit(reg_piInstance_t* pInst, reg_piInit_t const* pInit);

/** @brief regulate function
 *
 * @param[in] desired target value
 * @param[in] actual  actual value
 * @return the regulated output
 */
int32_t reg_PiRegulate(reg_piInstance_t* pInst, int32_t desired, int32_t actual);

/** @brief function to reset the integrator inside the regulator structure
 */
void reg_PiReset(reg_piInstance_t* pInst);

#endif // REGULATOR_H_INCLUDED

/**END OF FILE*****************************************************************/

