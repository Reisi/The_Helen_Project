/**
  ******************************************************************************
  * @file    regulator.c
  * @author  Thomas Reisnecker
  * @brief   tbd.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stddef.h>

#include "regulator.h"

/* External variables --------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/

/* Private defines -----------------------------------------------------------*/

/* Private read only variables -----------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/* Public functions ----------------------------------------------------------*/
void reg_PiInit(reg_piInstance_t* pInst, reg_piInit_t const* pInit)
{
    if (pInst == NULL || pInit == NULL)
        return;

    pInst->gainProp = pInit->gainProp;
    pInst->gainInt = pInit->gainInt;

    pInst->intUpperLimit = pInit->intUpperLimit;
    pInst->intLowerLimit = pInit->intLowerLimit;

    pInst->integrator = 0;
}
//#include "SEGGER_RTT.h"
int32_t reg_PiRegulate(reg_piInstance_t* pInst, int32_t desired, int32_t actual)
{
    int32_t deviation, p, i, output;
    int64_t y;

    deviation = desired - actual;
    y = (int64_t)pInst->integrator + deviation;

    if (y > pInst->intUpperLimit)
        pInst->integrator = pInst->intUpperLimit;
    else if (y < pInst->intLowerLimit)
        pInst->integrator = pInst->intLowerLimit;
    else
        pInst->integrator = y;

    p = ((int64_t)deviation * pInst->gainProp) >> 31;
    i = ((int64_t)pInst->integrator * pInst->gainInt) >> 31;
    y = (int64_t)p + i;

    if (y > INT32_MAX)
        output = INT32_MAX;
    else if (y < INT32_MIN)
        output = INT32_MIN;
    else
        output = y;

    //SEGGER_RTT_printf(0, "%d, %d, %d, %d\r\n", deviation, p, i, output);

    return output;
}

void reg_PiReset(reg_piInstance_t* pInst)
{
    pInst->integrator = 0;
}

/**END OF FILE*****************************************************************/
