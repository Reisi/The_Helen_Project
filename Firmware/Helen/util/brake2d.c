/**
  ******************************************************************************
  * @file    brake2d.c
  * @author  Thomas Reisnecker
  * @brief   2D braking algorithm.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "brake2d.h"
#include "filter.h"
#include "arm_math.h"
#include "math.h"
#include "stdlib.h"

/* External variables --------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/

/* Private defines -----------------------------------------------------------*/
#ifndef M_PIf
#define M_PIf (3.14159265358979323846f)
#endif

#ifndef M_2_PIf
#define M_2_PIf     (2.0f * M_PIf)
#endif // M_2_PI

#define ACCX    0
#define ACCZ    1

#define DECEL_THRESHOLD     1229    // ~0.3g

/* Private read only variables -----------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
FIL_LOWPASS_DEF(accX, 200);
FIL_LOWPASS_DEF(accZ, 200);

/* Private functions ---------------------------------------------------------*/
static q31_t getInclination(int32_t const* pAccel)
{
    float inc = atan2f(pAccel[ACCZ], pAccel[ACCX]);
    return inc * (float)INT32_MAX / M_2_PIf;
}

static void rotate(q31_t angle, int32_t* pAccel)
{
    int32_t rot[4];
    int32_t in[2] = {pAccel[ACCX], pAccel[ACCZ]};
    arm_matrix_instance_q31 matIn =  {.numRows = 2, .numCols = 1, .pData = in};
    arm_matrix_instance_q31 matRot =  {.numRows = 2, .numCols = 2, .pData = rot};
    arm_matrix_instance_q31 matOut =  {.numRows = 2, .numCols = 1, .pData = pAccel};

    // generate rotation matrix
    arm_sin_cos_q31(angle, &rot[2], &rot[3]);
    rot[0] = rot[3];
    rot[1] = -rot[2];

    // rotate
    arm_mat_mult_q31(&matRot, &matIn, &matOut);
}

static q3_12_t isBreaking(int32_t const* pAccel)
{
    if (pAccel[ACCX] < DECEL_THRESHOLD)
        return 0;

    if (abs(pAccel[ACCZ]) > (pAccel[ACCX] >> 2))
        return 0;

    /// TODO: check algo
    return 0;//pAccel[ACCX];
}

/* Public functions ----------------------------------------------------------*/
ret_code_t brake2d_Init(void)
{
    /// TODO: anything to do?

    return NRF_SUCCESS;
}

ret_code_t brake2d_Feed(q3_12_t const* pAccel, q3_12_t* pDecelleration, q15_t* pInlcination)
{
    if (pAccel == NULL || pDecelleration == NULL || pInlcination == NULL)
        return NRF_ERROR_NULL;

    // feed filter
    fil_LowPassFeed(&accX, pAccel[ACCX]);
    fil_LowPassFeed(&accZ, pAccel[ACCZ]);

    // generate low and high pass outputs
    int32_t lowPass[2], highPass[2];
    lowPass[ACCX] = fil_LowPassGet(&accX);
    lowPass[ACCZ] = fil_LowPassGet(&accZ);
    highPass[ACCX] = pAccel[ACCX] - lowPass[ACCX];
    highPass[ACCZ] = pAccel[ACCZ] - lowPass[ACCZ];

    // calculate inclination
    q31_t inclination = getInclination(lowPass);
    *pInlcination = inclination >> 16;

    // rotate high pass acceleration values depending on inclination
    rotate(-inclination, highPass);

    // check if braking condition is met
    *pDecelleration = isBreaking(highPass);

    return NRF_SUCCESS;
}

/**END OF FILE*****************************************************************/
