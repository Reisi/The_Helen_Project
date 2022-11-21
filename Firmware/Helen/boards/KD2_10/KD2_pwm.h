/**
  ******************************************************************************
  * @file    KD2_pwm.h
  * @author  Thomas Reisnecker
  * @brief   PWM output module
  *          This module uses a PWM peripheral of the NRF52 to generate up to 4
  *          PWM outputs with a frequency of 312.5Hz.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef KD2_PWM_H_INCLUDED
#define KD2_PWM_H_INCLUDED

/* Includes ------------------------------------------------------------------*/
#include "sdk_errors.h"

#include "KD2_10.h"

/* Exported constants --------------------------------------------------------*/
#define KD2_PWM_PIN_NOT_USED    0xFFFFFFFF
#define KD2_PWM_CHANNEL_CNT     4

#define KD2_PWM_INIT_ALL_DISABLED       \
{                                       \
    .channels =                         \
    {                                   \
        {.pinNo = KD2_PWM_PIN_NOT_USED},\
        {.pinNo = KD2_PWM_PIN_NOT_USED},\
        {.pinNo = KD2_PWM_PIN_NOT_USED},\
        {.pinNo = KD2_PWM_PIN_NOT_USED},\
    }                                   \
}

/* Exported types ------------------------------------------------------------*/
typedef enum
{
    KD2_PWM_PUSHPULL,
    KD2_PWM_PUSHPULLINVERTED,
    KD2_PWM_OPENDRAIN,
    KD2_PWM_OPENDRAININVERTED,
} KD2_Pwm_outType_t;

typedef struct
{
    uint32_t          pinNo;    // set to KD2_PWM_PIN_NOT_USED if not used
    KD2_Pwm_outType_t type;
} KD2_Pwm_pinDef_t;

typedef struct
{
    KD2_Pwm_pinDef_t channels[KD2_PWM_CHANNEL_CNT];
} KD2_Pwm_init_t;

typedef struct
{
   KD2_target_t channels[KD2_PWM_CHANNEL_CNT];
} KD2_Pwm_dc_t;

/* Exported macros -----------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
/** @brief function to initialize the pwm module
 *
 * @param[in] pInit  the pins and types of each channel
 * @return NRF_SUCCESS, NRF_ERROR_NULL or the error code of nrfx_pwm_init()
 */
ret_code_t KD2_Pwm_Init(KD2_Pwm_init_t const* pInit);

/** @brief function to set the power mode of the pwm module
 *
 * @param[in] newMode  the new power mode
 * @return NRF_SUCCESS
 */
ret_code_t KD2_Pwm_SetPowerMode(KD2_powerMode_t newMode);

/** @brief function to set the duty cycles
 *
 * @param[in] pDC  the duty cycles for each channel, unused channels will be ignored
 * @return NRF_SUCCESS, NRF_ERROR_NULL
 *         NRF_ERROR_INVALID_STATE if not in KD2_PWR_ON mode
 *         NRF_ERROR_INVALID_PARAMS if dutycycle are out of valid range
 */
ret_code_t KD2_Pwm_SetDC(KD2_Pwm_dc_t const* pDC);

#endif // KD2_PWM_H_INCLUDED

/**END OF FILE*****************************************************************/

