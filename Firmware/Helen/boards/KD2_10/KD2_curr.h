/**
  ******************************************************************************
  * @file    KD2_curr.h
  * @author  Thomas Reisnecker
  * @brief   current regulator module.
  *          This module uses a PWM module to drive the switching regulator fet
  *          with a frequency of 250kHz. At this frequency the PWM resolution is
  *          limited to 6bit, which is enhanced to a 8bit resolution with
  *          dithering. The internal regulator is expecting the actual values of
  *          the current measurement at a frequency of 7692 Hz (=250kHz / 32.5)
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef KD2_CURR_H_INCLUDED
#define KD2_CURR_H_INCLUDED

/* Includes ------------------------------------------------------------------*/
#include "sdk_errors.h"

#include "KD2_10.h"
#include "KD2_adc.h"
#include "channel_types.h"

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
/** @brief function to initialize the current regulator module
 *
 * @return NRF_SUCCESS or the return value of nrfx_pwm_init()
 */
ret_code_t KD2_Curr_Init(void);

/** @brief The execute function
 */
//void KD2_Curr_Execute(void);
#define KD2_Curr_Execute(...)   // no execute function necessary

/** @brief function to set the power mode
 *
 * @param[in] newMode  the new power mode
 * @return    NRF_SUCCESS
 */
ret_code_t KD2_Curr_SetPowerMode(KD2_powerMode_t newMode);

/** @brief function to set the desired output current
 *
 * @param[in] desired  the desired output current (relatively to full output rating)
 * @return NRF_SUCCESS
 *         NRF_ERROR_INVALID_STATE if in wrong power mode
 *         NRF_ERROR_INVALID_PARAM if desired value not withing permissible range
 */
ret_code_t KD2_Curr_SetCurrent(KD2_target_t desired);

/** @brief function to report the actual current
 *
 * @param[in] actual  actual output current (relatively to full output rating)
 *
 * @note the actual value is fed into the regulator which then calculates the
 *       new PWM setting. It is recommended to call this function at a high
 *       interrupt level to ensure proper regulation
 */
void KD2_Curr_ReportCurrent(KD2_target_t actual);

/** @brief Function to get the abort task of the PWM generator, this can be
 *         used to link the PWM generator with the adc high limit to abort at
 *         unforeseeable errors
 */
uint32_t KD2_Curr_GetAbortTask(void);

#endif // KD2_CURR_H_INCLUDED

/**END OF FILE*****************************************************************/

