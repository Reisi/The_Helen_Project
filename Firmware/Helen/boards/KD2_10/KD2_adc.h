/**
  ******************************************************************************
  * @file    KD2_adc.h
  * @author  Thomas Reisnecker
  * @brief   Adc module for Helen KD2 Hardware. In idle mode temperature and
  *          voltage is sampled with a sample rate of 1Hz, in on mode current
  *          and voltage are sampled with 7692Hz and temperature with 8Hz. The
  *          results are reported through the event handler, in on mode this
  *          handler uses APP_IRQ_PRIORITY_HIGH.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef KD2_ADC_H_INCLUDED
#define KD2_ADC_H_INCLUDED

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

#include "sdk_errors.h"

#include "KD2_10.h"

/* Exported types ------------------------------------------------------------*/
typedef uint16_t q3_13_t;   // used for current, range 0 .. 7.999A, resolution 122µA
typedef uint16_t q9_7_t;    // used for temperature, range 0 .. 511.996K, resolution 3.9mK
typedef uint16_t q5_11_t;   // used for voltage, range 0 .. 31.999V, resolution 488µV
typedef uint16_t q4_12_t;   // used for gain factors

typedef struct
{
    q5_11_t inputVoltage;
    q9_7_t  temperature;    // 0 if no value is available
    q3_13_t ledCurrent;     // 0 in idle mode
} KD2_adc_result_t;

/**< The adc result handler. Uses APP_IRQ_PRIORITY_LOW in idle mode and
 *   APP_IRQ_PRIORITY_HIGH in on mode
 */
typedef void (*KD2_adc_resultHandler_t)(KD2_adc_result_t const* pResult);

typedef struct
{
    KD2_adc_resultHandler_t resultHandler;
} KD2_adc_init_t;

/* Exported constants --------------------------------------------------------*/
// the default voltage gain factor to convert the raw adc output to q5_11_t
// the gain factor itself is represented in q4_12_t
//    0.6V   * (1Meg+47k)/47k  *  2^11   /   2^12  *  2^12
// reference   voltage divider   q5_11_t   adc_res   q4_12_t
#define KD2_ADC_VOLTGAIN_DEFAULT    27373

// the default current gain factor to convert the raw adc output to q3_13_t
// the gain factor itself is represented in q4_12_t
//    0.6V   / (820 * 0.01 * 0.021) * 2^13   /   2^12  *  2^12
// reference    R6    gain    R_L    q3_13_t   adc_res   q4_12_t
#define KD2_ADC_CURRGAIN_DEFAULT    28544

// the default temperature offset to convert the measurement to q9_7_t
// the offset itself is represented in q9_7_t
// the default offset converts the measurement from °C to °K
#define KD2_ADC_TEMPOFF_DEFAULT     (273 << 7)

/* Exported macros -----------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
/** @brief function to initialize the adc module
 *
 * @param[in]  pInit
 * @param[out] pInitResult  the initial values of temperature and voltage
 *                          (current is not measured and therefore 0), may be NULL
 * @return NRF_SUCCESS, NRF_ERROR_NULL
 */
ret_code_t KD2_Adc_Init(KD2_adc_init_t const* pInit, KD2_adc_result_t* pInitResult);

/** @brief function to set the power mode
 *
 * @param[in] newMode
 * @return NRF_SUCCESS
 */
ret_code_t KD2_Adc_SetPowerMode(KD2_powerMode_t newMode);

/** @brief The execute function of the hmi module
 */
void KD2_Adc_Execute(void);

/** @brief function to set a current limit
 *
 * @param[in] limit         the current limit
 * @param[in] taskToTrigger the task address to trigger when current exceeds limit
 * @return    the return value of nrfx_ppi_channel_assign()
 *
 * @note      this function has no effect at the moment, due to an unresolved error
 */
ret_code_t KD2_Adc_SetCurrentLimit(q3_13_t limit, uint32_t taskToTrigger);

/** @brief function to set the compensation values
 *
 * @param[in] gainCurr
 * @param[in] offTemp
 * @return    NRF_SUCCESS or NRF_ERROR_INVALID_PARAM if the currGain is below minimum value
 */
ret_code_t KD2_Adc_SetCompensation(q4_12_t gainCurr, q9_7_t offTemp);

#endif // KD2_ADC_H_INCLUDED

/**END OF FILE*****************************************************************/

