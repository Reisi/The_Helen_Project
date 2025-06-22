/**
  ******************************************************************************
  * @file    Drake_adc.h
  * @author  Thomas Reisnecker
  * @brief   Adc module for Helen Drake Hardware.
  *          This module samples input voltage, temperature, charge current and
  *          the voltage on both USB CC pins with a sample rate of 1Hz
  *          /// description haw charge pin is handled!!!
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DRAKE_ADC_H_INCLUDED
#define DRAKE_ADC_H_INCLUDED

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

#include "sdk_errors.h"

/* Exported types ------------------------------------------------------------*/
typedef uint16_t q16_t;     // used for current, range 0 .. 0.999985A, resolution 15.26µA
typedef uint16_t q9_7_t;    // used for temperature, range 0 .. 511.996K, resolution 3.9mK
typedef uint16_t q3_13_t;   // used for voltage, range 0 .. 7.999878V, resolution 122µV

typedef struct
{
    q3_13_t inputVoltage;
    q3_13_t cc1Voltage;
    q3_13_t cc2Voltage;
    q9_7_t  temperature;
    q16_t   chargeCurrent;
    /// TODO: USB D+ & D- for BC1.2?
} Drake_adc_result_t;

/**< The adc result handler. Reported in  APP_IRQ_PRIORITY_LOW context
 */
typedef void (*Drake_adc_resultHandler_t)(Drake_adc_result_t const* pResult);

typedef struct
{
    Drake_adc_resultHandler_t resultHandler;
} Drake_adc_init_t;

typedef enum
{
    DRAKE_ADC_OFF,
    DRAKE_ADC_1Hz
} Drake_adc_mode_t;

/* Exported constants --------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
/** @brief function to initialize the adc module
 *
 * @param[in]  pInit
 * @param[out] pInitResult  the initial values, may be NULL
 * @return NRF_SUCCESS, NRF_ERROR_NULL
 */
ret_code_t Drake_Adc_Init(Drake_adc_init_t const* pInit, Drake_adc_result_t* pInitResult);

/** @brief function to set the mode
 *
 * @param[in] newMode
 * @return NRF_SUCCESS
 */
ret_code_t Drake_Adc_SetMode(Drake_adc_mode_t newMode);

/** @brief The execute function of the adc module
 */
void Drake_Adc_Execute(void);

#endif // DRAKE_ADC_H_INCLUDED

/**END OF FILE*****************************************************************/

