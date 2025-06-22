/**
  ******************************************************************************
  * @file    Drake_light.h
  * @author  Thomas Reisnecker
  * @brief   Header for Drake light module
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DRAKE_LIGHT_H_INCLUDED
#define DRAKE_LIGHT_H_INCLUDED

/* Includes ------------------------------------------------------------------*/
#include "sdk_errors.h"
#include "stdint.h"
#include "stdbool.h"

/* Exported types ------------------------------------------------------------*/
typedef enum
{
    DRAKE_LIGHT_FRONT,      // lowest priority
    DRAKE_LIGHT_REAR,
    DRAKE_LIGHT_POS,
    DRAKE_LIGHT_BRAKE,
    DRAKE_LIGHT_INDIC_LEFT,
    DRAKE_LIGHT_INDIC_RIGHT,// highest priority
    DRAKE_LIGHT_CNT
} Drake_lightType_t;

typedef uint8_t q8_t;

typedef struct
{
    uint8_t len;     // length of pattern array in bytes
    uint8_t const* pPattern;
} Drake_lightPattern_t;

typedef struct
{
    q8_t intensity;
    bool activeClear;       // if false, lower priority data will be overridden only for intensity > 0, true always
    Drake_lightPattern_t pattern;
} Drake_light_t;

typedef uint16_t q2_14_t;
typedef uint16_t q3_13_t;

/* Exported constants --------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
/** @brief function to initialize the light module
 *
 * @param[in] addSecondInst  true if the second string should be used, too
 * @return NRF_SUCCESS or tbd.
 */
ret_code_t Drake_Light_Init(bool addSecondInst);

/** @brief function to update light
 *
 * @param[in]  pLight    pointer to array (size DRAKE_LIGHT_CNT) of light configurations
 * @param[in]  ledCnt    the number of mounted leds
 * @param[out] pCurrent  the calculated current consumption in A for this configuration, NULL if not needed
 * @param[out] pPower    the calculated output power for this configuration, NULL if not needed
 * @return NRF_SUCCESS or tbd. *
 */
ret_code_t Drake_Light_Update(Drake_light_t const* pLight, uint16_t ledCnt, q2_14_t* pCurrent, q3_13_t* pPower);

ret_code_t Drake_Light_ShowSOC(q8_t soc, uint16_t ledCnt, uint32_t timeout, q2_14_t* pCurrent, q3_13_t* pPower);

#endif // DRAKE_LIGHT_H_INCLUDED

/**END OF FILE*****************************************************************/

