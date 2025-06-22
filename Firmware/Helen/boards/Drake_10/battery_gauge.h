/**
  ******************************************************************************
  * @file    battery_gauge.h
  * @author  Thomas Reisnecker
  * @brief   battery gauge algorithm for li-ion/li-po batteries
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BAT_GAUGE_H_INCLUDED
#define BAT_GAUGE_H_INCLUDED

/* Includes ------------------------------------------------------------------*/
#include "sdk_errors.h"

/* Exported types ------------------------------------------------------------*/
typedef uint16_t q3_13_t;
typedef int32_t  q15_16_t;
typedef uint8_t  q8_t;

typedef enum
{
    BG_EVT_TYPE_CAP_RECALC = 0, // actual capacity has been recalculated
} bg_evtType_t;

typedef struct
{
    bg_evtType_t type;
    union
    {
        q15_16_t capActual;
    };
} bg_evt_t;

typedef void (*bg_eventHandler_t)(bg_evt_t const* pEvt);

typedef struct
{
    q3_13_t const*    pOCV;
    q15_16_t          capNominal;
    q15_16_t          capActual;
    q15_16_t          colCnt;
    q3_13_t           lastOCV;
    uint16_t          loadTimeout;
    bg_eventHandler_t handler;
} bg_inst_t;

typedef struct
{
    q3_13_t const*    pOCV;
    q15_16_t          capNominal;
    q15_16_t          capActual;
    bg_eventHandler_t handler;
} bg_init_t;

/* Exported constants --------------------------------------------------------*/
extern q3_13_t const bg_ocvLco[];   // open circuit voltage LUT for LCO Li-Ion batteries
extern q3_13_t const bg_ocvNmc[];   // open circuit voltage LUT for NMC Li-ion batteries

/* Exported macros -----------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
ret_code_t bg_Init(bg_inst_t* pInst, bg_init_t const* pInit);

/** @brief function to feed the battery gauge
 *
 * @param[in/out] pInst    the battery gauge instance
 * @param[in]     current  battery current (positive: charging, negative: discharging)
 * @param[in]     voltage  battery voltage
 * @return state of charge
 */
q8_t bg_Feed(bg_inst_t* pInst, q15_16_t current, q3_13_t voltage);

//q3_13_t const* getOCVTable(void);

#endif // BAT_GAUGE_H_INCLUDED

/**END OF FILE*****************************************************************/

