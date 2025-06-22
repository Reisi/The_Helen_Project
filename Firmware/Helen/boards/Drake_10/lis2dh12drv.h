/**
  ******************************************************************************
  * @file    lis2dh12drv.h
  * @author  Thomas Reisnecker
  * @brief   board specific driver for the LIS2DH12 acceleration sensor
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef LIS2DH12DRV_H_INCLUDED
#define LIS2DH12DRV_H_INCLUDED

/* Includes ------------------------------------------------------------------*/
#include "sdk_errors.h"

/* Exported types ------------------------------------------------------------*/
typedef int16_t q3_12_t;

typedef enum
{
    LIS2DH12_PWR_OFF,       // sensor completely off
    LIS2DH12_PWR_MOTION,    // just detecting motion
    LIS2DH12_PWR_DATA,      // fully operational, reporting data with 100Hz
} lis2dh12drv_powerMode_t;

typedef enum
{
    LIS2DH12_EVT_MOTION,    // motion detected
    LIS2DH12_EVT_DATA,      // new data available
} lis2dh12drv_EventType_t;

typedef struct
{
    q3_12_t accel[3];       //raw acceleration
} lis2dh12drv_data_t;

typedef struct
{
    lis2dh12drv_EventType_t type;
    lis2dh12drv_data_t      data;   // only filled for LIS2DH12_EVT_DATA
} lis2dh12drv_event_t;

typedef void (*lis2dh12drv_eventHandler_t)(lis2dh12drv_event_t const* pEvt);

typedef struct
{
    uint8_t addr;                       // device address to use, either 0x30 or 0x31
    uint32_t int1Pin;                   // GPIO pin number int1 is connected to
    lis2dh12drv_eventHandler_t handler; // event handler to report events, must not be NULL
} lis2dh12drv_init_t;

/* Exported constants --------------------------------------------------------*/
#define LIS2DH12_ADDR0   (0x30 >> 1)
#define LIS2DH12_ADDR1   (0x32 >> 1)

/* Exported macros -----------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
/** @brief function to initialize the acceleration sensor
 *
 * @param[in] pInit
 * @return NRF_SUCCESS, NRF_ERROR_NULL,
 *         NRF_ERROR_INVALID_ADDR for invalid device address,
 *         NRF_ERROR_INVALID_PARAM for invalid interrupt pin number
 */
ret_code_t lis2dh12drv_Init(lis2dh12drv_init_t const* pInit);

/** @brief function to set the power mode
 *
 * @param[in] powerMode the new power mode
 * @return NRF_SUCCESS or tbd.
 */
ret_code_t lis2dh12drv_SetPowerMode(lis2dh12drv_powerMode_t powerMode);

void lis2dh12drv_Execute();

#endif // LIS2DH12DRV_H_INCLUDED

/**END OF FILE*****************************************************************/

