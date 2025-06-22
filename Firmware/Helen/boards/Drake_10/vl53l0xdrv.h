/**
  ******************************************************************************
  * @file    vl53l0xdrv.h
  * @author  Thomas Reisnecker
  * @brief   board specific driver for the VL53L0X ToF sensor
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef VL53L0XDRV_H_INCLUDED
#define VL53L0XDRV_H_INCLUDED

/* Includes ------------------------------------------------------------------*/
#include "sdk_errors.h"

/* Exported types ------------------------------------------------------------*/
typedef int16_t q3_12_t;
typedef uint16_t q9_7_t;

typedef enum
{
    VL53L0X_PWR_OFF,       // sensor completely off
    VL53L0X_PWR_DATA,      // fully operational, reporting data tbd.
} vl53l0xdrv_powerMode_t;

typedef void (*vl53l0xdrv_eventHandler_t)(uint16_t distanceInmm);

typedef struct
{
    vl53l0xdrv_eventHandler_t handler; // event handler to report events, must not be NULL
    int32_t offsetInMicrom;
} vl53l0xdrv_init_t;

/* Exported constants --------------------------------------------------------*/
#define VL53L0X_ADDR    (0x52 >> 1)

/* Exported macros -----------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
/** @brief function to initialize the acceleration sensor
 *
 * @param[in] pInit
 * @return NRF_SUCCESS, NRF_ERROR_NULL,
 *         NRF_ERROR_NOT_FOUND if device not found
 */
ret_code_t vl53l0xdrv_Init(vl53l0xdrv_init_t const* pInit);

/** @brief function to set the power mode
 *
 * @param[in] powerMode the new power mode
 * @return NRF_SUCCESS or tbd.
 */
ret_code_t vl53l0xdrv_SetPowerMode(vl53l0xdrv_powerMode_t powerMode);

/** @brief execute function for the tof sensor
 *
 * @note call this in your main loop, it checks and reads for new data and
 *       performs temperature calibration if necessary
 *
 * @param[in] temperature  in K
 */
void vl53l0xdrv_Execute(q9_7_t temperature);

ret_code_t vl53l0xdrv_CalibrateOffset(uint16_t offsetInmm, int32_t* pNewOffsetInMikrom);

ret_code_t cl53l0xdrv_ClearOffsetCalibration(void);

#endif // VL53L0XDRV_H_INCLUDED

/**END OF FILE*****************************************************************/

