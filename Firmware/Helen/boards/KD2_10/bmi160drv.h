/**
  ******************************************************************************
  * @file    bmi160drv.h
  * @author  Thomas Reisnecker
  * @brief   board specific driver for the Bosch BMI160 IMU
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BMI160DRV_H_INCLUDED
#define BMI160DRV_H_INCLUDED

/* Includes ------------------------------------------------------------------*/
#include "sdk_errors.h"

/* Exported types ------------------------------------------------------------*/
typedef int16_t  q3_12_t;
typedef int16_t  q15_t;

typedef enum
{
    BMI160_PWR_OFF,     // off mode
    BMI160_PWR_STANDBY, // low power mode, only motion is reported
    //BMI160_PWR_LOWPOWER,// low power mode, only acceleration active NOT IMPLEMENTED YET
    BMI160_PWR_ON,      // fully operational, accel and gyro are reported
                        // with ~100Hz, motion as it occurs
} bmi160drv_powerMode_t;

typedef enum
{
    BMI160_EVT_TYPE_MOTION, // motion has been detected
    BMI160_EVT_TYPE_DATA,   // new data is available
} bmi160drv_eventType_t;

typedef struct
{
    q15_t   euler[3];   // pitch, roll and yaw angles in q15_t [0..1]->[0..2pi]
    //q3_12_t accel[3];   // raw x, y and z acceleration (range +-8g)
    //int16_t gyro[3];    // raw x, y and z gyro data (range +-2000 DPS)
} bmi160drv_data_t;

typedef struct
{
    bmi160drv_eventType_t type;
    bmi160drv_data_t      data; // used for event type BMI160_EVT_TYPE_DATA
} bmi160drv_event_t;

typedef void (*bmi160drv_eventHandler_t)(bmi160drv_event_t const* pEvt);

/** @brief function prototype for axis reorientation
 *
 * @note  The internal expects the sensor data in android device orientation.
 *        This function has to reorient the data from hardware orientation to
 *        the East-North-Up orientation
 * @param[in/out] pRawAccel, pGyro
 */
typedef void (*bmi160drv_reorientate_t)(int16_t *pRawAccel, int16_t *pGyro);

typedef struct
{
    uint8_t addr;       // I2C address to use, either 0x68 for grounded SDO pin, 0x69 if connected to VCC
    uint32_t int1Pin;   // GPIO pin number int1 is connected to, 0xFFFFFFFF if not connected
    bmi160drv_reorientate_t  reorient;  // the reorientation function, NULL if no reorientation is necessary
    bmi160drv_eventHandler_t handler;   // event handler to report events, must not be NULL
} bmi160drv_init_t;

/* Exported constants --------------------------------------------------------*/
#define BMI160DRV_NOINT 0xFFFFFFFF

/* Exported macros -----------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
/** @brief
 *
 * @param pInit bmi160drv_init_t const*
 * @return ret_code_t
 *
 */
ret_code_t bmi_Init(bmi160drv_init_t const* pInit);

ret_code_t bmi_SetPowerMode(bmi160drv_powerMode_t powerMode);

void bmi_Execute(void);

#endif // BMI160DRV

/**END OF FILE*****************************************************************/

