/**
  ******************************************************************************
  * @file    Drake_10.h
  * @author  Thomas Reisnecker
  * @brief   tbd.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DRAKE_10_H_INCLUDED
#define DRAKE_10_H_INCLUDED

/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "sdk_errors.h"
#include "stdbool.h"
#include "data_storage.h"
#include "Drake_light.h"
#include "nrfx_comp.h"
#include "motor_control.h"

/* Exported types ------------------------------------------------------------*/
typedef enum
{
    DRAKE_USAGE_DROPPER_ACTUATOR = 0,
    DRAKE_USAGE_TAILLIGHT_16LED  = 16,
    DRAKE_USAGE_TAILLIGHT_44LED  = 44,
    DRAKE_USAGE_BARLIGHTS_36LED  = 36
} drake_usage_t;

typedef uint8_t q8_t;
typedef uint16_t q2_14_t;
typedef uint32_t q16_16_t;
typedef uint32_t q18_14_t;

typedef enum
{
    DRAKE_BAT_TYPE_LCO = 0,
    DRAKE_BAT_TYPE_NMC = 1,
} drake_batteryType_t;

typedef struct
{
    drake_batteryType_t type;
    q16_16_t capNom;            // nominal capacity
    q16_16_t capAct;            // actual capacity
} drake_batteryInfo_t;

/*#if APP_TIMER_CONFIG_RTC_FREQUENCY != 1
#error "motor timeout values are stored in q18_14_t and need to be converted if APP_TIMER_CONFIG_RTC_FREQUENCY!== 1"
#endif // APP_TIMER_CONFIG_RTC_FREQUENCY

/// TODO: replace with structs of motor control

typedef struct
{
    q18_14_t openTimeout;
    q18_14_t closeTimeout;
} drake_motorConfig_t;*/

typedef enum
{
    DRAKE_MOTOR_SENS_CALIB_THRESHOLD = 0,
    DRAKE_MOTOR_SENS_CALIN_OPEN_CNT  = 1,
} drake_motorSensCalibType_t;

typedef union
{
    mtSens_thresholds_t threshold;
    uint16_t openCnt;
} drake_motorSensCalibData_t;

typedef struct
{
    uint16_t travelInmm;
    int32_t offsetInMikrom;
} drake_travelSensorData_t;

typedef struct
{
    uint32_t blockingTime;  // blocking time in app timer ticks
    uint32_t timeout;       // timeout in app timer ticks
    int16_t positions[4];
} drake_indexedTravelConfig_t;

/* Exported constants --------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
/** @brief function to retriev the current device usage
 * @return device usage
 */
drake_usage_t Drake_GetDeviceUsage(void);

/** @brief function to set the device usage
 *
 * @param[in] usage          new device usage
 * @param[in] resultHandler  the handler to call when the storage operation finished
 * @return NRF_ERROR_INVALID_PARAM or a storage related error code
 */
ret_code_t Drake_SetDeviceUsage(drake_usage_t usage, ds_reportHandler_t resultHandler);

/** @brief Fucntion to get the battery information
 *
 * @param[out] pInfo  battery information
 * @return NRF_SUCCESS, NRF_ERROR_NULL
 */
ret_code_t Drake_GetBatteryInfo(drake_batteryInfo_t* pInfo);

/** @brief Function to set the battery information
 *
 * @param[in] pInfo          new battery information
 * @param[in] resultHandler  the handler to call when the storage operation finished
 * @return NRF_ERROR_INVALID_PARAM or a storage related error code
 */
ret_code_t Drake_SetBatteryInfo(drake_batteryInfo_t const* pInfo, ds_reportHandler_t resultHandler);

/** @brief function to get the soc timeout
 *
 * @param[out] pTimeout  the requested timeout
 * @return NRF_SUCCESS, NRF_ERROR_NULL, NRF_ERROR_NOT_SUPPORTED
 */
ret_code_t Drake_GetSocTimeout(uint32_t* pTimeout);

/** @brief function to set the soc timeout
 *
 * @param[in] timeout  the requested timeout
 * @param[in] resultHandler  the handler to call when the storage operation finished
 * @return NRF_SUCCESS, NRF_ERROR_INVALID_PARAM, NRF_ERROR_NOT_SUPPORTED
 */
ret_code_t Drake_SetSocTimeout(uint32_t timeout, ds_reportHandler_t resultHandler);

/** @brief Function to get a ligth pattern
 *
 * @param[in]  type      the light type
 * @param[out] pPattern  the pattern structure
 * @return NRF_ERROR_SUCCESS, NRF_ERROR_NULL or NRF_ERROR_INVALID_PARAM
 */
ret_code_t Drake_GetLightPattern(Drake_lightType_t type, Drake_lightPattern_t* pPattern);

/** @brief Function to set a light pattern
 *
 * @param[in] type      the light type
 * @param[in] pPattern  the pattern structure
 * @param[in] resultHandler  the handler to call when the storage operation finished
 * @return NRF_ERROR_NULL, NRF_ERROR_INVALID_LENGTH,
 *         NRF_ERROR_INVALID_PARAM or a storage related error code
 */
ret_code_t Drake_SetLightPattern(Drake_lightType_t type, Drake_lightPattern_t const* pPattern, ds_reportHandler_t resultHandler);

/** @brief Function to set the plunger position
 *
 * @param[in] newPosition
 * @param[in] remote       true if remote request, false for local
 * @return NRF_ERROR_NOT_SUPPORTED or a propagated error code of
 *         either plctrl_SetPosition() or plctrli2cm_SetPosition()
 */
ret_code_t Drake_SetPlungerPosition(q8_t newPosition, bool remote);

/** @brief Function to retrieve the current plunger position
 *
 * @param[out] pPosition
 * @return NRF_SUCCESS, NRF_ERROR_NOT_SUPPORTED
 */
ret_code_t Drake_GetPlungerPosition(q8_t* pPosition);

/** @brief function to get the motor configuration
 *
 * @param[out] pMotorConfig
 * @return NRF_SUCCESS, NRF_ERROR_NULL, NRF_ERROR_NOT_SUPPORTED
 */
ret_code_t Drake_GetMotorConfig(mtCtrl_motorConfig_t* pMotorConfig);

/** @brief function to set the motor configuration
 *
 * @param[in] pMotorConfig
 * @param[in] resultHandler the handler to call with result
 * @return NRF_SUCCES, NRF_ERROR_NULL, NRF_ERROR_NOT_SUPPORTED, NRF_ERROR_INVALID_PARAM
 */
ret_code_t Drake_SetMotorConfiguration(mtCtrl_motorConfig_t const* pMotorConfig, ds_reportHandler_t resultHandler);

/** @brief function to get the motor sensor calibration data
 *
 * @param[in]  type
 * @param[out] pCalibData
 * @return NRF_SUCCESS, NRF_ERROR_NULL, NRF_ERROR_NOT_SUPPORTED
 */
ret_code_t Drake_GetMotorCalibrationData(drake_motorSensCalibType_t type, drake_motorSensCalibData_t* pCalibData);

/** @brief function to clear the motor sensor calibration data
 *
 * @param[in] type
 * @param[in] resultHandler the handler to call with result
 * @return NRF_SUCCES, NRF_ERROR_NOT_SUPPORTED, NRF_ERROR_INVALID_PARAM
 */
ret_code_t Drake_ClearMotorCalibrationData(drake_motorSensCalibType_t type, ds_reportHandler_t resultHandler);

/** @brief function to start the motor sensor calibration
 *
 * @param[in] type
 * @param[in] resultHandler the handler to call with result
 * @return NRF_SUCCES, NRF_ERROR_NOT_SUPPORTED, NRF_ERROR_INVALID_PARAM
 */
ret_code_t Drake_StartMotorCalibration(drake_motorSensCalibType_t type, ds_reportHandler_t resultHandler);

/** @brief function to get the travel sensor data
 *
 * @param[out] pData
 * @return NRF_SUCCESS, NRF_ERROR_NULL
 */
ret_code_t Drake_GetTravelSensorData(drake_travelSensorData_t* pData);

/** @brief function to clear the travel sensor data and reset to default values
 *
 * @param[in] resultHandler the handler to call with the result
 * @return NRF_SUCCESS or tbd.
 */
ret_code_t Drake_ClearTravelSensorData(ds_reportHandler_t resultHandler);

/** @brief function to start the travel sensor calibration
 *
 * @param[in] travelInmm     the seatpost travel in mm
 * @param[in] resultHandler  the handler to call with the result
 * @return NRF_SUCCESS, NRF_ERROR_INVALID_PARAM
 */
ret_code_t Drake_StartTravelSensorCalibration(uint16_t travelInmm, ds_reportHandler_t resultHandler);

/** @brief function to get the indexed travel configuration
 *
 * @param[out] pConfig
 * @return NRF_SUCCESS, NRF_ERROR_NULL
 */
ret_code_t Drake_GetIndexedTravelConfig(drake_indexedTravelConfig_t* pConfig);

/** @brief function to set the indexed travel configuration
 *
 * @param[in] pConfig        the new configuration
 * @param[in] resultHandler  the handler to call with the result
 * @return ret_code_t
 *
 */
ret_code_t Drake_SetIndexedTravelConfig(drake_indexedTravelConfig_t const* pConfig, ds_reportHandler_t resultHandler);

void Drake_Brake();

void Drake_IndicLeft();

void Drake_IndicRight();

uint16_t Drake_GetActivationCounter(void);

void Drake_SetActivationCounter(uint16_t newCounterValue);


#endif // DRAKE_10_H_INCLUDED

/**END OF FILE*****************************************************************/

