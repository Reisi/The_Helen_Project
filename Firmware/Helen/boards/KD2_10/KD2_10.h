/**
  ******************************************************************************
  * @file    KD2_10.h
  * @author  Thomas Reisnecker
  * @brief   tbd.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef KD2_10_H_INCLUDED
#define KD2_10_H_INCLUDED

/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "pitch_reg.h"
#include "helena_base_driver.h"

/* Exported types ------------------------------------------------------------*/
typedef int16_t  KD2_target_t;          // internal target values, max. value 100*256

typedef uint16_t q6_10_t;               // fix point integer used for power
typedef uint16_t q1_15_t;               // fix point integer used for gain factor

typedef enum
{
    KD2_PWR_OFF     = BRD_PM_OFF,       // complete off, CPU in system off, no wakeup expected
    KD2_PWR_STANDBY = BRD_PM_STANDBY,   // minimal current consumption, ready to wakeup
    KD2_PWR_IDLE    = BRD_PM_IDLE,      // operational, but light off
    KD2_PWR_ON                          // operational, light on
} KD2_powerMode_t;

typedef struct
{
    q6_10_t outputPower;
    KD2_target_t outputLimit;
    prg_optic_t optic;
} KD2_channelSetup_t;

typedef enum
{
    KD2_COMPIN_NOTUSED = 0,             // com pin not used at all
    KD2_COMPIN_COM,                     // com pin used for comreloaded
    KD2_COMPIN_BUTTON,                  // com pin used for external button
    KD2_COMPIN_PWM                      // com pin used as open drain PWM signal output
} KD2_comPinMode_t;

typedef struct
{
    q1_15_t gain;
    int16_t offset;
} KD2_compensation_t;

typedef struct
{
    KD2_compensation_t voltage;
    KD2_compensation_t current;
    KD2_compensation_t temperature;
} KD2_adcCompensation_t;

/* Exported constants --------------------------------------------------------*/
#define KD2_TARGET_MAX      (100 << 8)
#define KD2_TARGET_MIN      0

#define KD2_PITCH_OFF_MAX   (1 << 12)
#define KD2_PITCH_OFF_MIN   (-1 * KD2_PITCH_OFF_MAX)

#define KD2_OPTIC_TYPE_NA   0xFF

/* Exported macros -----------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
/** @brief function to retrieve a channel setup
 *
 * @param[out] pSetup   structure to store the setup
 * @param[in]  channel  the desired channel
 * @return NRF_SUCCESS, NRF_ERROR_NULL, NRF_ERROR_INVALID_PARAM for invalid channel
 */
ret_code_t KD2_GetChannelSetup(KD2_channelSetup_t* pSetup, uint8_t channel);

/** @brief function to set a channel setup
 *
 * @param[in]  pSetup   the new setup
 * @param[in]  channel  the desired channel
 * @return NRF_SUCCESS, NRF_ERROR_NULL, NRF_ERROR_INVALID_PARAM for invalid channel
 */
ret_code_t KD2_SetChannelSetup(KD2_channelSetup_t const* pSetup, uint8_t channel, ds_reportHandler_t resultHandler);

/** @brief function to get the adc compensation values
 *
 * @note currently only the current gain factor and temperature offset are
 *       supported
 *
 * @param[out] pComp
 * @return NRF_SUCCESS, NRF_ERROR_NULL
 */
ret_code_t KD2_GetCompensation(KD2_adcCompensation_t* pComp);

/** @brief function to get the adc compensation values
 *
 * @note currently only the current gain factor and temperature offset are
 *       supported, other values will be ignored
 *
 * @param[in] pComp
 * @return NRF_SUCCESS, NRF_ERROR_NULL
 */
ret_code_t KD2_SetCompensation(KD2_adcCompensation_t const* pComp, ds_reportHandler_t resultHandler);

/** @brief function to get the current com pin usage mode
 *
 * @param[out] pMode
 * @return NRF_SUCCESS, NRF_ERROR_NULL
 */
ret_code_t KD2_GetComPinMode(KD2_comPinMode_t* pMode);

/** @brief function to set the com pin usage
 *
 * @param[in] mode
 * @return NRF_SUCCESS, NRF_ERROR_NULL
 */
ret_code_t KD2_SetComPinMode(KD2_comPinMode_t mode, ds_reportHandler_t resultHandler);

/** @brief function to get the compensation values for the helena driver board
 *
 * @param[out] pComp
 * @return NRF_SUCCESS, NRF_ERROR_NULL, NRF_ERROR_NOT_FOUND, NRF_ERROR_NOT_SUPPORTED
 */
ret_code_t KD2_GetHelenaCompensation(hbd_calibData_t* pComp);

/** @brief function to set the compensation values for the helena driver board
 *
 * @param[out] pComp
 * @return NRF_SUCCESS, NRF_ERROR_NULL, NRF_ERROR_NOT_FOUND, NRF_ERROR_NOT_SUPPORTED, NRF_ERROR_INVALID_PARAM
 */
 /// TODO: this function used I2C and should be scheduled in main context
ret_code_t KD2_SetHelenaCompensation(hbd_calibData_t* pComp);

#endif // KD2_10_H_INCLUDED

/**END OF FILE*****************************************************************/

