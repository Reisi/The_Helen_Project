/**
  ******************************************************************************
  * @file    feature_tools.h
  * @author  Thomas Reisnecker
  * @brief   tbd.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef FEATURE_TOOLS_H_INCLUDED
#define FEATURE_TOOLS_H_INCLUDED

/* Includes ------------------------------------------------------------------*/
#include "sdk_errors.h"
#include "stdbool.h"
#include "stdint.h"

/* Exported types ------------------------------------------------------------*/
typedef enum
{
    FTOOL_BRAKE_INDICATOR,
    FTOOL_DIR_INDICATOR,
    /// TODO: blinking and sos -> separate functions
} ftools_type_t;

typedef uint16_t q6_10_t;

typedef struct
{
    ftools_type_t type;
    bool active;
    q6_10_t endTS;
} ftools_inst_t;

/* Exported constants --------------------------------------------------------*/
#define FTOOLS_BRAKELIGHT   1024
#define FTOOLS_BRAKEFLASH   2048
#define FTOOLS_INDICATOR    1536

/* Exported macros -----------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
/** @brief function to initialize the feature tools module
 *
 * @param[in/out] pInst      array of instances
 * @param[in]     numOfInst  number of instances in the pInst array
 * @return NRF_SUCCESS, NRF_ERROR_NULL, NRF_ERROR_INVALID_PARAM or a propagated
 *         error from app_timer_create()
 */
ret_code_t ftools_Init(ftools_inst_t* pInst, uint8_t numOfInst);

/** @brief function to update the timestamp reference to synchronize countdowns
 *         to other devices
 *
 * @param[in] newReference
 */
void ftools_UpdateReference(q6_10_t newReference);

/** @brief function to get the current timestamp reference to synchronize
 *         other devices
 * @return current reference timestamp
 */
q6_10_t ftools_GetReference(void);

/** @brief function to set a countdown.
 *
 * @note The new value will not be adopted if the current value of a running
 *       timer is already higher. Use this function to synchronize a countdown
 *       to a remote device.
 *
 * @param[in/out] pInst      the instance to set
 * @param[in]     countdown  countdown value
 * @return NRF_SUCCESS, NRF_ERROR_NULL, or a propagated error from
 *         app_timer_start() or app_timer_stop()
 */
ret_code_t ftools_SetCountdown(ftools_inst_t* pInst, q6_10_t countdown);

/** @brief function to update a countdown
 *
 * @note If the instance is inactive, the requested value will be used. If the
 *       instance is already active the running countdown value will be
 *       increased gradually with respect to the blinking frequency just until
 *       the requested countdown value is reached. Use this function to set or
 *       update from a local condition
 *
 * @param[in/out] pInst      the instance to set
 * @param[in]     countdown  countdown value
 * @return NRF_SUCCESS, NRF_ERROR_NULL, or a propagated error from
 *         app_timer_start() or app_timer_stop()
 */
ret_code_t ftools_UpdateCountdown(ftools_inst_t* pInst, q6_10_t countdown);

/** @brief function to retrieve a countdown value
 *
 * @param[in/out] pInst       the instance to get the value from
 * @param[out]    pCountdown
 * @return NRF_SUCCESS, NRF_ERROR_NULL, NRF_ERROR_INVALID_STATE
 */
ret_code_t ftools_GetCountdown(ftools_inst_t const* pInst, q6_10_t* pCountdown);

/** @brief function to determine if a light corresponding to the feature should
 *         be on
 *
 * @param[in] pInst
 * @return true if light should be on, otherwise false
 */
bool ftools_IsOn(ftools_inst_t const* pInst);

/* inline functions --------------------------------------------------------- */

#endif // FEATURE_TOOLS_H_INCLUDED

/**END OF FILE*****************************************************************/

