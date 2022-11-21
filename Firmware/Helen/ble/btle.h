/**
  ******************************************************************************
  * @file    btle.h
  * @author  Thomas Reisnecker
  * @brief   header file for bluetooth module
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BTLE_H_INCLUDED
#define BTLE_H_INCLUDED

/* Includes ------------------------------------------------------------------*/
#include "ble_lcs.h"

#define APP_BLE_CONN_CFG_TAG    1 // A tag identifying the SoftDevice BLE configuration.

/* Exported types ------------------------------------------------------------*/
typedef struct
{
    char const* pDevicename;            // the name of the device
    char const* pManufacturer;
    char const* pModelNumber;
    char const* pBoardHWVersion;
    uint16_t deviceAppearance;          // the used appearance
} btle_info_t;

typedef struct
{
    btle_info_t const* pInfo;
    bool useWhitelistAdvertising;       // if the whitelist should be also used for advertising
    //btle_eventHandler_t eventHandler;   // the event handler
    ble_lcs_init_t* pLcsInit;             // the initialization structure for the light control service
} btle_init_t;

typedef struct
{
    ble_lcs_hlmt_mode_t mode;               // mode configuration, setup will always be included, intensity just if spot and/or flood are/is enabled
    ble_lcs_lm_status_flags_t statusSpot;   // status of spot, will be included if spot is enabled
    uint16_t powerSpot;                     // spot output power in mW, will be included if spot is active
    int8_t temperature;                     // light temperature in °C, will be included if value is in range -40..85
    uint16_t inputVoltage;                  // input voltage in mV, will be included if value > 0
    int8_t pitch;                           // pitch angle in °, will be included if value is in range -90..+90
    uint8_t batterySoc;                     // battery state of charge, will be included if value is in range 0..100
    uint16_t powerTaillight;                // taillight output power in mW, will be included if value > 0
} btle_lcsMeasurement_t;

/* Exported macros -----------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
/** @brief function to initialize the bluetooth module
 *
 * @param[in] pInit
 * @return NRF_SUCCESS
 */
ret_code_t btle_Init(btle_init_t const* pInit);

/** @brief function to relay a mode change to connected lights
 *
 * @note use this function to relay a mode change to connected devices. Use
 *       BLE_CONN_HANDLE_ALL to relay to all connected devices (when mode
 *       change was initiated from this device). If the mode change was already
 *       relayed from another device use the connection Handle of this device
 *       to prevent a command back to the originator.
 * @param[in] modeNumber the mode to relay
 * @param[in] connHandle the connection to exclude
 * @return NRF_SUCCESS or tdb
 */
ret_code_t btle_RelayMode(uint8_t modeNumber, uint16_t connHandle);

/** @brief Function for updating light information. If a device is connected and
 *         light control service notifications are enabled a notification
 *         message will be sent.
 * @note call this function about once per second with actual values
 *
 * @param[in]   pData   light data to be sent
 * @return      NRF_SUCCESS, NRF_ERROR_INVALID_STATE if no notifications are to
 *              send or the propagated error of ble_lcs_light_measurement_send()
 */
ret_code_t btle_ReportMeasurements(btle_lcsMeasurement_t const* pData);

#endif // BTLE_H_INCLUDED

/**END OF FILE*****************************************************************/

