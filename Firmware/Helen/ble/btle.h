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
#include "ble_hps.h"
#include "ble_hps_c.h"
#include "nrf_section.h"

/* Exported types ------------------------------------------------------------*/
typedef enum
{
    BTLE_EVT_NAME_CHANGE,               // event indicating that the device name was changed
    BTLE_EVT_SUPPORT_NOTIF              // event indicating that a support notification has been received
} btle_evt_type_t;

typedef struct
{
    btle_evt_type_t type;
    union
    {
        char const*          pNewDeviceName;// the new device name for BTLE_EVT_NAME_CHANGE event
        ble_hps_c_s_t const* pSupport;      // the support data for BTLE_EVT_SUPPORT_NOTIF
    };
} btle_event_t;

typedef void (*btle_eventHandler_t)(btle_event_t const * pEvt);

typedef enum
{
    BTLE_ADV_TYPE_ALWAYS_OPEN,          // advertising is always open for unknown devices
    BTLE_ADV_TYPE_STARTUP,              // connections from unknown devices only possible at start
    BTLE_ADV_TYPE_AFTER_SEARCH          // connections from unknown devices only possible after search window
} btle_advType_t;

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
    btle_advType_t       advType;       // the type of advertising, that should be used
    uint16_t             maxDeviceNameLength;       // the maximum supported device name length, 0 is name change is not supported, max value is BLE_GAP_DEVNAME_DEFAULT_LEN (31 bytes)
    btle_info_t const*   pInfo;         // the information included into the GAP and Device Information Service
    btle_eventHandler_t  eventHandler;  // the event handler for btle events
    //ble_lcs_init_t*      pLcsInit;      // the initialization structure for the light control service
    ble_hps_init_t*      pHpsInit;      // the initialization structure for the Helen Project Service
    ble_user_mem_block_t qwrBuffer;     // the buffer for the queued write module
} btle_init_t;

typedef struct
{
    uint16_t connHandle;
    uint8_t  mode;
} btle_modeRelay_t;

/*typedef struct
{
    ble_lcs_hlmt_mode_t mode;               // mode configuration, setup will always be included, intensity just if spot and/or flood are/is enabled
    ble_lcs_lm_status_flags_t statusSpot;   // status of spot, will be included if spot is enabled
    uint16_t powerSpot;                     // spot output power in mW, will be included if spot is active
    int8_t temperature;                     // light temperature in °C, will be included if value is in range -40..85
    uint16_t inputVoltage;                  // input voltage in mV, will be included if value > 0
    int8_t pitch;                           // pitch angle in °, will be included if value is in range -90..+90
    uint8_t batterySoc;                     // battery state of charge, will be included if value is in range 0..100
    uint16_t powerTaillight;                // taillight output power in mW, will be included if value > 0
} btle_lcsMeasurement_t;*/

typedef struct
{
    uint8_t mode;           // current mode
    uint16_t outputPower;   // output power in mW, will be included if > 0
    int8_t temperature;     // temperature in °C, will be included if value is in range -40..85
    uint16_t inputVoltage;  // input voltage in mV, will be included if >0
} btle_hpsMeasurement_t;

/**@brief Function prototype to add board specific services to the Helen Project Service.
 *
 * @param[in]       uuid_type the base uuid for the helen project service
 * @param[in/out]   user context registered in macro
 **/
typedef uint32_t (*btle_serviceAdd_t)(void * pContext);

typedef struct
{
    btle_serviceAdd_t  serviceAdd;  /**< Function to add board specific services. */
    void             * pContext;    /**< context for board specific services */
} btle_service_t;

/** @brief macro to register additional services that should be added
 *
 * @param _name     the service name
 * @param _char     the service init function
 * @param _context  the context to be delivered to the init function
 */
#define BTLE_SERVICE_REGISTER(_name, _func, _context)                       \
    NRF_SECTION_ITEM_REGISTER(btle_service, static btle_service_t _name) =  \
    {                                                                       \
        .serviceAdd = _func,                                                \
        .pContext  = _context,                                             \
    }

/* Exported defines ----------------------------------------------------------*/
#define APP_BLE_CONN_CFG_TAG    1 // A tag identifying the SoftDevice BLE configuration.

/* Exported macros -----------------------------------------------------------*/
#define BTLE_QWR_BUFF_SIZE(max_char_size)   ((max_char_size / 18) + 1) * 24)

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
 * @param[in] pRelay  the source and the mode to relay
 * @return NRF_ERROR_NOT_FOUND if no device connected, otherwise NRF_SUCCESS
 */
ret_code_t btle_RelayMode(btle_modeRelay_t const* pRelay);

/** @brief Function for updating light information. If a device is connected and
 *         light control service notifications are enabled a notification
 *         message will be sent.
 * @note call this function about once per second with actual values
 *
 * @param[in]   pData   light data to be sent
 * @return      NRF_SUCCESS, NRF_ERROR_INVALID_STATE if no notifications are to
 *              send or the propagated error of ble_lcs_light_measurement_send()
 */
//ret_code_t btle_ReportLcsMeasurements(btle_lcsMeasurement_t const* pData);

/** @brief Function for updating light information. If a device is connected and
 *         light control service notifications are enabled a notification
 *         message will be sent.
 * @note call this function about once per second with actual values
 *
 * @param[in]   pData   light data to be sent
 * @return      NRF_SUCCESS, NRF_ERROR_INVALID_STATE if no notifications are to
 *              send or the propagated error of ble_lcs_light_measurement_send()
 */
ret_code_t btle_ReportHpsMeasurements(btle_hpsMeasurement_t const* pData);

bool btle_isHpsDevice(uint16_t connHandle);

ret_code_t btle_ReportHpsSipport(ble_hps_s_t const* pData);

#endif // BTLE_H_INCLUDED

/**END OF FILE*****************************************************************/

