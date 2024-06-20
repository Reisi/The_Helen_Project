/**
  ******************************************************************************
  * @file    link_management.c
  * @author  Thomas Reisnecker
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "ble_advertising.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_scan.h"
#include "ble_conn_state.h"
#include "ble_conn_params.h"
#include "app_timer.h"

#include "link_management.h"
#include "ble_hps.h"
#include "ble_lcs.h"
#include "btle.h"

/* logger configuration ----------------------------------------------_-------*/
#define BTLE_CONFIG_LOG_ENABLED 1 /// TODO: just temporary until project specific sdk_config
#define BTLE_CONFIG_LOG_LEVEL   3

#define NRF_LOG_MODULE_NAME btle
#if BTLE_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL BTLE_CONFIG_LOG_LEVEL
#else //BASE_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL 0
#endif //BASE_CONFIG_LOG_ENABLED
#include "nrf_log.h"

/* Private defines -----------------------------------------------------------*/
#define APP_BLE_OBSERVER_PRIO   3

#define SCAN_LOW_POWER          0
#define SCAN_LOW_LATENCY        1
#define SCAN_SEARCH             2

#define CONN_LOW_POWER          0
#define CONN_LOW_LATENCY        1

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
    lm_advState_t actual;
    lm_advState_t desired;
    btle_advType_t advType;     // the type of advertising
    bool isOpenAdvertising;     // indicating if current advertising should be open to unbonded devices, this will
                                // be reset by advertising timeout or when peripheral device connects,
} advState_t;

typedef struct
{
    lm_scanState_t actual;
    lm_scanState_t desired;
    uint8_t isPeriphControl;    // cumulative counter, if set from several services
} scanState_t;

typedef struct
{
    ble_gap_addr_t  address;    // the address of the device to be connected, this will be reset in the onConnected handler or on connection timeout
    rem_driver_t const* pDriver;// the used driver (if device is a remote, null pointer otherwise)
    uint16_t waitForDisconnect; // the connection handle of the remote, that has to be disconnected first ( if trying to connect to a remote)
    bool isSearchDevice;        // true if the device was found through active scanning, false for whitelist scanning
} deviceConnecting_t;

/* Private macros ------------------------------------------------------------*/

/* Private read only variables and configurations ----------------------------*/
static const ble_adv_modes_config_t advConfig =
{
    .ble_adv_fast_enabled = true,
    .ble_adv_fast_interval = MSEC_TO_UNITS(50, UNIT_0_625_MS),
    .ble_adv_fast_timeout = MSEC_TO_UNITS(30000, UNIT_10_MS),
    .ble_adv_slow_enabled = true,
    .ble_adv_slow_interval = MSEC_TO_UNITS(1000, UNIT_0_625_MS),
    .ble_adv_slow_timeout = 0,  // no timeout
};

static const ble_gap_scan_params_t scanParams[] =
{
    {   // scan settings for BTLE_SCAN_LOW_POWER
        .active        = 0,
        .interval      = MSEC_TO_UNITS(1280, UNIT_0_625_MS),
        .window        = MSEC_TO_UNITS(11.25, UNIT_0_625_MS),
        .filter_policy = BLE_GAP_SCAN_FP_WHITELIST,
        .timeout       = 0,
        .scan_phys     = BLE_GAP_PHY_1MBPS,
    },
    {   // scan settings for BTLE_SCAN_LOW_LATENCY
        .active        = 0,
        .interval      = MSEC_TO_UNITS(60, UNIT_0_625_MS),
        .window        = MSEC_TO_UNITS(30, UNIT_0_625_MS),
        .filter_policy = BLE_GAP_SCAN_FP_WHITELIST,
        .timeout       = 0,
        .scan_phys     = BLE_GAP_PHY_1MBPS,
    },
    {   // scan settings for BTLE_SCAN_SEARCH
        .active        = 0x01,
        .interval      = MSEC_TO_UNITS(60, UNIT_0_625_MS),
        .window        = MSEC_TO_UNITS(30, UNIT_0_625_MS),
        .filter_policy = BLE_GAP_SCAN_FP_ACCEPT_ALL,
        .timeout       = MSEC_TO_UNITS(30000, UNIT_10_MS),
        .scan_phys     = BLE_GAP_PHY_1MBPS,
    },
};

static const ble_gap_conn_params_t connParamsPeriph[] =
{
    {   // desired connection parameters for low power link
        .min_conn_interval = MSEC_TO_UNITS(500, UNIT_1_25_MS),
        .max_conn_interval = MSEC_TO_UNITS(1000, UNIT_1_25_MS),
        .slave_latency     = 0,
        .conn_sup_timeout  = MSEC_TO_UNITS(4000, UNIT_10_MS)
    },
    {   // desired connection parameters for low latency link
        .min_conn_interval = MSEC_TO_UNITS(100, UNIT_1_25_MS),
        .max_conn_interval = MSEC_TO_UNITS(200, UNIT_1_25_MS),
        .slave_latency     = 0,
        .conn_sup_timeout  = MSEC_TO_UNITS(4000, UNIT_10_MS)
    }
};

static const ble_gap_conn_params_t connParamsCentral =
{
    .min_conn_interval = MSEC_TO_UNITS(10, UNIT_1_25_MS),
    .max_conn_interval = MSEC_TO_UNITS(50, UNIT_1_25_MS),
    .slave_latency     = 0,
    .conn_sup_timeout  = MSEC_TO_UNITS(4000, UNIT_10_MS)
};

static const ble_gap_sec_params_t secParam =
{
    .bond           = 1,                    // perform bonding
    .mitm           = 0,                    // man in the middle protection not required
    .lesc           = 0,                    // LE secure connections not enabled
    .keypress       = 0,                    // keypress notifications not enabled
    .io_caps        = BLE_GAP_IO_CAPS_NONE, // no I/O capabilities
    .oob            = 0,                    // out of bound data not available
    .min_key_size   = 7,
    .max_key_size   = 16,
    .kdist_own.enc  = 1,
    .kdist_own.id   = 1,
    .kdist_peer.enc = 1,
    .kdist_peer.id  = 1,
};

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3

/* Private variables ---------------------------------------------------------*/
BLE_ADVERTISING_DEF(advInst);               // advertising instance
NRF_BLE_SCAN_DEF(scanInst);                 // scanner instance

static uint8_t uuidType;                    // the uuid type of the light control and helen project service
static advState_t advState;                 // the advertising state
static scanState_t scanState;               // the scanning state
static deviceConnecting_t pendingDevice;    // information of the device, helen is trying to connect to
static uint16_t openPeriphConnHandle;       // the connection handle of the peripheral that connected during open advertising
static uint16_t remoteConnHandle;           // the connection handle of the currently connected remote
static lm_eventHandler_t eventHandler;      // the event handler
static ds_reportHandler_t deleteHandler;    // the deletion report handler, only set if a deletion is pending

/* Private functions ---------------------------------------------------------*/

static void connect(ble_gap_addr_t const* pAddr)
{
    uint32_t errCode;

    // scanning has to be stopped to connect, it will be restarted when connected or on connection timeout
    nrf_ble_scan_stop();
    scanState.actual = LM_SCAN_OFF;

    errCode = sd_ble_gap_connect(pAddr, &scanParams[SCAN_SEARCH], &connParamsCentral, APP_BLE_CONN_CFG_TAG);
    if (errCode == NRF_SUCCESS)
    {
        NRF_LOG_INFO("Connecting...");
    }
    else
    {
        NRF_LOG_WARNING("Failed to initiate connection. Error: %d", errCode);
    }
}

static void disconnect(uint16_t connHandle, void * pContext)
{
    UNUSED_PARAMETER(pContext);

    ret_code_t errCode = sd_ble_gap_disconnect(connHandle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    if (errCode != NRF_SUCCESS)
    {
        NRF_LOG_WARNING("Failed to disconnect connection. Connection handle: %d Error: %d", connHandle, errCode);
    }
    else
    {
        NRF_LOG_INFO("Disconnecting connection handle %d", connHandle);
    }
}

static void sendScanEvent(lm_scanState_t state)
{
    NRF_LOG_INFO("scan state changed, desire %d, actual %d", scanState.desired, state)

    if (eventHandler == NULL)
        return;

    lm_evt_t evt;
    evt.type = LM_EVT_SCAN_MODE_CHANGED;
    evt.evt.newScanState = state;
    eventHandler(&evt);
}

static void sendAdvEvent(lm_advState_t state)
{
    NRF_LOG_INFO("advertising state changed, desired: %d, actual %d", advState.desired, state)

    if (eventHandler == NULL)
        return;

    lm_evt_t evt;
    evt.type = LM_EVT_ADV_MODE_CHANGED;
    evt.evt.newAdvState = state;
    eventHandler(&evt);
}

/** @brief function to retrieve the next peer manager id depending on the device role
 *
 * @param peerId    the peer id to start from, PM_PEER_ID_INVALID to get the first
 * @param role      BLE_GAP_ROLE_CENTRAL or BLE_GAP_ROLE_PERIPHERAL
 * @return the peer id or PM_PEER_ID_INVALID if no more available
 */
static pm_peer_id_t nextPeerIdGet(pm_peer_id_t peerId, uint8_t role)
{
    pm_peer_data_bonding_t peerData;

    while (1)
    {
        peerId = pm_next_peer_id_get(peerId);
        if (peerId == PM_PEER_ID_INVALID)
            break;

        uint32_t errCode = pm_peer_data_bonding_load(peerId, &peerData);
        if (errCode != NRF_SUCCESS)
        {
            NRF_LOG_ERROR("error loading bonding data, %d", errCode);
        }
        else if (peerData.own_role == role)
            break;
    }
    return peerId;
}

/**< returns the number of bonded but unconnected devices depending on the device role */
static uint8_t unconnectedDevices(uint8_t role)
{
    uint8_t cnt = 0;
    pm_peer_id_t peer = nextPeerIdGet(PM_PEER_ID_INVALID, role);

    while (peer != PM_PEER_ID_INVALID)
    {
        uint16_t connHandle;
        APP_ERROR_CHECK(pm_conn_handle_get(peer, &connHandle));
        if (ble_conn_state_status(connHandle) != BLE_CONN_STATUS_CONNECTED)
            cnt++;
        peer = nextPeerIdGet(peer, role);
    }

    return cnt;
}

/** @brief function to set the desired advertising mode
 *
 * @note the actual used advertising mode is reported through the event handler
 *
 * @param[in] mode                  the desired mode
 * @return the return value of ble_advertising_start
 */
static ret_code_t setAdvMode(lm_advState_t mode)
{
    ble_adv_modes_config_t config = advConfig;
    ble_adv_mode_t advMode = BLE_ADV_MODE_IDLE;
    ret_code_t errCode = NRF_SUCCESS;

    /// TODO: is it necessary to check if already in the requested mode? (desired or actual?)

    // convert BTLE_ADV_... modes to BLE_ADV_... modes
    switch (mode)
    {
    case LM_ADV_OFF:
        break;
    case LM_ADV_SLOW:
        advMode = BLE_ADV_MODE_SLOW;
        break;
    case LM_ADV_FAST:
    case LM_ADV_OPEN:
        advMode = BLE_ADV_MODE_FAST;
        break;
    }

    // stop advertising to be able to restart with different settings
    (void)sd_ble_gap_adv_stop(advInst.adv_handle);

    advState.isOpenAdvertising = mode == LM_ADV_OPEN ? true : advState.advType == BTLE_ADV_TYPE_ALWAYS_OPEN;

    // check if there are devices to advertise to
    if (!advState.isOpenAdvertising)
    {
        if(unconnectedDevices(BLE_GAP_ROLE_PERIPH) == 0 ||
           ble_conn_state_peripheral_conn_count() == NRF_SDH_BLE_PERIPHERAL_LINK_COUNT)
        {
            // no device to advertise to, advertising is not needed, manually update state and send event
            sendAdvEvent(LM_ADV_OFF);
            advState.actual = LM_ADV_OFF;
            return NRF_SUCCESS;
        }
    }

    if (mode == LM_ADV_OFF)
    {   // prevent module from restarting if a device disconnects
        config.ble_adv_on_disconnect_disabled = true;
        ble_advertising_modes_config_set(&advInst, &config);
        // set mode and event manually
        sendAdvEvent(LM_ADV_OFF);
        advState.actual = LM_ADV_OFF;
    }
    else
    {
        ble_advertising_modes_config_set(&advInst, &config);
        errCode = ble_advertising_start(&advInst, advMode);
        // state and event are handled in the advertising event handler
    }

    if (errCode != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("setting advertising mode error %d", errCode);
    }

    return errCode;
}

/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void advEventHandler(ble_adv_evt_t advEvt)
{
    lm_advState_t newState;

    switch (advEvt)
    {
        // the dedicated LM_ADV_OPEN mode is only used when advertising isn't open all the time
        case BLE_ADV_EVT_FAST:
            if (advState.desired == LM_ADV_SLOW)
            {
                setAdvMode(LM_ADV_SLOW);
                return; // we'll receive a new event with BLE_ADV_EVT_SLOW
            }
            if (advState.advType == BTLE_ADV_TYPE_ALWAYS_OPEN || !advState.isOpenAdvertising)
                newState = LM_ADV_FAST;
            else
                newState = LM_ADV_OPEN;
            break;

        case BLE_ADV_EVT_SLOW:
            newState = LM_ADV_SLOW;
            // open advertising has ended
            if (advState.isOpenAdvertising && advState.advType != BTLE_ADV_TYPE_ALWAYS_OPEN)
            {
                // check if there are devices to advertise to, if not stop advertising
                if (unconnectedDevices(BLE_GAP_ROLE_PERIPH) == 0)
                {
                    (void)sd_ble_gap_adv_stop(advInst.adv_handle);
                    newState = LM_ADV_OFF;
                }

                advState.isOpenAdvertising = false;
            }
            break;

        case BLE_ADV_EVT_IDLE:
            newState = LM_ADV_OFF;
            break;

        default:
            return;
    }

    if (newState != advState.actual)
    {
        sendAdvEvent(newState);
        advState.actual = newState;
    }
}

static void advErrorHandler(uint32_t errCode)
{
    NRF_LOG_ERROR("advertising error %d", errCode);
}

/** @brief sets the white list
 *  @note  whitelist is only used for scanning, so only devices with central role are added
 *
 * @return bool
 *
 */
static bool setWhitelist()
{
    uint32_t errCode, cnt = 0;
    pm_peer_id_t peers[8], id = nextPeerIdGet(PM_PEER_ID_INVALID, BLE_GAP_ROLE_CENTRAL);

    while (id != PM_PEER_ID_INVALID && cnt < ARRAY_SIZE(peers))
    {
        // check if the device is already connected
        uint16_t connHandle = BLE_CONN_HANDLE_INVALID;
        (void)pm_conn_handle_get(id, &connHandle);
        if (ble_conn_state_status(connHandle) != BLE_CONN_STATUS_CONNECTED) // add only unconnected devices
            peers[cnt++] = id;
        id = nextPeerIdGet(id, BLE_GAP_ROLE_CENTRAL);
    }

    if (cnt == 0)
        return false;

    errCode = pm_whitelist_set(peers, cnt);
    APP_ERROR_CHECK(errCode);

    // advertising and scanning needs to be disabled to change this
    errCode = pm_device_identities_list_set(peers, cnt);
    if (errCode != BLE_ERROR_GAP_DEVICE_IDENTITIES_IN_USE) /// TODO: how to handle correctly
    {
        APP_ERROR_CHECK(errCode);
    }

    return true;
}

static ret_code_t setScanMode(lm_scanState_t mode)
{
    /// TODO: how to handle searching, if no connections are available?

    ret_code_t errCode = NRF_SUCCESS;
    bool devicesAvailable = (bool)unconnectedDevices(BLE_GAP_ROLE_CENTRAL);
    bool connectionsAvailable = ble_conn_state_central_conn_count() < NRF_SDH_BLE_CENTRAL_LINK_COUNT;
    lm_scanState_t newMode = LM_SCAN_OFF;

    if (mode == LM_SCAN_OFF)
        nrf_ble_scan_stop();    // just stop, nothing more to do
    else
    {
        errCode = nrf_ble_scan_params_set(&scanInst, &scanParams[mode - 1]);
        APP_ERROR_CHECK(errCode);

        // don't scan if there is nothing to scan for
        if (connectionsAvailable && (mode == LM_SCAN_SEARCH || devicesAvailable))
        {
            errCode = nrf_ble_scan_start(&scanInst);
            newMode = mode;
        }
    }

    if (scanState.actual != newMode)
    {
        sendScanEvent(newMode);
        scanState.actual = newMode;
    }

    if (errCode != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("setting scan mode error %d", errCode);
    }

    return errCode;
}

static rem_driver_t const* isRemote(uint8_t const* pData, uint16_t len)
{
    uint8_t i;
    for (i = 0; i < REMOTE_CNT(); i++)
    {
        rem_driver_t const* pDriver = REMOTE_GET(i);
        if (pDriver->is_device(pData, len))
            return pDriver;
    }
    return NULL;
}

static bool isCustomService(uint16_t uuid, uint8_t const* pData, uint16_t len)
{
    ble_uuid_t serviceUuid;
    serviceUuid.uuid = uuid;
    serviceUuid.type = uuidType;

    return ble_advdata_uuid_find(pData, len, &serviceUuid);
}

static void scanningEventHandler(scan_evt_t const* pScanEvt)
{
    ble_gap_evt_adv_report_t const* pAdv;

    switch(pScanEvt->scan_evt_id)
    {
    case NRF_BLE_SCAN_EVT_WHITELIST_REQUEST:
    {   /// TODO: maybe setting the whitelist is only necessary if a new device is added?
        setWhitelist();
    }   return;

    // scanning timeout is only used when searching for new devices, thus this event can be used to start open advertising after
    case NRF_BLE_SCAN_EVT_SCAN_TIMEOUT:
        // restart previous scan mode after searching
        (void)setScanMode(scanState.desired);
        // advertising was stopped while searching, restart with previous or open mode
        (void)setAdvMode(advState.advType == BTLE_ADV_TYPE_AFTER_SEARCH ? LM_ADV_OPEN : advState.desired);
        return;

    case NRF_BLE_SCAN_EVT_FILTER_MATCH:
        pAdv = pScanEvt->params.filter_match.p_adv_report;
        break;
    case NRF_BLE_SCAN_EVT_WHITELIST_ADV_REPORT:
        pAdv = pScanEvt->params.p_whitelist_adv_report;
        break;

    default:
        return;
    }

    // additional handling for cases NRF_BLE_SCAN_EVT_FILTER_MATCH and NRF_BLE_SCAN_EVT_WHITELIST_ADV_REPORT
    // safety check, should not happen
    ble_gap_addr_t zero = {0};
    if (memcmp(&pendingDevice.address, &zero, sizeof(ble_gap_addr_t)) != 0)
    {
        NRF_LOG_ERROR("scanning while device connection is still pending")
    }

    // check if the device is a remote
    rem_driver_t const* pDriver = isRemote(pAdv->data.p_data, pAdv->data.len);
    if (pDriver != NULL)
    {
        NRF_LOG_INFO("remote control advertising report received");

        if (!scanState.isPeriphControl) // don't connect if under peripheral control
        {
            pendingDevice.address = pAdv->peer_addr;
            pendingDevice.pDriver = pDriver;
            pendingDevice.isSearchDevice = pScanEvt->scan_evt_id == NRF_BLE_SCAN_EVT_FILTER_MATCH;
            if (remoteConnHandle == BLE_CONN_HANDLE_INVALID)
                connect(&pAdv->peer_addr); // connect if no remote is connected yet
            else if (pScanEvt->scan_evt_id == NRF_BLE_SCAN_EVT_FILTER_MATCH)
            {                   // disconnect existing remote if a new one was found while searching
                if (pendingDevice.waitForDisconnect != remoteConnHandle) // don't disconnect more than once
                {
                    disconnect(remoteConnHandle, NULL);
                    pendingDevice.waitForDisconnect = remoteConnHandle;
                }
            }
        }
    }
    // check if device is another light
    if (isCustomService(BLE_UUID_LCS_SERVICE, pAdv->data.p_data, pAdv->data.len))
    {
        NRF_LOG_INFO("light control advertising report received");
        pendingDevice.address = pAdv->peer_addr;
        pendingDevice.pDriver = NULL;
        pendingDevice.isSearchDevice = pScanEvt->scan_evt_id == NRF_BLE_SCAN_EVT_FILTER_MATCH;
        connect(&pAdv->peer_addr);
    }
    if (isCustomService(BLE_UUID_HPS_SERVICE, pAdv->data.p_data, pAdv->data.len))
    {
        NRF_LOG_INFO("helen project advertising report received");
        pendingDevice.address = pAdv->peer_addr;
        pendingDevice.pDriver = NULL;
        pendingDevice.isSearchDevice = pScanEvt->scan_evt_id == NRF_BLE_SCAN_EVT_FILTER_MATCH;
        connect(&pAdv->peer_addr);
    }
}

static ret_code_t advertisingInit(lm_init_t const* pInit)
{
    ret_code_t errCode;
    ble_advertising_init_t init = {.config = advConfig};
    ble_uuid_t uuids[] =
    {
        {
            .uuid = BLE_UUID_HPS_SERVICE,
            .type = pInit->uuidType
        },
        {
            .uuid = BLE_UUID_LCS_SERVICE,
            .type = pInit->uuidType
        }
    };

    advState.advType                            = pInit->advType;

    init.advdata.name_type                      = BLE_ADVDATA_FULL_NAME;
    init.advdata.flags                          = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.advdata.uuids_more_available.uuid_cnt  = 1;
    init.advdata.uuids_more_available.p_uuids   = &uuids[0];
    init.srdata.include_appearance              = true;
    init.srdata.uuids_more_available.uuid_cnt   = 1;
    init.srdata.uuids_more_available.p_uuids    = &uuids[1];
    init.evt_handler                            = advEventHandler;
    init.error_handler                          = advErrorHandler;

    errCode = ble_advertising_init(&advInst, &init);
    if (errCode != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("advertising initialization error %d", errCode);
    }

    ble_advertising_conn_cfg_tag_set(&advInst, APP_BLE_CONN_CFG_TAG);

    errCode = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, advInst.adv_handle, 4);
    if (errCode != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("setting advertising power error %d", errCode);
    }

    openPeriphConnHandle = BLE_CONN_HANDLE_INVALID;

    return errCode;
}

static ret_code_t scanningInit(uint8_t uuidType)
{
    ret_code_t errCode;
    nrf_ble_scan_init_t init = {0};
    ble_uuid_t uuids[] =
    {
        {
            .uuid = BLE_UUID_HUMAN_INTERFACE_DEVICE_SERVICE,
            .type = BLE_UUID_TYPE_BLE
        },
        {
            .uuid = BLE_UUID_HPS_SERVICE,
            .type = uuidType
        },
        {
            .uuid = BLE_UUID_LCS_SERVICE,
            .type = uuidType
        }
    };

    pendingDevice.waitForDisconnect = BLE_CONN_HANDLE_INVALID;
    remoteConnHandle = BLE_CONN_HANDLE_INVALID;

    init.p_scan_param = &scanParams[SCAN_LOW_POWER];
    init.p_conn_param = &connParamsCentral;
    init.conn_cfg_tag = APP_BLE_CONN_CFG_TAG;

    errCode = nrf_ble_scan_init(&scanInst, &init, scanningEventHandler);
    if (errCode != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("scanning initialization error %d", errCode);
        return errCode;
    }

    for (uint_fast8_t i = 0; i < ARRAY_SIZE(uuids); i++)
    {
        errCode = nrf_ble_scan_filter_set(&scanInst, SCAN_UUID_FILTER, &uuids[i]);
        if (errCode != NRF_SUCCESS)
        {
            NRF_LOG_ERROR("adding scan filter error %d", errCode);
            return errCode;
        }
    }

    errCode = nrf_ble_scan_filters_enable(&scanInst, NRF_BLE_SCAN_ALL_FILTER, false);
    if (errCode != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("enable scan filters error %d", errCode);
    }
    return errCode;
}

/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void connParamsErrorHandler(uint32_t error)
{
    APP_ERROR_HANDLER(error);
}

/**@brief Function for initializing the Connection Parameters module.
 */
static ret_code_t connParamsInit(void)
{
    ret_code_t             errCode;
    ble_conn_params_init_t cpInit;

    memset(&cpInit, 0, sizeof(cpInit));

    cpInit.p_conn_params                  = (ble_gap_conn_params_t*)&connParamsPeriph[CONN_LOW_LATENCY];
    cpInit.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cpInit.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cpInit.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cpInit.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cpInit.disconnect_on_fail             = true;
    cpInit.evt_handler                    = NULL;
    cpInit.error_handler                  = connParamsErrorHandler;

    errCode = ble_conn_params_init(&cpInit);
    if (errCode != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("connection parameter initialization error %d", errCode);
    }
    return errCode;
}

/**@brief Function for handling Peer Manager events.
 *
 * @param[in] pEvt  Peer Manager event.
 */
static void pmEventHandler(pm_evt_t const * pEvt)
{
    ret_code_t errCode = NRF_SUCCESS;

    pm_handler_on_pm_evt(pEvt);
    pm_handler_flash_clean(pEvt);

    switch (pEvt->evt_id)
    {
        case PM_EVT_PEERS_DELETE_FAILED:
            errCode = pEvt->params.peers_delete_failed_evt.error;
            // fall through!
        case PM_EVT_PEERS_DELETE_SUCCEEDED:
            if (deleteHandler)
                deleteHandler(errCode);
            setScanMode(scanState.desired);     // nothing to scan for, but to generate event
            setAdvMode(advState.desired);       // to generate event and in case of open advertising
            break;
        case PM_EVT_CONN_SEC_CONFIG_REQ:
        {   // This happens if we are the peripheral and the central has lost its key. We allow
            // rebonding only in open advertise mode
            NRF_LOG_INFO("the other side lost its key")
            pm_conn_sec_config_t config =
            {
                .allow_repairing = advState.advType == BTLE_ADV_TYPE_ALWAYS_OPEN ||
                                   pEvt->conn_handle == openPeriphConnHandle   // allow rebonding only in open advertising mode
            };
            pm_conn_sec_config_reply(pEvt->conn_handle, &config);

        }   break;
        case PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED:
            pm_local_database_has_changed();
            break;
        case PM_EVT_CONN_SEC_FAILED:
            if (pEvt->params.conn_sec_failed.error == PM_CONN_SEC_ERROR_PIN_OR_KEY_MISSING)
            {   // this error occurs when connecting to a peripheral, which has lost its key.
                // If we are the central we can force rebonding if we are in the search state
                if (ble_conn_state_role(pEvt->conn_handle) == BLE_GAP_ROLE_CENTRAL && pendingDevice.isSearchDevice)
                {
                    NRF_LOG_INFO("peripheral lost its key, force rebonding");
                    errCode = pm_conn_secure(pEvt->conn_handle, true);    // force rebonding
                    if (errCode != NRF_SUCCESS)
                    {
                        NRF_LOG_WARNING("link secure with forced rebonding failed, error %d", errCode);
                    }
                    break;
                }
                // if we are in the peripheral role we cannot do anything, the central has to rebond, but we
                // only allow it in the open advertising mode
                else if (ble_conn_state_role(pEvt->conn_handle) == BLE_GAP_ROLE_PERIPH &&
                         (pEvt->conn_handle == openPeriphConnHandle))
                {
                    NRF_LOG_INFO("central tries to bond with key, but we don't have one");
                }
                else
                    pm_handler_disconnect_on_sec_failure(pEvt);
            }

            else if (pEvt->params.conn_sec_failed.error != PM_CONN_SEC_ERROR_DISCONNECT)
                pm_handler_disconnect_on_sec_failure(pEvt);
            break;
        default:
            break;
    }
}

/**@brief Function for the Peer Manager initialization.
 */
static ret_code_t peerManagerInit()
{
    ble_gap_sec_params_t secParams = secParam;
    ret_code_t           errCode;

    errCode = pm_init();
    if (errCode != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("peer manager initialization error %d", errCode);
        return errCode;
    }

    errCode = pm_sec_params_set(&secParams);
    if (errCode != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("setting security parameter error %d", errCode);
        return errCode;
    }

    errCode = pm_register(pmEventHandler);
    if (errCode != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("peer manager handler registration error %d", errCode);
        return errCode;
    }

    return errCode;
}

/** @brief handling disconnect events, just clears the connection handle and relays event
 *
 * @param connHandle
 * @return void
 */
static void onDisconnected(uint16_t connHandle, ble_gap_evt_disconnected_t const* pEvt)
{
    uint8_t role = ble_conn_state_role(connHandle);
    lm_evt_t evt;

    if (role == BLE_GAP_ROLE_PERIPH)
    {
        NRF_LOG_INFO("Peripheral disconnected, connection handle %d, reason %d", connHandle, pEvt->reason);

        if (connHandle == openPeriphConnHandle)
            openPeriphConnHandle = BLE_CONN_HANDLE_INVALID;
    }
    else if (role == BLE_GAP_ROLE_CENTRAL)
    {
        if (connHandle == remoteConnHandle)
        {
            remoteConnHandle = BLE_CONN_HANDLE_INVALID;
            NRF_LOG_INFO("Remote disconnected, connection handle %d, reason %d", connHandle, pEvt->reason);
        }
        else
        {
            NRF_LOG_INFO("Central disconnected, connection handle %d, reason %d", connHandle, pEvt->reason);
        }
    }

    // send a disconnect event
    if (eventHandler != NULL)
    {
        evt.type = LM_EVT_DISCONNECTED;
        evt.evt.conn.connHandle = connHandle;
        evt.evt.conn.role = role;
        eventHandler(&evt);
    }

    // check if peer deletion is pending
    if (deleteHandler)
    {   // wait until all devices are disconnected
        if (ble_conn_state_conn_count() == 0)
        {
            ret_code_t errCode = pm_peers_delete();
            if (errCode != NRF_SUCCESS)
            {   // inform main if peer deletion procedure can not be started
                deleteHandler(errCode);
            }
            deleteHandler = NULL;
        }
        return; // return to prevent restart of scanning
    }

    if (role == BLE_GAP_ROLE_CENTRAL)
    {
        // initiate connection to pending device
        if (connHandle == pendingDevice.waitForDisconnect)
        {
            connect(&pendingDevice.address);
            pendingDevice.waitForDisconnect = BLE_CONN_HANDLE_INVALID;
        }
        // otherwise restart scanning
        else if (scanState.actual != scanState.desired)
            (void)setScanMode(scanState.desired);   // error logging in setScanMode
    }
}

static void onConnected(uint16_t connHandle, ble_gap_evt_connected_t const* pConnected)
{
    uint8_t role = ble_conn_state_role(connHandle);
    lm_evt_t evt = {0};
    uint32_t errCode;

    evt.type = LM_EVT_CONNECTED;
    evt.evt.conn.connHandle = connHandle;
    evt.evt.conn.role = role;

    errCode = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_CONN, connHandle, 4);
    if (errCode != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("setting connection power error %d", errCode);
    }

    if (role == BLE_GAP_ROLE_CENTRAL)
    {
        if (memcmp(&pConnected->peer_addr, &pendingDevice.address, sizeof(pendingDevice.address)) != 0)
        {
            NRF_LOG_ERROR("unexpected device connected")
        }
        else
        {
            // initiate bonding if this link isn't bonded yet
            pm_conn_sec_status_t connStatus = {0};
            (void)pm_conn_sec_status_get(connHandle, &connStatus);
            if (!connStatus.bonded)
            {
                errCode = pm_conn_secure(connHandle, false);
                if (errCode != NRF_SUCCESS)
                {
                    NRF_LOG_WARNING("link secure failed, error %d", errCode);
                }
            }

            // add remote driver to event if this is a expected remote
            if (pendingDevice.pDriver != NULL)
            {
                evt.evt.conn.pRemDriver = pendingDevice.pDriver;
                remoteConnHandle = connHandle;
                NRF_LOG_INFO("Remote connected, connection handle %d", connHandle);
            }
            // nothing else to do for HPS or LCS device
            else
            {
                NRF_LOG_INFO("Central connected, connection handle %d", connHandle);
            }
        }

        // pending device connected, clear
        memset(&pendingDevice.address, 0, sizeof(pendingDevice.address));

        // advertising is deactivated during search, reactivate
        if (scanState.actual == SCAN_SEARCH)
            (void)setAdvMode(advState.desired);

        // scanner has to be deactivated to connect, so restart
        (void)setScanMode(scanState.desired); // error is logged inside setScanMode
    }
    else if (role == BLE_GAP_ROLE_PERIPH)
    {
        // in open advertising reset flag do defaults and save connection handle to identify this device later for bonding
        if (advState.isOpenAdvertising)
        {
            advState.isOpenAdvertising = advState.advType == BTLE_ADV_TYPE_ALWAYS_OPEN;
            openPeriphConnHandle = connHandle;
            // report changed advertising state
            if (advState.advType != BTLE_ADV_TYPE_ALWAYS_OPEN)
            {
                // the event can only change form LM_ADV_OPEN to LM_ADV_FAST
                sendAdvEvent(LM_ADV_FAST);
                advState.actual = LM_ADV_FAST;
            }
        }
        // otherwise check if the device is bonded, if not disconnect immediately (manual whitelisting)
        else
        {
            pm_peer_id_t peerId;
            (void)pm_peer_id_get(connHandle, &peerId);
            if (peerId == PM_PEER_ID_INVALID)
                disconnect(connHandle, NULL);
        }

        // deactivation of advertising on connection is not reported through event handler
        if (ble_conn_state_peripheral_conn_count() == NRF_SDH_BLE_PERIPHERAL_LINK_COUNT ||
            unconnectedDevices(BLE_GAP_ROLE_PERIPH) == 0)
        {
            sendAdvEvent(LM_ADV_OFF);
            advState.actual = LM_ADV_OFF;
        }

        NRF_LOG_INFO("Peripheral connected, connection handle %d", connHandle);
    }

    // send a connect event
    if (eventHandler != NULL)
        eventHandler(&evt);
}

/**@brief Function for handling BLE events.
 */
static void bleEventHandler(ble_evt_t const * pBleEvt, void * pContext)
{
    uint32_t errCode = NRF_SUCCESS;

    switch (pBleEvt->header.evt_id)
    {
        case BLE_GAP_EVT_DISCONNECTED:
            onDisconnected(pBleEvt->evt.gap_evt.conn_handle, &pBleEvt->evt.gap_evt.params.disconnected);
            break;

        case BLE_GAP_EVT_CONNECTED:
            onConnected(pBleEvt->evt.gap_evt.conn_handle, &pBleEvt->evt.gap_evt.params.connected);
            break;

        case BLE_GAP_EVT_TIMEOUT:
            if (pBleEvt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                NRF_LOG_INFO("connection timeout, restarting scanning");
                // reset pending device and restart scanner and advertising (if deactivated because of search)
                memset(&pendingDevice.address, 0, sizeof(pendingDevice.address));
                // restart advertising if it was deactivated for searching
                if (scanState.actual == LM_SCAN_SEARCH)
                    setAdvMode(advState.desired);
                // restart scanner
                setScanMode(scanState.desired);
            }

            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            errCode = sd_ble_gap_phy_update(pBleEvt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(errCode);
            break;
        }

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("connection parameter update request from the peer");
            // Accept parameters requested by peer.
            errCode = sd_ble_gap_conn_param_update(pBleEvt->evt.gap_evt.conn_handle,
                                        &pBleEvt->evt.gap_evt.params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(errCode);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout. Disconnecting...");
            errCode = sd_ble_gap_disconnect(pBleEvt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(errCode);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout. Disconnecting...");
            errCode = sd_ble_gap_disconnect(pBleEvt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(errCode);
            break;

        default:
            // No implementation needed.
            break;
    }
}

/* Public functions ----------------------------------------------------------*/
ret_code_t lm_Init(lm_init_t const* pInit)
{
    ret_code_t errCode;

    if (pInit == NULL)
        return NRF_ERROR_NULL;

    eventHandler = pInit->eventHandler;

    errCode = advertisingInit(pInit);
    if (errCode != NRF_SUCCESS)
        return errCode;

    uuidType = pInit->uuidType;
    errCode = scanningInit(pInit->uuidType);
    if (errCode != NRF_SUCCESS)
        return errCode;

    errCode = peerManagerInit();
    if (errCode != NRF_SUCCESS)
        return errCode;

    errCode = connParamsInit();
    if (errCode != NRF_SUCCESS)
        return errCode;

    NRF_SDH_BLE_OBSERVER(bleObserver, APP_BLE_OBSERVER_PRIO, bleEventHandler, NULL);

    return NRF_SUCCESS;
}

ret_code_t lm_SetExposureMode(lm_exposureMode_t mode)
{
    static bool started;    // indicator for BTLE_ADV_TYPE_STARTUP

    ret_code_t errCode = NRF_SUCCESS;
    lm_scanState_t scanModeReq = (lm_scanState_t)mode;
    lm_advState_t advModeReq = (lm_advState_t)mode;

    // update connection params if necessary
    if ((advModeReq <= LM_ADV_SLOW) != (advState.desired <= LM_ADV_SLOW))
    {
        ble_gap_conn_params_t const* pNewParams = &connParamsPeriph[advModeReq <= LM_ADV_SLOW ? CONN_LOW_POWER : CONN_LOW_LATENCY];
        ble_conn_state_conn_handle_list_t connHandles = ble_conn_state_periph_handles();
        for (uint32_t i = 0; i < connHandles.len; i++)
        {
            if (ble_conn_state_status(connHandles.conn_handles[i]) != BLE_CONN_STATUS_CONNECTED)
                continue;

            errCode = ble_conn_params_change_conn_params(connHandles.conn_handles[i], (ble_gap_conn_params_t*)pNewParams);
            if (errCode != NRF_SUCCESS && errCode != BLE_ERROR_INVALID_CONN_HANDLE) // ignore error if not connected
                NRF_LOG_ERROR("conn params change error %d", errCode);
        }
    }

    // there is no search mode for advertising (but advertising is restarted
    // without whitelist in scanning timeout handler), so use previous mode
    if (mode == LM_EXP_SEARCHING)
        advModeReq = advState.desired;

    // searching is just temporary, start searching but don't save
    if (scanModeReq == LM_SCAN_SEARCH && scanState.actual != LM_SCAN_SEARCH)
    {
        // stop advertising during search, to prevent unwanted connections
        (void)sd_ble_gap_adv_stop(advInst.adv_handle);
        sendAdvEvent(LM_ADV_OFF);
        advState.actual = LM_ADV_OFF;
        advState.desired = advModeReq;

        errCode = setScanMode(scanModeReq);

        return errCode; // advertising already handled for this case
    }
    // otherwise start scanning if not already in desired mode
    else if (scanModeReq != scanState.desired && scanModeReq != LM_SCAN_SEARCH)
    {
        scanState.desired = scanModeReq;
        errCode = setScanMode(scanModeReq);
    }

    // if startup advertising is selected override advertising on first call
    if (advState.advType == BTLE_ADV_TYPE_STARTUP && !started)
    {
        started = true;
        advState.desired = advModeReq;
        if (errCode != NRF_SUCCESS) // preserve error code
            (void)setAdvMode(LM_ADV_OPEN);
    }
    // change adv mode if necessary
    else if (advState.desired != advModeReq)
    {
        advState.desired = advModeReq;
        if (errCode != NRF_SUCCESS) // preserve error code
            (void)setAdvMode(advModeReq);
        else
            errCode = setAdvMode(advModeReq);
    }

    // finally disconnect devices if shutting down
    if (mode == LM_EXP_OFF)
        ble_conn_state_for_each_connected(disconnect, NULL);

    // no log, because already logging in setScanMode and setAdvMode

    return errCode;
}

lm_advState_t lm_GetAdvertisingState()
{
    return advState.actual;
}

lm_scanState_t lm_GetScanningState()
{
    return scanState.actual;
}

uint8_t lm_GetPeriphCnt()
{
    return ble_conn_state_peripheral_conn_count();
}

uint8_t lm_GetCentralCnt()
{
    return ble_conn_state_central_conn_count();
}

ble_gap_conn_params_t const* lm_getPreferredConnParams()
{
    return &connParamsPeriph[CONN_LOW_LATENCY];
}

void lm_disconnectAll()
{
    (void)ble_conn_state_for_each_connected(disconnect, NULL);
}

void lm_diconnectAndIgnoreRemote(bool enable)
{
    if (enable)
        scanState.isPeriphControl++;
    else if (scanState.isPeriphControl)
        scanState.isPeriphControl--;

    if (enable && remoteConnHandle != BLE_CONN_HANDLE_INVALID)
        disconnect(remoteConnHandle, NULL);
}

ret_code_t lm_DeleteBonds(ds_reportHandler_t pHandler)
{
    ret_code_t errCode = NRF_SUCCESS;

    // stop scanning and advertising to prevent any new connections
    // not updating desired modes, to be able to restart after successful deletion
    nrf_ble_scan_stop();
    (void)setAdvMode(LM_ADV_OFF);

    deleteHandler = pHandler;

    // if any devices are connected, they need to be disconnected first
    uint32_t connCount = ble_conn_state_for_each_connected(disconnect, NULL);

    // if no devices are connected, content can be deleted immediately
    if (!connCount)
    {
        errCode = pm_peers_delete();
        if (errCode != NRF_SUCCESS)
        {
            deleteHandler(errCode);
            deleteHandler = NULL;
        }
    }

    return errCode;
}

/**END OF FILE*****************************************************************/
