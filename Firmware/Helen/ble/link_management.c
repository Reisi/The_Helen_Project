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
#include "ble_lcs_c.h"
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
    bool useWhitelistAdvertising;
} advState_t;

typedef struct
{
    ble_gap_addr_t  address;    // the address of the remote to be connected
    rem_driver_t const* pDriver;// the used driver
    uint16_t waitForDisconnect; // the connection handle of the device, that has to be disconnected first
} remoteConnecting_t;

typedef struct
{
    lm_scanState_t actual;
    lm_scanState_t desired;
    bool isPeriphControl;
} scanState_t;

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

static advState_t advState;                 // the advertising state
static scanState_t scanState;               // the scanning state
static remoteConnecting_t pendingRemote;    // information of the remote device helen is trying to connect to
static uint8_t lcsUuidType;                 // the uuid type of the light control service
static uint16_t remoteConnHandle;           // the connection handle of the currently connected remote
static lm_eventHandler_t eventHandler;      // the event handler
static ds_reportHandler_t deleteHandler;    // the deletion report handler, only set if a deletion is pending

/* Private functions ---------------------------------------------------------*/

static void connect(ble_gap_addr_t const* pAddr)
{
    uint32_t errCode;

    // scanning has to be stopped to connect
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

    ret_code_t err_code = sd_ble_gap_disconnect(connHandle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_WARNING("Failed to disconnect connection. Connection handle: %d Error: %d", connHandle, err_code);
    }
    else
    {
        NRF_LOG_INFO("Disconnecting connection handle %d", connHandle);
    }
}

/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void advEventHandler(ble_adv_evt_t advEvt)
{
    switch (advEvt)
    {
        case BLE_ADV_EVT_FAST:
            advState.actual = LM_ADV_FAST;
            break;

        case BLE_ADV_EVT_SLOW:
            advState.actual = LM_ADV_SLOW;
            break;

        case BLE_ADV_EVT_FAST_WHITELIST:
            advState.actual = LM_ADV_FAST_WHITELIST;
            break;

        case BLE_ADV_EVT_SLOW_WHITELIST:
            advState.actual = LM_ADV_SLOW_WHITELIST;
            break;

        case BLE_ADV_EVT_IDLE:
            advState.actual = LM_ADV_OFF;
            break;

        case BLE_ADV_EVT_WHITELIST_REQUEST:
        {
            ble_gap_addr_t addrs[8];
            ble_gap_irk_t irks[8];
            uint32_t errCode, addrCnt = ARRAY_SIZE(addrs), irkCnt = ARRAY_SIZE(irks);
            errCode = pm_whitelist_get(addrs, &addrCnt, irks, &irkCnt);
            if (errCode != NRF_SUCCESS)
            {
                NRF_LOG_WARNING("whitelist fetching error &d", errCode);
            }
            errCode = ble_advertising_whitelist_reply(&advInst, addrs, addrCnt, irks, irkCnt);
            if (errCode != NRF_SUCCESS)
            {
                NRF_LOG_WARNING("whitelist fetching error &d", errCode);
            }
        }   return;

        default:
            return;
    }

    NRF_LOG_INFO("advertising state changed, desired: %d, actual %d", advState.desired, advState.actual);

    if (eventHandler != NULL)
    {
        lm_evt_t evt;
        evt.type = LM_EVT_ADV_MODE_CHANGED;
        evt.evt.newAdvState = advState.actual;
        eventHandler(&evt);
    }
}

static void advErrorHandler(uint32_t errCode)
{
    NRF_LOG_ERROR("advertising error %d", errCode);
}

/** @brief function to set the desired advertising mode
 *
 * @note the actual used advertising mode is reported through the event handler
 *
 * @param[in] mode the desired mode
 * @param[in] deactivateWhitelist  true if whitelist should be disabled until next change
 * @return the return value of ble_advertising_start
 */
static ret_code_t setAdvMode(lm_advState_t mode, bool deactivateWhitelist)
{
    ble_adv_modes_config_t config = advConfig;
    ble_adv_mode_t advMode = BLE_ADV_MODE_IDLE;
    ret_code_t errCode = NRF_SUCCESS;

    // convert BTLE_ADV_... modes to BLE_ADV_... modes
    if (mode == LM_ADV_FAST)
        advMode = BLE_ADV_MODE_FAST;
    else if (mode == LM_ADV_SLOW)
        advMode = BLE_ADV_MODE_SLOW;
    else if (mode != LM_ADV_OFF)
        return NRF_ERROR_INVALID_PARAM;

    // stop advertising to be able to restart with different settings
    (void)sd_ble_gap_adv_stop(advInst.adv_handle);

    // not using ble_advertising_restart_without_whitelist() because whitelist
    // would be disabled until next disconnect event, I want it enabled
    // automatically the next time advertising is changed.
    config.ble_adv_whitelist_enabled = advState.useWhitelistAdvertising;
    advInst.whitelist_temporarily_disabled = deactivateWhitelist;

    if (mode == LM_ADV_OFF)
    {   // prevent module from restarting if a device disconnects
        config.ble_adv_on_disconnect_disabled = true;
        ble_advertising_modes_config_set(&advInst, &config);
    }
    else
    {
        ble_advertising_modes_config_set(&advInst, &config);
        errCode = ble_advertising_start(&advInst, advMode);
        if (errCode == NRF_ERROR_CONN_COUNT)    /// TODO: maybe catch this case and don't even try to start advertising
            errCode = NRF_SUCCESS;
        /// TODO: advertising is started without whitelist if no devices are stored
    }

    // make sure whitelist is used the next time
    advInst.whitelist_temporarily_disabled = false;

    if (errCode != NRF_SUCCESS && errCode != NRF_ERROR_CONN_COUNT)  // no advertising when already connected
    {
        NRF_LOG_ERROR("setting advertising mode error %d", errCode);
    }

    return errCode;
}

static bool setWhitelist()
{
    uint32_t errCode, cnt = 0;
    pm_peer_id_t peers[8], id = pm_next_peer_id_get(PM_PEER_ID_INVALID);

    while (id != PM_PEER_ID_INVALID && cnt < ARRAY_SIZE(peers))
    {
        peers[cnt++] = id;
        id = pm_next_peer_id_get(id);
    }

    if (cnt == 0)
        return false;

    errCode = pm_whitelist_set(peers, cnt);
    APP_ERROR_CHECK(errCode);

    // advertising and scanning needs to be disabled to change this
    //errCode = pm_device_identities_list_set(peers, cnt);
    //APP_ERROR_CHECK(errCode);

    return true;
}

static pm_peer_id_t nextCentralIdGet(pm_peer_id_t peerId)
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
        else if (peerData.own_role == BLE_GAP_ROLE_CENTRAL)
            break;
    }
    return peerId;
}

/**< returns the number of stored central devices that are not connected */
static uint8_t devicesToScanFor()
{
    /// TODO: how to exclude remotes if device is under peripheral control?
    ///       don't forget to reset scanning if this peripheral disconnects

    uint8_t cnt = 0;
    pm_peer_id_t peer = nextCentralIdGet(PM_PEER_ID_INVALID);

    while (peer != PM_PEER_ID_INVALID)
    {
        uint16_t connHandle;
        APP_ERROR_CHECK(pm_conn_handle_get(peer, &connHandle));
        if (ble_conn_state_status(connHandle) != BLE_CONN_STATUS_CONNECTED)
            cnt++;
        peer = nextCentralIdGet(peer);
    }

    return cnt;
}

static ret_code_t setScanMode(lm_scanState_t mode)
{
    /// TODO: how to handle searching, if no connections are available?

    ret_code_t errCode = NRF_SUCCESS;
    bool devicesAvailable = (bool)devicesToScanFor();
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

    scanState.actual = newMode;
    NRF_LOG_INFO("scan state changed, desired: %d, actual %d", scanState.desired, scanState.actual);

    if (eventHandler != NULL)
    {
        lm_evt_t evt;
        evt.type = LM_EVT_SCAN_MODE_CHANGED;
        evt.evt.newAdvState = scanState.actual;
        eventHandler(&evt);
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

static bool isLcs(uint8_t const* pData, uint16_t len)
{
    ble_uuid_t lcsServiceUuid;
    lcsServiceUuid.uuid = BLE_UUID_LCS_SERVICE;
    lcsServiceUuid.type = lcsUuidType;

    return ble_advdata_uuid_find(pData, len, &lcsServiceUuid);
}

static void scanningEventHandler(scan_evt_t const* pScanEvt)
{
    ble_gap_evt_adv_report_t const* pAdv;

    switch(pScanEvt->scan_evt_id)
    {
    case NRF_BLE_SCAN_EVT_WHITELIST_REQUEST:
        /// TODO: maybe setting the whitelist is only necessary if a new device is added?
        setWhitelist(); /// TODO: stop advertising?
        break;

    case NRF_BLE_SCAN_EVT_SCAN_TIMEOUT:
        // restart previous scan mode after searching
        (void)setScanMode(scanState.desired);
        // start advertising without whitelist
        if (advState.useWhitelistAdvertising)
            (void)setAdvMode(LM_ADV_FAST, true);
        break;

    case NRF_BLE_SCAN_EVT_FILTER_MATCH:
        pAdv = pScanEvt->params.filter_match.p_adv_report;
    case NRF_BLE_SCAN_EVT_WHITELIST_ADV_REPORT:
        pAdv = pScanEvt->params.p_whitelist_adv_report;
        // check if the device is a remote
        rem_driver_t const* pDriver = isRemote(pAdv->data.p_data, pAdv->data.len);
        if (pDriver != NULL)
        {
            NRF_LOG_INFO("remote control advertising report received");

            if (!scanState.isPeriphControl) // don't connect if under peripheral control
            {
                pendingRemote.pDriver = pDriver;
                pendingRemote.address = pAdv->peer_addr;
                if (remoteConnHandle == BLE_CONN_HANDLE_INVALID)
                    connect(&pAdv->peer_addr); // connect if no remote is connected yet
                else if (pScanEvt->scan_evt_id == NRF_BLE_SCAN_EVT_FILTER_MATCH)
                {                   // disconnect existing remote if a new one was found while searching
                    disconnect(remoteConnHandle, NULL);
                    pendingRemote.waitForDisconnect = remoteConnHandle;
                }
            }
        }
        // check if device is another light
        if (isLcs(pAdv->data.p_data, pAdv->data.len))
        {
            NRF_LOG_INFO("light control advertising report received");
            connect(&pAdv->peer_addr);
        }
        break;

    default:
        break;
    }
}

static ret_code_t advertisingInit(lm_init_t const* pInit)
{
    ret_code_t errCode;
    ble_advertising_init_t init = {.config = advConfig};
    ble_uuid_t uuids[] =
    {
        {
            .uuid = BLE_UUID_LCS_SERVICE,
            .type = pInit->lcsUuidType
        }
    };

    advState.useWhitelistAdvertising = pInit->useWhitelistAdvertising;

    init.advdata.name_type                      = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance             = false;
    init.advdata.flags                          = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.advdata.uuids_more_available.uuid_cnt  = ARRAY_SIZE(uuids);
    init.advdata.uuids_more_available.p_uuids   = uuids;
    init.config.ble_adv_whitelist_enabled       = pInit->useWhitelistAdvertising;
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

    return errCode;
}

static ret_code_t scanningInit(uint8_t lcsUuidType)
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
            .uuid = BLE_UUID_LCS_SERVICE,
            .type = lcsUuidType
        }
    };

    pendingRemote.waitForDisconnect = BLE_CONN_HANDLE_INVALID;
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
            setAdvMode(advState.desired, false);// to generate event and in case of non whitelist advertising
            break;
        case PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED:
            pm_local_database_has_changed();
            break;
        case PM_EVT_CONN_SEC_FAILED:
            if (pEvt->params.conn_sec_failed.error != PM_CONN_SEC_ERROR_DISCONNECT)
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
static void onDisconnected(uint16_t connHandle)
{
    uint8_t role = ble_conn_state_role(connHandle);
    lm_evt_t evt;

    // check if peer deletion is pending
    if (deleteHandler)
    {   // wait until all devices are disconnected
        if (ble_conn_state_conn_count() == 0)
        {
            ret_code_t errCode = pm_peers_delete();
            if (errCode != NRF_SUCCESS)
            {   // inform main if peer deletion procedure can not be started
                deleteHandler(errCode);
                deleteHandler = NULL;
            }
        }
        return; // return to prevent restart of scanning
    }

    if (role == BLE_GAP_ROLE_PERIPH)
    {
        // is there anything to do?
        NRF_LOG_INFO("Peripheral disconnected, connection handle %d", connHandle);
    }
    else if (role == BLE_GAP_ROLE_CENTRAL)
    {
        if (connHandle == remoteConnHandle)
        {
            remoteConnHandle = BLE_CONN_HANDLE_INVALID;
            NRF_LOG_INFO("Remote disconnected, connection handle %d", connHandle);
        }
        else
        {
            NRF_LOG_INFO("Central disconnected, connection handle %d", connHandle);
        }

        // initiate connection to pending device
        if (connHandle == pendingRemote.waitForDisconnect)
        {
            connect(&pendingRemote.address);
            pendingRemote.waitForDisconnect = BLE_CONN_HANDLE_INVALID;
        }
        // otherwise restart scanning
        else if (scanState.actual != scanState.desired)
            (void)setScanMode(scanState.desired);   // error logging in setScanMode
    }

    // send a disconnect event
    if (eventHandler != NULL)
    {
        evt.type = LM_EVT_DISCONNECTED;
        evt.evt.conn.connHandle = connHandle;
        evt.evt.conn.role = role;
        eventHandler(&evt);
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
        // initiate bonding if this is a new device
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
        if (memcmp(&pConnected->peer_addr, &pendingRemote.address, sizeof(pendingRemote.address)) == 0)
        {
            evt.evt.conn.pRemDriver = pendingRemote.pDriver;
            memset(&pendingRemote.address, 0, sizeof(pendingRemote.address));
            remoteConnHandle = connHandle;
            NRF_LOG_INFO("Remote connected, connection handle %d", connHandle);
        }
        else
        {
            NRF_LOG_INFO("Central connected, connection handle %d", connHandle);
        }
        // scanner has to deactivated to connect, so restart
        (void)setScanMode(scanState.desired); // error is logged inside setScanMode
    }
    else if (role == BLE_GAP_ROLE_PERIPH)
    {
        // deactivation of advertising on connection is not reported through event handler ans
        // as long as only one peripheral link is supported advertising is definitely off when
        // a device is connected
        advState.actual = LM_ADV_OFF;
        if (eventHandler != NULL)
        {
            lm_evt_t advEvt;
            advEvt.type = LM_EVT_ADV_MODE_CHANGED;
            advEvt.evt.newAdvState = advState.actual;
            eventHandler(&advEvt);
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
            onDisconnected(pBleEvt->evt.gap_evt.conn_handle);
            break;

        case BLE_GAP_EVT_CONNECTED:
            onConnected(pBleEvt->evt.gap_evt.conn_handle, &pBleEvt->evt.gap_evt.params.connected);
            break;

        case BLE_GAP_EVT_TIMEOUT:
            if (pBleEvt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {   // scanning was stopped for connection, restart
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

    lcsUuidType = pInit->lcsUuidType;
    errCode = scanningInit(pInit->lcsUuidType);
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
    /// TODO: handle advertising with temporary disabled whitelist if scan search is not available

    ret_code_t errCode = NRF_SUCCESS;
    lm_scanState_t scanModeReq = (lm_scanState_t)mode;
    lm_advState_t advModeReq = (lm_advState_t)mode;

    // there is no search mode for advertising (but advertising is restarted
    // without whitelist in scanning timeout handler), so use previous mode
    if (mode == LM_EXP_SEARCHING)
        advModeReq = advState.desired;

    // searching is just temporary, start searching but don't save
    if (scanModeReq == LM_SCAN_SEARCH && scanState.actual != LM_SCAN_SEARCH)
    {
        errCode = setScanMode(scanModeReq);
    }
    // otherwise start scanning if not already in desired mode
    else if (scanModeReq != scanState.desired)
    {
        scanState.desired = scanModeReq;
        errCode = setScanMode(scanModeReq);
    }

    // change adv mode if necesary
    if (advState.desired != advModeReq)
    {
        advState.desired = advModeReq;
        if (errCode != NRF_SUCCESS) // preserve error code
            (void)setAdvMode(advModeReq, false);
        else
            errCode = setAdvMode(advModeReq, false);
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
    scanState.isPeriphControl = enable;
    if (enable && remoteConnHandle != BLE_CONN_HANDLE_INVALID)
        disconnect(remoteConnHandle, NULL);
}

ret_code_t lm_DeleteBonds(ds_reportHandler_t pHandler)
{
    ret_code_t errCode = NRF_SUCCESS;

    // stop scanning and advertising to prevent any new connections
    // not updating desired modes, to be able to restart after successful deletion
    nrf_ble_scan_stop();
    (void)setAdvMode(LM_ADV_OFF, false);

    deleteHandler = pHandler;

    // if any devices are connected, they need to be disconnected first
    uint32_t connCount = ble_conn_state_for_each_connected(disconnect, NULL);

    // if no devices are connected, content can be deleted immediately
    if (!connCount)
        errCode = pm_peers_delete();

    return errCode;
}

/**END OF FILE*****************************************************************/
