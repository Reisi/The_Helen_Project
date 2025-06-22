/**
  ******************************************************************************
  * @file    btle.c
  * @author  Thomas Reisnecker
  * @brief   bluetooth module
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "nrf_ble_gatt.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_state.h"
#include "ble_dfu.h"
#include "ble_dis.h"
#include "ble_hids_c.h"
#include "app_timer.h"
#include "nrf_ble_scan.h"
#include "nrf_ble_qwr.h"

#include "debug.h"
#include "btle.h"
#include "remote.h"
#include "ble_hps.h"
#include "ble_hps_c.h"
//#include "ble_lcs.h"
//#include "ble_lcs_c.h"
#include "link_management.h"
#include "version.h"

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
NRF_LOG_MODULE_REGISTER();

/* Private typedef -----------------------------------------------------------*/

/* Private defines -----------------------------------------------------------*/
#define NRF_BLE_GQ_QUEUE_SIZE 8
#define QWR_BUFFER_SIZE       128

#ifndef BLE_BTLE_BLE_OBSERVER_PRIO
#define BLE_BTLE_BLE_OBSERVER_PRIO   2
#endif // BLE_BTLE_BLE_OBSERVER_PRIO

/* Section Variable ----------------------------------------------------------*/
NRF_SECTION_DEF(btle_service, btle_service_t);
#define BTLE_SERVICE_CNT()      NRF_SECTION_ITEM_COUNT(btle_service, btle_service_t)
#define BTLE_SERVICE_GET(cnt)   NRF_SECTION_ITEM_GET(btle_service, btle_service_t, cnt)

/* Private functions forward declaration -------------------------------------*/
static void onBleEventHandler(ble_evt_t const * pBleEvt, void * pContext);

/* Private variables ---------------------------------------------------------*/
NRF_BLE_GATT_DEF(gattInst);                             // gatt instance
//BLE_LCS_DEF(lcsGattS, NRF_SDH_BLE_TOTAL_LINK_COUNT);    // light control service server
BLE_HPS_DEF(hpsGattS, NRF_SDH_BLE_TOTAL_LINK_COUNT);    // Helen Project Service server
NRF_BLE_QWR_DEF(qwrInst);                               // The queued writes instance

BLE_DB_DISCOVERY_DEF(discInst);                         // database discovery
NRF_BLE_GQ_DEF(gattQueueInst, NRF_SDH_BLE_CENTRAL_LINK_COUNT, NRF_BLE_GQ_QUEUE_SIZE); // gatt queued writes module
BLE_HIDS_C_DEF(hidsInst);                               // hid service client
//BLE_LCS_C_DEF(lcsGattC, NRF_SDH_BLE_TOTAL_LINK_COUNT);  // light control service client
BLE_HPS_C_DEF(hpsGattC, NRF_SDH_BLE_TOTAL_LINK_COUNT);  // helen project service client

NRF_SDH_BLE_OBSERVER(btle_obs, BLE_BTLE_BLE_OBSERVER_PRIO, onBleEventHandler, NULL);// registering the btle event observer
static btle_eventHandler_t pEventHandler;               // the event handler to report events related to this module
static ble_hps_evt_handler_t pHpsServerHandler;         // the event handler

/* Private functions ---------------------------------------------------------*/
/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static ret_code_t bleStackInit()
{
    ret_code_t errCode;

    errCode = nrf_sdh_enable_request();
    LOG_ERROR_CHECK("softdevice handler enable request error %d", errCode);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ramStart = 0;
    errCode = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ramStart);
    LOG_ERROR_CHECK("setting stack configuration error %d", errCode);

    // Enable BLE stack.
    errCode = nrf_sdh_ble_enable(&ramStart);
    if (errCode != NRF_SUCCESS)
    LOG_ERROR_CHECK("softdevice enable error %d", errCode);

    return NRF_SUCCESS;
}

/**@brief Function for the GAP initialization.
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static ret_code_t gapParamsInit(btle_info_t const* pInfo, uint16_t maxDeviceNameLenght)
{
    ret_code_t              errCode;
    ble_gap_conn_params_t   gapConnParams = *lm_getPreferredConnParams();
    ble_gap_conn_sec_mode_t secMode;

    if (maxDeviceNameLenght > BLE_GAP_DEVNAME_DEFAULT_LEN)
    {
        NRF_LOG_ERROR("maximum device name too long");
        return NRF_ERROR_INVALID_LENGTH;
    }
    else if (maxDeviceNameLenght)
        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&secMode);
    else
        BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&secMode);

    errCode = sd_ble_gap_device_name_set(&secMode,
                                          (const uint8_t *)pInfo->pDevicename,
                                          strlen(pInfo->pDevicename));
    if (errCode != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("setting device name error %d", errCode)
        return errCode;
    }

    errCode = sd_ble_gap_appearance_set(pInfo->deviceAppearance);
    if (errCode != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("setting device appearance error %d", errCode)
        return errCode;
    }

    errCode = sd_ble_gap_ppcp_set(&gapConnParams);
    if (errCode != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("setting device preferred connection parameters error %d", errCode)
        return errCode;
    }

    return NRF_SUCCESS;
}

/**@brief Function for handling dfu events from the Buttonless Secure DFU service
 */
static void dfrServerHandler(ble_dfu_buttonless_evt_type_t event)
{
    switch (event)
    {
        case BLE_DFU_EVT_BOOTLOADER_ENTER_PREPARE:
            NRF_LOG_INFO("Device is preparing to enter bootloader mode.");
            (void)lm_SetExposureMode(LM_EXP_OFF);   // shut off scanning and advertising to prevent new connections
            lm_disconnectAll();                     // and disconnect
            break;

        case BLE_DFU_EVT_BOOTLOADER_ENTER:
            NRF_LOG_INFO("Device will enter bootloader mode.");
            break;

        case BLE_DFU_EVT_BOOTLOADER_ENTER_FAILED:
            NRF_LOG_ERROR("Request to enter bootloader mode failed asynchroneously.");
            break;

        case BLE_DFU_EVT_RESPONSE_SEND_ERROR:
            NRF_LOG_ERROR("Request to send a response to client failed.");
            APP_ERROR_CHECK(false);
            break;

        default:
            NRF_LOG_ERROR("Unknown event from ble_dfu_buttonless.");
            break;
    }
}

/** @brief light control service server event handler
 *  @details monitors if indications has been enabled from a peripheral connection
 */
/*static void lcsServerHandler(ble_lcs_evt_t* pEvt)
{
    if (ble_conn_state_role(pEvt->conn_handle) != BLE_GAP_ROLE_PERIPH)
        return;

    static bool indicEnabled;

    switch (pEvt->evt_type)
    {
    case BLE_LCS_EVT_LCP_INDICATION_ENABLED:
        if (!indicEnabled)
        {
            NRF_LOG_INFO("device is under peripheral control (LCS), disabling remote control");
            lm_diconnectAndIgnoreRemote(true);
            indicEnabled = true;
        }
        break;
    case BLE_LCS_EVT_LCP_INDICATION_DISABLED:
        if (indicEnabled)
        {
            NRF_LOG_INFO("device is no longer under peripheral control (LCS), enabling remote control");
            lm_diconnectAndIgnoreRemote(false);
            indicEnabled = false;
        }
        break;
    default:
        break;
    }
}*/

static bool hpsServerHandler(ble_hps_evt_t const * pEvt)
{
    static bool indicEnabled;

    // check if control point indications have been en/disbaled
    if (ble_conn_state_role(pEvt->p_client->conn_handle) == BLE_GAP_ROLE_PERIPH)
    {
        switch (pEvt->evt_type)
        {
        case BLE_HPS_EVT_CP_INDICATION_ENABLED:
            if (!indicEnabled)
            {
                NRF_LOG_INFO("device is under peripheral control (HPS), disabling remote control");
                lm_diconnectAndIgnoreRemote(true);
                indicEnabled = true;
            }
            break;
        case BLE_HPS_EVT_CP_INDICATION_DISABLED:
            if (indicEnabled)
            {
                NRF_LOG_INFO("device is no longer under peripheral control (HPS), enabling remote control");
                lm_diconnectAndIgnoreRemote(false);
                indicEnabled = false;
            }
            break;
        default:
            break;
        }
    }

    // relay event to main
    if (pHpsServerHandler != NULL)
        return pHpsServerHandler(pEvt);
    else
        return false;
}

static void hpsServerErrorHandler(uint32_t errCode)
{
    NRF_LOG_ERROR("hps error %d", errCode);
}

static void qwrErrorHandler(uint32_t errCode)
{
    NRF_LOG_ERROR("qwr error %d", errCode);
}

static uint16_t qwrHandler(nrf_ble_qwr_t * pQwr, nrf_ble_qwr_evt_t * pEvt)
{
    return ble_hps_on_qwr_evt(&hpsGattS, pQwr, pEvt);
}

/**@brief Function for initializing services that will be used by the application.
 */
static ret_code_t gattServerInit(ble_user_mem_block_t const* pBuf, ble_hps_init_t* pHpsInit,/* ble_lcs_init_t* pLcsInit,*/ btle_info_t const* pInfo)
{
    ret_code_t                errCode;
    nrf_ble_qwr_init_t        qwrInit = {0};
    ble_dfu_buttonless_init_t dfusInit = {0};
    ble_dis_init_t            disInit = {0};

    // initialize the queued writes module
    if (pBuf != NULL)
    {
        qwrInit.mem_buffer = *pBuf;
        qwrInit.error_handler = qwrErrorHandler;
        qwrInit.callback = qwrHandler;
        errCode = nrf_ble_qwr_init(&qwrInst, &qwrInit);
        if (errCode != NRF_SUCCESS)
        {
            NRF_LOG_ERROR("qwr init error %d", errCode);
            return errCode;
        }
    }

    // initialize the device information service
    if (pInfo->pManufacturer != NULL)
        ble_srv_ascii_to_utf8(&disInit.manufact_name_str, (char*)pInfo->pManufacturer);
    if (pInfo->pModelNumber != NULL)
        ble_srv_ascii_to_utf8(&disInit.model_num_str, (char*)pInfo->pModelNumber);
    if (pInfo->pBoardHWVersion != NULL)
        ble_srv_ascii_to_utf8(&disInit.hw_rev_str, (char*)pInfo->pBoardHWVersion);
    ble_srv_ascii_to_utf8(&disInit.fw_rev_str, VERSION_STRING);
    disInit.dis_char_rd_sec = SEC_OPEN;
    errCode = ble_dis_init(&disInit);
    if (errCode != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("device information service init error %d", errCode);
        return errCode;
    }

    // Initialize the lcs service
    /*if (pLcsInit != NULL)
    {
        pLcsInit->lcs_lm_cccd_wr_sec = SEC_OPEN;
        pLcsInit->lcs_lf_rd_sec = SEC_OPEN;
#ifdef DEBUG_EXT
#pragma message ( "debug configuration set to just works temporary" )
        pLcsInit->lcs_lcp_cccd_wr_sec = SEC_JUST_WORKS;//SEC_OPEN;
        pLcsInit->lcs_lcp_wr_sec = SEC_JUST_WORKS;//SEC_OPEN;
#else
        pLcsInit->lcs_lcp_cccd_wr_sec = SEC_JUST_WORKS;
        pLcsInit->lcs_lcp_wr_sec = SEC_JUST_WORKS;
#endif // DEBUG_EXT
        pLcsInit->evt_handler = lcsServerHandler;
        errCode = ble_lcs_init(&lcsGattS, ARRAY_SIZE(lcsGattS_client_data), pLcsInit);
        if (errCode != NRF_SUCCESS)
        {
            NRF_LOG_ERROR("light control service init error %d", errCode);
            return errCode;
        }
    }*/

    // Initialize the hp service
    if (pHpsInit != NULL)
    {
#ifdef DEBUG_EXT
#pragma message ( "debug configuration set to just works temporary" )
        pHpsInit->modes_wr_sec = SEC_JUST_WORKS;//SEC_OPEN;
        pHpsInit->cp_wr_sec = SEC_JUST_WORKS;//SEC_OPEN;
#else
        pHpsInit->modes_wr_sec = SEC_JUST_WORKS;
        pHpsInit->cp_wr_sec = SEC_JUST_WORKS;
#endif // DEBUG_EXT
        pHpsServerHandler = pHpsInit->evt_handler;  // save handler, we need to listen to events here, too
        pHpsInit->evt_handler = hpsServerHandler;
        pHpsInit->error_handler = hpsServerErrorHandler;
        pHpsInit->p_qwr_ctx = qwrInst.initialized ? &qwrInst : NULL;
        errCode = ble_hps_init(&hpsGattS, ARRAY_SIZE(hpsGattS_client_data), pHpsInit);
        if (errCode != NRF_SUCCESS)
        {
            NRF_LOG_ERROR("helen project service init error %d", errCode);
            return errCode;
        }
    }

    // Initialize dfu service
    dfusInit.evt_handler = dfrServerHandler;
    errCode = ble_dfu_buttonless_init(&dfusInit);
    if (errCode != NRF_SUCCESS && errCode != NRF_ERROR_NOT_FOUND)
    {
        NRF_LOG_ERROR("dfu service init error %d", errCode);
        return errCode;
    }

    // add board specific services
    for (uint8_t i = 0; i < BTLE_SERVICE_CNT(); i++)
    {
        btle_service_t const* pBrdService = BTLE_SERVICE_GET(i);
        errCode = pBrdService->serviceAdd(pBrdService->pContext);
        if (errCode != NRF_SUCCESS)
            return errCode;
    }

    return NRF_SUCCESS;
}

/** @brief database discovery event handler
 */
static void dbDiscoveryHandler(ble_db_discovery_evt_t * pEvt)
{
    // hids handles are assigned manually
    //if (pEvt->conn_handle == hidsInst.conn_handle)
    //    ble_hids_c_on_db_disc_evt(&hidsInst, pEvt);

    //ble_lcs_c_on_db_disc_evt(&lcsGattC, pEvt);
    ble_hps_c_on_db_disc_evt(&hpsGattC, pEvt);
}

/** @brief hid service client error handler
 */
static void hidsClientErrorHandler(uint32_t errCode)
{
    APP_ERROR_HANDLER(errCode);
}

/*static void lcsClientErrorHandler(ble_lcs_c_t * pLcsC, ble_lcs_c_error_evt_t * pEvt)
{

}

static void lcsClientHandler(ble_lcs_c_t * pLcsC, ble_lcs_c_evt_t * pEvt)
{
    uint32_t errCode;

    if (pEvt->evt_type == BLE_LCS_C_EVT_DISCOVERY_COMPLETE)
    {
        NRF_LOG_INFO("lcs at peer detected");
        // handles are automatically assigned
        //errCode = ble_lcs_c_handles_assign(pLcsC, pEvt->conn_handle, &pEvt->data.lcs_db);
        //APP_ERROR_CHECK(errCode);
        // enable indications to be able to relay mode changes
        errCode = ble_lcs_c_cp_indic_enable(pLcsC, pEvt->conn_handle, true);
        APP_ERROR_CHECK(errCode);
    }
}*/

static void hpsClientErrorHandler(uint32_t errCode)
{
    NRF_LOG_ERROR("hps client error %d", errCode);
}

static void hpsClientHandler(ble_hps_c_t * pHpsC, ble_hps_c_evt_t * pEvt)
{
    uint32_t errCode;

    if (pEvt->evt_type == BLE_HPS_C_EVT_DISCOVERY_COMPLETE)
    {
        NRF_LOG_INFO("hps at peer detected");
        // handles are automatically assigned
        //errCode = ble_hps_c_handles_assign(pHpsC, pEvt->conn_handle, &pEvt->data.hps_db);
        //APP_ERROR_CHECK(errCode);
        // enable indications to be able to relay mode changes

        errCode = ble_hps_c_cp_indic_enable(pHpsC, pEvt->conn_handle, true);
        if (errCode != NRF_SUCCESS && errCode != NRF_ERROR_NOT_SUPPORTED)
            NRF_LOG_ERROR("error %d enabling control point indications", errCode);

        errCode = ble_hps_c_s_notify_enable(pHpsC, pEvt->conn_handle, true);
        if (errCode != NRF_SUCCESS && errCode != NRF_ERROR_NOT_SUPPORTED)
            NRF_LOG_ERROR("error %d enabling support notifications", errCode);
    }
    else if (pEvt->evt_type == BLE_HPS_C_EVT_SUPPORT_NOTIF && pEventHandler != NULL)
    {
        btle_event_t evt;
        evt.type = BTLE_EVT_SUPPORT_NOTIF;
        evt.pSupport = &pEvt->data.support;

        pEventHandler(&evt);
    }
}

/** @brief handler to listen to events related to device name change
 */
static void onBleEventHandler(ble_evt_t const * pBleEvt, void * pContext)
{
    (void) pContext;

    if (pBleEvt == NULL || pEventHandler == NULL)
        return;

    // check if there was a write operation to the device name characteristic and report the new name
    if (pBleEvt->header.evt_id == BLE_GATTS_EVT_WRITE)
    {
        if (pBleEvt->evt.gatts_evt.params.write.uuid.uuid == BLE_UUID_GAP_CHARACTERISTIC_DEVICE_NAME)
        {
            uint16_t len = pBleEvt->evt.gatts_evt.params.write.len;
            char newName[len + 1];

            memcpy(newName, pBleEvt->evt.gatts_evt.params.write.data, len);
            newName[len] = 0;

            btle_event_t evt;
            evt.type = BTLE_EVT_NAME_CHANGE;
            evt.pNewDeviceName = newName;

            pEventHandler(&evt);
        }
    }
}

static ret_code_t gattClientInit()
{
    ret_code_t errCode;

    // initialize database discovery
    ble_db_discovery_init_t dbInit = {0};
    dbInit.evt_handler = dbDiscoveryHandler;
    dbInit.p_gatt_queue = &gattQueueInst;
    errCode = ble_db_discovery_init(&dbInit);
    if (errCode != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("database discovery init error %d", errCode)
        return errCode;
    }

    // initialize hids client
    ble_hids_c_init_t hidsInit = {0};
    hidsInit.error_handler = hidsClientErrorHandler;  /// TODO: necessary?
    hidsInit.p_gatt_queue = &gattQueueInst;
    errCode = ble_hids_c_init(&hidsInst, &hidsInit);
    if (errCode != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("hids client init error %d", errCode)
        return errCode;
    }

    // initialize lcs client
    /*ble_lcs_c_init_t lcsInit = {0};
    lcsInit.error_handler = lcsClientErrorHandler;
    lcsInit.evt_handler = lcsClientHandler;
    lcsInit.p_gatt_queue = &gattQueueInst;
    errCode = ble_lcs_c_init(&lcsGattC, ARRAY_SIZE(lcsGattC_server_data), &lcsInit);
    if (errCode != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("light control service client init error %d", errCode)
        return errCode;
    }*/

    // initialize hps client
    ble_hps_c_init_t hpsInit = {0};
    hpsInit.error_handler = hpsClientErrorHandler;
    hpsInit.evt_handler = hpsClientHandler;
    hpsInit.p_gatt_queue = &gattQueueInst;
    errCode = ble_hps_c_init(&hpsGattC, ARRAY_SIZE(hpsGattC_server_data), &hpsInit);
    if (errCode != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("helen project service client init error %d", errCode)
        return errCode;
    }

    return NRF_SUCCESS;
}

/**@brief   Function for initializing the GATT module.
 * @details The GATT module handles ATT_MTU and Data Length update procedures automatically.
 */
static ret_code_t gattInit(ble_user_mem_block_t const * pBuf, ble_hps_init_t* pHpsInit, /*ble_lcs_init_t* pLcsInit,*/ btle_info_t const* pInfo)
{
    ret_code_t errCode;

    errCode = nrf_ble_gatt_init(&gattInst, NULL);
    if (errCode != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("gatt init error %d", errCode)
        return errCode;
    }

    errCode = gattServerInit(pBuf, pHpsInit,/* pLcsInit,*/ pInfo);
    if (errCode != NRF_SUCCESS)
        return errCode;

    errCode = gattClientInit();
    if (errCode != NRF_SUCCESS)
        return errCode;

    return NRF_SUCCESS;
}

static void onConnected(lm_connEvt_t const* pConn)
{
    uint32_t errCode;

    // assign peripheral connection handle to queued writes module
    if (pConn->role == BLE_GAP_ROLE_PERIPH)
    {
        errCode = nrf_ble_qwr_conn_handle_assign(&qwrInst, pConn->connHandle);
        if (errCode != NRF_SUCCESS)
        {
            NRF_LOG_WARNING("qwr handle assign error %d", errCode);
        }
    }

    // if device is a remote control, assign handles directly
    if (pConn->pRemDriver != NULL)
    {
        NRF_LOG_INFO("remote control connected");
        hidsInst.evt_handler = pConn->pRemDriver->evt_handler;
        ble_hids_c_db_t const* pHandles = pConn->pRemDriver->get_handles();
        errCode = ble_hids_c_handles_assign(&hidsInst, pConn->connHandle, pHandles);
        if (errCode != NRF_SUCCESS)
        {
            NRF_LOG_WARNING("hid handle assign error %d", errCode);
        }
        errCode = pConn->pRemDriver->notif_enable(&hidsInst, true);
        if (errCode != NRF_SUCCESS)
        {
            NRF_LOG_WARNING("hid notification enable error %d", errCode);
        }
    }
    // otherwise start database discovery
    else
    {
        // start database discovery
        /*errCode = ble_lcs_c_handles_assign(&lcsGattC, pConn->connHandle, NULL); /// TODO: maybe better to assign handles in discovery event in case of unexpected disconnection
        if (errCode != NRF_SUCCESS)
        {
            NRF_LOG_WARNING("lcs handle assign error %d", errCode);
        }*/
        errCode = ble_hps_c_handles_assign(&hpsGattC, pConn->connHandle, NULL); /// TODO: maybe better to assign handles in discovery event in case of unexpected disconnection
        if (errCode != NRF_SUCCESS)
        {
            NRF_LOG_WARNING("hps handle assign error %d", errCode);
        }
        errCode = ble_db_discovery_start(&discInst, pConn->connHandle);
        if (errCode != NRF_SUCCESS)
        {
            NRF_LOG_WARNING("database discovery start error %d", errCode);
        }
    }
}

/** @brief link manager event handler
 */
static void lmEventHandler(lm_evt_t const* pEvt)
{
    switch (pEvt->type)
    {
    case LM_EVT_CONNECTED:
        onConnected(&pEvt->evt.conn);
        break;

    case LM_EVT_ADV_MODE_CHANGED:
    case LM_EVT_SCAN_MODE_CHANGED:
    case LM_EVT_DISCONNECTED:
    case LM_EVT_DELETED:
    default:
        break;
    }

}

static void relayMode(uint16_t connHandle, void* pContext)
{
    uint32_t errCode;
    btle_modeRelay_t* pRelay = (btle_modeRelay_t*)pContext;
    //ble_lcs_c_cp_write_t lcsCmd;
    ble_hps_c_cp_write_t hpsCmd;

    // don't relay mode to source handle
    if (pRelay->connHandle == connHandle)
        return;

    //lcsCmd.command = BLE_LCS_C_CP_CMD_SET_MODE;
    //lcsCmd.params.mode_to_set = pRelay->mode;

    hpsCmd.command = BLE_HPS_C_CP_CMD_SET_MODE;
    hpsCmd.params.mode_to_set = pRelay->mode;

    errCode = ble_hps_c_cp_write(&hpsGattC, connHandle, &hpsCmd);
    //if (errCode == NRF_ERROR_NOT_FOUND) // if helen project not available, try light control service
    //    errCode = ble_lcs_c_cp_write(&lcsGattC, connHandle, &lcsCmd);

    if (errCode != NRF_SUCCESS && errCode != NRF_ERROR_NOT_FOUND)
    {
        NRF_LOG_ERROR("error %d relaying mode", errCode);
    }
}

//{ Public functions ----------------------------------------------------------*/
ret_code_t btle_Init(btle_init_t const* pInit)
{
    ret_code_t errCode;

    if (pInit == NULL || pInit->eventHandler == NULL)
        return NRF_ERROR_NULL;

    pEventHandler = pInit->eventHandler;

    errCode = ble_dfu_buttonless_async_svci_init();
    if (errCode != NRF_SUCCESS &&
        errCode != NRF_ERROR_NO_MEM)  // in case bootloader is not present
    {
        NRF_LOG_ERROR("dfu svci init error %d", errCode);
        return errCode;
    }

    errCode = bleStackInit();
    if (errCode != NRF_SUCCESS)
        return errCode;

    errCode = gapParamsInit(pInit->pInfo, pInit->maxDeviceNameLength);
    if (errCode != NRF_SUCCESS)
        return errCode;

    errCode = gattInit(&pInit->qwrBuffer, pInit->pHpsInit, /*pInit->pLcsInit,*/ pInit->pInfo);
    if (errCode != NRF_SUCCESS)
        return errCode;

    lm_init_t lmInit;
    lmInit.advType = pInit->advType;
    lmInit.uuidType = hpsGattC.uuid_type;
    lmInit.eventHandler = lmEventHandler;
    errCode = lm_Init(&lmInit);
    if (errCode != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("link manager init error %d", errCode);
        return errCode;
    }

    return NRF_SUCCESS;
}

ret_code_t btle_RelayMode(btle_modeRelay_t const* pRelay)
{
    uint32_t cnt;

    cnt = ble_conn_state_for_each_connected(relayMode, (void*)pRelay);

    return cnt ? NRF_SUCCESS : NRF_ERROR_NOT_FOUND;
}

/*ret_code_t btle_ReportLcsMeasurements(btle_lcsMeasurement_t const* pData)
{
    if (pData == NULL)
        return NRF_ERROR_NULL;

    ret_code_t errCode = NRF_ERROR_INVALID_STATE;
    ble_lcs_lm_t message = {0};

    message.light_type = BLE_LCS_LT_HELMET_LIGHT;
    message.hlmt.mode = pData->mode;
    if (pData->mode.setup.spot)
    {
        message.flags.hlmt.intensity_present = 1;
        message.flags.hlmt.spot_status_present = 1;
        message.hlmt.spot_status = pData->statusSpot;
        message.flags.hlmt.spot_power_present = 1;
        message.hlmt.spot_power = pData->powerSpot;
    }
    if (pData->temperature >= -40 && pData->temperature <= 85)
    {
        message.flags.hlmt.temperature_present = 1;
        message.temperature = pData->temperature;
    }
    if (pData->inputVoltage != 0)
    {
        message.flags.hlmt.input_voltage_present = 1;
        message.input_voltage = pData->inputVoltage;;
    }
    if (pData->pitch >= -90 && pData->pitch <= 90)
    {
        message.flags.hlmt.pitch_present = 1;
        message.pitch = pData->pitch;
    }
    if (pData->batterySoc <= 100)
    {
        message.flags.hlmt.battery_soc_present = 1;
        message.battery_soc = pData->batterySoc;
    }
    if (pData->powerTaillight > 0)
    {
        message.flags.hlmt.taillight_power_present = 1;
        message.taillight_power = pData->powerTaillight;
    }

    ret_code_t localErrCode = NRF_SUCCESS;
    for (uint_fast8_t i = 0; i < ARRAY_SIZE(lcsGattS_client_data); i++)
    {
        if (lcsGattS_client_data[i].conn_handle == BLE_CONN_HANDLE_INVALID ||
            lcsGattS_client_data[i].is_lm_notfy_enabled == false)
            continue;

        errCode = NRF_SUCCESS;

        if (localErrCode == NRF_SUCCESS)
            localErrCode = ble_lcs_light_measurement_send(&lcsGattS,
                                                        lcsGattS_client_data[i].conn_handle,
                                                        &message);
        else
            (void)ble_lcs_light_measurement_send(&lcsGattS,
                                                 lcsGattS_client_data[i].conn_handle,
                                                 &message);
    }

    return errCode == NRF_SUCCESS ? localErrCode : errCode;
}*/

ret_code_t btle_ReportHpsMeasurements(btle_hpsMeasurement_t const* pData)
{
    if (pData == NULL)
        return NRF_ERROR_NULL;

    ret_code_t errCode = NRF_SUCCESS;
    ble_hps_m_t message = {0};

    message.mode = pData->mode;
    if (pData->outputPower != 0)
    {
        message.flags.output_power_present = 1;
        message.output_power = pData->outputPower;
    }
    if (pData->temperature >= -40 && pData->temperature <= 85)
    {
        message.flags.temperature_present = 1;
        message.temperature = pData->temperature;
    }
    if (pData->inputVoltage != 0)
    {
        message.flags.input_voltage_present = 1;
        message.input_voltage = pData->inputVoltage;;
    }

    for (uint_fast8_t i = 0; i < ARRAY_SIZE(hpsGattS_client_data); i++)
    {
        if (hpsGattS_client_data[i].conn_handle == BLE_CONN_HANDLE_INVALID ||
            hpsGattS_client_data[i].is_m_notfy_enabled == false)
            continue;

        if (errCode == NRF_SUCCESS)
            errCode = ble_hps_measurement_send(&hpsGattS, hpsGattS_client_data[i].conn_handle, &message);
        else
            (void)ble_hps_measurement_send(&hpsGattS, hpsGattS_client_data[i].conn_handle, &message);
    }

    return errCode;
}

bool btle_isHpsDevice(uint16_t connHandle)
{
    for (uint_fast8_t i = 0; i < ARRAY_SIZE(hpsGattS_client_data); i++)
    {
        if (hpsGattS_client_data[i].conn_handle == connHandle)
            return true;
    }

    return false;
}

ret_code_t btle_ReportHpsSupport(uint16_t connHandle, ble_hps_s_t const* pData)
{
    return ble_hps_support_send(&hpsGattS, connHandle, pData);
}

//}

/**END OF FILE*****************************************************************/
