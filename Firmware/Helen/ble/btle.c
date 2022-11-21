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

#include "debug.h"
#include "btle.h"
#include "remote.h"
#include "ble_lcs.h"
#include "ble_lcs_c.h"
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

/* Private variables ---------------------------------------------------------*/
NRF_BLE_GATT_DEF(gattInst);                                         // gatt instance
BLE_LCS_DEF(lcsGattS, NRF_SDH_BLE_TOTAL_LINK_COUNT);                             // light control service server

BLE_DB_DISCOVERY_DEF(discInst);                                     // database discovery
NRF_BLE_GQ_DEF(gattQueueInst, NRF_SDH_BLE_CENTRAL_LINK_COUNT, NRF_BLE_GQ_QUEUE_SIZE); // queued writes module
BLE_HIDS_C_DEF(hidsInst);                                           // hid service client
BLE_LCS_C_DEF(lcsGattC, NRF_SDH_BLE_TOTAL_LINK_COUNT);                           // light control service client

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
static ret_code_t gapParamsInit(btle_info_t const* pInfo)
{
    ret_code_t              errCode;
    ble_gap_conn_params_t   gapConnParams = *lm_getPreferredConnParams();
    ble_gap_conn_sec_mode_t secMode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&secMode);   /// TODO: just read, or allow changing?

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
static void dfuEventHandler(ble_dfu_buttonless_evt_type_t event)
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
static void lightControlServiceHandler(ble_lcs_evt_t* pEvt)
{
    if (ble_conn_state_role(pEvt->conn_handle) != BLE_GAP_ROLE_PERIPH)
        return;

    switch (pEvt->evt_type)
    {
    case BLE_LCS_EVT_LCP_INDICATION_ENABLED:
        NRF_LOG_INFO("device is under peripheral control, disabling remote control");
        lm_diconnectAndIgnoreRemote(true);
        break;
    case BLE_LCS_EVT_LCP_INDICATION_DISABLED:
        NRF_LOG_INFO("device is no longer under peripheral control, enabling remote control");
        lm_diconnectAndIgnoreRemote(false);
        break;
    default:
        break;
    }
}

/**@brief Function for initializing services that will be used by the application.
 */
static ret_code_t gattServerInit(ble_lcs_init_t* pLcsInit, btle_info_t const* pInfo)
{
    ret_code_t                errCode;
    ble_dfu_buttonless_init_t dfusInit = {0};
    ble_dis_init_t            disInit = {0};

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
        NRF_LOG_ERROR("device information service init error %d", errCode)
        return errCode;
    }

    // Initialize the lcs service
    if (pLcsInit != NULL)
    {
        pLcsInit->lcs_lm_cccd_wr_sec = SEC_OPEN;
        pLcsInit->lcs_lf_rd_sec = SEC_OPEN;
        pLcsInit->lcs_lcp_cccd_wr_sec = SEC_OPEN; //SEC_JUST_WORKS;
        pLcsInit->lcs_lcp_wr_sec = SEC_OPEN; //SEC_JUST_WORKS;
        pLcsInit->evt_handler = lightControlServiceHandler;
        errCode = ble_lcs_init(&lcsGattS, ARRAY_SIZE(lcsGattS_client_data), pLcsInit);
        if (errCode != NRF_SUCCESS)
        {
            NRF_LOG_ERROR("light control service init error %d", errCode)
            return errCode;
        }
    }

    // Initialize dfu service
    dfusInit.evt_handler = dfuEventHandler;
    errCode = ble_dfu_buttonless_init(&dfusInit);
    if (errCode != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("dfu service init error %d", errCode)
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

    ble_lcs_c_on_db_disc_evt(&lcsGattC, pEvt);
}

/** @brief hid service client error handler
 */
static void hidsErrorHandler(uint32_t errCode)
{
    APP_ERROR_HANDLER(errCode);
}

static void lcsErrorHandler(ble_lcs_c_t * pLcsC, ble_lcs_c_error_evt_t * pEvt)
{

}

static void lcsEventHandler(ble_lcs_c_t * pLcsC, ble_lcs_c_evt_t * pEvt)
{
    uint32_t errCode;

    if (pEvt->evt_type == BLE_LCS_C_EVT_DISCOVERY_COMPLETE)
    {
        NRF_LOG_INFO("another light connected");
        // handles are automatically assigned
        //errCode = ble_lcs_c_handles_assign(pLcsC, pEvt->conn_handle, &pEvt->data.lcs_db);
        //APP_ERROR_CHECK(errCode);
        // enable indications to be able to relay mode changes
        errCode = ble_lcs_c_cp_indic_enable(pLcsC, pEvt->conn_handle, true);
        APP_ERROR_CHECK(errCode);
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
    hidsInit.error_handler = hidsErrorHandler;  /// TODO: necessary?
    hidsInit.p_gatt_queue = &gattQueueInst;
    errCode = ble_hids_c_init(&hidsInst, &hidsInit);
    if (errCode != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("hids client init error %d", errCode)
        return errCode;
    }

    // initialize lcs client
    ble_lcs_c_init_t lcsInit = {0};
    lcsInit.error_handler = lcsErrorHandler;
    lcsInit.evt_handler = lcsEventHandler;
    lcsInit.p_gatt_queue = &gattQueueInst;
    errCode = ble_lcs_c_init(&lcsGattC, ARRAY_SIZE(lcsGattC_server_data), &lcsInit);
    if (errCode != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("light control service client init error %d", errCode)
        return errCode;
    }

    return NRF_SUCCESS;
}

/**@brief   Function for initializing the GATT module.
 * @details The GATT module handles ATT_MTU and Data Length update procedures automatically.
 */
static ret_code_t gattInit(ble_lcs_init_t* pLcsInit, btle_info_t const* pInfo)
{
    ret_code_t errCode;

    errCode = nrf_ble_gatt_init(&gattInst, NULL);
    if (errCode != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("gatt init error %d", errCode)
        return errCode;
    }

    errCode = gattServerInit(pLcsInit, pInfo);
    if (errCode != NRF_SUCCESS)
        return errCode;

    gattClientInit();
    if (errCode != NRF_SUCCESS)
        return errCode;

    return NRF_SUCCESS;
}

static void onConnected(lm_connEvt_t const* pConn)
{
    uint32_t errCode;

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
        errCode = ble_lcs_c_handles_assign(&lcsGattC, pConn->connHandle, NULL); /// TODO: maybe better to assign handles in discovery event in case of unexpected disconnection
        if (errCode != NRF_SUCCESS)
        {
            NRF_LOG_WARNING("lcs handle assign error %d", errCode);
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
/*
case BLE_GATTS_EVT_WRITE:

    if (p_ble_evt->evt.gatts_evt.params.write.context.char_uuid.uuid == BLE_UUID_GAP_CHARACTERISTIC_DEVICE_NAME)
    {
        for(uint32_t counter=0;counter<p_ble_evt->evt.gatts_evt.params.write.len;counter++)
            {
            CUSTOM_DEVICE_NAME[counter] = p_ble_evt->evt.gatts_evt.params.write.data[counter];
            beacon_write_handler(&m_bcs, beacon_device_name, CUSTOM_DEVICE_NAME);
            }

}
    break;
*/

//{ Public functions ----------------------------------------------------------*/
ret_code_t btle_Init(btle_init_t const* pInit)
{
    ret_code_t errCode;

    errCode = ble_dfu_buttonless_async_svci_init();
    if (errCode != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("dfu svci init error %d", errCode);
        return errCode;
    }

    errCode = bleStackInit();
    if (errCode != NRF_SUCCESS)
        return errCode;

    errCode = gapParamsInit(pInit->pInfo);
    if (errCode != NRF_SUCCESS)
        return errCode;

    errCode = gattInit(pInit->pLcsInit, pInit->pInfo);
    if (errCode != NRF_SUCCESS)
        return errCode;

    lm_init_t lmInit;
    lmInit.useWhitelistAdvertising = pInit->useWhitelistAdvertising;
    lmInit.lcsUuidType = lcsGattC.uuid_type;
    lmInit.eventHandler = lmEventHandler;
    errCode = lm_Init(&lmInit);
    if (errCode != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("link manager init error %d", errCode);
        return errCode;
    }

    return NRF_SUCCESS;
}

ret_code_t btle_RelayMode(uint8_t modeNumber, uint16_t connHandle)
{
    ret_code_t errCode = NRF_SUCCESS;
    ble_lcs_c_cp_write_t cmd;

    cmd.command = BLE_LCS_C_CP_CMD_SET_MODE;
    cmd.params.mode_to_set = modeNumber;

    for (uint_fast8_t i = 0; i < ARRAY_SIZE(lcsGattC_server_data); i++)
    {
        if (lcsGattC_server_data[i].conn_handle != BLE_CONN_HANDLE_INVALID && // only send to connected devices
            (connHandle == BLE_CONN_HANDLE_ALL ||                             // too all
             lcsGattC_server_data[i].conn_handle != connHandle))              // or to all except the one
        {
            if (errCode == NRF_SUCCESS)
                errCode = ble_lcs_c_cp_write(&lcsGattC, lcsGattC_server_data[i].conn_handle, &cmd);
            else
                (void)ble_lcs_c_cp_write(&lcsGattC, lcsGattC_server_data[i].conn_handle, &cmd);
        }
    }
    return errCode;
}

ret_code_t btle_ReportMeasurements(btle_lcsMeasurement_t const* pData)
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
}
//}

/**END OF FILE*****************************************************************/
