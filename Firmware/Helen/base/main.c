/**
  ******************************************************************************
  * @file    main.c
  * @author  Thomas Reisnecker
  * @brief   main module
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "app_timer.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "fds.h"
#include "nrf_pwr_mgmt.h"
//#include "nrf_drv_clock.h"
#include "nrf_power.h"
#include "nrf_bootloader_info.h"

#include "main.h"
#include "debug.h"
#include "btle.h"
#include "link_management.h"
#include "board.h"
#include "hmi.h"
#include "mode_management.h"
#include "channel_types.h"
#include "watchdog.h"
#include "com_message_handling.h"

#include "ble_types.h"

/* logger configuration ----------------------------------------------_-------*/
#define BASE_CONFIG_LOG_ENABLED 1 /// TODO: just temporary until project specific sdk_config
#define BASE_CONFIG_LOG_LEVEL   3

#define NRF_LOG_MODULE_NAME base
#if BASE_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL BASE_CONFIG_LOG_LEVEL
#else //BASE_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL 0
#endif //BASE_CONFIG_LOG_ENABLED
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

/* Private defines -----------------------------------------------------------*/
#define IDLE_TIMEOUT APP_TIMER_TICKS(180000)
#define IDLE_PERIOD  APP_TIMER_TICKS(10000)

/* Private typedef -----------------------------------------------------------*/
typedef enum
{
    MAINSTATE_OFF,      // device enters system off mode
    MAINSTATE_STANDBY,
    MAINSTATE_IDLE
} mState_t;

typedef enum
{
    FLOPSCR_NONE,   // no source
    FLOPSCR_HMI,    // flash operation request from hmi module
    FLOPSCR_LCS,    // flash operation request from light control service
} pendingFlashOpSource_t;

typedef struct
{
    volatile uint8_t pendingResponses;           // the number of responses to be expected
    pendingFlashOpSource_t source;
    uint32_t firstErrorCode;
    union
    {
        struct
        {
            ble_lcs_ctrlpt_evt_type_t event; // the event from light control service (0 if not the source)
            uint16_t connHandle;
            ble_lcs_ctrlpt_t * pLcsCtrlpt;
        } lcs;
        ds_reportHandler_t hmiResponse;
    };                           // the source for the flash operation, only one is allowed at a timer
} pendingFlashOp_t;

/* Private macros ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static mState_t mainState;
static uint32_t idleCounter;
APP_TIMER_DEF(idleTimer);
static pendingFlashOp_t pendingFlashOp;

/* Private functions ---------------------------------------------------------*/

/**@brief Handler for shutdown preparation.
 *
 * @details During shutdown procedures, this function will be called at a 1 second interval
 *          untill the function returns true. When the function returns true, it means that the
 *          app is ready to reset to DFU mode.
 *
 * @param[in]   event   Power manager event.
 *
 * @retval  True if shutdown is allowed by this power manager handler, otherwise false.
 */
static bool app_shutdown_handler(nrf_pwr_mgmt_evt_t event)
{
    switch (event)
    {
        case NRF_PWR_MGMT_EVT_PREPARE_DFU:
            NRF_LOG_INFO("Power management wants to reset to DFU mode.");
            break;

        default:
            return true;
    }

    NRF_LOG_INFO("Power management allowed to reset to DFU mode.");
    return true;
}

//lint -esym(528, m_app_shutdown_handler)
/**@brief Register application shutdown handler with priority 0.
 */
NRF_PWR_MGMT_HANDLER_REGISTER(app_shutdown_handler, 0);

static void softdeviceStateObserverHandler(nrf_sdh_state_evt_t state, void * p_context)
{
    if (state == NRF_SDH_EVT_STATE_DISABLED)
    {
        // Softdevice was disabled before going into reset. Inform bootloader to skip CRC on next boot.
        nrf_power_gpregret2_set(BOOTLOADER_DFU_SKIP_CRC);

        //Go to system off.
        nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
    }
}

/* nrf_sdh state observer. */
NRF_SDH_STATE_OBSERVER(softdeviceStateObserver, 0) =
{
    .handler = softdeviceStateObserverHandler,
};

void idleTimerCallback(void * pContext)
{
    (void)pContext;

    if (idleCounter == 0 || (--idleCounter != 0))
        return;

    mainState = MAINSTATE_STANDBY;
    NRF_LOG_INFO("going to standby mode");

    ret_code_t errCode;
    errCode = lm_SetExposureMode(LM_EXP_LOW_POWER);
    LOG_ERROR_CHECK("exposure mode set error %d", errCode);
    errCode = brd_SetPowerMode(BRD_PM_STANDBY);
    LOG_ERROR_CHECK("board power mode set error %d", errCode);
}

/**@brief Function for the main module initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static ret_code_t mainInit(void)
{
    ret_code_t errCode, retCode = NRF_SUCCESS;

    // Initialize timer module.
    errCode = app_timer_init();
    LOG_ERROR_CHECK("timer init error %d", errCode);
    if (retCode == NRF_SUCCESS)
        retCode = errCode;

    // Initialize power management module
    errCode = nrf_pwr_mgmt_init();
    LOG_ERROR_CHECK("power management init error %d", errCode);
    if (retCode == NRF_SUCCESS)
        retCode = errCode;

    errCode = app_timer_create(&idleTimer, APP_TIMER_MODE_REPEATED, &idleTimerCallback);
    LOG_ERROR_CHECK("idle timer create error %d", errCode);
    if (retCode == NRF_SUCCESS)
        retCode = errCode;

    mainState = MAINSTATE_IDLE;
    idleCounter = IDLE_TIMEOUT / IDLE_PERIOD;
    errCode = app_timer_start(idleTimer, IDLE_PERIOD, NULL);
    LOG_ERROR_CHECK("starting idle timer error %d", errCode);
    if (retCode == NRF_SUCCESS)
        retCode = errCode;

    return retCode;
}

static void statusLedHandling(bool limiterActive)
{
    static uint32_t const processRate = APP_TIMER_TICKS(20);
    static uint32_t lastCheck;
    uint32_t timestamp;
    hmi_ledState_t ledState;

    timestamp = app_timer_cnt_get();
    if (app_timer_cnt_diff_compute(timestamp, lastCheck) < processRate)
        return;

    lastCheck = timestamp;

    // in standby and off mode deactivate leds to save energy
    if (mainState <= MAINSTATE_STANDBY)
    {
        hmi_SetLed(HMI_LT_RED, HMI_LS_OFF);
        hmi_SetLed(HMI_LT_GREEN, HMI_LS_OFF);
        hmi_SetLed(HMI_LT_BLUE, HMI_LS_OFF);
        return;
    }

    // red indicates limiter status
    ledState = limiterActive ? HMI_LS_ON : HMI_LS_OFF;
    hmi_SetLed(HMI_LT_RED, ledState);

    // green indicates a peripheral connection
    /// TODO: maybe indicate advertising with blinking?
    uint8_t periphCnt = lm_GetPeriphCnt();
    ledState = periphCnt ? HMI_LS_ON : HMI_LS_OFF;
    hmi_SetLed(HMI_LT_GREEN, ledState);

    // blue indicates scanner and central connection
    lm_scanState_t scanMode = lm_GetScanningState();
    uint8_t centralCnt = lm_GetCentralCnt();
    ledState = scanMode == LM_SCAN_SEARCH ? HMI_LS_BLINKFAST :        // blink fast if search (highest priority
               centralCnt ? HMI_LS_ON :                               // on if at least one central is connected
               scanMode == LM_SCAN_LOW_LATENCY ? HMI_LS_BLINKSLOW :   // slow blinking if searching for known devices
               HMI_LS_OFF;                                            // or off
    hmi_SetLed(HMI_LT_BLUE, ledState);
}

/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void main_Execute(bool limiting)
{
    statusLedHandling(limiting);
}


/** @brief function to encode the helen group configuration to bluetooth lcs congiguration
 *
 * @param[out]    pGroups the array to be filled
 * @param[in/out] pCnt    in: the size of the provides array, out: the number of elements used
 */
static void encodeGroupConfig(uint8_t* pGroups, uint8_t* pCnt)
{
    if (pCnt == NULL)
        return;
    if (pGroups == NULL)
    {
        *pCnt = 0;
        return;
    }

    memset(pGroups, 0, *pCnt);

    uint_fast8_t i = 0, j = 0;
    mm_modeConfig_t const* pMode;// = mm_GetModeConfig(i);
    (void)mm_GetModeConfigs(&pMode);
    while (i < MM_NUM_OF_MODES && j < *pCnt)
    {
        pGroups[j] += 1;
        if (pMode[i++].lastInGroup)
            j++;
        //pMode = mm_GetModeConfig(++i);
    }
    *pCnt = j;
}

/** @brief function to encode the helen mode configuration to bluetooth lcs config
 *
 * @param[in]     start    the start index to start with
 * @param[out]    pList    pointer to list, that will be filled
 * @param[in/out] pCnt     in: the size of the list, out: the number of filled elements
 */
static void encodeModeConfig(uint8_t start, ble_lcs_hlmt_mode_t* pList, uint8_t* pCnt)
{
    if (pCnt == NULL)   return;

    if (pList == NULL)
    {
        *pCnt = 0;
        return;
    }

    memset(pList, 0, sizeof(ble_lcs_hlmt_mode_t) * *pCnt);

    cht_5int3mode_t const* pChannels;
    uint16_t channelSize;
    brd_GetChannelConfig((void const**)&pChannels, &channelSize);
    channelSize = channelSize < 8 ? channelSize : 8;   // ignoring PMW channel so far, will fail if helena base board is available

    uint_fast8_t j = 0;
    mm_modeConfig_t const* pMode;// = mm_GetModeConfig(start);
    (void)mm_GetModeConfigs(&pMode);
    while (start < MM_NUM_OF_MODES && j < *pCnt)
    {
        pList[j].setup.spot = pMode[start].ignore ? 0 : 1;
        pList[j].setup.taillight = 0;
        pList[j].setup.brakelight = 0;
        if (j < channelSize && !pMode[start++].ignore)
        {
            pList[j].intensity = CHT_53_TO_PERCENT(pChannels[j].intensity);
            if (pChannels[j].intensity == 0)
                pList[j].setup.spot = 0;
            pList[j].setup.pitchCompensation = pChannels[j].mode == 1 ? 1 : 0;
        }                                                       //  ^ just temporary for LCS
        else
        {
            pList[j].intensity = 0;
            pList[j].setup.spot = 0;
        }
        //pMode = mm_GetModeConfig(++start);
        j++;
    }
    *pCnt = j;
}

static void flashOpResultHandler(ret_code_t errCode)
{
    if (pendingFlashOp.pendingResponses == 0)
        return; // not expecting anything

    // preserve the first error code
    if (pendingFlashOp.firstErrorCode == NRF_SUCCESS)
        pendingFlashOp.firstErrorCode = errCode;

    if (--pendingFlashOp.pendingResponses == 0)
    {
        switch (pendingFlashOp.source)
        {
        case FLOPSCR_LCS:
        {
            ret_code_t errCode;
            ble_lcs_ctrlpt_rsp_t rsp = {0};
            rsp.opcode = pendingFlashOp.lcs.event +
                         BLE_LCS_CTRLPT_OP_CODE_REQ_MODE_CNT - BLE_LCS_CTRLPT_EVT_REQ_MODE_CNT;
            if (pendingFlashOp.firstErrorCode == NRF_SUCCESS)
                rsp.status = BLE_LCS_CTRLPT_RSP_CODE_SUCCESS;
            else if (pendingFlashOp.firstErrorCode == NRF_ERROR_INVALID_PARAM)
                rsp.status = BLE_LCS_CTRLPT_RSP_CODE_INVALID;
            else
                rsp.status = BLE_LCS_CTRLPT_RSP_CODE_FAILED;
            errCode = ble_lcs_ctrlpt_mode_resp(pendingFlashOp.lcs.pLcsCtrlpt, pendingFlashOp.lcs.connHandle, &rsp);
            LOG_ERROR_CHECK("error %d sending control point response", errCode);
        }   break;

        case FLOPSCR_HMI:
            if (pendingFlashOp.hmiResponse != NULL)
                pendingFlashOp.hmiResponse(pendingFlashOp.firstErrorCode);
            break;

        default:
            break;
        }

        memset(&pendingFlashOp, 0, sizeof(pendingFlashOp));
    }
}

static void lcsCpErrorHandler(uint32_t errCode)
{
    NRF_LOG_ERROR("lcs cp error: %d", errCode);
}

static bool isBlockZero(void const* pBlock, unsigned int size)
{
    unsigned char* pMem = (unsigned char*)pBlock;
    for (unsigned int i = 0; i < size; i++)
    {
        if(pMem[i] != 0)
            return false;
    }
    return true;
}

static void lcsCpEventHandler(ble_lcs_ctrlpt_t * pLcsCtrlpt, ble_lcs_ctrlpt_evt_t * pEvt)
{
    ret_code_t errCode;
    ble_lcs_ctrlpt_rsp_t rsp = {0};
    rsp.opcode = pEvt->evt_type + BLE_LCS_CTRLPT_OP_CODE_REQ_MODE_CNT - BLE_LCS_CTRLPT_EVT_REQ_MODE_CNT;
    rsp.status = BLE_LCS_CTRLPT_RSP_CODE_SUCCESS;

    union
    {
        uint8_t groupCfg[MM_NUM_OF_MODES];
        ble_lcs_hlmt_mode_t modeList[MM_NUM_OF_MODES];
    } data;

    switch (pEvt->evt_type)
    {
    case BLE_LCS_CTRLPT_EVT_REQ_MODE_CNT:
        NRF_LOG_DEBUG("number of modes requested");
        rsp.params.mode_cnt = MM_NUM_OF_MODES;
        break;
    case BLE_LCS_CTRLPT_EVT_SET_MODE:
    {
        NRF_LOG_DEBUG("request to set mode to %d", pEvt->p_params->set_mode);
        hmi_evtParams_t modeReq;
        modeReq.mode.connHandle = pEvt->conn_handle;
        modeReq.mode.modeNumber = pEvt->p_params->set_mode;
        if (hmi_RequestEvent(HMI_EVT_MODENUMBER, &modeReq) != NRF_SUCCESS)
            rsp.status = BLE_LCS_CTRLPT_RSP_CODE_FAILED;
    }   break;
    case BLE_LCS_CTRLPT_EVT_REQ_GRP_CNFG:
    {
        NRF_LOG_DEBUG("group organization requested");
        uint8_t cnt = ARRAY_SIZE(data.groupCfg);
        encodeGroupConfig(data.groupCfg, &cnt);
        rsp.params.group_config.num_of_groups = cnt;
        rsp.params.group_config.p_list_of_modes_per_group = data.groupCfg;
    }   break;

    case BLE_LCS_CTRLPT_EVT_CNFG_GROUP:
        NRF_LOG_DEBUG("new group organization received");

        if (pendingFlashOp.pendingResponses)
        {
            rsp.status = BLE_LCS_CTRLPT_RSP_CODE_NOT_SUPPORTED;
            break;
        }

        pendingFlashOp.pendingResponses = 1;
        pendingFlashOp.source = FLOPSCR_LCS;
        pendingFlashOp.lcs.event = pEvt->evt_type;
        pendingFlashOp.lcs.pLcsCtrlpt = pLcsCtrlpt;
        pendingFlashOp.lcs.connHandle = pEvt->conn_handle;
        errCode = mm_UpdateGroupConfig(&pEvt->p_params->group_config, flashOpResultHandler);
        if (errCode != NRF_SUCCESS)     // report manually if request fails
            flashOpResultHandler(errCode);
        return;

    case BLE_LCS_CTRLPT_EVT_REQ_MODE_CNFG:
    {
        NRF_LOG_DEBUG("mode organization requested");
        uint8_t cnt = ARRAY_SIZE(data.modeList);
        encodeModeConfig(pEvt->p_params->mode_list_start, data.modeList, &cnt);
        rsp.params.mode_config_list.num_of_entries = cnt;
        rsp.params.mode_config_list.p_list_hlmt = data.modeList;
    }   break;

    case BLE_LCS_CTRLPT_EVT_CNFG_MODE:
        NRF_LOG_DEBUG("new mode organization received");

        if (pendingFlashOp.pendingResponses)
        {
            rsp.status = BLE_LCS_CTRLPT_RSP_CODE_NOT_SUPPORTED;
            break;
        }

        pendingFlashOp.pendingResponses = 2;
        pendingFlashOp.source = FLOPSCR_LCS;
        pendingFlashOp.lcs.event = pEvt->evt_type;
        pendingFlashOp.lcs.pLcsCtrlpt = pLcsCtrlpt;
        pendingFlashOp.lcs.connHandle = pEvt->conn_handle;

        errCode = mm_UpdateModeConfig(&pEvt->p_params->mode_config, flashOpResultHandler);
        if (errCode != NRF_SUCCESS)     // report manually if request fails
            flashOpResultHandler(errCode);

        errCode = brd_UpdateChannelConfig(&pEvt->p_params->mode_config, flashOpResultHandler);
        if (errCode != NRF_SUCCESS)     // report manually if request fails
            flashOpResultHandler(errCode);

        return;

    case BLE_LCS_CTRLPT_EVT_REQ_LED_CNFG:
        NRF_LOG_DEBUG("led configuration requested");
        rsp.params.led_config.cnt_spot = 2; // hardcoded for now
        break;

    case BLE_LCS_CTRLPT_EVT_REQ_PREF_MODE:
        NRF_LOG_DEBUG("preferred mode requested");
        rsp.params.pref_mode = mm_GetPrefMode();
        break;

    case BLE_LCS_CTRLPT_EVT_SET_PREF_MODE:
        NRF_LOG_DEBUG("new preferred mode (%d) received", pEvt->p_params->pref_mode);

        if (pendingFlashOp.pendingResponses)
        {
            rsp.status = BLE_LCS_CTRLPT_RSP_CODE_NOT_SUPPORTED;
            break;
        }

        pendingFlashOp.pendingResponses = 1;
        pendingFlashOp.source = FLOPSCR_LCS;
        pendingFlashOp.lcs.event = pEvt->evt_type;
        pendingFlashOp.lcs.pLcsCtrlpt = pLcsCtrlpt;
        pendingFlashOp.lcs.connHandle = pEvt->conn_handle;
        errCode = mm_SetPrefMode(pEvt->p_params->pref_mode, flashOpResultHandler);
        if (errCode != NRF_SUCCESS)     // report manually if request fails
            flashOpResultHandler(errCode);
        return;

    case BLE_LCS_CTRLPT_EVT_REQ_TEMP_MODE:
        NRF_LOG_DEBUG("temporary mode requested");
        rsp.params.temp_mode = mm_GetTempMode();
        break;

    case BLE_LCS_CTRLPT_EVT_SET_TEMP_MODE:
        NRF_LOG_DEBUG("new temporary mode (%d) received", pEvt->p_params->pref_mode);

        if (pendingFlashOp.pendingResponses)
        {
            rsp.status = BLE_LCS_CTRLPT_RSP_CODE_NOT_SUPPORTED;
            break;
        }

        pendingFlashOp.pendingResponses = 1;
        pendingFlashOp.source = FLOPSCR_LCS;
        pendingFlashOp.lcs.event = pEvt->evt_type;
        pendingFlashOp.lcs.pLcsCtrlpt = pLcsCtrlpt;
        pendingFlashOp.lcs.connHandle = pEvt->conn_handle;
        errCode = mm_SetTempMode(pEvt->p_params->temp_mode, flashOpResultHandler);
        if (errCode != NRF_SUCCESS)     // report manually if request fails
            flashOpResultHandler(errCode);
        return;

    case BLE_LCS_CTRLPT_EVT_CHK_LED_CNFG:
    case BLE_LCS_CTRLPT_EVT_REQ_SENS_OFF:
    case BLE_LCS_CTRLPT_EVT_CALIB_SENS_OFF:
    case BLE_LCS_CTRLPT_EVT_REQ_LIMITS:
    case BLE_LCS_CTRLPT_EVT_SET_LIMITS:
    default:
        rsp.status = BLE_LCS_CTRLPT_RSP_CODE_NOT_SUPPORTED;
        break;
    }

    errCode = ble_lcs_ctrlpt_mode_resp(pLcsCtrlpt, pEvt->conn_handle, &rsp);
    APP_ERROR_CHECK(errCode);
}

static bool hpsEventHandler(ble_hps_evt_t const * pEvt)
{
    if (pEvt->evt_type == BLE_HPS_EVT_MODE_CONFIG_CHANGED)
    {
        ret_code_t errCode;
        uint16_t size;

        size = (uint32_t)pEvt->params.modes.p_channel_config - (uint32_t)pEvt->params.modes.p_mode_config;

        errCode = mm_CheckModeConfig((mm_modeConfig_t const*)(pEvt->params.modes.p_mode_config), size);
        if (errCode != NRF_SUCCESS)
            return false;

        errCode = brd_SetChannelConfig(pEvt->params.modes.p_channel_config, pEvt->params.modes.total_size - size, NULL);
        if (errCode == NRF_ERROR_INVALID_LENGTH || errCode == NRF_ERROR_INVALID_PARAM)
            return false;
        LOG_ERROR_CHECK("setting channel config error %d", errCode);

        errCode = mm_SetModeConfig((mm_modeConfig_t const*)pEvt->params.modes.p_mode_config, size, NULL);
        LOG_ERROR_CHECK("setting mode config error %d", errCode);
    }
    else if (pEvt->evt_type == BLE_HPS_EVT_CP_WRITE)
    {
        hmi_evtParams_t hmiParam;

        switch (pEvt->params.ctrl_pt.opcode)
        {
        case BLE_HPS_CP_SET_MODE:
            hmiParam.mode.modeNumber = pEvt->params.ctrl_pt.mode_request;
            hmiParam.mode.connHandle = pEvt->p_client->conn_handle;
            (void)hmi_RequestEvent(HMI_EVT_MODENUMBER, &hmiParam);
            break;

        case BLE_HPS_CP_REQ_SEARCH:
            (void)hmi_RequestEvent(HMI_EVT_SEARCHREMOTE, NULL);
            break;

        case BLE_HPS_CP_REQ_FACTORY_RESET:
            hmiParam.resultHandler = NULL;
            (void)hmi_RequestEvent(HMI_EVT_FACTORYRESET, &hmiParam);
            break;

        case BLE_HPS_CP_SET_MODE_OVERRIDE:
        {
            ret_code_t errCode = brd_OverrideMode(&pEvt->params.ctrl_pt.channel_config);
            if (errCode == NRF_SUCCESS)
                return true;
            else
                return false;
        }
        default:
            return false;
        }
    }
    return true;
}

static void btleEventHandler(btle_event_t const * pEvt)
{
    if (pEvt->type == BTLE_EVT_NAME_CHANGE)
    {
        NRF_LOG_INFO("new device name received: %s", pEvt->pNewDeviceName);

        ret_code_t errCode;
        errCode = brd_SetDeviceName(pEvt->pNewDeviceName, NULL);
        LOG_ERROR_CHECK("setting device name error %d", errCode);
    }
}

static ret_code_t bluetoothInit(brd_info_t const* pBrdInfo)//btle_info_t const* const pInfo, brd_features_t const* pFeatures, ble_hps_modes_init_t const* pModes)
{
    ble_lcs_init_t lcsInit = {0};
    lcsInit.cp_evt_handler = lcsCpEventHandler;
    lcsInit.error_handler = lcsCpErrorHandler;
    lcsInit.feature.light_type = BLE_LCS_LT_HELMET_LIGHT;
    lcsInit.feature.hlmt_features.spot_supported = 1;
    lcsInit.feature.hlmt_features.pitch_comp_supported = pBrdInfo->pFeatures->isIMUPresent ? 1 : 0;
    lcsInit.feature.cfg_features.mode_change_supported = 1;
    lcsInit.feature.cfg_features.mode_config_supported = 1;
    lcsInit.feature.cfg_features.mode_grouping_supported = 1;
    lcsInit.feature.cfg_features.preferred_mode_supported = 1;
    lcsInit.feature.cfg_features.temporary_mode_supported = 1;

    ble_hps_init_t hpsInit = {0};
    ble_hps_f_ch_size_t chSize[pBrdInfo->pFeatures->channelCount];
    hpsInit.features.mode_count = MM_NUM_OF_MODES;
    hpsInit.features.channel_count = pBrdInfo->pFeatures->channelCount;
    for (uint8_t i = 0; i < ARRAY_SIZE(chSize); i++)
    {
        chSize[i].channel_bitsize = 8;  /// TODO this cannot be hardcoded here, it should be shifted to board specific code
        chSize[i].sp_ftr_bitsize = 3;
        chSize[i].channel_description = pBrdInfo->pFeatures->pChannelTypes[i];
    }
    hpsInit.features.p_channel_size = chSize;
    hpsInit.features.flags.mode_set_supported = true;   /// TODO shift to board specific code
    hpsInit.features.flags.search_request_supported = true;
    hpsInit.features.flags.factory_reset_supported = true;
    hpsInit.features.flags.mode_override_supported = true;
    hpsInit.modes = *(pBrdInfo->pModes);
    hpsInit.evt_handler = hpsEventHandler;

    btle_init_t btleInit = {0};
    btleInit.advType = BTLE_ADV_TYPE_AFTER_SEARCH,            /// TODO board specific and options for no button
    btleInit.maxDeviceNameLength = pBrdInfo->pFeatures->maxNameLenght;
    btleInit.pInfo = pBrdInfo->pInfo;
    btleInit.eventHandler = btleEventHandler;
    btleInit.pLcsInit = &lcsInit;
    btleInit.pHpsInit = &hpsInit;
    btleInit.qwrBuffer = pBrdInfo->qwrBuffer;

    return btle_Init(&btleInit);
}

ret_code_t hmiEventHandler(hmi_evt_t const* pEvent)
{
    ret_code_t errCode = NRF_SUCCESS;
    (void)main_ResetIdleTimer();

    switch (pEvent->type)
    {
    case HMI_EVT_SEARCHREMOTE:
        errCode = lm_SetExposureMode(LM_EXP_SEARCHING);
        LOG_ERROR_CHECK("setting exposure mode error %d", errCode);
        break;

    case HMI_EVT_FACTORYRESET:
        NRF_LOG_INFO("initiating factory reset");
        if (pendingFlashOp.pendingResponses)
            return NRF_ERROR_INVALID_STATE;

        pendingFlashOp.pendingResponses = 3;
        pendingFlashOp.source = FLOPSCR_HMI;
        pendingFlashOp.hmiResponse = pEvent->params.resultHandler;

        errCode = mm_FactoryReset(flashOpResultHandler);
        if (errCode != NRF_SUCCESS)
        {
            pendingFlashOp.pendingResponses--;
            if (pendingFlashOp.firstErrorCode != NRF_SUCCESS)
                pendingFlashOp.firstErrorCode = errCode;
            NRF_LOG_ERROR("mode factory reset error %d", errCode);
        }

        errCode = brd_FactoryReset(flashOpResultHandler);
        if (errCode != NRF_SUCCESS)
        {
            pendingFlashOp.pendingResponses--;
            if (pendingFlashOp.firstErrorCode != NRF_SUCCESS)
                pendingFlashOp.firstErrorCode = errCode;
            NRF_LOG_ERROR("board factory reset error %d", errCode);
        }

        errCode = lm_DeleteBonds(flashOpResultHandler);
        if (errCode != NRF_SUCCESS)
        {
            pendingFlashOp.pendingResponses--;
            if (pendingFlashOp.firstErrorCode != NRF_SUCCESS)
                pendingFlashOp.firstErrorCode = errCode;
            NRF_LOG_ERROR("deleting bonds error %d", errCode);
        }
        // if all requests failed, result might to be sent manually
        if (pendingFlashOp.hmiResponse == 0 && !isBlockZero(&pendingFlashOp, sizeof(pendingFlashOp)))
        {
            pEvent->params.resultHandler(pendingFlashOp.firstErrorCode);
            memset(&pendingFlashOp, 0, sizeof(pendingFlashOp));
        }
        break;

    case HMI_EVT_DELETEBONDS:
        NRF_LOG_INFO("deleting bonds");
        if (pendingFlashOp.pendingResponses)
            return NRF_ERROR_INVALID_STATE;

        pendingFlashOp.pendingResponses = 1;
        pendingFlashOp.source = FLOPSCR_HMI;
        pendingFlashOp.hmiResponse = pEvent->params.resultHandler;

        errCode = lm_DeleteBonds(flashOpResultHandler);
        if (errCode != NRF_SUCCESS)
        {
            pendingFlashOp.pendingResponses--;
            if (pendingFlashOp.firstErrorCode != NRF_SUCCESS)
                pendingFlashOp.firstErrorCode = errCode;
            NRF_LOG_ERROR("deleting bonds error %d", errCode);
        }
        // if request failed, result might to be sent manually
        if (pendingFlashOp.hmiResponse == 0 && !isBlockZero(&pendingFlashOp, sizeof(pendingFlashOp)))
        {
            pEvent->params.resultHandler(pendingFlashOp.firstErrorCode);
            memset(&pendingFlashOp, 0, sizeof(pendingFlashOp));
        }
        break;

    default:    // the other events are handled by the hmi module
        return mm_HmiEventHandler(pEvent);
    }

    return errCode;
}

static void comHandler(cmh_event_t const* pEvent)
{
    switch (pEvent->type)
    {
    case CMH_EVENT_MODE_REQUEST:
    {
        hmi_evtParams_t req;
        req.mode.connHandle = BLE_CONN_HANDLE_INVALID;
        req.mode.modeNumber = pEvent->mode;
        (void)hmi_RequestEvent(HMI_EVT_MODENUMBER, &req);
    }   break;
    default:
        break;
    }
}

static ret_code_t comInit(brd_comPins_t const* pPins)
{
    ret_code_t errCode;
    cmh_init_t init;

    init.rxPin = pPins->rx;
    init.txPin = pPins->tx;
    init.dummyPin = pPins->dummy;
    init.handler = comHandler;

    errCode = cmh_Init(&init);
    LOG_ERROR_CHECK("com init error %d", errCode);

    return errCode;
}

static void modeChanged(uint8_t newMode)
{
    if (newMode == MM_MODE_OFF)
        newMode = mm_GetOffMode();
    ret_code_t errCode = brd_SetLightMode(newMode);
    LOG_ERROR_CHECK("setting light mode error %d", errCode);
}

/* main ----------------------------------------------------------------------*/
/**@brief Function for application main entry.
 */
int main(void)
{
    ret_code_t errCode, criticalError = NRF_SUCCESS;
    brd_info_t const* pBrdInfos;
    uint8_t initMode;

    errCode = dbg_Init();
    APP_ERROR_CHECK(errCode);

    errCode = mainInit();
    APP_ERROR_CHECK(errCode);

    // First of all initialize the mode management, so that the modes are
    // already for the board initialization
    errCode = mm_Init(modeChanged, &initMode);
    APP_ERROR_CHECK(errCode);

    errCode = brd_Init(&pBrdInfos);
    LOG_ERROR_CHECK("board init error %d", errCode);
    if (criticalError == NRF_SUCCESS)
        criticalError = errCode;

    hmi_init_t hmiInit = {.eventHandler = hmiEventHandler};
    errCode = hmi_Init(&hmiInit);
    APP_ERROR_CHECK(errCode);

    errCode = bluetoothInit(pBrdInfos);
    APP_ERROR_CHECK(errCode);

    errCode = comInit(&pBrdInfos->pFeatures->comSupported);
    APP_ERROR_CHECK(errCode);

    if (criticalError)
        hmi_SetLed(HMI_LT_RED, HMI_LS_BLINKFAST);
    while (criticalError)
    {
        wdg_Feed();
    }

    errCode = brd_SetPowerMode(BRD_PM_IDLE);
    APP_ERROR_CHECK(errCode);

    if (initMode == MM_MODE_OFF)
        initMode = mm_GetOffMode();
    errCode = brd_SetLightMode(initMode);
    APP_ERROR_CHECK(errCode);

    errCode = lm_SetExposureMode(LM_EXP_LOW_LATENCY);
    APP_ERROR_CHECK(errCode);

    // Enter main loop.
    for (;;)
    {
        bool limiting;

        limiting = brd_Execute();                       /// TODO: changed, test again! 518 - 52000 cycles

        hmi_Execute();                                  // ~7 cycles

        main_Execute(limiting);                         // ~72-1300 cycles

        if (dbg_Execute() == false)
        {
            nrf_pwr_mgmt_run();
        }
    }
}

/* Public functions ----------------------------------------------------------*/
ret_code_t main_ResetIdleTimer()
{
    ret_code_t errCode = NRF_SUCCESS;

    if (mainState <= MAINSTATE_STANDBY)
    {
        mainState = MAINSTATE_IDLE;
        NRF_LOG_INFO("going to idle mode");
        errCode = lm_SetExposureMode(LM_EXP_LOW_LATENCY);
        LOG_ERROR_CHECK("exposure mode set error %d", errCode);
        errCode = brd_SetPowerMode(BRD_PM_IDLE);
        LOG_ERROR_CHECK("board power mode set error %d", errCode);
    }

    idleCounter = IDLE_TIMEOUT / IDLE_PERIOD;

    return errCode;
}

/**END OF FILE*****************************************************************/

