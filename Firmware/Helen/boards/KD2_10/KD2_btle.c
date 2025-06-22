/**
  ******************************************************************************
  * @file    KD2_service.c
  * @author  Thomas Reisnecker
  * @brief   tbd.
  ******************************************************************************
  */

/* logger configuration ----------------------------------------------_-------*/
#define BRD_CONFIG_LOG_ENABLED 1 /// TODO: just temporary until project specific sdk_config
#define BRD_CONFIG_LOG_LEVEL   3

#define NRF_LOG_MODULE_NAME brd
#if BRD_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL BRD_CONFIG_LOG_LEVEL
#else //BASE_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL 0
#endif //BASE_CONFIG_LOG_ENABLED
#include "nrf_log.h"

/* Includes ------------------------------------------------------------------*/
#include "debug.h"
#include "ble_KD2.h"
#include "KD2_10.h"
#include "btle.h"
#include "ble_bas.h"

/* External variables --------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
    brd_features_t const* pFtrs;
    bool isImuPresent;
} init_t;

/* Private macros ------------------------------------------------------------*/

/* Private defines -----------------------------------------------------------*/
static uint32_t initKD2Service(void* pContext);

/* Private variables ---------------------------------------------------------*/
static init_t init;
BLE_KD2_DEF(KD2Gatts, NRF_SDH_BLE_TOTAL_LINK_COUNT);
BLE_BAS_DEF(basGatts);
BTLE_SERVICE_REGISTER(KD2Service, initKD2Service, &init);
static uint16_t pendingConnHandle = BLE_CONN_HANDLE_INVALID;
static ble_KD2_cp_op_code_t pendingOpCode;

/* Private read only variables -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
static void resultHandler(ret_code_t errCode)
{
    ble_KD2_cp_rsp_t rsp;

    if (pendingConnHandle == BLE_CONN_HANDLE_INVALID)
        return; // something went wrong

    rsp.opcode = pendingOpCode;
    rsp.status = errCode == NRF_SUCCESS ? BLE_KD2_CP_RSP_VAL_SUCCESS : BLE_KD2_CP_RSP_VAL_FAILED;

    errCode = ble_KD2_cp_response(&KD2Gatts, pendingConnHandle, &rsp);   /// TODO: try again in case of error?
    LOG_ERROR_CHECK("error %d sending response in result handler", errCode);

    pendingConnHandle = BLE_CONN_HANDLE_INVALID;
}

static void onChannelSetupRequest(uint16_t connHandle, uint8_t channel)
{
    ret_code_t errCode;
    KD2_channelSetup_t setup;
    ble_KD2_cp_rsp_t rsp;
    int32_t conv;

    rsp.opcode = BLE_KD2_CP_OP_REQ_CHN_CONFIG;

    errCode = KD2_GetChannelSetup(&setup, channel);
    if (errCode == NRF_SUCCESS)
    {
        rsp.status = BLE_KD2_CP_RSP_VAL_SUCCESS;
        rsp.params.channel_config.channel = channel;
        conv = setup.outputPower * 1000l;
        conv = (conv + 512) / 1024;
        rsp.params.channel_config.config.output_power = conv;
        conv = (setup.outputLimit + 128) / 256;
        rsp.params.channel_config.config.output_limit = conv;
        if (setup.optic.type >= PRG_TYPE_CNT)
            rsp.params.channel_config.config.optic_type = BLE_KD2_OPTIC_NA;
        else
            rsp.params.channel_config.config.optic_type = setup.optic.type - PRG_TYPE_15 + BLE_KD2_OPTIC_15D;
        conv = setup.optic.offset * 36000l;
        conv = conv >= 0 ? conv + 16384 : conv - 16384; // rounding away from zero
        rsp.params.channel_config.config.optic_offset = conv / 32768;
    }
    else
    {
        rsp.status = BLE_KD2_CP_RSP_VAL_FAILED;
        NRF_LOG_ERROR("error %d getting channel setup", errCode);
    }

    errCode = ble_KD2_cp_response(&KD2Gatts, connHandle, &rsp);
    LOG_ERROR_CHECK("error %d sending channel setup response", errCode);
}

static void onChannelSetupSet(uint16_t connHandle, uint8_t channel, ble_KD2_channel_config_t const* pSetup)
{
    ret_code_t errCode;
    KD2_channelSetup_t setup;
    ble_KD2_cp_rsp_t rsp;
    int32_t conv;

    rsp.opcode = BLE_KD2_CP_OP_SET_CHN_CONFIG;
    rsp.status = BLE_KD2_CP_RSP_VAL_FAILED;

    if (pendingConnHandle == BLE_CONN_HANDLE_INVALID)
    {
        conv = pSetup->output_power * 1024l;
        setup.outputPower = (conv + 500) / 1000;
        setup.outputLimit = pSetup->output_limit << 8;
        if (pSetup->optic_type == BLE_KD2_OPTIC_NA)
            setup.optic.type = KD2_OPTIC_TYPE_NA;
        else
            setup.optic.type = pSetup->optic_type - BLE_KD2_OPTIC_15D + PRG_TYPE_15;
        conv = pSetup->optic_offset * 32768l;
        conv = conv >= 0 ? conv + 180 : conv - 180; // rounding away from zero
        setup.optic.offset = conv / 36000;

        errCode = KD2_SetChannelSetup(&setup, channel, resultHandler);
        if (errCode == NRF_ERROR_INVALID_PARAM)
            rsp.status = BLE_KD2_CP_RSP_VAL_INVALID;
        else if (errCode == NRF_ERROR_INVALID_STATE)    // nothing changed
            rsp.status = BLE_KD2_CP_RSP_VAL_SUCCESS;
        else if (errCode == NRF_SUCCESS)
        {
            pendingConnHandle = connHandle;
            pendingOpCode = rsp.opcode;
            return; // response is sent in result handler
        }
    }

    errCode = ble_KD2_cp_response(&KD2Gatts, connHandle, &rsp);
    LOG_ERROR_CHECK("error %d sending channel setup response", errCode);
}

static void onComPinRequest(uint16_t connHandle)
{
    ret_code_t errCode;
    KD2_comPinMode_t comPinMode;
    ble_KD2_cp_rsp_t rsp;

    rsp.opcode = BLE_KD2_CP_OP_REQ_COM_PIN_CONFIG;

    errCode = KD2_GetComPinMode(&comPinMode);
    if (errCode == NRF_SUCCESS)
    {
        rsp.status = BLE_KD2_CP_RSP_VAL_SUCCESS;
        rsp.params.com_pin = comPinMode;
    }
    else
    {
        rsp.status = BLE_KD2_CP_RSP_VAL_FAILED;
        NRF_LOG_ERROR("error %d getting com pin mode", errCode);
    }

    errCode = ble_KD2_cp_response(&KD2Gatts, connHandle, &rsp);
    LOG_ERROR_CHECK("error %d sending com pin response", errCode);
}

static void onComPinSet(uint16_t connHandle, ble_KD2_com_pin_t comPinMode)
{
    ret_code_t errCode;
    ble_KD2_cp_rsp_t rsp;

    rsp.opcode = BLE_KD2_CP_OP_SET_COM_PIN_CONFIG;
    rsp.status = BLE_KD2_CP_RSP_VAL_FAILED;

    if (pendingConnHandle == BLE_CONN_HANDLE_INVALID)
    {
        errCode = KD2_SetComPinMode(comPinMode, resultHandler);
        if (errCode == NRF_ERROR_INVALID_PARAM)
            rsp.status = BLE_KD2_CP_RSP_VAL_INVALID;
        else if (errCode == NRF_ERROR_INVALID_STATE)    // nothing changed
            rsp.status = BLE_KD2_CP_RSP_VAL_SUCCESS;
        else if (errCode == NRF_SUCCESS)
        {
            pendingConnHandle = connHandle;
            pendingOpCode = rsp.opcode;
            return; // response is sent in result handler
        }
    }

    errCode = ble_KD2_cp_response(&KD2Gatts, connHandle, &rsp);
    LOG_ERROR_CHECK("error %d sending com pin response", errCode);
}

static void onIntCompRequest(uint16_t connHandle)
{
    ret_code_t errCode;
    KD2_adcCompensation_t comp;
    ble_KD2_cp_rsp_t rsp;

    rsp.opcode = BLE_KD2_CP_OP_REQ_INT_COMP;

    errCode = KD2_GetCompensation(&comp);
    if (errCode == NRF_SUCCESS)
    {
        rsp.status = BLE_KD2_CP_RSP_VAL_SUCCESS;
        rsp.params.int_comp.voltage_gain = comp.voltage.gain;
        rsp.params.int_comp.voltage_offset = comp.voltage.offset;
        rsp.params.int_comp.current_gain = comp.current.gain;
        rsp.params.int_comp.current_offset = comp.current.offset;
        rsp.params.int_comp.temperature_gain = comp.temperature.gain;
        rsp.params.int_comp.temperature_offset = comp.temperature.offset;
    }
    else
    {
        rsp.status = BLE_KD2_CP_RSP_VAL_FAILED;
        NRF_LOG_ERROR("error %d getting internal compensation", errCode);
    }

    errCode = ble_KD2_cp_response(&KD2Gatts, connHandle, &rsp);
    LOG_ERROR_CHECK("error %d sending internal compensation response", errCode);
}

static void onIntCompSet(uint16_t connHandle, ble_KD2_int_comp_t const *pComp)
{
    ret_code_t errCode;
    KD2_adcCompensation_t comp;
    ble_KD2_cp_rsp_t rsp;

    rsp.opcode = BLE_KD2_CP_OP_SET_INT_COMP;
    rsp.status = BLE_KD2_CP_RSP_VAL_FAILED;

    if (pendingConnHandle == BLE_CONN_HANDLE_INVALID)
    {
        comp.voltage.gain = pComp->voltage_gain;
        comp.voltage.offset = pComp->voltage_offset;
        comp.current.gain = pComp->current_gain;
        comp.current.offset = pComp->current_offset;
        comp.temperature.gain = pComp->temperature_gain;
        comp.temperature.offset = pComp->temperature_offset;

        errCode = KD2_SetCompensation(&comp, resultHandler);
        if (errCode == NRF_ERROR_INVALID_PARAM)
            rsp.status = BLE_KD2_CP_RSP_VAL_INVALID;
        else if (errCode == NRF_ERROR_INVALID_STATE)  // nothing changed
            rsp.status = BLE_KD2_CP_RSP_VAL_SUCCESS;
        else if (errCode == NRF_SUCCESS)
        {
            pendingConnHandle = connHandle;
            pendingOpCode = rsp.opcode;
            return; // response is sent in result handler
        }
    }

    errCode = ble_KD2_cp_response(&KD2Gatts, connHandle, &rsp);
    LOG_ERROR_CHECK("error %d sending internal compensation response", errCode);
}

static void onExtCompRequest(uint16_t connHandle)
{
    ret_code_t errCode;
    hbd_calibData_t comp;
    ble_KD2_cp_rsp_t rsp;

    rsp.opcode = BLE_KD2_CP_OP_REQ_EXT_COMP;

    errCode = KD2_GetHelenaCompensation(&comp);
    if (errCode == NRF_SUCCESS)
    {
        rsp.status = BLE_KD2_CP_RSP_VAL_SUCCESS;
        rsp.params.ext_comp.temperature_offset = comp.temperatureOffset;
        rsp.params.ext_comp.left_current_gain = comp.gainLeft;
        rsp.params.ext_comp.right_current_gain = comp.gainRight;
    }
    else
    {
        rsp.status = BLE_KD2_CP_RSP_VAL_FAILED;
        NRF_LOG_ERROR("error %d getting external compensation", errCode);
    }

    errCode = ble_KD2_cp_response(&KD2Gatts, connHandle, &rsp);
    LOG_ERROR_CHECK("error %d sending external compensation response", errCode);
}

static void onExtCompSet(uint16_t connHandle, ble_KD2_ext_comp_t const *pComp)
{
    ret_code_t errCode;
    hbd_calibData_t comp;
    ble_KD2_cp_rsp_t rsp;

    rsp.opcode = BLE_KD2_CP_OP_SET_EXT_COMP;
    rsp.status = BLE_KD2_CP_RSP_VAL_FAILED;

    comp.temperatureOffset = pComp->temperature_offset;
    comp.gainLeft = pComp->left_current_gain;
    comp.gainRight = pComp->right_current_gain;

    errCode = KD2_SetHelenaCompensation(&comp);
    if (errCode == NRF_ERROR_INVALID_PARAM)
        rsp.status = BLE_KD2_CP_RSP_VAL_INVALID;
    else if (errCode == NRF_SUCCESS)
        rsp.status = BLE_KD2_CP_RSP_VAL_SUCCESS;

    errCode = ble_KD2_cp_response(&KD2Gatts, connHandle, &rsp);
    LOG_ERROR_CHECK("error %d sending external compensation response", errCode);
}

static void onImuStateRequest(uint16_t connHandle)
{
    ret_code_t errCode;
    ble_KD2_cp_rsp_t rsp;

    rsp.opcode = BLE_KD2_CP_OP_REQ_IMU_CALIB_STATE;
    rsp.status = BLE_KD2_CP_RSP_VAL_NOT_SUPPORTED;

    errCode = ble_KD2_cp_response(&KD2Gatts, connHandle, &rsp);
    LOG_ERROR_CHECK("error %d sending com pin response", errCode);
}

static void onImuCalibStart(uint16_t connHandle)
{
    ret_code_t errCode;
    ble_KD2_cp_rsp_t rsp;

    rsp.opcode = BLE_KD2_CP_OP_START_IMU_CALIB;
    rsp.status = BLE_KD2_CP_RSP_VAL_NOT_SUPPORTED;

    errCode = ble_KD2_cp_response(&KD2Gatts, connHandle, &rsp);
    LOG_ERROR_CHECK("error %d sending imu calib start response", errCode);
}

static void eventHandler(ble_KD2_evt_t const *pEvt)
{
    if (pEvt->evt_type != BLE_KD2_EVT_CP_EVT)
        return;

    switch (pEvt->cp_evt)
    {
    case BLE_KD2_CP_OP_REQ_CHN_CONFIG:
        onChannelSetupRequest(pEvt->p_client->conn_handle, pEvt->p_params->channel_config.channel);
        break;

    case BLE_KD2_CP_OP_SET_CHN_CONFIG:
        onChannelSetupSet(pEvt->p_client->conn_handle, pEvt->p_params->channel_config.channel, &pEvt->p_params->channel_config.config);
        break;

    case BLE_KD2_CP_OP_REQ_COM_PIN_CONFIG:
        onComPinRequest(pEvt->p_client->conn_handle);
        break;

    case BLE_KD2_CP_OP_SET_COM_PIN_CONFIG:
        onComPinSet(pEvt->p_client->conn_handle, pEvt->p_params->com_pin);
        break;

    case BLE_KD2_CP_OP_REQ_INT_COMP:
        onIntCompRequest(pEvt->p_client->conn_handle);
        break;

    case BLE_KD2_CP_OP_SET_INT_COMP:
        onIntCompSet(pEvt->p_client->conn_handle, &pEvt->p_params->int_comp);
        break;

    case BLE_KD2_CP_OP_REQ_EXT_COMP:
        onExtCompRequest(pEvt->p_client->conn_handle);
        break;

    case BLE_KD2_CP_OP_SET_EXT_COMP:
        onExtCompSet(pEvt->p_client->conn_handle, &pEvt->p_params->ext_comp);
        break;

    case BLE_KD2_CP_OP_REQ_IMU_CALIB_STATE:
        onImuStateRequest(pEvt->p_client->conn_handle);
        break;

    case BLE_KD2_CP_OP_START_IMU_CALIB:
        onImuCalibStart(pEvt->p_client->conn_handle);
        break;

    default:
        break;
    }
}

static void errorHandler(uint32_t errCode)
{
    LOG_ERROR_CHECK("KD2 service error %d", errCode);
}

static uint32_t initKD2Service(void* pContext)
{
    ble_KD2_init_t kd2Init = {0};

    if (init.pFtrs == NULL)
        return NRF_ERROR_NULL;

    // if com is available, battery soc might be available, so add battery service
    if (init.pFtrs->comSupported.rx != COM_PIN_NOT_USED)
    {
        ble_bas_init_t basInit = {0};

        uint8_t batteryLevel         = 0; /// TODO: is there a way to initialize with actual value?
        basInit.support_notification = true;
        basInit.initial_batt_level   = batteryLevel;
        basInit.bl_rd_sec            = SEC_OPEN;
        basInit.bl_cccd_wr_sec       = SEC_OPEN;
        ret_code_t errCode = ble_bas_init(&basGatts, &basInit);
        if (errCode != NRF_SUCCESS)
            return errCode;
    }

    // com pin is available last channel is a pwm channel
    bool comSupported = init.pFtrs->pChannelSize[init.pFtrs->channelCount - 1].channel_description == BLE_HPS_CH_DESC_PWM ? true : false;

    kd2Init.feature.config.channel_config_supported = 1;
    kd2Init.feature.config.com_pin_mode_supported   = comSupported ? 1 : 0;
    kd2Init.feature.channel.adaptive_supported      = init.isImuPresent ? 1 : 0;

#ifdef DEBUG_EXT
    // helena driver board is available with at least 3 channels and the second one is a current channel
    bool helenaSupported = init.pFtrs->channelCount >= 3 && init.pFtrs->pChannelSize[1].channel_description == BLE_HPS_CH_DESC_CURRENT ? true : false;

    kd2Init.feature.config.internal_comp_supported  = 1;
    kd2Init.feature.config.external_comp_supported  = helenaSupported ? 1 : 0;

#pragma message ( "debug configuration set to just works temporary" )
    kd2Init.KD2_cp_wr_sec      = SEC_JUST_WORKS;//SEC_OPEN;
    kd2Init.KD2_cp_cccd_wr_sec = SEC_JUST_WORKS;//SEC_OPEN;
#else
    kd2Init.KD2_cp_wr_sec      = SEC_JUST_WORKS;
    kd2Init.KD2_cp_cccd_wr_sec = SEC_JUST_WORKS;
#endif // DEBUG_EXT
    kd2Init.KD2_f_rd_sec       = SEC_OPEN;

    kd2Init.error_handler = errorHandler;
    kd2Init.evt_handler   = eventHandler;

    return ble_KD2_init(&KD2Gatts, ARRAY_SIZE(KD2Gatts_client_data), &kd2Init);
}

/* Public functions ----------------------------------------------------------*/
ret_code_t KD2_btle_Init(brd_features_t const* pFeatures, bool isImuPresent)//uint8_t channel_cnt, bool comSupported, bool helenaSupported)
{
    if (pFeatures == NULL)
        return NRF_ERROR_NULL;

    if (pFeatures->channelCount == 0 || pFeatures->channelCount > BLE_KD2_MAX_CHANNELS)
        return NRF_ERROR_INVALID_PARAM;

    init.pFtrs = pFeatures;
    init.isImuPresent = isImuPresent;

    // ble isn't initialized yet, impossible to add service here
    return NRF_SUCCESS;
}

ret_code_t KD2_btle_BatteryLevelUpdate(q8_t batterySoc)
{
    if (init.pFtrs->comSupported.rx == COM_PIN_NOT_USED)
        return NRF_ERROR_NOT_SUPPORTED;

    uint8_t batteryLevel = ((uint16_t)batterySoc * 100) >> 8;

    return ble_bas_battery_level_update(&basGatts, batteryLevel, BLE_CONN_HANDLE_ALL);
}

/**END OF FILE*****************************************************************/

