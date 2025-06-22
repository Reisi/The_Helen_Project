/**
  ******************************************************************************
  * @file    Drake_service.c
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
#include "ble_drake.h"
#include "ble_bas.h"
#include "Drake_10.h"
//#include "plunger_control.h"
#include "btle.h"

/* External variables --------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
    brd_features_t const* pFtrs;
    q8_t batterySoc;
    bool isActuatorAvailable;
    bool isMotorSensorPresent;
    bool isTravelSensorPresent;
} init_t;

/* Private macros ------------------------------------------------------------*/

/* Private defines -----------------------------------------------------------*/
static uint32_t initDrakeService(void* pContext);

/* Private variables ---------------------------------------------------------*/
static init_t init;
BLE_DRAKE_DEF(drakeGatts, NRF_SDH_BLE_TOTAL_LINK_COUNT);
BLE_BAS_DEF(basGatts);
BTLE_SERVICE_REGISTER(drakeService, initDrakeService, &init);
static uint16_t pendingConnHandle = BLE_CONN_HANDLE_INVALID;
static ble_drake_cp_op_code_t pendingOpCode;

/* Private read only variables -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
static void resultHandler(ret_code_t errCode)
{
    ble_drake_cp_rsp_t rsp;

    if (pendingConnHandle == BLE_CONN_HANDLE_INVALID)
        return; // something went wrong

    rsp.opcode = pendingOpCode;
    rsp.status = errCode == NRF_SUCCESS ? BLE_DRAKE_CP_RSP_VAL_SUCCESS : BLE_DRAKE_CP_RSP_VAL_FAILED;

    if (rsp.opcode == BLE_DRAKE_CP_OP_START_MOTOR_SENS_CALIB && rsp.status == BLE_DRAKE_CP_RSP_VAL_SUCCESS)
    {
        /// TODO: send new data
    }

    errCode = ble_drake_cp_response(&drakeGatts, pendingConnHandle, &rsp);   /// TODO: try again in case of error?
    LOG_ERROR_CHECK("error %d sending response in result handler", errCode);

    pendingConnHandle = BLE_CONN_HANDLE_INVALID;
}

static void onUsageRequest(uint16_t connHandle)
{
    ret_code_t errCode;
    drake_usage_t usage;
    ble_drake_cp_rsp_t rsp;

    rsp.opcode = BLE_DRAKE_CP_OP_REQ_DEV_USAGE;

    usage = Drake_GetDeviceUsage();

    rsp.params.device_usage = usage;
    rsp.status = BLE_DRAKE_CP_RSP_VAL_SUCCESS;

    errCode = ble_drake_cp_response(&drakeGatts, connHandle, &rsp);
    LOG_ERROR_CHECK("error %d sending request usage response", errCode);
}

static void onUsageSet(uint16_t connHandle, ble_drake_cp_dev_usage_t usage)
{
    ret_code_t errCode;
    ble_drake_cp_rsp_t rsp;

    rsp.opcode = BLE_DRAKE_CP_OP_SET_DEV_USAGE;
    rsp.status = BLE_DRAKE_CP_RSP_VAL_FAILED;

    if (pendingConnHandle == BLE_CONN_HANDLE_INVALID)
    {
        errCode = Drake_SetDeviceUsage(usage, resultHandler);
        if (errCode == NRF_SUCCESS)
        {
            pendingConnHandle = connHandle;
            pendingOpCode = rsp.opcode;
            return; // response is sent in result handler
        }
    }

    errCode = ble_drake_cp_response(&drakeGatts, connHandle, &rsp);
    LOG_ERROR_CHECK("error %d sending set usage response", errCode);
}

static void onBatInfoRequest(uint16_t connHandle)
{
    ret_code_t errCode;
    drake_batteryInfo_t batInfo;
    ble_drake_cp_rsp_t rsp;

    rsp.opcode = BLE_DRAKE_CP_OP_REQ_BAT_INFO;

    errCode = Drake_GetBatteryInfo(&batInfo);
    if (errCode == NRF_SUCCESS)
    {
        rsp.params.battery_info.type = batInfo.type;
        rsp.params.battery_info.cap_nom = batInfo.capNom >> 16;
        rsp.params.battery_info.cap_act = batInfo.capAct >> 16;
        rsp.status = BLE_DRAKE_CP_RSP_VAL_SUCCESS;
    }
    else
        rsp.status = BLE_DRAKE_CP_RSP_VAL_FAILED;

    errCode = ble_drake_cp_response(&drakeGatts, connHandle, &rsp);
    LOG_ERROR_CHECK("error %d sending request battery info response", errCode);
}

static void onBatInfoSet(uint16_t connHandle, ble_drake_cp_bat_info_t const* pBatInfo)
{
    ret_code_t errCode;
    ble_drake_cp_rsp_t rsp;
    drake_batteryInfo_t batInfo;

    rsp.opcode = BLE_DRAKE_CP_OP_SET_BAT_INFO;
    rsp.status = BLE_DRAKE_CP_RSP_VAL_FAILED;

    if (pendingConnHandle == BLE_CONN_HANDLE_INVALID)
    {
        batInfo.type = pBatInfo->type;
        batInfo.capAct = pBatInfo->cap_act << 16;
        batInfo.capNom = pBatInfo->cap_nom << 16;

        errCode = Drake_SetBatteryInfo(&batInfo, resultHandler);
        if (errCode == NRF_SUCCESS)
        {
            pendingConnHandle = connHandle;
            pendingOpCode = rsp.opcode;
            return; // response is sent in result handler
        }
    }

    errCode = ble_drake_cp_response(&drakeGatts, connHandle, &rsp);
    LOG_ERROR_CHECK("error %d sending set battery info response", errCode);
}

static void onSocTimeoutRequest(uint16_t connHandle)
{
    ret_code_t errCode;
    uint32_t socTimeout;
    ble_drake_cp_rsp_t rsp;

    rsp.opcode = BLE_DRAKE_CP_OP_REQ_SOC_TIMEOUT;

    errCode = Drake_GetSocTimeout(&socTimeout);
    if (errCode == NRF_SUCCESS)
    {
        rsp.params.soc_timeout = socTimeout >> 4;
        rsp.status = BLE_DRAKE_CP_RSP_VAL_SUCCESS;
    }
    else if (errCode == NRF_ERROR_NOT_SUPPORTED)
        rsp.status = BLE_DRAKE_CP_RSP_VAL_NOT_SUPPORTED;
    else
        rsp.status = BLE_DRAKE_CP_RSP_VAL_FAILED;

    errCode = ble_drake_cp_response(&drakeGatts, connHandle, &rsp);
    LOG_ERROR_CHECK("error %d sending request soc timeout response", errCode);
}

static void onSocTimeoutSet(uint16_t connHandle, uint16_t socTimeout)
{
    ret_code_t errCode;
    ble_drake_cp_rsp_t rsp;

    rsp.opcode = BLE_DRAKE_CP_OP_SET_SOC_TIMEOUT;
    rsp.status = BLE_DRAKE_CP_RSP_VAL_FAILED;

    if (pendingConnHandle == BLE_CONN_HANDLE_INVALID)
    {
        errCode = Drake_SetSocTimeout(socTimeout << 4, resultHandler);
        if (errCode == NRF_SUCCESS)
        {
            pendingConnHandle = connHandle;
            pendingOpCode = rsp.opcode;
            return; // response is sent in result handler
        }
        else if (errCode == NRF_ERROR_NOT_SUPPORTED)
            rsp.status = BLE_DRAKE_CP_RSP_VAL_NOT_SUPPORTED;
    }

    errCode = ble_drake_cp_response(&drakeGatts, connHandle, &rsp);
    LOG_ERROR_CHECK("error %d sending set battery info response", errCode);
}

/*static ret_code_t getPatternTypeFromOpCode(ble_drake_cp_op_code_t opCode, Drake_lightType_t* pType)
{
    switch (opCode)
    {
    case BLE_DRAKE_CP_OP_REQ_TAIL_PATTERN:
    case BLE_DRAKE_CP_OP_SET_TAIL_PATTERN:
        *pType = DRAKE_LIGHT_REAR;
        return NRF_SUCCESS;

    case BLE_DRAKE_CP_OP_REQ_FRONT_PATTERN:
    case BLE_DRAKE_CP_OP_SET_FRONT_PATTERN:
        *pType = DRAKE_LIGHT_FRONT;
        return NRF_SUCCESS;

    case BLE_DRAKE_CP_OP_REQ_POS_PATTERN:
    case BLE_DRAKE_CP_OP_SET_POS_PATTERN:
        *pType = DRAKE_LIGHT_POS;
        return NRF_SUCCESS;

    case BLE_DRAKE_CP_OP_REQ_BRAKE_PATTERN:
    case BLE_DRAKE_CP_OP_SET_BRAKE_PATTERN:
        *pType = DRAKE_LIGHT_BRAKE;
        return NRF_SUCCESS;

    case BLE_DRAKE_CP_OP_REQ_LEFT_INDIC_PATTERN:
    case BLE_DRAKE_CP_OP_SET_LEFT_INDIC_PATTERN:
        *pType = DRAKE_LIGHT_INDIC_LEFT;
        return NRF_SUCCESS;

    case BLE_DRAKE_CP_OP_REQ_RIGHT_INDIC_PATTERN:
    case BLE_DRAKE_CP_OP_SET_RIGHT_INDIC_PATTERN:
        *pType = DRAKE_LIGHT_INDIC_RIGHT;
        return NRF_SUCCESS;

    default:
        return NRF_ERROR_INVALID_PARAM;
    }
}*/

static void onPatternRequest(uint16_t connHandle, ble_drake_cp_light_type_t type)
{
    ret_code_t errCode;
    ble_drake_cp_rsp_t rsp;
    Drake_lightPattern_t pattern;

    rsp.opcode = BLE_DRAKE_CP_OP_REQ_LIGHT_PATTERN;
    rsp.params.pattern.type = type;

    errCode = Drake_GetLightPattern((Drake_lightType_t)type, &pattern);
    if (errCode == NRF_SUCCESS)
    {
        rsp.status = BLE_DRAKE_CP_RSP_VAL_SUCCESS;
        rsp.params.pattern.p_pattern = pattern.pPattern;
        rsp.params.pattern.len = pattern.len;
    }
    else if (errCode == NRF_ERROR_INVALID_PARAM)
        rsp.status = BLE_DRAKE_CP_RSP_VAL_INVALID;
    else
        rsp.status = BLE_DRAKE_CP_RSP_VAL_FAILED;

    errCode = ble_drake_cp_response(&drakeGatts, connHandle, &rsp);
    LOG_ERROR_CHECK("error %d sending request pattern response", errCode);
}

static void onPatternSet(uint16_t connHandle, ble_drake_cp_light_type_t type, ble_drake_cp_pattern_t const* pPattern)
{
    ret_code_t errCode;
    ble_drake_cp_rsp_t rsp;

    rsp.opcode = BLE_DRAKE_CP_OP_SET_LIGHT_PATTERN;
    rsp.status = BLE_DRAKE_CP_RSP_VAL_FAILED;

    if (pendingConnHandle == BLE_CONN_HANDLE_INVALID)
    {
        errCode = Drake_SetLightPattern((Drake_lightType_t)type, (Drake_lightPattern_t const*)pPattern, resultHandler);
        if (errCode == NRF_SUCCESS)
        {
            pendingConnHandle = connHandle;
            pendingOpCode = rsp.opcode;
            return; // response is sent in result handler
        }
        else if (errCode == NRF_ERROR_INVALID_PARAM)
            rsp.status = BLE_DRAKE_CP_RSP_VAL_INVALID;
    }

    errCode = ble_drake_cp_response(&drakeGatts, connHandle, &rsp);
    LOG_ERROR_CHECK("error %d sending set charge current response", errCode);
}



static void onSetPlungerPosition(uint16_t connHandle, uint8_t position)
{
    ret_code_t errCode;
    q8_t plPos;
    ble_drake_cp_rsp_t rsp;

    rsp.opcode = BLE_DRAKE_CP_OP_SET_PL_POS;

    if (position > 200)
        rsp.opcode = BLE_DRAKE_CP_RSP_VAL_INVALID;
    else
    {
        plPos = position >= 200 ? 0xFF : (position * 256ul) / 200;

        errCode = Drake_SetPlungerPosition(plPos, true);
        if (errCode == NRF_SUCCESS)
            rsp.status = BLE_DRAKE_CP_RSP_VAL_SUCCESS;
        else
            rsp.status = BLE_DRAKE_CP_RSP_VAL_FAILED;
    }

    errCode = ble_drake_cp_response(&drakeGatts, connHandle, &rsp);
    LOG_ERROR_CHECK("error %d sending plunger position response", errCode);
}

static void onMotorConfigRequest(uint16_t connHandle)
{
    ret_code_t errCode;
    mtCtrl_motorConfig_t motorConfig;
    ble_drake_cp_rsp_t rsp;

    rsp.opcode = BLE_DRAKE_CP_OP_REQ_MOTOR_CONFIG;
    errCode = Drake_GetMotorConfig(&motorConfig);
    if (errCode == NRF_SUCCESS)
    {
        rsp.status = BLE_DRAKE_CP_RSP_VAL_SUCCESS;
        rsp.params.motor_config.open_timeout = motorConfig.openTimeout >> 4;
        rsp.params.motor_config.close_timeout = motorConfig.closeTimeout >> 4;
    }
    else if (errCode == NRF_ERROR_INVALID_PARAM)
        rsp.status = BLE_DRAKE_CP_RSP_VAL_INVALID;
    else if (errCode == NRF_ERROR_NOT_SUPPORTED)
        rsp.status = BLE_DRAKE_CP_RSP_VAL_NOT_SUPPORTED;
    else
        rsp.status = BLE_DRAKE_CP_RSP_VAL_FAILED;

    errCode = ble_drake_cp_response(&drakeGatts, connHandle, &rsp);
    LOG_ERROR_CHECK("error %d sending request motor config response", errCode);
}

static void onMotorConfigSet(uint16_t connHandle, ble_drake_cp_motor_config_t const* pMotorConfig)
{
    ret_code_t errCode;
    ble_drake_cp_rsp_t rsp;
    mtCtrl_motorConfig_t motorConfig;

    rsp.opcode = BLE_DRAKE_CP_OP_SET_MOTOR_CONFIG;
    rsp.status = BLE_DRAKE_CP_RSP_VAL_FAILED;

    if (pendingConnHandle == BLE_CONN_HANDLE_INVALID)
    {
        errCode = Drake_GetMotorConfig(&motorConfig);
        if (errCode == NRF_SUCCESS)
        {
            motorConfig.openTimeout = pMotorConfig->open_timeout << 4;
            motorConfig.closeTimeout = pMotorConfig->close_timeout << 4;

            errCode = Drake_SetMotorConfiguration(&motorConfig, resultHandler);
            if (errCode == NRF_SUCCESS)
            {
                pendingConnHandle = connHandle;
                pendingOpCode = rsp.opcode;
                return; // response is sent in result handler
            }
        }
        else if (errCode == NRF_ERROR_INVALID_PARAM)
            rsp.status = BLE_DRAKE_CP_RSP_VAL_INVALID;
        else if (errCode == NRF_ERROR_NOT_SUPPORTED)
            rsp.status = BLE_DRAKE_CP_RSP_VAL_NOT_SUPPORTED;
    }

    errCode = ble_drake_cp_response(&drakeGatts, connHandle, &rsp);
    LOG_ERROR_CHECK("error %d sending set motor config response", errCode);
}

static void onMotorSensorCalibDataRequest(uint16_t connHandle, ble_drake_cp_motor_calib_type_t type)
{
    ret_code_t errCode;
    drake_motorSensCalibData_t calibData;
    ble_drake_cp_rsp_t rsp;

    rsp.opcode = BLE_DRAKE_CP_OP_REQ_MOTOR_SENS_CALIB;
    rsp.params.motor_calib.type = type;
    errCode = Drake_GetMotorCalibrationData((drake_motorSensCalibType_t)type, &calibData);
    if (errCode == NRF_SUCCESS)
    {
        rsp.status = BLE_DRAKE_CP_RSP_VAL_SUCCESS;
        switch (type)
        {
        case BLE_DRAKE_MOTOR_CALIB_TYPE_COMP_THRESHOLDS:
            rsp.params.motor_calib.params.thresholds.upper = calibData.threshold.upper;
            rsp.params.motor_calib.params.thresholds.lower = calibData.threshold.lower;
            break;

        case BLE_DRAKE_MOTOR_CALIB_TYPE_FULL_OPEN_CNT:
            rsp.params.motor_calib.params.full_open_cnt = calibData.openCnt;
            break;

        //default:
        //    break;
        }
    }
    else if (errCode == NRF_ERROR_INVALID_PARAM)
        rsp.status = BLE_DRAKE_CP_RSP_VAL_INVALID;
    else if (errCode == NRF_ERROR_NOT_SUPPORTED)
        rsp.status = BLE_DRAKE_CP_RSP_VAL_NOT_SUPPORTED;
    else
        rsp.status = BLE_DRAKE_CP_RSP_VAL_FAILED;

    errCode = ble_drake_cp_response(&drakeGatts, connHandle, &rsp);
    LOG_ERROR_CHECK("error %d sending request motor sensor calib response", errCode);
}

static void onMotorSensorCalibStart(uint16_t connHandle, ble_drake_cp_motor_calib_type_t type, bool startOrClear)
{
    ret_code_t errCode;
    ble_drake_cp_rsp_t rsp;

    rsp.opcode = BLE_DRAKE_CP_OP_START_MOTOR_SENS_CALIB;
    rsp.status = BLE_DRAKE_CP_RSP_VAL_FAILED;

    if (pendingConnHandle == BLE_CONN_HANDLE_INVALID)
    {
        if (startOrClear)
            errCode = Drake_StartMotorCalibration((drake_motorSensCalibType_t)type, resultHandler);
        else
            errCode = Drake_ClearMotorCalibrationData((drake_motorSensCalibType_t)type, resultHandler);

        if (errCode == NRF_SUCCESS)
        {
            pendingConnHandle = connHandle;
            pendingOpCode = rsp.opcode;
            return; // response is sent in result handler
        }
        else if (errCode == NRF_ERROR_INVALID_PARAM)
            rsp.status = BLE_DRAKE_CP_RSP_VAL_INVALID;
        else if (errCode == NRF_ERROR_NOT_SUPPORTED)
            rsp.status = BLE_DRAKE_CP_RSP_VAL_NOT_SUPPORTED;
    }

    errCode = ble_drake_cp_response(&drakeGatts, connHandle, &rsp);
    LOG_ERROR_CHECK("error %d sending start motor sens calib response", errCode);
}

static void onTravelSensorRequest(uint16_t connHandle)
{
    ret_code_t errCode;
    drake_travelSensorData_t sensData;
    ble_drake_cp_rsp_t rsp;

    rsp.opcode = BLE_DRAKE_CP_OP_REQ_TRAVEL_SENS_DATA;
    errCode = Drake_GetTravelSensorData(&sensData);
    if (errCode == NRF_SUCCESS)
    {
        rsp.status = BLE_DRAKE_CP_RSP_VAL_SUCCESS;
        rsp.params.travel_sens_data.travel = sensData.travelInmm;
        rsp.params.travel_sens_data.offset = sensData.offsetInMikrom;
    }
    else if (errCode == NRF_ERROR_INVALID_PARAM)
        rsp.status = BLE_DRAKE_CP_RSP_VAL_INVALID;
    else if (errCode == NRF_ERROR_NOT_SUPPORTED)
        rsp.status = BLE_DRAKE_CP_RSP_VAL_NOT_SUPPORTED;
    else
        rsp.status = BLE_DRAKE_CP_RSP_VAL_FAILED;

    errCode = ble_drake_cp_response(&drakeGatts, connHandle, &rsp);
    LOG_ERROR_CHECK("error %d sending request travel sensor data response", errCode);
}

static void onTravelSensorCalibStart(uint16_t connHandle, uint16_t travel)
{
    ret_code_t errCode;
    ble_drake_cp_rsp_t rsp;

    rsp.opcode = BLE_DRAKE_CP_OP_START_TRAVEL_SENS_CALIB;
    rsp.status = BLE_DRAKE_CP_RSP_VAL_FAILED;

    if (pendingConnHandle == BLE_CONN_HANDLE_INVALID)
    {
        if (travel)
            errCode = Drake_StartTravelSensorCalibration(travel, resultHandler);
        else
            errCode = Drake_ClearTravelSensorData(resultHandler);

        if (errCode == NRF_SUCCESS)
        {
            pendingConnHandle = connHandle;
            pendingOpCode = rsp.opcode;
            return; // response is sent in result handler
        }
        else if (errCode == NRF_ERROR_INVALID_PARAM)
            rsp.status = BLE_DRAKE_CP_RSP_VAL_INVALID;
        else if (errCode == NRF_ERROR_NOT_SUPPORTED)
            rsp.status = BLE_DRAKE_CP_RSP_VAL_NOT_SUPPORTED;
    }

    errCode = ble_drake_cp_response(&drakeGatts, connHandle, &rsp);
    LOG_ERROR_CHECK("error %d sending start travel sens calib response", errCode);
}

static void onIndexedConfigRequest(uint16_t connHandle)
{
    ret_code_t errCode;
    drake_indexedTravelConfig_t config;
    ble_drake_cp_rsp_t rsp;

    rsp.opcode = BLE_DRAKE_CP_OP_REQ_INDEXED_CONFIG;
    errCode = Drake_GetIndexedTravelConfig(&config);
    if (errCode == NRF_SUCCESS)
    {
        rsp.status = BLE_DRAKE_CP_RSP_VAL_SUCCESS;
        rsp.params.indexed_config.blocking_time = config.blockingTime >> 4;
        rsp.params.indexed_config.timeout = config.timeout;
        for (uint8_t i = 0; i < ARRAY_SIZE(rsp.params.indexed_config.positions); i++)
        {
            rsp.params.indexed_config.positions[i] = i < ARRAY_SIZE(config.positions) ? config.positions[i] : 0;
        }
    }
    else if (errCode == NRF_ERROR_INVALID_PARAM)
        rsp.status = BLE_DRAKE_CP_RSP_VAL_INVALID;
    else if (errCode == NRF_ERROR_NOT_SUPPORTED)
        rsp.status = BLE_DRAKE_CP_RSP_VAL_NOT_SUPPORTED;
    else
        rsp.status = BLE_DRAKE_CP_RSP_VAL_FAILED;

    errCode = ble_drake_cp_response(&drakeGatts, connHandle, &rsp);
    LOG_ERROR_CHECK("error %d sending request travel sensor data response", errCode);
}

static void onIndexedConfigSet(uint16_t connHandle, ble_drake_cp_indexed_config_t const* pConfig)
{
    ret_code_t errCode;
    ble_drake_cp_rsp_t rsp;
    drake_indexedTravelConfig_t config;

    rsp.opcode = BLE_DRAKE_CP_OP_SET_INDEXED_CONFIG;
    rsp.status = BLE_DRAKE_CP_RSP_VAL_FAILED;

    if (pendingConnHandle == BLE_CONN_HANDLE_INVALID)
    {
        config.blockingTime = pConfig->blocking_time << 4;
        config.timeout = pConfig->timeout << 4;
        for (uint8_t i = 0; i < ARRAY_SIZE(config.positions); i++)
        {
            config.positions[i] = i < ARRAY_SIZE(pConfig->positions) ? pConfig->positions[i] : 0;
        }

        errCode = Drake_SetIndexedTravelConfig(&config, resultHandler);
        if (errCode == NRF_SUCCESS)
        {
            pendingConnHandle = connHandle;
            pendingOpCode = rsp.opcode;
            return; // response is sent in result handler
        }
        else if (errCode == NRF_ERROR_INVALID_PARAM)
            rsp.status = BLE_DRAKE_CP_RSP_VAL_INVALID;
        else if (errCode == NRF_ERROR_NOT_SUPPORTED)
            rsp.status = BLE_DRAKE_CP_RSP_VAL_NOT_SUPPORTED;
    }

    errCode = ble_drake_cp_response(&drakeGatts, connHandle, &rsp);
    LOG_ERROR_CHECK("error %d sending indexed config response", errCode);
}

static void eventHandler(ble_drake_evt_t const *pEvt)
{
    if (pEvt->evt_type != BLE_DRAKE_EVT_CP_EVT)
        return;

    switch (pEvt->cp_evt)
    {
    case BLE_DRAKE_CP_OP_REQ_DEV_USAGE:
        onUsageRequest(pEvt->p_client->conn_handle);
        break;

    case BLE_DRAKE_CP_OP_SET_DEV_USAGE:
        onUsageSet(pEvt->p_client->conn_handle, pEvt->p_params->device_usage);
        break;

    case BLE_DRAKE_CP_OP_REQ_BAT_INFO:
        onBatInfoRequest(pEvt->p_client->conn_handle);
        break;

    case BLE_DRAKE_CP_OP_SET_BAT_INFO:
        onBatInfoSet(pEvt->p_client->conn_handle, &pEvt->p_params->battery_info);
        break;

    case BLE_DRAKE_CP_OP_REQ_SOC_TIMEOUT:
        onSocTimeoutRequest(pEvt->p_client->conn_handle);
        break;

    case BLE_DRAKE_CP_OP_SET_SOC_TIMEOUT:
        onSocTimeoutSet(pEvt->p_client->conn_handle, pEvt->p_params->soc_timeout);
        break;

    case BLE_DRAKE_CP_OP_REQ_LIGHT_PATTERN:
        onPatternRequest(pEvt->p_client->conn_handle, pEvt->p_params->pattern.type);
        break;

    case BLE_DRAKE_CP_OP_SET_LIGHT_PATTERN:
        onPatternSet(pEvt->p_client->conn_handle, pEvt->p_params->pattern.type, &pEvt->p_params->pattern);
        break;

    case BLE_DRAKE_CP_OP_SET_PL_POS:
        onSetPlungerPosition(pEvt->p_client->conn_handle, pEvt->p_params->pl_pos);
        break;

    case BLE_DRAKE_CP_OP_REQ_MOTOR_CONFIG:
        onMotorConfigRequest(pEvt->p_client->conn_handle);
        break;

    case BLE_DRAKE_CP_OP_SET_MOTOR_CONFIG:
        onMotorConfigSet(pEvt->p_client->conn_handle, &pEvt->p_params->motor_config);
        break;

    case BLE_DRAKE_CP_OP_REQ_MOTOR_SENS_CALIB:
        onMotorSensorCalibDataRequest(pEvt->p_client->conn_handle, pEvt->p_params->motor_calib.type);
        break;

    case BLE_DRAKE_CP_OP_START_MOTOR_SENS_CALIB:
        onMotorSensorCalibStart(pEvt->p_client->conn_handle, pEvt->p_params->motor_calib.type, pEvt->p_params->motor_calib.params.start_clear);
        break;

    case BLE_DRAKE_CP_OP_REQ_TRAVEL_SENS_DATA:
        onTravelSensorRequest(pEvt->p_client->conn_handle);
        break;

    case BLE_DRAKE_CP_OP_START_TRAVEL_SENS_CALIB:
        onTravelSensorCalibStart(pEvt->p_client->conn_handle, pEvt->p_params->travel_sens_data.travel);
        break;

    case BLE_DRAKE_CP_OP_REQ_INDEXED_CONFIG:
        onIndexedConfigRequest(pEvt->p_client->conn_handle);
        break;

    case BLE_DRAKE_CP_OP_SET_INDEXED_CONFIG:
        onIndexedConfigSet(pEvt->p_client->conn_handle, &pEvt->p_params->indexed_config);
        break;

    default:
        break;
    }
}

static void errorHandler(uint32_t errCode)
{
    LOG_ERROR_CHECK("drake service error %d", errCode);
}

static uint32_t initDrakeService(void* pContext)
{
    uint32_t errCode;
    ble_drake_init_t drakeInit = {0};
    ble_bas_init_t basInit = {0};

    if (init.pFtrs == NULL)
        return NRF_ERROR_NULL;

    // add battery service
    uint8_t batteryLevel         = ((uint16_t)init.batterySoc * 100) >> 8;
    basInit.support_notification = true;
    basInit.initial_batt_level   = batteryLevel;
    basInit.bl_rd_sec            = SEC_OPEN;
    basInit.bl_cccd_wr_sec       = SEC_OPEN;
    errCode = ble_bas_init(&basGatts, &basInit);
    if (errCode != NRF_SUCCESS)
        return errCode;

    drakeInit.feature.device.light_supported = init.pFtrs->channelCount ? 1 : 0;
    drakeInit.feature.device.seatpost_actuator_supported = init.isActuatorAvailable ? 1 : 0;

    drakeInit.feature.light.tail_light_pattern_supported = init.pFtrs->channelCount ? 1 : 0;
    drakeInit.feature.light.front_light_pattern_supported = init.pFtrs->channelCount ? 1 : 0;
    drakeInit.feature.light.pos_light_pattern_supported = init.pFtrs->channelCount ? 1 : 0;
    drakeInit.feature.light.brake_light_pattern_supported = init.pFtrs->channelCount ? 1 : 0;
    drakeInit.feature.light.left_indic_pattern_supported = init.pFtrs->channelCount ? 1 : 0;
    drakeInit.feature.light.right_indic_pattern_supported = init.pFtrs->channelCount ? 1 : 0;

    drakeInit.feature.actuator.motor_config_supported = init.isActuatorAvailable ? 1 : 0;
    drakeInit.feature.actuator.motor_sensor_supported = init.isMotorSensorPresent ? 1 : 0;
    drakeInit.feature.actuator.travel_sensor_supported = init.isTravelSensorPresent ? 1 : 0;
    drakeInit.feature.actuator.travel_indexing_supported = init.isTravelSensorPresent ? 1 : 0;

#ifdef DEBUG_EXT
#pragma message ( "debug configuration set to just works temporary" )
    drakeInit.drake_s_cccd_wr_sec  = SEC_JUST_WORKS;//SEC_OPEN;
    drakeInit.drake_cp_wr_sec      = SEC_JUST_WORKS;//SEC_OPEN;
    drakeInit.drake_cp_cccd_wr_sec = SEC_JUST_WORKS;//SEC_OPEN;
#else
    drakeInit.drake_s_cccd_wr_sec  = SEC_JUST_WORKS;
    drakeInit.drake_cp_wr_sec      = SEC_JUST_WORKS;
    drakeInit.drake_cp_cccd_wr_sec = SEC_JUST_WORKS;
#endif // DEBUG_EXT
    drakeInit.drake_s_rd_sec       = SEC_OPEN;
    drakeInit.drake_f_rd_sec       = SEC_OPEN;

    drakeInit.error_handler = errorHandler;
    drakeInit.evt_handler = eventHandler;

    errCode = ble_drake_init(&drakeGatts, ARRAY_SIZE(drakeGatts_client_data), &drakeInit);
    if (errCode != NRF_SUCCESS)
        return errCode;

    return NRF_SUCCESS;
}

/* Public functions ----------------------------------------------------------*/
ret_code_t Drake_btle_Init(brd_features_t const* pFeatures, q8_t batterySoc, bool isActuatorPresent, bool isMotorSensorPresent, bool isTravelSensorPresent)
{
    if (pFeatures == NULL)
        return NRF_ERROR_NULL;

    init.pFtrs = pFeatures;
    init.batterySoc = batterySoc;
    init.isActuatorAvailable = isActuatorPresent;
    init.isMotorSensorPresent = isMotorSensorPresent;
    init.isTravelSensorPresent = isTravelSensorPresent;

    // ble isn't initialized yet, impossible to add service here
    return NRF_SUCCESS;
}

ret_code_t Drake_btle_BatteryLevelUpdate(q8_t batterySoc)
{
    uint8_t batteryLevel = ((uint16_t)batterySoc * 100) >> 8;

    return ble_bas_battery_level_update(&basGatts, batteryLevel, BLE_CONN_HANDLE_ALL);
}

/// TODO: update function s for motor position and travel

/**END OF FILE*****************************************************************/

