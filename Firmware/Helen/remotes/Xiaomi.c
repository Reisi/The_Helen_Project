/**
  ******************************************************************************
  * @file    Xiaomi.h
  * @author  Thomas Reisnecker
  * @brief   Xiaomi Yi Remote Control driver
  *
  * @note this is a project specific driver for the Xiaomi Yi Remote control.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "nrf_log.h"

#include "remote.h"
#include "hmi.h"
#include "button.h"
#include "debug.h"

/* Defines -------------------------------------------------------------------*/
#define UUID16_SIZE     2       /**< Size of 16 bit UUID. */
#define VOLUME_UP       (1<<6)
#define VOLUME_DOWN     (1<<7)

/* Read only variables -------------------------------------------------------*/
static const ble_hids_c_db_t device_handles =
{
    .hid_info_handle                 = 0x001F,
    .report_map_handle               = 0x0023,
    {
        {
            .report_handle           = 0x002C,
            .cccd_handle             = 0x002E,
            .report_reference_handle = 0,
        },
        {
            .report_handle           = 0x0030,
            .cccd_handle             = 0,
            .report_reference_handle = 0,
        },
        {
            .report_handle           = 0x0033,
            .cccd_handle             = 0x0035,
            .report_reference_handle = 0,
        }
    }
};

/* Private variables ---------------------------------------------------------*/
static uint8_t evalIdMain = BUTTON_EVALUATE_CNT;
static uint8_t evalIdSec  = BUTTON_EVALUATE_CNT;

/* Private functions ---------------------------------------------------------*/
static bool isInit()
{
    return !(evalIdMain == BUTTON_EVALUATE_CNT || evalIdSec == BUTTON_EVALUATE_CNT);
}

static void buttonEval(uint8_t evalId, but_evalEvent_t evt, uint16_t stillPressCnt)
{
    ret_code_t errCode;

    if (evalId == evalIdMain)
    {
        switch (evt)
        {
        case BUTTON_CLICKSHORT:
            errCode = hmi_RequestEvent(HMI_EVT_NEXTMODE, NULL);
            LOG_ERROR_CHECK("[XDRV]: hmi request error %d", errCode);
            break;
        case BUTTON_CLICKLONG:
            errCode = hmi_RequestEvent(HMI_EVT_NEXTGROUP, NULL);
            LOG_ERROR_CHECK("[XDRV]: hmi request error %d", errCode);
            break;
        default:
            break;
        }
    }
    else if (evalId == evalIdSec)
    {
        switch (evt)
        {
        case BUTTON_CLICKSHORT:
        case BUTTON_CLICKLONG:
            errCode = hmi_RequestEvent(HMI_EVT_MODEPREF, NULL);
            LOG_ERROR_CHECK("[XDRV]: hmi request error %d", errCode);
            break;
        case BUTTON_IS_STILL_PRESSED:
            if (stillPressCnt == MSEC_TO_UNITS(500, UNIT_10_MS))
            {
                errCode = hmi_RequestEvent(HMI_EVT_MODETEMP, NULL);
                LOG_ERROR_CHECK("[XDRV]: hmi request error %d", errCode);
            }
            break;
        case BUTTON_IS_RELEASED:
            errCode = hmi_RequestEvent(HMI_EVT_MODETEMP, NULL);
            LOG_ERROR_CHECK("[XDRV]: hmi request error %d", errCode);
            break;
        default:
            break;
        }
    }
}

static void init()
{
    but_evaluateInit_t evalInit = {0};
    evalInit.reportHandler      = buttonEval;
    evalInit.longClickCnt       = MSEC_TO_UNITS(500, UNIT_10_MS);
    evalInit.StillPressedPeriod = MSEC_TO_UNITS(2000, UNIT_10_MS);
    evalInit.abortCnt           = MSEC_TO_UNITS(30000, UNIT_10_MS);
    evalIdMain = but_EvaluateInit(&evalInit);

    evalInit.longClickCnt       = MSEC_TO_UNITS(250, UNIT_10_MS);
    evalInit.StillPressedPeriod = MSEC_TO_UNITS(500, UNIT_10_MS);
    evalInit.abortCnt           = MSEC_TO_UNITS(30000, UNIT_10_MS);
    evalIdSec = but_EvaluateInit(&evalInit);
}

/* Exported functions --------------------------------------------------------*/
bool rem_xiaomi_is_device(uint8_t const* p_data, uint16_t len)
{
    uint_fast8_t index = 0;

    bool match_uuid = false, match_name = false;

    while (index < len)
    {
        uint8_t length = p_data[index];
        uint8_t type   = p_data[index+1];

        // check if the device advertises as a HID device
        if (type == BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_COMPLETE ||
            type == BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_MORE_AVAILABLE)
        {
            for (uint_fast8_t i = 0; i < length/UUID16_SIZE; i++)
            {
                uint16_t uuid;
                uuid = uint16_decode(&p_data[index + 2 + i * UUID16_SIZE]);
                if (uuid == BLE_UUID_HUMAN_INTERFACE_DEVICE_SERVICE)
                    match_uuid = true;
            }
        }
        // check if the device name matches
        else if (type == BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME ||
                 type == BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME)
        {
            if (strncmp("XiaoYi_RC", (char*)&p_data[index + 2], strlen("XiaoYi")) == 0)
                match_name = true;
        }

        index += length + 1;
    }
    return match_uuid && match_name;
}

ble_hids_c_db_t const* rem_xiaomi_get_handles(void)
{
    return &device_handles;
}

uint32_t rem_xiaomi_notif_enable(ble_hids_c_t * p_ble_hids_c, bool enable)
{
    if (!isInit())
        init();

    return ble_hids_c_report_notif_enable(p_ble_hids_c, 2, enable);
}

void rem_xiaomi_on_hids_c_evt(ble_hids_c_t * p_ble_hids_c, ble_hids_c_evt_t * p_evt)
{
    (void)p_ble_hids_c;

    if (p_evt->evt_type != BLE_HIDS_C_EVT_REPORT_NOTIFICATION || p_evt->params.report.index != 2 || !isInit())
    {
        return;
    }

    static uint8_t last_state;
    uint8_t change, state;
    uint32_t err_code;

    state = p_evt->params.report.p_report[0];
    change = last_state ^ state;
    last_state = state;

    if (change & VOLUME_UP)
    {
        err_code = but_EvaluateButtonChange(evalIdMain, (state & VOLUME_UP) ? BUTTON_PRESSED : BUTTON_RELEASED);
        LOG_ERROR_CHECK("[XDRV]: evaluate button change error %d", err_code);
    }
    if (change & VOLUME_DOWN)
    {
        err_code = but_EvaluateButtonChange(evalIdSec, (state & VOLUME_DOWN) ? BUTTON_PRESSED : BUTTON_RELEASED);
        LOG_ERROR_CHECK("[XDRV]: evaluate button change error %d", err_code);
    }

    /*if (change & VOLUME_UP && state & VOLUME_UP)
    {
        err_code = hmi_RequestEvent(HMI_EVT_NEXTMODE, 0);
        APP_ERROR_CHECK(err_code);
    }
    if (change & VOLUME_DOWN)
    {
        err_code = hmi_RequestEvent(HMI_EVT_MODEPREF, 0);
        APP_ERROR_CHECK(err_code);
    }*/
}

/* section variable ----------------------------------------------------------*/
REMOTE_REGISTER(const rem_driver_t rem_xiaomi) =
{
    .is_device = rem_xiaomi_is_device,
    .get_handles = rem_xiaomi_get_handles,
    .notif_enable = rem_xiaomi_notif_enable,
    .evt_handler = rem_xiaomi_on_hids_c_evt,
};

/**END OF FILE*****************************************************************/
