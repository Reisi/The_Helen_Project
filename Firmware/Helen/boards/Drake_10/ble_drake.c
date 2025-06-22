/**
  ******************************************************************************
  * @file    ble_drake.c
  * @author  Thomas Reisnecker
  * @brief   Drake Control Service implementation
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "ble_hps.h"
#include "ble_drake.h"
#include <string.h>

/* Private defines -----------------------------------------------------------*/
#define BLE_UUID_DRAKE_SERVICE            0x0601  // The UUID of the Drake Control Service
#define BLE_UUID_DRAKE_S_CHARACTERISTIC   0x0602  // The UUID of the Drake Status characteristic
#define BLE_UUID_DRAKE_F_CHARACTERISTIC   0x0603  // The UUID of the Drake feature characteristic
#define BLE_UUID_DRAKE_CP_CHARACTERISTIC  0x0604  // The UUID of the Drake control point characteristic

#define BLE_DRAKE_S_MIN_CHAR_LEN          1       // minimum length of the status characteristic */
#define BLE_DRAKE_S_MAX_CHAR_LEN          2       // maximum length of the status characteristic */

#define DRAKE_DEF_F_LIGHT_MASK            (1 << 0)// light supported
#define DRAKE_DEF_F_SP_ACT_MASK           (1 << 1)// seat-post actuator supported

#define DRAKE_LGHT_F_FRONT_PATTERN_MASK   (1 << 0)
#define DRAKE_LGHT_F_REAR_PATTERN_MASK    (1 << 1)
#define DRAKE_LGHT_F_POS_PATTERN_MASK     (1 << 2)
#define DRAKE_LGHT_F_BRAKE_PATTERN_MASK   (1 << 3)
#define DRAKE_LGHT_F_LEFT_PATTERN_MASK    (1 << 4)
#define DRAKE_LGHT_F_RIGHT_PATTERN_MASK   (1 << 5)

#define DRAKE_ACT_F_MTR_CFG_MASK          (1 << 0)
#define DRAKE_ACT_F_MTR_SENS_MASK         (1 << 1)
#define DRAKE_ACT_F_TRV_SENS_MASK         (1 << 2)
#define DRAKE_ACT_F_INDEXED_TRV_MASK      (1 << 3)

#define SIZE_OF_PL_POS                    1
#define SIZE_OF_USAGE                     1
#define SIZE_OF_BAT_INFO_REQ              5
#define SIZE_OF_BAT_INFO_SET              3
#define SIZE_OF_SOC_TIMEOUT               2
#define SIZE_OF_MOTOR_CONFIG              4
#define SIZE_OF_MOTOR_SENS_CALIB          2
#define SIZE_OF_TRAVEL_SENS_CLEAR         0
#define SIZE_OF_TRAVEL_SENS_CALIB_START   2
#define SIZE_OF_TRAVEL_SENS_DATA          4
#define SIZE_OF_INDEXED_CONFIG_MIN        4
#define SIZE_OF_INDEXED_CONFIG_MAX        (SIZE_OF_INDEXED_CONFIG_MIN + BLE_DRAKE_CP_MAX_INDEXED_POS * sizeof(uint16_t))

/* Private macros -----------------------------------------------------------*/
#define VERIFY_LEN(actual, target)      \
do                                      \
{                                       \
    if (target != actual)               \
    {                                   \
        return NRF_ERROR_INVALID_PARAM; \
    }                                   \
} while (0)

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
    ble_drake_cp_op_code_t opcode;
    ble_drake_cp_params_t  params;
} cp_val_t;

/* Private variables ---------------------------------------------------------*/
static uint8_t m_num_of_clients;

/* Private functions ---------------------------------------------------------*/
static uint32_t status_char_add(ble_drake_t * p_drake, security_req_t rd_sec, security_req_t cccd_wr_sec)
{
    ble_add_char_params_t add_char_params;
    uint8_t               init_value[BLE_DRAKE_S_MIN_CHAR_LEN] = {0};

    /// TODO: initial values?

    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid              = BLE_UUID_DRAKE_S_CHARACTERISTIC;
    add_char_params.uuid_type         = p_drake->uuid_type;
    add_char_params.init_len          = BLE_DRAKE_S_MIN_CHAR_LEN;
    add_char_params.max_len           = BLE_DRAKE_S_MAX_CHAR_LEN;
    add_char_params.is_var_len        = true;
    add_char_params.p_init_value      = init_value;
    add_char_params.char_props.notify = 1;
    add_char_params.cccd_write_access = cccd_wr_sec;
    add_char_params.read_access       = rd_sec;

    return characteristic_add(p_drake->service_handle, &add_char_params, &p_drake->s_handles);
}

static uint32_t feature_char_add(ble_drake_t * p_drake, security_req_t rd_sec)
{
    ble_add_char_params_t add_char_params;
    uint8_t dev_flags = 0;
    uint8_t light_flags = 0;
    uint8_t act_flags = 0;
    uint8_t init_value_encoded[3];

    // device features
    if (p_drake->feature.device.light_supported)
    {
        dev_flags |= DRAKE_DEF_F_LIGHT_MASK;
    }
    if (p_drake->feature.device.seatpost_actuator_supported)
    {
        dev_flags |= DRAKE_DEF_F_SP_ACT_MASK;
    }

    init_value_encoded[0] = dev_flags;

    // light features
    if (p_drake->feature.light.tail_light_pattern_supported)
    {
        light_flags |= DRAKE_LGHT_F_REAR_PATTERN_MASK;
    }
    if (p_drake->feature.light.front_light_pattern_supported)
    {
        light_flags |= DRAKE_LGHT_F_FRONT_PATTERN_MASK;
    }
    if (p_drake->feature.light.pos_light_pattern_supported)
    {
        light_flags |= DRAKE_LGHT_F_POS_PATTERN_MASK;
    }
    if (p_drake->feature.light.brake_light_pattern_supported)
    {
        light_flags |= DRAKE_LGHT_F_BRAKE_PATTERN_MASK;
    }
    if (p_drake->feature.light.left_indic_pattern_supported)
    {
        light_flags |= DRAKE_LGHT_F_LEFT_PATTERN_MASK;
    }
    if (p_drake->feature.light.right_indic_pattern_supported)
    {
        light_flags |= DRAKE_LGHT_F_RIGHT_PATTERN_MASK;
    }

    init_value_encoded[1] = light_flags;

    // actuator features
    if (p_drake->feature.actuator.motor_config_supported)
    {
        act_flags |= DRAKE_ACT_F_MTR_CFG_MASK;
    }
    if (p_drake->feature.actuator.motor_sensor_supported)
    {
        act_flags |= DRAKE_ACT_F_MTR_SENS_MASK;
    }
    if (p_drake->feature.actuator.travel_sensor_supported)
    {
        act_flags |= DRAKE_ACT_F_TRV_SENS_MASK;
    }
    if (p_drake->feature.actuator.travel_indexing_supported)
    {
        act_flags |= DRAKE_ACT_F_INDEXED_TRV_MASK;
    }

    init_value_encoded[2] = act_flags;

    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid            = BLE_UUID_DRAKE_F_CHARACTERISTIC;
    add_char_params.uuid_type       = p_drake->uuid_type;
    add_char_params.init_len        = sizeof(init_value_encoded);
    add_char_params.max_len         = sizeof(init_value_encoded);
    add_char_params.p_init_value    = init_value_encoded;
    add_char_params.char_props.read = 1;
    add_char_params.read_access     = rd_sec;

    return characteristic_add(p_drake->service_handle, &add_char_params, &p_drake->f_handles);
}

static uint32_t control_point_char_add(ble_drake_t * p_drake, security_req_t wr_sec, security_req_t cccd_wr_sec)
{
    ble_add_char_params_t add_char_params;

    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid                = BLE_UUID_DRAKE_CP_CHARACTERISTIC;
    add_char_params.uuid_type           = p_drake->uuid_type;
    add_char_params.max_len             = BLE_DRAKE_CP_MAX_LEN;
    add_char_params.is_var_len          = true;
    add_char_params.char_props.indicate = 1;
    add_char_params.char_props.write    = 1;
    add_char_params.is_defered_write    = true;
    add_char_params.write_access        = wr_sec;
    add_char_params.cccd_write_access   = cccd_wr_sec;

    return characteristic_add(p_drake->service_handle, &add_char_params, &p_drake->cp_handles);
}

static ble_drake_client_spec_t * get_client_data_by_conn_handle(uint16_t conn_handle, ble_drake_client_spec_t * p_context)
{
    uint_fast8_t index = m_num_of_clients;

    while (index--)
    {
        if (p_context[index].conn_handle == conn_handle)
        {
            return &p_context[index];
        }
    }

    return NULL;
}

static bool is_notification_enabled(ble_drake_t const * p_drake, uint16_t conn_handle)
{
    uint32_t err_code;
    uint8_t  cccd_value_buf[BLE_CCCD_VALUE_LEN];
    bool     is_cccp_notify_enabled = false;
    ble_gatts_value_t gatts_value;

    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len     = BLE_CCCD_VALUE_LEN;
    gatts_value.offset  = 0;
    gatts_value.p_value = cccd_value_buf;

    err_code = sd_ble_gatts_value_get(conn_handle, p_drake->s_handles.cccd_handle, &gatts_value);
    if (err_code != NRF_SUCCESS)
    {
        // Report error to application
        if (p_drake->error_handler != NULL)
        {
            p_drake->error_handler(err_code);
        }
        return false;
    }

    is_cccp_notify_enabled = ble_srv_is_notification_enabled(cccd_value_buf);

    return is_cccp_notify_enabled;
}

static bool is_indication_enabled(ble_drake_t const * p_drake, uint16_t conn_handle)
{
    uint32_t err_code;
    uint8_t  cccd_value_buf[BLE_CCCD_VALUE_LEN];
    bool     is_cccp_indic_enabled = false;
    ble_gatts_value_t gatts_value;

    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len     = BLE_CCCD_VALUE_LEN;
    gatts_value.offset  = 0;
    gatts_value.p_value = cccd_value_buf;

    err_code = sd_ble_gatts_value_get(conn_handle, p_drake->cp_handles.cccd_handle, &gatts_value);
    if (err_code != NRF_SUCCESS)
    {
        // Report error to application
        if (p_drake->error_handler != NULL)
        {
            p_drake->error_handler(err_code);
        }
        return false;
    }

    is_cccp_indic_enabled = ble_srv_is_indication_enabled(cccd_value_buf);

    return is_cccp_indic_enabled;
}

static void on_connect(ble_drake_t const * p_drake, ble_evt_t const * p_ble_evt)
{
    ble_drake_evt_t           evt;
    ble_drake_client_spec_t * p_client;

    p_client = get_client_data_by_conn_handle(BLE_CONN_HANDLE_INVALID, p_drake->p_client);
    if (p_client == NULL)
    {
        if (p_drake->error_handler != NULL)
        {
            p_drake->error_handler(NRF_ERROR_NOT_FOUND);
        }
        return;
    }
    p_client->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

    if (p_drake->evt_handler)
    {
        return;
    }

    // check the hosts CCCD value to inform the application if it has to send indications or notifications
   if (is_notification_enabled(p_drake, p_ble_evt->evt.gap_evt.conn_handle))
    {
        p_client->is_drake_s_notify_enabled = true;

        memset(&evt, 0, sizeof(ble_drake_evt_t));
        evt.evt_type    = BLE_DRAKE_EVT_S_NOTIFICATION_ENABLED;
        evt.p_drake     = p_drake;
        evt.p_client    = p_client;

        p_drake->evt_handler(&evt);
    }

    if (is_indication_enabled(p_drake, p_ble_evt->evt.gap_evt.conn_handle))
    {
        p_client->is_drake_cp_indic_enabled = true;

        memset(&evt, 0, sizeof(ble_drake_evt_t));
        evt.evt_type    = BLE_DRAKE_EVT_CP_INDICATION_ENABLED;
        evt.p_drake     = p_drake;
        evt.p_client    = p_client;

        p_drake->evt_handler(&evt);
    }
}

static void on_disconnect(ble_drake_t * p_drake, ble_evt_t const * p_ble_evt)
{
    ble_drake_evt_t evt;
    ble_drake_client_spec_t * p_client;

    p_client = get_client_data_by_conn_handle(p_ble_evt->evt.gap_evt.conn_handle, p_drake->p_client);
    if (p_client == NULL)
    {
        return;
    }

    if (p_client->is_drake_s_notify_enabled)
    {
        p_client->is_drake_s_notify_enabled = false;

        if (p_drake->evt_handler != NULL)
        {
            memset(&evt, 0, sizeof(ble_drake_evt_t));
            evt.evt_type    = BLE_DRAKE_EVT_S_NOTIFICATION_DISABLED;
            evt.p_drake     = p_drake;
            evt.p_client    = p_client;

            p_drake->evt_handler(&evt);
        }
    }

    if (p_client->is_drake_cp_indic_enabled)
    {
        p_client->is_drake_cp_indic_enabled = false;

        if (p_drake->evt_handler != NULL)
        {
            memset(&evt, 0, sizeof(ble_drake_evt_t));
            evt.evt_type    = BLE_DRAKE_EVT_CP_INDICATION_DISABLED;
            evt.p_drake     = p_drake;
            evt.p_client    = p_client;

            p_drake->evt_handler(&evt);
        }
    }

    p_client->conn_handle = BLE_CONN_HANDLE_INVALID;
    p_client->procedure_status = BLE_DRAKE_CP_PROC_STATUS_FREE;
}

static void on_s_cccd_write(ble_drake_t * p_drake, ble_gatts_evt_write_t const * p_evt_write, uint16_t conn_handle)
{
    ble_drake_evt_t evt;
    ble_drake_client_spec_t * p_client;

    p_client = get_client_data_by_conn_handle(conn_handle, p_drake->p_client);
    if (p_client == NULL)
    {
        if (p_drake->error_handler != NULL)
        {
            p_drake->error_handler(NRF_ERROR_NOT_FOUND);
        }
        return;
    }

    if (p_evt_write->len == 2)
    {
        // CCCD written, update notification state
        memset(&evt, 0, sizeof(ble_drake_evt_t));
        evt.p_drake     = p_drake;
        evt.p_client    = p_client;


        if (ble_srv_is_notification_enabled(p_evt_write->data))
        {
            p_client->is_drake_s_notify_enabled = true;
            evt.evt_type = BLE_DRAKE_EVT_S_NOTIFICATION_ENABLED;
        }
        else
        {
            p_client->is_drake_s_notify_enabled = false;
            evt.evt_type = BLE_DRAKE_EVT_S_NOTIFICATION_DISABLED;
        }

        if (p_drake->evt_handler != NULL)
        {
            p_drake->evt_handler(&evt);
        }
    }
}

static void on_cp_cccd_write(ble_drake_t * p_drake, ble_gatts_evt_write_t const * p_evt_write, uint16_t conn_handle)
{
    ble_drake_evt_t evt;
    ble_drake_client_spec_t * p_client;

    p_client = get_client_data_by_conn_handle(conn_handle, p_drake->p_client);
    if (p_client == NULL)
    {
        if (p_drake->error_handler != NULL)
        {
            p_drake->error_handler(NRF_ERROR_NOT_FOUND);
        }
        return;
    }

    if (p_evt_write->len == 2)
    {
        // CCCD written, update indication state
        memset(&evt, 0, sizeof(ble_drake_evt_t));
        evt.p_drake     = p_drake;
        evt.p_client    = p_client;


        if (ble_srv_is_indication_enabled(p_evt_write->data))
        {
            p_client->is_drake_cp_indic_enabled = true;
            evt.evt_type = BLE_DRAKE_EVT_CP_INDICATION_ENABLED;
        }
        else
        {
            p_client->is_drake_cp_indic_enabled = false;
            evt.evt_type = BLE_DRAKE_EVT_CP_INDICATION_DISABLED;
        }

        if (p_drake->evt_handler != NULL)
        {
            p_drake->evt_handler(&evt);
        }
    }
}

static void on_write(ble_drake_t * p_drake, ble_gatts_evt_t const * p_gatts_evt)
{
    ble_gatts_evt_write_t const * p_evt_write = &p_gatts_evt->params.write;

    if (p_evt_write->handle == p_drake->s_handles.cccd_handle)
    {
        on_s_cccd_write(p_drake, p_evt_write, p_gatts_evt->conn_handle);
    }

    if (p_evt_write->handle == p_drake->cp_handles.cccd_handle)
    {
        on_cp_cccd_write(p_drake, p_evt_write, p_gatts_evt->conn_handle);
    }
}

uint32_t cp_decode(uint8_t const     * p_rcvd_val,
                   uint8_t             len,
                   ble_drake_t const * p_drake,
                   cp_val_t          * p_val)
{
   uint8_t pos = 0;

   if (len < BLE_DRAKE_CP_MIN_LEN)
   {
       return NRF_ERROR_INVALID_PARAM;
   }

   p_val->opcode = (ble_drake_cp_op_code_t)p_rcvd_val[pos++];

   switch (p_val->opcode)
   {
    case BLE_DRAKE_CP_OP_SET_DEV_USAGE:
        VERIFY_LEN(len, 1 + SIZE_OF_USAGE);
        p_val->params.device_usage = p_rcvd_val[pos];
        return NRF_SUCCESS;

    case BLE_DRAKE_CP_OP_SET_BAT_INFO:
        VERIFY_LEN(len, 1 + SIZE_OF_BAT_INFO_SET);
        p_val->params.battery_info.type = p_rcvd_val[pos++];
        p_val->params.battery_info.cap_nom = uint16_decode(&p_rcvd_val[pos]);
        return NRF_SUCCESS;

    case BLE_DRAKE_CP_OP_SET_SOC_TIMEOUT:
        VERIFY_LEN(len, 1 + SIZE_OF_SOC_TIMEOUT);
        p_val->params.soc_timeout = uint16_decode(&p_rcvd_val[pos]);
        return NRF_SUCCESS;

    case BLE_DRAKE_CP_OP_SET_LIGHT_PATTERN:
        if (len > BLE_DRAKE_CP_MAX_LEN)
        {
            return NRF_ERROR_INVALID_PARAM;
        }
        p_val->params.pattern.type = p_rcvd_val[pos++];
        p_val->params.pattern.p_pattern = &p_rcvd_val[pos];
        p_val->params.pattern.len = len - 2;
        return NRF_SUCCESS;

    case BLE_DRAKE_CP_OP_SET_MOTOR_CONFIG:
        VERIFY_LEN(len, 1 + SIZE_OF_MOTOR_CONFIG);
        p_val->params.motor_config.open_timeout = uint16_decode(&p_rcvd_val[pos]);
        pos += 2;
        p_val->params.motor_config.close_timeout = uint16_decode(&p_rcvd_val[pos]);
        pos += 2;
        return NRF_SUCCESS;

    case BLE_DRAKE_CP_OP_START_MOTOR_SENS_CALIB:
        VERIFY_LEN(len, 1 + SIZE_OF_MOTOR_SENS_CALIB);
        p_val->params.motor_calib.type = p_rcvd_val[pos++];
        p_val->params.motor_calib.params.start_clear = p_rcvd_val[pos];
        return NRF_SUCCESS;

    case BLE_DRAKE_CP_OP_REQ_LIGHT_PATTERN:
        VERIFY_LEN(len, 2);
        p_val->params.pattern.type = p_rcvd_val[pos++];
        return NRF_SUCCESS;

    case BLE_DRAKE_CP_OP_REQ_MOTOR_SENS_CALIB:
        VERIFY_LEN(len, 2);
        p_val->params.motor_calib.type = p_rcvd_val[pos++];
        return NRF_SUCCESS;

    case BLE_DRAKE_CP_OP_START_TRAVEL_SENS_CALIB:
        if (len == 1 + SIZE_OF_TRAVEL_SENS_CLEAR)
        {
            p_val->params.travel_sens_data.travel = 0;
        }
        else if (len == 1 + SIZE_OF_TRAVEL_SENS_CALIB_START)
        {
            p_val->params.travel_sens_data.travel = uint16_decode(&p_rcvd_val[pos]);
        }
        else
        {
            return NRF_ERROR_INVALID_PARAM;
        }
        return NRF_SUCCESS;

    case BLE_DRAKE_CP_OP_SET_INDEXED_CONFIG:
        if ((len < 1 + SIZE_OF_INDEXED_CONFIG_MIN) || (len > 1 + SIZE_OF_INDEXED_CONFIG_MAX))
        {
            return NRF_ERROR_INVALID_PARAM;
        }
        p_val->params.indexed_config.blocking_time = uint16_decode(&p_rcvd_val[pos]);
        pos += 2;
        p_val->params.indexed_config.timeout = uint16_decode(&p_rcvd_val[pos]);
        pos += 2;
        for (uint8_t i = 0; i < BLE_DRAKE_CP_MAX_INDEXED_POS; i++)
        {
            if (pos >= len)
            {
                p_val->params.indexed_config.positions[i] = 0;
            }
            else
            {
                p_val->params.indexed_config.positions[i] = uint16_decode(&p_rcvd_val[pos]);
                pos += 2;
            }
        }
        return NRF_SUCCESS;

    case BLE_DRAKE_CP_OP_REQ_DEV_USAGE:
    case BLE_DRAKE_CP_OP_REQ_BAT_INFO:
    case BLE_DRAKE_CP_OP_REQ_SOC_TIMEOUT:
    case BLE_DRAKE_CP_OP_REQ_MOTOR_CONFIG:
    case BLE_DRAKE_CP_OP_REQ_TRAVEL_SENS_DATA:
    case BLE_DRAKE_CP_OP_REQ_INDEXED_CONFIG:
        VERIFY_LEN(len, 1);
        return NRF_SUCCESS;

    default:
        return NRF_ERROR_INVALID_PARAM;
   }
}

static bool is_feature_supported(ble_drake_f_t const * p_feature, ble_drake_cp_op_code_t opcode, ble_drake_cp_light_type_t light_type)
{
    /// TODO: double check config features with device features?

    switch (opcode)
    {
    case BLE_DRAKE_CP_OP_REQ_DEV_USAGE:
    case BLE_DRAKE_CP_OP_SET_DEV_USAGE:
        return true;

    case BLE_DRAKE_CP_OP_REQ_BAT_INFO:
    case BLE_DRAKE_CP_OP_SET_BAT_INFO:
        return true;

    case BLE_DRAKE_CP_OP_REQ_SOC_TIMEOUT:
    case BLE_DRAKE_CP_OP_SET_SOC_TIMEOUT:
        return true;

    case BLE_DRAKE_CP_OP_REQ_LIGHT_PATTERN:
    case BLE_DRAKE_CP_OP_SET_LIGHT_PATTERN:
    {
        switch (light_type)
        {
        case BLE_DRAKE_LIGHT_TYPE_FRONT:
            return p_feature->light.front_light_pattern_supported;

        case BLE_DRAKE_LIGHT_TYPE_REAR:
            return p_feature->light.tail_light_pattern_supported;

        case BLE_DRAKE_LIGHT_TYPE_POS:
            return p_feature->light.pos_light_pattern_supported;

        case BLE_DRAKE_LIGHT_TYPE_BRAKE:
            return p_feature->light.brake_light_pattern_supported;

        case BLE_DRAKE_LIGHT_TYPE_LEFT_IND:
            return p_feature->light.left_indic_pattern_supported;

        case BLE_DRAKE_LIGHT_TYPE_RIGHT_IND:
            return p_feature->light.right_indic_pattern_supported;

        //default:
        //    return false;
        }
    }

    case BLE_DRAKE_CP_OP_SET_PL_POS:
        return p_feature->device.seatpost_actuator_supported;

    case BLE_DRAKE_CP_OP_REQ_MOTOR_CONFIG:
    case BLE_DRAKE_CP_OP_SET_MOTOR_CONFIG:
        return p_feature->actuator.motor_config_supported;

    case BLE_DRAKE_CP_OP_REQ_MOTOR_SENS_CALIB:
    case BLE_DRAKE_CP_OP_START_MOTOR_SENS_CALIB:
        return p_feature->actuator.motor_sensor_supported;

    case BLE_DRAKE_CP_OP_REQ_TRAVEL_SENS_DATA:
    case BLE_DRAKE_CP_OP_START_TRAVEL_SENS_CALIB:
        return p_feature->actuator.travel_sensor_supported;

    case BLE_DRAKE_CP_OP_REQ_INDEXED_CONFIG:
    case BLE_DRAKE_CP_OP_SET_INDEXED_CONFIG:
        return p_feature->actuator.travel_indexing_supported;

    default:
        return false;
   }
}

static void on_cp_write(ble_drake_t * p_drake, ble_gatts_evt_write_t const * p_evt_write, uint16_t conn_handle)
{
    uint32_t                              err_code;
    ble_gatts_rw_authorize_reply_params_t auth_reply;
    ble_drake_client_spec_t             * p_client;
    ble_drake_evt_t                       evt;
    cp_val_t                              decoded;

    p_client = get_client_data_by_conn_handle(conn_handle, p_drake->p_client);
    if (p_client == NULL)
    {
        if (p_drake->error_handler != NULL)
        {
            p_drake->error_handler(NRF_ERROR_NOT_FOUND);
        }
    }

    auth_reply.type                     = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
    auth_reply.params.write.offset      = 0;
    auth_reply.params.write.len         = 0;
    auth_reply.params.write.p_data      = NULL;
    auth_reply.params.write.update      = 1;

    if (p_client == NULL)
    {
        auth_reply.params.write.gatt_status = BLE_GATT_STATUS_UNKNOWN;
    }
    else if (is_indication_enabled(p_drake, conn_handle))
    {
        if (p_client->procedure_status == BLE_DRAKE_CP_PROC_STATUS_FREE)
        {
            auth_reply.params.write.gatt_status = BLE_GATT_STATUS_SUCCESS;
        }
        else
        {
            auth_reply.params.write.gatt_status = BLE_GATT_STATUS_ATTERR_CPS_PROC_ALR_IN_PROG;
        }
    }
    else
    {
        auth_reply.params.write.gatt_status = BLE_GATT_STATUS_ATTERR_CPS_CCCD_CONFIG_ERROR;
    }

    err_code = sd_ble_gatts_rw_authorize_reply(conn_handle, &auth_reply);
    if (err_code != NRF_SUCCESS)
    {
        // Report error to application.
        if (p_drake->error_handler != NULL)
        {
            p_drake->error_handler(err_code);
        }
        return;
    }

    if (auth_reply.params.write.gatt_status != BLE_GATT_STATUS_SUCCESS || p_client == NULL)
    {
        return;
    }

    p_client->procedure_status = BLE_DRAKE_CP_PROC_IN_PROGRESS;

    err_code = cp_decode(p_evt_write->data, p_evt_write->len, p_drake, &decoded);
    if (err_code != NRF_SUCCESS)
    {
        ble_drake_cp_rsp_t rsp;
        rsp.opcode = decoded.opcode;
        rsp.status = BLE_DRAKE_CP_RSP_VAL_INVALID;
        err_code = ble_drake_cp_response(p_drake, conn_handle, &rsp);
        if (err_code != NRF_SUCCESS && p_drake->error_handler)
        {
            p_drake->error_handler(err_code);
        }
    }
    else if (p_drake->evt_handler && is_feature_supported(&p_drake->feature, decoded.opcode, decoded.params.pattern.type))
    {
        evt.evt_type = BLE_DRAKE_EVT_CP_EVT;
        evt.p_drake = p_drake;
        evt.p_client = p_client;
        evt.cp_evt = decoded.opcode;
        evt.p_params = &decoded.params;

        p_drake->evt_handler(&evt);
    }
    else
    {
        ble_drake_cp_rsp_t rsp;
        rsp.opcode = decoded.opcode;
        rsp.status = BLE_DRAKE_CP_RSP_VAL_NOT_SUPPORTED;
        err_code = ble_drake_cp_response(p_drake, conn_handle, &rsp);
        if (err_code != NRF_SUCCESS && p_drake->error_handler)
        {
            p_drake->error_handler(err_code);
        }
    }
}

static void on_rw_authorize_request(ble_drake_t * p_drake, ble_gatts_evt_t const * p_gatts_evt)
{
    ble_gatts_evt_rw_authorize_request_t const * p_auth_req = &p_gatts_evt->params.authorize_request;

    if (p_auth_req->type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
    {
        if ((p_auth_req->request.write.op != BLE_GATTS_OP_PREP_WRITE_REQ) &&
            (p_auth_req->request.write.op != BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) &&
            (p_auth_req->request.write.op != BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
        {
            if (p_auth_req->request.write.handle == p_drake->cp_handles.value_handle)
            {
                on_cp_write(p_drake, &p_auth_req->request.write, p_gatts_evt->conn_handle);
            }
        }
    }
}

static uint16_t s_encode(ble_drake_s_t const * p_data, uint8_t * p_buffer)
{
    uint16_t len = 0;

    union
    {
        ble_drake_s_flags_t raw;
        uint8_t           encoded;
    } flags;

    flags.raw = p_data->flags;
    p_buffer[len++] = flags.encoded;

    if (flags.raw.pl_status_present)
    {
        p_buffer[len++] = p_data->pl_status;
    }

    return len;
}

static uint8_t cp_encode(ble_drake_cp_rsp_t const* p_rsp, ble_drake_t const * p_drake, uint8_t * p_data)
{
    uint8_t len = 0;

    p_data[len++] = BLE_DRAKE_CP_OP_RSP_CODE;
    p_data[len++] = p_rsp->opcode;
    p_data[len++] = p_rsp->status;

    if (p_rsp->status == BLE_DRAKE_CP_RSP_VAL_SUCCESS)
    {
        switch (p_rsp->opcode)
        {
        case BLE_DRAKE_CP_OP_REQ_DEV_USAGE:
            p_data[len++] = p_rsp->params.device_usage;
            break;

        case BLE_DRAKE_CP_OP_REQ_BAT_INFO:
            p_data[len++] = p_rsp->params.battery_info.type;
            len += uint16_encode(p_rsp->params.battery_info.cap_nom, &p_data[len]);
            len += uint16_encode(p_rsp->params.battery_info.cap_act, &p_data[len]);
            break;

        case BLE_DRAKE_CP_OP_REQ_SOC_TIMEOUT:
            len += uint16_encode(p_rsp->params.soc_timeout, &p_data[len]);
            break;

        case BLE_DRAKE_CP_OP_REQ_LIGHT_PATTERN:
            p_data[len++] = p_rsp->params.pattern.type;
            for (uint16_t i = 0; i < p_rsp->params.pattern.len; i++)
            {
                p_data[len++] = p_rsp->params.pattern.p_pattern[i];
            }
            break;

        case BLE_DRAKE_CP_OP_REQ_MOTOR_CONFIG:
            len += uint16_encode(p_rsp->params.motor_config.open_timeout, &p_data[len]);
            len += uint16_encode(p_rsp->params.motor_config.close_timeout, &p_data[len]);
            break;

        case BLE_DRAKE_CP_OP_REQ_MOTOR_SENS_CALIB:
        case BLE_DRAKE_CP_OP_START_MOTOR_SENS_CALIB:
            p_data[len++] = p_rsp->params.motor_calib.type;
            switch (p_rsp->params.motor_calib.type)
            {
            case BLE_DRAKE_MOTOR_CALIB_TYPE_COMP_THRESHOLDS:
                p_data[len++] = p_rsp->params.motor_calib.params.thresholds.upper;
                p_data[len++] = p_rsp->params.motor_calib.params.thresholds.lower;
                break;

            case BLE_DRAKE_MOTOR_CALIB_TYPE_FULL_OPEN_CNT:
                len += uint16_encode(p_rsp->params.motor_calib.params.full_open_cnt, &p_data[len]);
                break;

            default:
                break;
            }
            break;

        case BLE_DRAKE_CP_OP_REQ_TRAVEL_SENS_DATA:
        case BLE_DRAKE_CP_OP_START_TRAVEL_SENS_CALIB:
            len += uint16_encode(p_rsp->params.travel_sens_data.travel, &p_data[len]);
            len += uint32_encode(p_rsp->params.travel_sens_data.offset, &p_data[len]);
            break;

        case BLE_DRAKE_CP_OP_REQ_INDEXED_CONFIG:
            len += uint16_encode(p_rsp->params.indexed_config.blocking_time, &p_data[len]);
            len += uint16_encode(p_rsp->params.indexed_config.timeout, &p_data[len]);
            for (uint8_t i = 0; i < BLE_DRAKE_CP_MAX_INDEXED_POS; i++)
            {
                if (p_rsp->params.indexed_config.positions[i] != 0)
                {
                    len += uint16_encode(p_rsp->params.indexed_config.positions[i], &p_data[len]);
                }
                else
                {
                    break;
                }
            }
            break;

        default:
            break;
        }
    }

    return len;
}

static void cp_rsp_send(ble_drake_t const * p_drake, uint16_t conn_handle)
{
    uint32_t                  err_code;
    uint16_t                  hvx_len;
    ble_gatts_hvx_params_t    hvx_params;
    ble_drake_client_spec_t * p_client;

    p_client = get_client_data_by_conn_handle(conn_handle, p_drake->p_client);
    if (p_client == NULL)
    {
        if (p_drake->error_handler != NULL)
        {
            p_drake->error_handler(NRF_ERROR_NOT_FOUND);
        }
        return;
    }

    if ((p_client->procedure_status == BLE_DRAKE_CP_RSP_CODE_IND_PENDING))
    {
        hvx_len = p_client->response.len;
        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_drake->cp_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_INDICATION;
        hvx_params.offset = 0;
        hvx_params.p_len  = &hvx_len;
        hvx_params.p_data = p_client->response.encoded_rsp;

        err_code = sd_ble_gatts_hvx(conn_handle, &hvx_params);

        // Error handling
        if ((err_code == NRF_SUCCESS) && (hvx_len != p_client->response.len))
        {
            err_code = NRF_ERROR_DATA_SIZE;
        }

        switch (err_code)
        {
            case NRF_SUCCESS:
                p_client->procedure_status = BLE_DRAKE_CP_RSP_CODE_IND_CONFIRM_PENDING;
                // Wait for HVC event
                break;

            case NRF_ERROR_RESOURCES:
                // Wait for TX_COMPLETE event to retry transmission
                p_client->procedure_status = BLE_DRAKE_CP_RSP_CODE_IND_PENDING;
                break;

            default:
                // Report error to application
                p_client->procedure_status = BLE_DRAKE_CP_PROC_STATUS_FREE;
                if (p_drake->error_handler != NULL)
                {
                    p_drake->error_handler(err_code);
                }
                break;
        }
    }
}

static void on_tx_complete(ble_drake_t const * p_drake)
{
    uint8_t i = m_num_of_clients;

    while (i--)
    {
        if (p_drake->p_client[i].procedure_status == BLE_DRAKE_CP_RSP_CODE_IND_PENDING)
        {
            cp_rsp_send(p_drake, p_drake->p_client[i].conn_handle);
        }
    }
}

static void on_sc_hvc_confirm(ble_drake_t * p_drake, ble_evt_t const * p_ble_evt)
{
    ble_drake_client_spec_t * p_client;

    p_client = get_client_data_by_conn_handle(p_ble_evt->evt.gatts_evt.conn_handle, p_drake->p_client);
    if (p_client == NULL)
    {
        if (p_drake->error_handler != NULL)
        {
            p_drake->error_handler(NRF_ERROR_NOT_FOUND);
        }
        return;
    }

    if (p_ble_evt->evt.gatts_evt.params.hvc.handle == p_drake->cp_handles.value_handle)
    {
        if (p_client->procedure_status == BLE_DRAKE_CP_RSP_CODE_IND_CONFIRM_PENDING)
        {
            p_client->procedure_status = BLE_DRAKE_CP_PROC_STATUS_FREE;
        }
    }
}

/* Public functions ----------------------------------------------------------*/
uint32_t ble_drake_init(ble_drake_t * p_drake, uint8_t max_clients, ble_drake_init_t const * p_drake_init)
{
    uint32_t err_code;
    ble_uuid_t ble_uuid;
    ble_uuid128_t drake_base_uuid = {{0x1B, 0xC5, 0xD5, 0xA5, 0x02, 0x00, 0x0e, 0x84, 0xe4, 0x11, 0x7d, 0xed, 0x00, 0x00, 0x77, 0x4f}};

    if (p_drake == NULL || p_drake_init == NULL || p_drake_init->evt_handler == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (max_clients == 0)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    m_num_of_clients = max_clients;

    // Initialize service structure
    p_drake->evt_handler    = p_drake_init->evt_handler;
    p_drake->error_handler  = p_drake_init->error_handler;
    p_drake->feature        = p_drake_init->feature;

    while (max_clients--)
    {
        p_drake->p_client[max_clients].conn_handle      = BLE_CONN_HANDLE_INVALID;
        p_drake->p_client[max_clients].procedure_status = BLE_DRAKE_CP_PROC_STATUS_FREE;
    }

    // add custom base uuid
    err_code = sd_ble_uuid_vs_add(&drake_base_uuid, &p_drake->uuid_type);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // add service
    ble_uuid.type = p_drake->uuid_type;
    ble_uuid.uuid = BLE_UUID_DRAKE_SERVICE;
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_drake->service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // add status characterisitc
    err_code = status_char_add(p_drake, p_drake_init->drake_s_rd_sec, p_drake_init->drake_s_cccd_wr_sec);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // add feature characteristic
    err_code = feature_char_add(p_drake, p_drake_init->drake_f_rd_sec);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // add control control point characteristic
    err_code = control_point_char_add(p_drake, p_drake_init->drake_cp_wr_sec, p_drake_init->drake_cp_cccd_wr_sec);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}

void ble_drake_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    if (p_context == NULL || p_ble_evt == NULL)
    {
        return;
    }

    ble_drake_t * p_drake = (ble_drake_t *)p_context;

    switch (p_ble_evt->header.evt_id)
    {
    case BLE_GAP_EVT_CONNECTED:
        on_connect(p_drake, p_ble_evt);
        break;

    case BLE_GAP_EVT_DISCONNECTED:
        on_disconnect(p_drake, p_ble_evt);
        break;

    case BLE_GATTS_EVT_WRITE:
        on_write(p_drake, &p_ble_evt->evt.gatts_evt);
        break;

    case BLE_GATTS_EVT_HVC:
        on_sc_hvc_confirm(p_drake, p_ble_evt);
        break;

    case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        on_rw_authorize_request(p_drake, &p_ble_evt->evt.gatts_evt);
        break;

    case BLE_GATTS_EVT_HVN_TX_COMPLETE:
        on_tx_complete(p_drake);
        break;

    default:
        break;
    }
}

uint32_t ble_drake_status_update(ble_drake_t const * p_drake, uint16_t conn_handle, ble_drake_s_t const * p_data)
{
    if (p_drake == NULL || p_data == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint32_t               err_code;
    uint8_t                encoded_data[BLE_DRAKE_S_MAX_CHAR_LEN];
    uint16_t               len;
    uint16_t               hvx_len;
    ble_gatts_hvx_params_t hvx_params;

    len     = s_encode(p_data, encoded_data);
    hvx_len = len;

    memset(&hvx_params, 0, sizeof(ble_gatts_hvx_params_t));
    hvx_params.handle = p_drake->s_handles.value_handle;
    hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
    hvx_params.offset = 0;
    hvx_params.p_len  = &hvx_len;
    hvx_params.p_data = encoded_data;

    err_code = sd_ble_gatts_hvx(conn_handle, &hvx_params);
    if (err_code == NRF_SUCCESS && hvx_len != len)
    {
        err_code = NRF_ERROR_DATA_SIZE;
    }

    return err_code;
}

uint32_t ble_drake_cp_response(ble_drake_t * p_drake, uint16_t conn_handle, ble_drake_cp_rsp_t const * p_rsp)
{
    ble_drake_client_spec_t * p_client;

    if (p_drake == NULL || p_rsp == NULL)
    {
        return NRF_ERROR_NULL;
    }

    p_client = get_client_data_by_conn_handle(conn_handle, p_drake->p_client);
    if (p_client == NULL)
    {
        return NRF_ERROR_NOT_FOUND;
    }

    if (p_client->procedure_status != BLE_DRAKE_CP_PROC_IN_PROGRESS)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    p_client->procedure_status = BLE_DRAKE_CP_RSP_CODE_IND_PENDING;

    p_client->response.len = cp_encode(p_rsp, p_drake, p_client->response.encoded_rsp);

    cp_rsp_send(p_drake, conn_handle);

    return NRF_SUCCESS;
}

/**END OF FILE*****************************************************************/



