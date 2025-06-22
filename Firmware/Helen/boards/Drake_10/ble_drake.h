/**
  ******************************************************************************
  * @file    ble_drake.h
  * @author  Thomas Reisnecker
  * @version V1.0
  * @brief   Drake Control Service implementation
  *
  * @details This module implements the Drake Status, Drake Feature and Drake
  *          Control Point characteristics.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BLE_DRAKE_H_INCLUDED
#define BLE_DRAKE_H_INCLUDED

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"

/* Exported defines ----------------------------------------------------------*/
#ifndef BLE_DRAKE_BLE_OBSERVER_PRIO
#define BLE_DRAKE_BLE_OBSERVER_PRIO     2
#endif // BLE_DRAKE_BLE_OBSERVER_PRIO

#define BLE_DRAKE_CP_MAX_INDEXED_POS    4

#define BLE_DRAKE_CP_MAX_LEN            15
#define BLE_DRAKE_CP_MIN_LEN            1

/* Exported types ------------------------------------------------------------*/
/**@brief Drake Control Service Event Types */
typedef enum
{
    BLE_DRAKE_EVT_S_NOTIFICATION_ENABLED,   // Drake Status Notification enabled event
    BLE_DRAKE_EVT_S_NOTIFICATION_DISABLED,  // Drake Status Notification disabled event
    BLE_DRAKE_EVT_CP_INDICATION_ENABLED,    // Drake Control Point Indication enabled event
    BLE_DRAKE_EVT_CP_INDICATION_DISABLED,   // Drake Control Point Indication disabled event
    BLE_DRAKE_EVT_CP_EVT,                   // write event form the Control Point
} ble_drake_evt_type_t;

typedef enum
{
    BLE_DRAKE_CP_OP_REQ_DEV_USAGE           = 1,  // request devise usage
    BLE_DRAKE_CP_OP_SET_DEV_USAGE           = 2,  // set device usage
    BLE_DRAKE_CP_OP_REQ_BAT_INFO            = 3,  // request battery information
    BLE_DRAKE_CP_OP_SET_BAT_INFO            = 4,  // set battery information
    BLE_DRAKE_CP_OP_REQ_SOC_TIMEOUT         = 5,  // request the soc timeout
    BLE_DRAKE_CP_OP_SET_SOC_TIMEOUT         = 6,  // set the soc timeout
    BLE_DRAKE_CP_OP_REQ_LIGHT_PATTERN       = 7,  // request light pattern
    BLE_DRAKE_CP_OP_SET_LIGHT_PATTERN       = 8,  // set light pattern
    BLE_DRAKE_CP_OP_SET_PL_POS              = 10, // set plunger position
    BLE_DRAKE_CP_OP_REQ_MOTOR_CONFIG        = 11, // request motor configuration
    BLE_DRAKE_CP_OP_SET_MOTOR_CONFIG        = 12, // set motor configuration
    BLE_DRAKE_CP_OP_REQ_MOTOR_SENS_CALIB    = 13, // request motor sensor calibration
    BLE_DRAKE_CP_OP_START_MOTOR_SENS_CALIB  = 14, // start/clear motor sensor calibration
    BLE_DRAKE_CP_OP_REQ_TRAVEL_SENS_DATA    = 15, // request the travel sensor data
    BLE_DRAKE_CP_OP_START_TRAVEL_SENS_CALIB = 16, // start/clear the travel sensor calibration
    BLE_DRAKE_CP_OP_REQ_INDEXED_CONFIG      = 17, // request the indexed travel configuration
    BLE_DRAKE_CP_OP_SET_INDEXED_CONFIG      = 18, // set the indexed travel configuration
    BLE_DRAKE_CP_OP_RSP_CODE                = 32, // response code
} ble_drake_cp_op_code_t;

/**@brief Drake Control Control Point response parameter */
typedef enum
{
    BLE_DRAKE_CP_RSP_VAL_SUCCESS       = 1,
    BLE_DRAKE_CP_RSP_VAL_NOT_SUPPORTED = 2,
    BLE_DRAKE_CP_RSP_VAL_INVALID       = 3,
    BLE_DRAKE_CP_RSP_VAL_FAILED        = 4
} ble_drake_cp_rsp_val_t;

/**@brief Drake Control Control Point procedure status */
typedef enum
{
    BLE_DRAKE_CP_PROC_STATUS_FREE             = 0,
    BLE_DRAKE_CP_PROC_IN_PROGRESS             = 1,
    BLE_DRAKE_CP_RSP_CODE_IND_PENDING         = 2,
    BLE_DRAKE_CP_RSP_CODE_IND_CONFIRM_PENDING = 3
} ble_drake_cp_proc_status_t;

/**@brief Drake Control Control Point response indication structure */
typedef struct
{
    ble_drake_cp_rsp_val_t rsp_code;
    uint8_t                len;
    uint8_t                encoded_rsp[BLE_DRAKE_CP_MAX_LEN];
} ble_drake_cp_rsp_ind_t;

/**@brief Drake Control Control Point Device Usage Enumeration */
typedef enum
{
    BLE_DRAKE_CP_DEV_USAGE_DR_ACT       = 0,
    BLE_DRAKE_CP_DEV_USAGE_REARLIGHT_16 = 16,
    BLE_DRAKE_CP_DEV_USAGE_REARLIGHT_44 = 44,
    BLE_DRAKE_CP_DEV_USAGE_BARLIGHT_36  = 36
} ble_drake_cp_dev_usage_t;

typedef enum
{
    BLE_DRAKE_BAT_TYPE_LCO = 0,
    BLE_DRAKE_BAT_TYPE_NMC = 1,
} ble_drake_cp_bat_type_t;

typedef struct
{
    ble_drake_cp_bat_type_t type;               // battery type
    uint16_t                cap_nom;            // nominal capacity in C
    uint16_t                cap_act;            // actual capacity in C
} ble_drake_cp_bat_info_t;

typedef enum
{
    BLE_DRAKE_LIGHT_TYPE_FRONT     = 0,
    BLE_DRAKE_LIGHT_TYPE_REAR      = 1,
    BLE_DRAKE_LIGHT_TYPE_POS       = 3,
    BLE_DRAKE_LIGHT_TYPE_BRAKE     = 4,
    BLE_DRAKE_LIGHT_TYPE_LEFT_IND  = 5,
    BLE_DRAKE_LIGHT_TYPE_RIGHT_IND = 6,
} ble_drake_cp_light_type_t;

typedef struct
{
    ble_drake_cp_light_type_t type;             // light type
    uint8_t const           * p_pattern;        // pattern array
    uint8_t                   len;              // length of pattern array
} ble_drake_cp_pattern_t;

typedef struct
{
    uint16_t open_timeout;                      // motor timeout for opening in 1/1024 s
    uint16_t close_timeout;                     // motor timeout for closing in 1/1024 s
} ble_drake_cp_motor_config_t;

typedef enum
{
    BLE_DRAKE_MOTOR_CALIB_TYPE_COMP_THRESHOLDS = 0,
    BLE_DRAKE_MOTOR_CALIB_TYPE_FULL_OPEN_CNT   = 1,
} ble_drake_cp_motor_calib_type_t;

typedef struct
{
    uint8_t upper;                              // upper thresholds
    uint8_t lower;                              // lower thresholds
} ble_drake_cp_motor_calib_comp_th_t;

typedef struct
{
    ble_drake_cp_motor_calib_type_t type;
    union
    {
        bool                               start_clear;     // used when receiving, true to start, false to clear
        ble_drake_cp_motor_calib_comp_th_t thresholds;      // used when sending,
        uint16_t                           full_open_cnt;   // used when sending
    } params;
} ble_drake_cp_motor_calib_t;

typedef struct
{
    uint16_t travel;
    int32_t  offset;
} ble_drake_cp_travel_sens_data_t;

typedef struct
{
    uint16_t blocking_time;
    uint16_t timeout;
    int16_t  positions[BLE_DRAKE_CP_MAX_INDEXED_POS];
} ble_drake_cp_indexed_config_t;

/**@brief Control Point event parameters */
typedef union
{
    ble_drake_cp_dev_usage_t        device_usage;    // device usage
    ble_drake_cp_bat_info_t         battery_info;    // battery information
    uint16_t                        soc_timeout;     // soc timeout
    ble_drake_cp_pattern_t          pattern;         // light pattern
    uint8_t                         pl_pos;          // plunger position in 0.5%
    ble_drake_cp_motor_config_t     motor_config;    // motor configuration
    ble_drake_cp_motor_calib_t      motor_calib;     // motor calibration
    ble_drake_cp_travel_sens_data_t travel_sens_data;// travel sensor data
    ble_drake_cp_indexed_config_t   indexed_config;  // indexed travel configuration
} ble_drake_cp_params_t;

/**@brief client specific data */
typedef struct
{
    uint16_t                   conn_handle;               // Handle if the current connection (as provided by the BLE stack)
    bool                       is_drake_s_notify_enabled; // Indicator if status notifications are enabled
    bool                       is_drake_cp_indic_enabled; // Indicator if control point indications are enabled
    ble_drake_cp_proc_status_t procedure_status;          // status of possible procedure
    ble_drake_cp_rsp_ind_t     response;                  // pending response data
} ble_drake_client_spec_t;

// Forward declaration of the ble_drake_t type.
typedef struct ble_drake_s ble_drake_t;

/**@brief Drake Control Service Event */
typedef struct
{
    ble_drake_evt_type_t            evt_type; // the type of the event
    ble_drake_t const             * p_drake;  // a pointer to the instance
    ble_drake_client_spec_t const * p_client; // a pointer to the client data
    ble_drake_cp_op_code_t          cp_evt;   // the control point event in case of @ref BLE_DRAKE_EVT_CP_EVT
    ble_drake_cp_params_t const   * p_params; // the event parameters
} ble_drake_evt_t;

/**@brief Drake Control Service event handler type */
typedef void (*ble_drake_evt_handler_t)(ble_drake_evt_t const * p_evt);

/**@brief Drake Device feature structure. This contains the supported device features */
typedef struct
{
    uint8_t light_supported                : 1;
    uint8_t seatpost_actuator_supported    : 1;
} ble_drake_device_f_t;

/**@brief Drake light feature structure. This contains the supported light configuration features */
typedef struct
{
    uint8_t front_light_pattern_supported  : 1;
    uint8_t tail_light_pattern_supported   : 1;
    uint8_t pos_light_pattern_supported    : 1;
    uint8_t brake_light_pattern_supported  : 1;
    uint8_t left_indic_pattern_supported   : 1;
    uint8_t right_indic_pattern_supported  : 1;
} ble_drake_light_f_t;

/**@brief Drake actuator feature structure. This contains the supported actuator configuration features */
typedef struct
{
    uint8_t motor_config_supported    : 1;
    uint8_t motor_sensor_supported    : 1;
    uint8_t travel_sensor_supported   : 1;
    uint8_t travel_indexing_supported : 1;
} ble_drake_act_f_t;

/**@brief Drake feature structure. */
typedef struct
{
    ble_drake_device_f_t device;
    ble_drake_light_f_t  light;
    ble_drake_act_f_t    actuator;
} ble_drake_f_t;

/**@brief Drake Control Service init structure. This contains all options and data
 *        needed for initialization of the service */
typedef struct
{
    security_req_t          drake_s_rd_sec;       // Security requirement for reading Drake status characteristic.
    security_req_t          drake_s_cccd_wr_sec;  // Security requirement for writing Drake status CCCD.
    security_req_t          drake_f_rd_sec;       // Security requirement for reading Drake feature characteristic.
    security_req_t          drake_cp_cccd_wr_sec; // Security requirement for writing Drake control point characteristic CCCD.
    security_req_t          drake_cp_wr_sec;      // Security requirement for writing Drake control point characteristic.
    ble_drake_evt_handler_t evt_handler;          // Event handler to be called for handling events in the Drake Control Service
    ble_srv_error_handler_t error_handler;        // Function to be called in case of an error.
    ble_drake_f_t           feature;              // supported features
} ble_drake_init_t;

/**@brief Drake Control Service structure. This contains various status
 *        information for the service */
struct ble_drake_s
{
    uint8_t                   uuid_type;
    uint16_t                  service_handle; // Handle of the Drake Control Service (as provided by the BLE stack)
    ble_gatts_char_handles_t  s_handles;      // Handles related to the status characteristic
    ble_gatts_char_handles_t  f_handles;      // Handles related to the feature characteristic
    ble_gatts_char_handles_t  cp_handles;     // Handles related to the control point characteristic
    ble_drake_evt_handler_t   evt_handler;    // Event handler to be called for handling events in the Drake Control Service
    ble_srv_error_handler_t   error_handler;  // Function to be called in case of an error.
    ble_drake_f_t             feature;        // supported features
    ble_drake_client_spec_t * p_client;       // client specific data
};

/**@brief Drake Control Control Point Response parameter structure */
typedef struct
{
    ble_drake_cp_op_code_t    opcode;
    ble_drake_cp_rsp_val_t    status;
    ble_drake_cp_params_t     params;
} ble_drake_cp_rsp_t;

/** @brief the drake status flags field */
typedef struct
{
    uint8_t pl_status_present : 1;  // Flag indication that the plunger status field is present
} ble_drake_s_flags_t;

/** @brief the measurement structure */
typedef struct
{
    ble_drake_s_flags_t flags;            // Flags indicating which fields are present
    uint8_t             pl_status;        // plunger status with resolution of 0.5 %
} ble_drake_s_t;

/* Exported macros ---------------------------------------------------------- */
#define GLUE(X, Y)  X ## Y
#define BLE_DRAKE_DEF(_name, _drake_max_clients)                              \
static ble_drake_client_spec_t GLUE(_name, _client_data)[_drake_max_clients]; \
static ble_drake_t _name =                                                    \
{                                                                             \
    .p_client = GLUE(_name, _client_data),                                    \
};                                                                            \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                           \
                     BLE_DRAKE_BLE_OBSERVER_PRIO,                             \
                     ble_drake_on_ble_evt, &_name)

/* Exported functions ------------------------------------------------------- */
/** @brief Function for initializing the Drake Control Service
 *
 * @param[out]  p_drake       Drake Control Service structure. This structure
 *                            will have to be supplied by the application. It
 *                            will be initialized by this function, and will
 *                            later be usedto identify this particular service
 *                            instance
 * @param[in]   p_drake_init  Information needed to initialize th service
 * @return      NRF_SUCCESS on successful initialization of service, otherwise
 *              an error code *
 */
uint32_t ble_drake_init(ble_drake_t * p_drake, uint8_t max_clients, ble_drake_init_t const * p_drake_init);

/** @brief Function for handling the Applications's BLE stack events.
 *
 * @param[in]   p_ble_evt   Event received from the BLE stack
 * @param[in]   p_context   Drake Control Service Structure
 */
void ble_drake_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);

/** @brief Function to update status data if notifications have been enabled
 *
 * @param[in] p_drake      the drake control service structure
 * @param[in] conn_handle  the connection to send the message to
 * @param[in] p_data       the data to send
 * @return NRF_SUCCESS, NRF_ERROR_NULL, NRF_ERROR_INVALID_STATE *
 */
uint32_t ble_drake_status_update(ble_drake_t const * p_drake, uint16_t conn_handle, ble_drake_s_t const * p_data);

/** @brief Function for sending a control point response
 *
 * @param[in] p_drake      Drake Control Service structure
 * @param[in] conn_handle  the connection handle to send the response to
 * @param[in] p_rsp        Response data structure
 * @return    NRF_SUCCESS on successful response, otherwise an error code
 */
uint32_t ble_drake_cp_response(ble_drake_t * p_drake, uint16_t conn_handle, ble_drake_cp_rsp_t const * p_rsp);

#endif /* BLE_DRAKE_H_INCLUDED */

/**END OF FILE*****************************************************************/
