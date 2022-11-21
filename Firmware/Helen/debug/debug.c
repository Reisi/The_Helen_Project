/**
  ******************************************************************************
  * @file    debug.c
  * @author  Thomas Reisnecker
  * @brief   debug and error handling module
  ******************************************************************************
  */

/* Debug flag checking -------------------------------------------------------*/

/* logger configuration ----------------------------------------------_-------*/
#define DBG_CONFIG_LOG_ENABLED 1 /// TODO: just temporary until project specific sdk_config
#define DBG_CONFIG_LOG_LEVEL   3

#define NRF_LOG_MODULE_NAME dbg
#if DBG_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL DBG_CONFIG_LOG_LEVEL
#else //BASE_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL 0
#endif //BASE_CONFIG_LOG_ENABLED
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

/* Includes ------------------------------------------------------------------*/
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_power.h"
#include "nrf_bootloader_info.h"

#include "watchdog.h"

#ifdef DEBUG_EXT
#include "ble_nus.h"
#include "debug.h"
#endif // DEBUG_EXT

/* External variables --------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/

/* Private defines -----------------------------------------------------------*/
#define DEAD_BEEF   0xDEADBEEF  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

/* Private function prototype ------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
#ifdef DEBUG_EXT
BLE_NUS_DEF(nusGattS, NRF_SDH_BLE_TOTAL_LINK_COUNT);
#endif

/* Private functions ---------------------------------------------------------*/
static ret_code_t logInit(void)
{
    ret_code_t errCode = NRF_LOG_INIT(NULL);
    //APP_ERROR_CHECK(errCode);

    NRF_LOG_DEFAULT_BACKENDS_INIT();

    return errCode;
}

#ifdef DEBUG_EXT
static void nusDataHandler(ble_nus_evt_t* pEvt)
{
    if (pEvt->type != BLE_NUS_EVT_RX_DATA)
        return;

    char const* pString = (char const*)pEvt->params.rx_data.p_data;
    uint16_t length = pEvt->params.rx_data.length;

    // search command
    for (uint_fast8_t i = 0; i < DEBUG_CNT(); i++)
    {
        dbg_commands_t const* pCommand = DEBUG_GET(i);
        int16_t len = strlen(pCommand->pCommandName);
        if (length < len)
            continue;   // no need to compare if received message is shorter than command
        if (strncmp(pString, pCommand->pCommandName, len) == 0)
        {
            if (pCommand->pCommandFunc != NULL)
                (*pCommand->pCommandFunc)(&pString[len], length - len, pEvt->conn_handle);
            return;
        }
    }

    // subcommand not found, send list of available commands
    ret_code_t errCode;
    uint16_t len;
    char list[DBG_MAX_DATA_LEN + 1];

    strcpy(list, "not found, cmds:\r\n");
    len = strlen(list);

    for (uint_fast8_t i = 0; i < DEBUG_CNT(); i++)
    {
        dbg_commands_t const* pCommand = DEBUG_GET(i);

        // send message if there is not enough space for the next command name (incl. linefeed)
        if (len + 2 + strlen(pCommand->pCommandName) > sizeof(list))  // additional 2 bytes for \r\n
        {
            errCode = dbg_DataSend((uint8_t*)list, &len, pEvt->conn_handle);
            if (errCode != NRF_SUCCESS)
                return;

            len = 0;
            list[0] = '\0';
        }

        strcat(list, pCommand->pCommandName);
        strcat(list, "\r\n");
        len = strlen(list);
    }

    // send last message
    (void)dbg_DataSend((uint8_t*)list, &len, pEvt->conn_handle);
}

static void initNus()
{
    ble_nus_init_t nusInit = {.data_handler = nusDataHandler};
    APP_ERROR_CHECK(ble_nus_init(&nusGattS, &nusInit));
}

static bool isNusInit()
{
    if (nusGattS.uuid_type == BLE_UUID_TYPE_UNKNOWN)
        return false;
    else
        return true;
}
#endif

/* Public functions ----------------------------------------------------------*/
ret_code_t dbg_Init()
{
    ret_code_t errCode;

    // set DebugMonitor Interrupt priority
    NVIC_SetPriority(DebugMonitor_IRQn, APP_IRQ_PRIORITY_MID);
    // disable CRC check of bootloader for better testing while developing
    nrf_power_gpregret2_set(BOOTLOADER_DFU_SKIP_CRC);

    errCode = logInit();

#ifdef DEBUG_EXT
    for (uint_fast8_t i = 0; i < DEBUG_CNT(); i++)
    {
        dbg_commands_t const* pCommand = DEBUG_GET(i);
        if (pCommand->pCommandInit != NULL)
            pCommand->pCommandInit();
    }

#endif

    if (errCode == NRF_SUCCESS)
        errCode = wdg_Init();
    else
        (void)wdg_Init();

    return errCode;
}

bool dbg_Execute(void)
{
#ifdef DEBUG_EXT
    if (!isNusInit())
        initNus();

    for (uint_fast8_t i = 0; i < DEBUG_CNT(); i++)
    {
        dbg_commands_t const* pCommand = DEBUG_GET(i);
        if (pCommand->pCommandExecute != NULL)
            pCommand->pCommandExecute();
    }
#endif

    wdg_Feed();

    return NRF_LOG_PROCESS();
}

#ifdef DEBUG_EXT
ret_code_t dbg_DataSend(uint8_t* pData, uint16_t* pLength, uint16_t connHandle)
{
    return ble_nus_data_send(&nusGattS, pData, pLength, connHandle);
}
#endif

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**END OF FILE*****************************************************************/
