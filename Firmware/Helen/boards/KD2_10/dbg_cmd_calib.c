/**
  ******************************************************************************
  * @file    dbg_cmd_calib.c
  * @author  Thomas Reisnecker
  * @brief   calibration over debug interface for KD2 hardware
  ******************************************************************************
  */

#ifdef DEBUG_EXT

/* Includes ------------------------------------------------------------------*/
#include "debug.h"
#include "KD2_10.h"
#include "itoa.h"

/* External variables --------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/
typedef void (*subCommandFunc_t)(char const* pValue, uint16_t lenght, uint16_t connHandle);

typedef struct
{
    char* pName;
    char* pHelp;
    subCommandFunc_t func;
} subCommand_t;

/* Private macros ------------------------------------------------------------*/

/* Private defines -----------------------------------------------------------*/

/* Private function prototype ------------------------------------------------*/
static void readCalib(char const* pValue, uint16_t lenght, uint16_t connHandle);
static void setCurrent(char const* pValue, uint16_t lenght, uint16_t connHandle);
static void setTemperature(char const* pValue, uint16_t lenght, uint16_t connHandle);
static void readCalibHelena(char const* pValue, uint16_t lenght, uint16_t connHandle);
static void setCurrentHelena(char const* pValue, uint16_t lenght, uint16_t connHandle);
static void setTemperatureHelena(char const* pValue, uint16_t lenght, uint16_t connHandle);

/* Private variables ---------------------------------------------------------*/
static const subCommand_t subCmds[] =
{
    {.pName = " --read",  .pHelp = "",                  .func = readCalib},
    {.pName = " --cur",   .pHelp = " [gain]",           .func = setCurrent},
    {.pName = " --temp",  .pHelp = " [offset]",         .func = setTemperature},
    {.pName = " --hread", .pHelp = "",                  .func = readCalibHelena},
    {.pName = " --hc",    .pHelp = " [gainL], [gainR]", .func = setCurrentHelena},
    {.pName = " --ht",    .pHelp = " [offset]",         .func = setTemperatureHelena},
};

static uint16_t responseConnHandle = BLE_CONN_HANDLE_INVALID;

/* Private functions ---------------------------------------------------------*/
static void storageDoneHandler(ret_code_t errCode)
{
    char const success[] = "done\r\n";
    char const failed[] = "failed\r\n";
    uint16_t len;

    if (responseConnHandle == BLE_CONN_HANDLE_INVALID)
        return;

    if (errCode == NRF_SUCCESS)
    {
        len = strlen(success);
        (void)dbg_DataSend((uint8_t*)success, &len, responseConnHandle);
    }
    else
    {
        len = strlen(failed);
        (void)dbg_DataSend((uint8_t*)failed, &len, responseConnHandle);
    }

    responseConnHandle = BLE_CONN_HANDLE_INVALID;
}

static void readCalib(char const* pValue, uint16_t length, uint16_t connHandle)
{
    (void)pValue;
    (void)length;

    ret_code_t errCode;
    char rsp[DBG_MAX_DATA_LEN + 1];     // +1 for \0
    uint16_t len;
    KD2_adcCompensation_t comp;

    (void)KD2_GetCompensation(&comp);

    strcpy(rsp, "curr_gain: ");
    itoa(comp.current.gain, &rsp[strlen(rsp)], 10);
    strcat(rsp, "\r\n");
    len = strlen(rsp);
    errCode = dbg_DataSend((uint8_t*)rsp, &len, connHandle);
    if (errCode != NRF_SUCCESS)
        return;

    strcpy(rsp, "temp_off: ");
    itoa(comp.temperature.offset, &rsp[strlen(rsp)], 10);
    strcat(rsp, "\r\n");
    len = strlen(rsp);
    errCode = dbg_DataSend((uint8_t*)rsp, &len, connHandle);
    if (errCode != NRF_SUCCESS)
        return;
}

static void setCurrent(char const* pValue, uint16_t lenght, uint16_t connHandle)
{
    int32_t gain;
    ret_code_t errCode = NRF_ERROR_INVALID_PARAM;
    char const invalidParam[] = "invalid parameter\r\n";
    uint16_t len = strlen(invalidParam);

    if (lenght != 0 && sscanf(pValue, "%ld", &gain) == 1 && gain > 0 && gain < UINT16_MAX)
    {
        KD2_adcCompensation_t comp;
        (void)KD2_GetCompensation(&comp);
        comp.current.gain = gain;
        errCode = KD2_SetCompensation(&comp, storageDoneHandler);
    }

    if (errCode != NRF_SUCCESS)
        (void)dbg_DataSend((uint8_t*)invalidParam, &len, connHandle);
    else
        responseConnHandle = connHandle;
}

static void setTemperature(char const* pValue, uint16_t lenght, uint16_t connHandle)
{
    int16_t offset;
    ret_code_t errCode = NRF_ERROR_INVALID_PARAM;
    char const invalidParam[] = "invalid parameter\r\n";
    uint16_t len = strlen(invalidParam);

    /// TODO: maybe check plausibility
    if (lenght != 0 && sscanf(pValue, "%hd", &offset) == 1)
    {
        KD2_adcCompensation_t comp;
        (void)KD2_GetCompensation(&comp);
        comp.temperature.offset = offset;
        errCode = KD2_SetCompensation(&comp, storageDoneHandler);
    }

    if (errCode != NRF_SUCCESS)
        (void)dbg_DataSend((uint8_t*)invalidParam, &len, connHandle);
    else
        responseConnHandle = connHandle;
}

static void readCalibHelena(char const* pValue, uint16_t length, uint16_t connHandle)
{
    (void)pValue;
    (void)length;

    ret_code_t errCode;
    char rsp[DBG_MAX_DATA_LEN + 1];     // +1 for \0
    uint16_t len;
    hbd_calibData_t comp;

    errCode = KD2_GetHelenaCompensation(&comp);
    if (errCode == NRF_ERROR_NOT_FOUND)
    {
        strcpy(rsp, "no board available\r\n");
        len = strlen(rsp);
        (void)dbg_DataSend((uint8_t*)rsp, &len, connHandle);
        return;
    }
    else if (errCode == NRF_ERROR_NOT_SUPPORTED)
    {
        strcpy(rsp, "not supported\r\n");
        len = strlen(rsp);
        (void)dbg_DataSend((uint8_t*)rsp, &len, connHandle);
        return;
    }
    else if (errCode == NRF_SUCCESS)
    {
        strcpy(rsp, "gain_left: ");
        itoa(comp.gainLeft, &rsp[strlen(rsp)], 10);
        strcat(rsp, "\r\n");
        len = strlen(rsp);
        errCode = dbg_DataSend((uint8_t*)rsp, &len, connHandle);
        if (errCode != NRF_SUCCESS)
            return;

        strcpy(rsp, "gain_right: ");
        itoa(comp.gainRight, &rsp[strlen(rsp)], 10);
        strcat(rsp, "\r\n");
        len = strlen(rsp);
        errCode = dbg_DataSend((uint8_t*)rsp, &len, connHandle);
        if (errCode != NRF_SUCCESS)
            return;

        strcpy(rsp, "temp_off: ");
        itoa(comp.temperatureOffset, &rsp[strlen(rsp)], 10);
        strcat(rsp, "\r\n");
        len = strlen(rsp);
        errCode = dbg_DataSend((uint8_t*)rsp, &len, connHandle);
        if (errCode != NRF_SUCCESS)
            return;
    }
    else
    {
        strcpy(rsp, "failed\r\n");
        len = strlen(rsp);
        (void)dbg_DataSend((uint8_t*)rsp, &len, connHandle);
        return;
    }
}

static void setCurrentHelena(char const* pValue, uint16_t lenght, uint16_t connHandle)
{
    int32_t gainLeft, gainRight;
    ret_code_t errCode;
    char rsp[DBG_MAX_DATA_LEN + 1];     // +1 for \0
    uint16_t len;
    hbd_calibData_t comp;

    errCode = KD2_GetHelenaCompensation(&comp);
    if (errCode == NRF_SUCCESS)
    {
        if (lenght != 0 && sscanf(pValue, "%ld, %ld", &gainLeft, &gainRight) == 2)
        {
            comp.gainLeft = gainLeft;
            comp.gainRight = gainRight;
            errCode = KD2_SetHelenaCompensation(&comp);
        }
        else
            errCode = NRF_ERROR_INVALID_PARAM;
    }

    if (errCode == NRF_ERROR_NOT_FOUND)
    {
        strcpy(rsp, "no board available\r\n");
        len = strlen(rsp);
        (void)dbg_DataSend((uint8_t*)rsp, &len, connHandle);
    }
    else if (errCode == NRF_ERROR_NOT_SUPPORTED)
    {
        strcpy(rsp, "not supported\r\n");
        len = strlen(rsp);
        (void)dbg_DataSend((uint8_t*)rsp, &len, connHandle);
    }
    else if (errCode == NRF_ERROR_INVALID_PARAM)
    {
        strcpy(rsp, "invalid parameter\r\n");
        len = strlen(rsp);
        (void)dbg_DataSend((uint8_t*)rsp, &len, connHandle);
    }
    else if (errCode != NRF_SUCCESS)
    {
        strcpy(rsp, "failed\r\n");
        len = strlen(rsp);
        (void)dbg_DataSend((uint8_t*)rsp, &len, connHandle);
    }
}

static void setTemperatureHelena(char const* pValue, uint16_t lenght, uint16_t connHandle)
{
    int32_t offset;
    ret_code_t errCode;
    char rsp[DBG_MAX_DATA_LEN + 1];     // +1 for \0
    uint16_t len;
    hbd_calibData_t comp;

    errCode = KD2_GetHelenaCompensation(&comp);
    if (errCode == NRF_SUCCESS)
    {
        if (lenght != 0 && sscanf(pValue, "%ld", &offset) == 1)
        {
            comp.temperatureOffset = offset;
            errCode = KD2_SetHelenaCompensation(&comp);
        }
        else
            errCode = NRF_ERROR_INVALID_PARAM;
    }

    if (errCode == NRF_ERROR_NOT_FOUND)
    {
        strcpy(rsp, "no board available\r\n");
        len = strlen(rsp);
        (void)dbg_DataSend((uint8_t*)rsp, &len, connHandle);
    }
    else if (errCode == NRF_ERROR_NOT_SUPPORTED)
    {
        strcpy(rsp, "not supported\r\n");
        len = strlen(rsp);
        (void)dbg_DataSend((uint8_t*)rsp, &len, connHandle);
    }
    else if (errCode == NRF_ERROR_INVALID_PARAM)
    {
        strcpy(rsp, "invalid parameter\r\n");
        len = strlen(rsp);
        (void)dbg_DataSend((uint8_t*)rsp, &len, connHandle);
    }
    else if (errCode != NRF_SUCCESS)
    {
        strcpy(rsp, "failed\r\n");
        len = strlen(rsp);
        (void)dbg_DataSend((uint8_t*)rsp, &len, connHandle);
    }
}

static void dbg_cmd_Calib(char const* pSubcommand, uint16_t length, uint16_t connHandle)
{
    // search subcommand
    for (uint_fast8_t i = 0; i < ARRAY_SIZE(subCmds); i++)
    {
        int16_t len = strlen(subCmds[i].pName);
        if (length < len)
            continue;   // no need to compare if received message is shorter than command
        if (strncmp(pSubcommand, subCmds[i].pName, len) == 0)
        {
            subCmds[i].func(&pSubcommand[len], length - len, connHandle);
            return;
        }
    }

    // subcommand not found, send list of available commands
    ret_code_t errCode;
    uint16_t len;
    char list[DBG_MAX_DATA_LEN + 1];

    strcpy(list, "usage:\r\n");
    len = strlen(list);

    for (uint_fast8_t i = 0; i < ARRAY_SIZE(subCmds); i++)
    {
        // send message if there is not enough space for the next command name (incl. linefeed)
        if (len + strlen(subCmds[i].pName) > sizeof(list))  // additional 2 bytes for \r\n
        {
            errCode = dbg_DataSend((uint8_t*)list, &len, connHandle);
            if (errCode != NRF_SUCCESS)
                return;

            len = 0;
            list[0] = '\0';
        }

        strcat(list, subCmds[i].pName);
        len = strlen(list);

        // send message if there is not enough space for the help string (incl. linefeed)
        if (len + 2 + strlen(subCmds[i].pHelp) > sizeof(list))  // additional 2 bytes for \r\n
        {
            errCode = dbg_DataSend((uint8_t*)list, &len, connHandle);
            if (errCode != NRF_SUCCESS)
                return;

            len = 0;
            list[0] = '\0';
        }

        strcat(list, subCmds[i].pHelp);
        strcat(list, "\r\n");
        len = strlen(list);
    }

    // send last message
    (void)dbg_DataSend((uint8_t*)list, &len, connHandle);
}

/* section variable ----------------------------------------------------------*/
DEBUG_REGISTER(const dbg_commands_t dbg_cmd_calib) =
{
    .pCommandName    = "calib",
    .pCommandInit    = NULL,
    .pCommandFunc    = dbg_cmd_Calib,
    .pCommandExecute = NULL,
};

#endif // DEBUG_EXT

/**END OF FILE*****************************************************************/
