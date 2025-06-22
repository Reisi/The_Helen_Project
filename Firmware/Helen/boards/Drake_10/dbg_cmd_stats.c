/**
  ******************************************************************************
  * @file    dbg_cmd_stats.c
  * @author  Thomas Reisnecker
  * @brief   statistics over debug interface for KD2 hardware
  ******************************************************************************
  */

#ifdef DEBUG_EXT

/* Includes ------------------------------------------------------------------*/
#include "debug.h"
#include "Drake_10.h"
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
static void getActCounter(char const* pValue, uint16_t lenght, uint16_t connHandle);
static void setActCounter(char const* pValue, uint16_t lenght, uint16_t connHandle);

/* Private variables ---------------------------------------------------------*/
static const subCommand_t subCmds[] =
{
    {.pName = " --get", .pHelp = "",                .func = getActCounter},
    {.pName = " --set", .pHelp = " [new value]",    .func = setActCounter},
};

/* Private functions ---------------------------------------------------------*/
static void getActCounter(char const* pValue, uint16_t length, uint16_t connHandle)
{
    (void)pValue;
    (void)length;

    ret_code_t errCode;
    char rsp[DBG_MAX_DATA_LEN + 1];     // +1 for \0
    uint16_t len;
    uint16_t counter = Drake_GetActivationCounter();

    strcpy(rsp, "act counter: ");
    itoa(counter, &rsp[strlen(rsp)], 10);
    strcat(rsp, "\r\n");
    len = strlen(rsp);
    errCode = dbg_DataSend((uint8_t*)rsp, &len, connHandle);
    if (errCode != NRF_SUCCESS)
        return;
}

static void setActCounter(char const* pValue, uint16_t lenght, uint16_t connHandle)
{
    int32_t counter;
    uint16_t len;
    char const invalidParam[] = "invalid parameter\r\n";
    char const done[] = "done\r\n";

    if (lenght != 0 && sscanf(pValue, "%ld", &counter) == 1 && counter >= 0 && counter < UINT16_MAX)
    {
        Drake_SetActivationCounter(counter);
        len = strlen(done);
        (void)dbg_DataSend((uint8_t*)done, &len, connHandle);
    }
    else
    {
        len = strlen(invalidParam);
        (void)dbg_DataSend((uint8_t*)invalidParam, &len, connHandle);
    }
}

static void dbg_cmd_Stats(char const* pSubcommand, uint16_t length, uint16_t connHandle)
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
DEBUG_REGISTER(const dbg_commands_t dbg_cmd_stats) =
{
    .pCommandName    = "stats",
    .pCommandInit    = NULL,
    .pCommandFunc    = dbg_cmd_Stats,
    .pCommandExecute = NULL,
};

#endif // DEBUG_EXT

/**END OF FILE*****************************************************************/
