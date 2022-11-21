/**
  ******************************************************************************
  * @file    debug.h
  * @author  Thomas Reisnecker
  * @brief   header file for error handling module
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DEBUG_H_INCLUDED
#define DEBUG_H_INCLUDED

/* Includes ------------------------------------------------------------------*/
//#include "nrf_log.h"
#include "ble_nus.h"
#include "nrf_section.h"

/* Exported types ------------------------------------------------------------*/
#if defined DEBUG_EXT
typedef void (*dbg_commandInit_t)(void);
typedef void (*dbg_commandFunc_t)(char const* pSubcommand, uint16_t lenght, uint16_t connHandle);
typedef void (*dbg_commandExecute_t)(void);

typedef struct
{
    char const* pCommandName;
    dbg_commandInit_t pCommandInit;
    dbg_commandFunc_t pCommandFunc;
    dbg_commandExecute_t pCommandExecute;
} dbg_commands_t;

NRF_SECTION_DEF(debug_commands, dbg_commands_t);
#endif // DEBUG_EXT

/* Exported constants --------------------------------------------------------*/
#if defined DEBUG_EXT
#define DBG_MAX_DATA_LEN    BLE_NUS_MAX_DATA_LEN
#endif // DEBUG_EXT

/* Exported macros -----------------------------------------------------------*/
#if defined DEBUG_EXT
#define DEBUG_REGISTER(command) NRF_SECTION_ITEM_REGISTER(debug_commands, command)
#define DEBUG_CNT()             NRF_SECTION_ITEM_COUNT(debug_commands, dbg_commands_t)
#define DEBUG_GET(cnt)          NRF_SECTION_ITEM_GET(debug_commands, dbg_commands_t, cnt)
#endif // DEBUG_EXT

#define dbg_ResetCycleCnt() DWT->CYCCNT = 0
#define dbg_StartCycleCnt() DWT->CTRL |= 1
#define dbg_StopCycleCnt()  DWT->CTRL &= ~1
#define dbg_GetCycleCnt()   DWT->CYCCNT

#define LOG_ERROR_CHECK(string, errCode) \
{                                        \
    if (errCode != NRF_SUCCESS)          \
    {                                    \
        NRF_LOG_ERROR(string, errCode);  \
    }                                    \
}

#define LOG_ERROR_RETURN(string, errCode)\
{                                        \
    if (errCode != NRF_SUCCESS)          \
    {                                    \
        NRF_LOG_ERROR(string, errCode);  \
        return errCode;                  \
    }                                    \
}

#define LOG_WARNING_CHECK(string, errCode) \
{                                          \
    if (errCode != NRF_SUCCESS)            \
    {                                      \
        NRF_LOG_WARNING(string, errCode);  \
    }                                      \
}

#define IS_DEBUGGER_ATTACHED()                              \
    ((CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) ||  \
     (CoreDebug->DEMCR & CoreDebug_DEMCR_MON_EN_Msk))

/* Exported functions ------------------------------------------------------- */

/** @brief function to initialize the debug module
 */
ret_code_t dbg_Init(void);

/** @brief execute function of the debug module
 *
 * @note call this function regularly (preferably in the main loop)
 *
 * @return true  there is still data to process
 * @return false no more data to process
 */
bool dbg_Execute(void);

ret_code_t dbg_DataSend(uint8_t* pData, uint16_t* pLength, uint16_t connHandle);

#endif // DEBUG_H_INCLUDED

/**END OF FILE*****************************************************************/
