/**
  ******************************************************************************
  * @file    mode management.c
  * @author  Thomas Reisnecker
  * @brief   mode management module
  ******************************************************************************
  */

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

/* Includes ------------------------------------------------------------------*/
#include "nordic_common.h"
#include "ble_types.h"

#include "mode_management.h"
#include "btle.h"
#include "com_message_handling.h"
#include "debug.h"
#include "data_storage.h"

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
    uint32_t isValid;       // valid pattern to check if noinit data needs to be initialized
    uint8_t currentMode;    // containing the current mode
    uint8_t lastMode;       // last mode in case of temporary mode
} mmState_t;

typedef struct
{
    uint8_t numOfModes;     // the number of modes stored
    uint8_t version;        // mode version (to convert if changed in future)
    uint16_t dummy;         // dummy bytes for alignment
    mm_modeConfig_t modes[MM_NUM_OF_MODES];
} storageFormat_t;

/* Private macros ------------------------------------------------------------*/

/* Private defines -----------------------------------------------------------*/
#define VALID_PATTERN       0x30d334bd

#define MODES_FILE_ID       0x30de
#define MODES_RECORD_BASE   0x0001

#define MODES_VERSION       0
#define DEFAULT_SET         0

#define MODES_DEFAULTS                          \
{ /* ignore first  last   pref   temp   off   */\
    {false, true,  false, false, false, false}, \
    {false, false, true,  false, false, false}, \
    {false, true,  false, false, false, false}, \
    {false, false, true,  false, false, false}, \
    {true,  true,  false, false, false, false}, \
    {true,  false, false, false, false, false}, \
    {true,  false, false, false, false, false}, \
    {true,  false, true,  false, false, false}, \
}

#define MODES_STR_DEFAULTS                      \
{                                               \
    .numOfModes = MM_NUM_OF_MODES,              \
    .version    = MODES_VERSION,                \
}

/* Private function prototype ------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static mm_modeChangedHandler_t modeChangeHandler;
static mmState_t               state __attribute__((section(".noinit")));
static mm_modeConfig_t         modes[MM_NUM_OF_MODES] = MODES_DEFAULTS;
static storageFormat_t         newModes  __ALIGN(4) = MODES_STR_DEFAULTS;

/* Private functions ---------------------------------------------------------*/
/** @brief function to jump to the next mode
 *
 * @param[in] currentMode
 * @return    the next available mode
 *
 * @note this function relies on a correct setup with at least one available
 *       mode and correctly set up group boarders, otherwise it will end up in
 *       a dead loop or hard fault
 */
static uint8_t getNextMode(uint8_t currentMode)
{
    uint8_t newMode;

    // in off mode just use the first available mode
    if (currentMode == MM_MODE_OFF) /// TODO: SOS
    {
        newMode = 0;
        while (modes[newMode].ignore) {newMode++;}
        return newMode;
    }

    newMode = currentMode;
    do
    {
        // if this is not the last, go to the next mode
        if (!modes[newMode].lastInGroup)
            newMode++;
        // otherwise roll back to the first
        else
        {
            while (!modes[newMode].firstInGroup) {newMode--;}
        }
        // check if the mode is enabled
        if (!modes[newMode].ignore)
            return newMode;
    } while (newMode != currentMode); // in case of no available modes in current group

    // if there are no active modes in the current group, just go to the next
    // available mode ignoring group limits
    while (modes[newMode].ignore)
    {
        if (++newMode >= MM_NUM_OF_MODES)
            newMode = 0;
    }

    return newMode;
}

/** @brief function to jump to the previous mode
 *
 * @param[in] currentMode
 * @return    the previous available mode
 *
 * @note this function relies on a correct setup with at least one available
 *       mode and correctly set up group boarders, otherwise it will end up in
 *       a dead loop or hard fault
 */
static uint8_t getPreviousMode(uint8_t currentMode)
{
    uint8_t newMode;

    // in off mode just use the first available mode
    if (currentMode == MM_MODE_OFF) /// TODO: SOSO
    {
        newMode = MM_NUM_OF_MODES - 1;
        while (modes[newMode].ignore) {newMode--;}
        return newMode;
    }

    newMode = currentMode;
    do
    {
        // if this is not the first, go to the previous mode
        if (!modes[newMode].firstInGroup)
            newMode--;
        // otherwise roll back to the last
        else
        {
            while (!modes[newMode].lastInGroup) {newMode++;}
        }
        // check if the mode is enabled
        if (!modes[newMode].ignore)
            return newMode;
    } while (newMode != currentMode); // in case of no available modes in current group

    // if there are no active modes in the current group, just go to the next
    // available mode ignoring group limits
    while (modes[newMode].ignore)
    {
        if (newMode-- == 0)
            newMode = MM_NUM_OF_MODES - 1;
    }

    return newMode;
}

/**< returns true is the group containing mode has at least one valid member */
static bool isValidGroup(uint8_t mode)
{
    while (!modes[mode].firstInGroup) {mode--;}
    while (!modes[mode].lastInGroup)
    {
        if (!modes[mode++].ignore)
            return true;
    }
    return false;
}

/** @brief function to jump to the next group
 *
 * @param[in] currentMode
 * @return    the correspondent mode in the next group
 *
 * @note this function relies on a correct setup with at least one available
 *       mode and correctly set up group boarders, otherwise it will end up in
 *       a dead loop or hard fault
 */
static uint8_t getNextGroup(uint8_t currentMode)
{
    uint8_t posInGroup = 0;

    // in off mode start with the first available mode
    if (currentMode == MM_MODE_OFF)  /// TODO: SOS
    {
        currentMode = 0;
        while (modes[currentMode].ignore)
        {
            currentMode++;
            posInGroup++;
        }
    }
    // if not off, determine the position of the current mode in its group
    else
    {
        while (!modes[currentMode].firstInGroup)
        {
            currentMode--;
            posInGroup++;
        }
    }

    // now go to the first mode in the next valid group
    do
    {
        while (!modes[currentMode++].lastInGroup) {}
        if (currentMode >= MM_NUM_OF_MODES)    // roll over to first mode
            currentMode = 0;
    } while (!isValidGroup(currentMode));

    // finally go to the same position in this group (if possible)
    while (posInGroup-- && !modes[currentMode].lastInGroup) {currentMode++;}

    if (modes[currentMode].ignore)
        currentMode = getNextMode(currentMode);

    return currentMode;
}

/** @brief function to jump to the previous group
 *
 * @param[in] currentMode
 * @return    the correspondent mode in the previous group
 *
 * @note this function relies on a correct setup with at least one available
 *       mode and correctly set up group boarders, otherwise it will end up in
 *       a dead loop or hard fault
 */
static uint8_t getPreviousGroup(uint8_t currentMode)
{
    uint8_t posInGroup = 0;

    // in off mode start with the first available mode
    if (currentMode == MM_MODE_OFF) /// TODO: SOS
    {
        currentMode = MM_NUM_OF_MODES - 1;
        while (modes[currentMode].ignore) {currentMode--;}
    }
    // if not off, determine the position of the current mode in its group, referenced to the last member
    else
    {
        while (!modes[currentMode].lastInGroup)
        {
            currentMode++;
            posInGroup++;
        }
    }

    // now go to the last mode in the previous valid group
    do
    {
        while (!modes[currentMode--].firstInGroup) {}
        if (currentMode >= MM_NUM_OF_MODES)    // roll over to last mode
            currentMode = MM_NUM_OF_MODES - 1;
    } while (!isValidGroup(currentMode));

    // finally go to the same position in this group (if possible)
    while (posInGroup-- && !modes[currentMode].firstInGroup) {currentMode--;}

    if (modes[currentMode].ignore)
        currentMode = getPreviousMode(currentMode);

    return currentMode;
}

static uint8_t getOffMode()
{
    for (uint_fast8_t i = 0; i < ARRAY_SIZE(modes); i++)
    {
        if (modes[i].isOffMode)
            return i;
    }
    return MM_MODE_OFF;
}

static uint8_t getPrefMode()
{
    for (uint_fast8_t i = 0; i < ARRAY_SIZE(modes); i++)
    {
        if (modes[i].isPrefMode)
            return i;
    }
    return MM_MODE_OFF;
}

static uint8_t getTempMode()
{
    for (uint_fast8_t i = 0; i < ARRAY_SIZE(modes); i++)
    {
        if (modes[i].isTempMode)
            return i;
    }
    return MM_MODE_OFF;
}

/*static uint8_t sizeOfGroup(uint8_t mode)
{
    uint8_t size = 1;
    while (!modeCfg.pGeneral[mode--].firstInGroup) {}
    while (!modeCfg.pGeneral[mode++].lastInGroup)
    {
        size++;
    }
    return size;
}*/

static bool isModeConfigValid(mm_modeConfig_t const* pModes)
{
    int8_t group = 0;
    uint8_t temp = 0, pref = 0, off = 0;
    for (uint_fast8_t i = 0; i < ARRAY_SIZE(modes); i++)
    {
        if (pModes[i].firstInGroup)
            group++;
        if (pModes[i].lastInGroup)
            group--;
        if (pModes[i].isPrefMode)
            pref++;
        if (pModes[i].isTempMode)
            temp++;
        if (pModes[i].isOffMode)
            off++;

        if (group > 1 || group < 0 || pref > 1 || temp > 1 || off > 1)
            return false;
    }
    return true;
}

static ret_code_t loadModes(uint8_t set)
{
    ret_code_t             errCode;
    storageFormat_t const* pModes;
    uint16_t               lengthWords;

    errCode = ds_Read(MODES_FILE_ID, MODES_RECORD_BASE + set, (void const**)&pModes, &lengthWords);
    if (errCode == NRF_SUCCESS)
    {
        if (lengthWords == BYTES_TO_WORDS(sizeof(storageFormat_t)) &&
            isModeConfigValid(pModes->modes))
            memcpy(modes, pModes->modes, sizeof(modes));
    }
    else if (errCode != FDS_ERR_NOT_FOUND)
    {
        NRF_LOG_ERROR("mode record load error %d", errCode);
    }

    return errCode;
}

static ret_code_t storeModes(uint8_t set, ds_reportHandler_t handler)
{
    ret_code_t errCode;
    void const* pData      = (void const*)&newModes;
    uint16_t lenghtInWords = BYTES_TO_WORDS(sizeof(storageFormat_t));

    errCode = ds_Write(MODES_FILE_ID, MODES_RECORD_BASE + set, pData, lenghtInWords, handler);

    LOG_ERROR_CHECK("error %d storing modes", errCode);
    return errCode;
}

/*static ret_code_t deleteModes(uint8_t set)
{
    /// HINT: if more sets are used and they should be deleted all use fds_file_delete()

    ret_code_t errCode;
    fds_record_desc_t desc;
    fds_find_token_t  tok = {0};

    errCode = fds_record_find(MODES_FILE_ID, MODES_RECORD_BASE + set, &desc, &tok);
    if (errCode == NRF_SUCCESS)
        errCode = fds_record_delete(&desc);
    else if(errCode == FDS_ERR_NOT_FOUND)
        errCode = NRF_SUCCESS;

    return errCode;
}*/

static bool isValidMode(uint8_t mode)
{
    if (mode == MM_MODE_OFF || /* mode == MM_MODE_SOS || */
        mode < ARRAY_SIZE(modes))
        return true;
    else
        return false;
}

/* Public functions ----------------------------------------------------------*/
ret_code_t mm_Init(mm_modeChangedHandler_t modeHandler, uint8_t* pInitMode)
{
    ret_code_t errCode;

    if (modeHandler == NULL || pInitMode == NULL)
        return NRF_ERROR_NULL;

    modeChangeHandler = modeHandler;

    // check if state is valid, if not initialize
    if (state.isValid != VALID_PATTERN ||
        !isValidMode(state.currentMode) ||
        !isValidMode(state.lastMode))
    {
        state.isValid = VALID_PATTERN;
        state.currentMode = MM_MODE_OFF;
        state.lastMode = MM_MODE_OFF;
    }

    *pInitMode = state.currentMode;

    errCode = loadModes(DEFAULT_SET);
    if (errCode != NRF_SUCCESS && errCode != FDS_ERR_NOT_FOUND)
        return errCode;

    return NRF_SUCCESS;
}

ret_code_t mm_HmiEventHandler(hmi_evt_t const* pEvent)
{
    bool relay = true;
    uint16_t connHandle = BLE_CONN_HANDLE_ALL;

    switch (pEvent->type)
    {
    case HMI_EVT_MODEOFF:
        state.currentMode = MM_MODE_OFF;//getOffMode();
        state.lastMode = state.currentMode;
        break;

    case HMI_EVT_MODEPREF:
    {
        uint8_t prefMode = getPrefMode();
        if (state.currentMode == prefMode)
            state.currentMode = MM_MODE_OFF;
        else
            state.currentMode = prefMode;
        state.lastMode = state.currentMode;
    }   break;

    case HMI_EVT_MODETEMP:
    {
        uint8_t tempMode = getTempMode();
        relay = false;
        if (state.currentMode == tempMode)
            state.currentMode = state.lastMode;
        else
            state.currentMode = tempMode;
    }   break;

    case HMI_EVT_MODENUMBER:
        connHandle = pEvent->params.mode.connHandle;
        if (pEvent->params.mode.modeNumber >= MM_NUM_OF_MODES)
            state.currentMode = MM_MODE_OFF;    /// TODO: it's better to relay the received mode (in case of other devices support more modes)
        else
            state.currentMode = pEvent->params.mode.modeNumber;
        state.lastMode = state.currentMode;
        break;

    case HMI_EVT_NEXTMODE:
        state.currentMode = getNextMode(state.currentMode);
        state.lastMode = state.currentMode;
        break;

    case HMI_EVT_NEXTGROUP:
        state.currentMode = getNextGroup(state.currentMode);
        state.lastMode = state.currentMode;
        break;

    case HMI_EVT_PREVMODE:
        state.currentMode = getPreviousMode(state.currentMode);
        state.lastMode = state.currentMode;
        break;

    case HMI_EVT_PREVGROUP:
        state.currentMode = getPreviousGroup(state.currentMode);
        state.lastMode = state.currentMode;
        break;

    default:
    case HMI_EVT_MODESOS:
        /// todo:
        break;
    }

    /// TODO: check if mode has changed at all
    if ((state.currentMode == MM_MODE_OFF) && (mm_GetOffMode() != MM_MODE_OFF))
    {
        NRF_LOG_INFO("new mode %d (%d)", state.currentMode, mm_GetOffMode())
    }
    else
    {
        NRF_LOG_INFO("new mode %d", state.currentMode);
    }

    if (relay)
    {
        ret_code_t errCode;
        btle_modeRelay_t relay;

        // if the conn handle is invalid, the mode change was initiated through
        // the wired com, so no relay to com, but to all ble devices
        if (connHandle != BLE_CONN_HANDLE_INVALID)
        {
            errCode = cmh_RelayMode(state.currentMode);
            if (errCode != NRF_SUCCESS &&
                errCode != NRF_ERROR_INVALID_STATE && // no com at all
                errCode != NRF_ERROR_NOT_SUPPORTED)   // no transmitter, just receiving
            {
                NRF_LOG_WARNING("com relay error %d", errCode);
            }
        }
        else
            connHandle = BLE_CONN_HANDLE_ALL;

        relay.connHandle = connHandle;
        relay.mode = state.currentMode;
        errCode = btle_RelayMode(&relay);
        if (errCode != NRF_SUCCESS)
            LOG_WARNING_CHECK("ble relay error %d", errCode);
    }

    modeChangeHandler(state.currentMode);

    return NRF_SUCCESS;
}

uint8_t mm_GetCurrentMode()
{
    return state.currentMode;
}

uint8_t mm_GetOffMode()
{
    return getOffMode();
}

/*mm_modeConfig_t const* mm_GetModeConfig(uint8_t mode)
{
    if (mode >= ARRAY_SIZE(modes))
        return NULL;
    else
        return &modes[mode];
}*/

ret_code_t mm_GetModeConfigs(mm_modeConfig_t const** ppModeConfigs)
{
    if (ppModeConfigs == NULL)
        return NRF_ERROR_NULL;

    *ppModeConfigs = modes;

    return NRF_SUCCESS;
}

ret_code_t mm_CheckModeConfig(mm_modeConfig_t const* pModeConfig, uint16_t size)
{
    if (pModeConfig == NULL)
        return NRF_ERROR_NULL;

    if (size != sizeof(modes))
        return NRF_ERROR_INVALID_LENGTH;

    return isModeConfigValid(pModeConfig) ? NRF_SUCCESS : NRF_ERROR_INVALID_PARAM;
}

ret_code_t mm_SetModeConfig(mm_modeConfig_t const* pModeConfig, uint16_t size, ds_reportHandler_t resultHandler)
{
    if (pModeConfig == NULL)
        return NRF_ERROR_NULL;

    if (size != sizeof(modes))
        return NRF_ERROR_INVALID_LENGTH;

    if (!isModeConfigValid(pModeConfig))
        return NRF_ERROR_INVALID_PARAM;

    memcpy(modes, pModeConfig, sizeof(modes));

    memcpy(newModes.modes, modes, sizeof(modes));

    return storeModes(DEFAULT_SET, resultHandler);
}

ret_code_t mm_FactoryReset(ds_reportHandler_t resultHandler)
{
    if (resultHandler == NULL)
        return NRF_ERROR_NULL;

    // set defaults setting
    mm_modeConfig_t defaults[] = MODES_DEFAULTS;
    memcpy(modes, defaults, sizeof(defaults));

    // initiate deletion
    return ds_Reset(MODES_FILE_ID, resultHandler);
}

/**< temporary functions to support light control service */

/*#if (MM_NUM_OF_MODES & (MM_NUM_OF_MODES - 1)) != 0
#error temporary lcs functions only working with mode numbers of power of two
#endif

ret_code_t mm_UpdateGroupConfig(ble_lcs_ctrlpt_group_cnfg_t const* pLcs, ds_reportHandler_t resultHandler)
{
    if (resultHandler == NULL)
        return NRF_ERROR_NULL;

    if (pLcs->number_of_groups == 0 || pLcs->number_of_groups >= ARRAY_SIZE(modes))
        return NRF_ERROR_INVALID_PARAM;

    memcpy(newModes.modes, modes, sizeof(modes));

    // delete old configuration
    for (uint_fast8_t i = 0; i < ARRAY_SIZE(modes); i++)
    {
        newModes.modes[i].firstInGroup = false;
        newModes.modes[i].lastInGroup = false;
    }

    // regular distributed groups
    if (pLcs->p_list_of_modes_per_group == NULL)
    {
        uint_fast8_t numOfModesPerGroup = ARRAY_SIZE(newModes.modes) / pLcs->number_of_groups;
        for (uint_fast8_t i = 0; i < ARRAY_SIZE(newModes.modes); i += numOfModesPerGroup)
        {
            newModes.modes[i].firstInGroup = true;
            newModes.modes[i + numOfModesPerGroup - 1].lastInGroup = true;
        }
    }
    else    // individual grouping
    {
        uint8_t sumOfModes = 0;
        for (uint_fast8_t i = 0; i < pLcs->number_of_groups; i++)
        {
            sumOfModes += pLcs->p_list_of_modes_per_group[i];
        }
        if (sumOfModes != ARRAY_SIZE(newModes.modes))
            return NRF_ERROR_INVALID_PARAM;

        uint8_t const* pNumOfModesPerGroup = pLcs->p_list_of_modes_per_group;
        for (uint_fast8_t i = 0; i < ARRAY_SIZE(newModes.modes); )
        {
            newModes.modes[i].firstInGroup = true;
            i += *pNumOfModesPerGroup;
            newModes.modes[i - 1].lastInGroup = true;
            pNumOfModesPerGroup++;
        }
    }

    if (!isModeConfigValid(newModes.modes))
        return NRF_ERROR_INVALID_PARAM;

    memcpy(modes, newModes.modes, sizeof(modes));

    return storeModes(DEFAULT_SET, resultHandler);
}

ret_code_t mm_UpdateModeConfig(ble_lcs_ctrlpt_mode_cnfg_t const* pLcs, ds_reportHandler_t resultHandler)
{
    if (resultHandler == NULL)
        return NRF_ERROR_NULL;

    if (pLcs->mode_number_start >= ARRAY_SIZE(modes) ||
        pLcs->mode_number_start + pLcs->mode_entries > ARRAY_SIZE(modes))
        return NRF_ERROR_INVALID_PARAM;

    memcpy(newModes.modes, modes, sizeof(modes));

    for (uint8_t i = 0; i < pLcs->mode_entries; i++)
    {
        newModes.modes[i + pLcs->mode_number_start].ignore =
            pLcs->config_hlmt[i].setup.spot && pLcs->config_hlmt[i].intensity ? false : true;
    }

    if (!isModeConfigValid(newModes.modes))
        return NRF_ERROR_INVALID_PARAM;

    memcpy(modes, newModes.modes, sizeof(modes));

    return storeModes(DEFAULT_SET, resultHandler);
}

uint8_t mm_GetPrefMode()
{
    return getPrefMode();
}

ret_code_t mm_SetPrefMode(uint8_t mode, ds_reportHandler_t resultHandler)
{
    if (resultHandler == NULL)
        return NRF_ERROR_NULL;

    memcpy(newModes.modes, modes, sizeof(modes));

    uint8_t oldPref = getPrefMode();
    if (oldPref < ARRAY_SIZE(newModes.modes))
        newModes.modes[oldPref].isPrefMode = false;
    if (mode < ARRAY_SIZE(newModes.modes))
        newModes.modes[mode].isPrefMode = true;

    if (!isModeConfigValid(newModes.modes))  // can only fail if more than one pref mode was set before
        return NRF_ERROR_INVALID_PARAM;

    memcpy(modes, newModes.modes, sizeof(modes));

    return storeModes(DEFAULT_SET, resultHandler);
}

uint8_t mm_GetTempMode()
{
    return getTempMode();
}

ret_code_t mm_SetTempMode(uint8_t mode, ds_reportHandler_t resultHandler)
{
    if (resultHandler == NULL)
        return NRF_ERROR_NULL;

    memcpy(newModes.modes, modes, sizeof(modes));

    uint8_t oldTemp = getTempMode();
    if (oldTemp < ARRAY_SIZE(newModes.modes))
        newModes.modes[getTempMode()].isTempMode = false;
    if (mode < ARRAY_SIZE(newModes.modes))
        newModes.modes[mode].isTempMode = true;

    if (!isModeConfigValid(newModes.modes))  // can only fail if more than one temp mode was set before
        return NRF_ERROR_INVALID_PARAM;

    memcpy(modes, newModes.modes, sizeof(modes));

    return storeModes(DEFAULT_SET, resultHandler);
}*/

/**END OF FILE*****************************************************************/
