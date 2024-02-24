/**
  ******************************************************************************
  * @file    KD20_10.c
  * @author  Thomas Reisnecker
  * @brief   board managment module for KD2 Hardware 1.0
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
NRF_LOG_MODULE_REGISTER();

/* Includes ------------------------------------------------------------------*/
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "fds.h"
#include "nrfx_nvmc.h"

#include "board.h"
#include "mode_management.h"
#include "btle.h"
#include "button.h"
#include "debug.h"
#include "KD2_10.h"
#include "KD2_hmi.h"
#include "KD2_adc.h"
#include "KD2_pwm.h"
#include "KD2_curr.h"
#include "debug.h"
#include "limiter.h"
#include "i2c.h"
#include "main.h"
#include "bmi160drv.h"
#include "bmi160_defs.h"
#include "pitch_reg.h"
#include "com_message_handling.h"
#include "helena_base_driver.h"
#include "KD2_btle.h"

/* External variables --------------------------------------------------------*/

/* Private defines -----------------------------------------------------------*/
#define COMP_ADDR_UICR          (&NRF_UICR->CUSTOMER[0])

#define SDA_PIN                 31
#define SCL_PIN                 30

#define COM_PIN                 27
#define COM_DUMMY               8

#define BUTTON_PIN              21

#define PWM_PIN                 14

#define BMI_ONBOARD             0
#define BMI_MODULE              1

#define PRG_TYPE_NA             KD2_OPTIC_TYPE_NA

#define DEVICENAME_LENGHT       BLE_GAP_DEVNAME_DEFAULT_LEN

#define DEFAULT_SET             0           // the default config set, different sets not implemented yet, so 0
#define CHANNELS_VERSION        0           // channel version to identify changes for future extensions
#define CONFIG_VERSION          0
#define CHANNELS_FILE_ID        0x13D2      // the file id for this module
#define CHANNELS_RECORD_BASE    0x0101      // the base for channel configuration
#define CONFIG_RECORD_BASE      0x0111      // the base for config (should be renamed to setup as in the app)
#define NAME_RECORD_BASE        0x0121      // the base for the device name

#define CHANNEL_DEFAULTS                                                      \
{                                                                             \
    {{ 0, 0}, { 0, 0}, { 0, 0}, { 0, 0}, {0,  0}, {0,  0}, {0,  0}, {0,  0}}, \
    {{ 0, 0}, { 0, 0}, { 0, 0}, { 0, 0}, {0,  0}, {0,  0}, {0,  0}, {0,  0}}, \
    {{ 0, 0}, { 0, 0}, { 5, 0}, {16, 0}, {0,  0}, {0,  0}, {0,  0}, {0,  0}}, \
    {{ 0, 0}, { 0, 0}, { 0, 0}, { 0, 0}, {0,  0}, {0,  0}, {0,  0}, {0,  0}}, \
    {{ 0, 0}, { 0, 0}, { 0, 0}, { 0, 0}, {0,  0}, {0,  0}, {0,  0}, {0,  0}}, \
}

#define CHANNEL_STR_DEFAULTS                \
{                                           \
    .numOfModesPerChannel = MM_NUM_OF_MODES,\
    .version      = CHANNELS_VERSION,       \
}

// default compensation is just gain 1 and no offset
#define CONFIG_COMP_DEFAULTS            \
{                                       \
    {.gain = (1 << 15)},                \
    {.gain = (1 << 15)},                \
    {.gain = (1 << 15)},                \
}

// defaults for prototype
/*#define CONFIG_COMP_DEFAULTS            \
{                                       \
    {.gain = (1 << 15)},                \
    {.gain = 34576},                    \
    {.gain = (1 << 15), .offset = -448} \
}*/

#define CHANNEL_SETUP_DEFAULT_CURR  \
{                                   \
    .outputPower = (18 << 10),      \
    .outputLimit = (85 << 8),       \
    .optic.type = PRG_TYPE_15,      \
}

#define CHANNEL_SETUP_DEFAULT_PWM   \
{                                   \
    .outputLimit = KD2_TARGET_MAX,  \
    .optic.type = PRG_TYPE_NA       \
}

#define CHANNEL_SETUP_DEFAULT       \
{                                   \
    CHANNEL_SETUP_DEFAULT_CURR,     \
    CHANNEL_SETUP_DEFAULT_CURR,     \
    CHANNEL_SETUP_DEFAULT_CURR,     \
    CHANNEL_SETUP_DEFAULT_PWM,      \
    CHANNEL_SETUP_DEFAULT_PWM       \
}

#define CONFIG_DEFAULTS                     \
{                                           \
    .channelSetup = CHANNEL_SETUP_DEFAULT,  \
    .comp = CONFIG_COMP_DEFAULTS,           \
    .comPinMode = KD2_COMPIN_NOTUSED,       \
    .deviceName = "Helen" \
}

#define CONFIG_STR_DEFAULTS         \
{                                   \
    .numOfChannels = CHANNEL_CNT,   \
    .version       = CONFIG_VERSION,\
}

// prescale values for 1Hz message rate
#define MSG_PRESCALE_IDLE       1
#define MSG_PRESCALE_ON         7692

// prescale values for channel updates
// 1Hz in idle (currently not used, but maybe in future), 60Hz in on mode
#define UPD_PRESCALE_IDLE       1
#define UPD_PRESCALE_ON         128

// 3A in q3_13_t, used for all current channels
#define FULL_CURRENT            (3 << 13)

#define SIZE_OF_MODES           (sizeof(channels_t) + MM_NUM_OF_MODES * sizeof(mm_modeConfig_t))
#define SIZE_OF_QWR_BUFFER      (((SIZE_OF_MODES / 18) + 1) * 24)

/* Private typedef -----------------------------------------------------------*/
/**< the channel configurations */
typedef enum
{
    CHANNEL_HELENAFLOOD = 0,    // the left current driver on the helena base
    CHANNEL_HELENASPOT,         // the right current driver on the helena base
    CHANNEL_CURRENT,            // the onboard current driver
    CHANNEL_PWM,                // the onboard PWM channel
    CHANNEL_COMPWM,             // the PWM channel assigned to the com pin
    CHANNEL_CNT                 // the number of channels
} channelTypes_t;

typedef enum
{
    CHMODE_CONST = 0,
    CHMODE_PITCH,
} channelModes_t;

typedef struct
{
    cht_5int3mode_t helenaFlood[MM_NUM_OF_MODES]; // helena driver flood
    cht_5int3mode_t helenaSpot[MM_NUM_OF_MODES];  // helena driver spot
    cht_5int3mode_t current[MM_NUM_OF_MODES];     // onboard current driver
    cht_5int3mode_t pwm[MM_NUM_OF_MODES];         // onboard PWM output
    cht_5int3mode_t comPwm[MM_NUM_OF_MODES];      // PWM signal on com pin
} channels_t;

/**< the channel configurations storage format*/
typedef struct
{
    uint8_t      numOfModesPerChannel;
    uint8_t      version;
    uint16_t     dummy;
    channels_t   channels;
} channels_str_t;

/**< device configuration */
typedef struct
{
    KD2_channelSetup_t    channelSetup[CHANNEL_CNT];        // the channel configurations
    KD2_adcCompensation_t comp;                             // the adc compensation data
    uint8_t               IMUAddr;                          // device address of IMU, 0 if not available
    uint8_t               helenaBaseAddr;                   // device address of helena base, 0 if not available
    bool                  backsideAssembled;                // indicator if back side of board is fully assembled
    KD2_comPinMode_t      comPinMode;                       // mode of com pin
    char                  deviceName[DEVICENAME_LENGHT + 1];// the device name if changed
} config_t;

/**< device configuration storage format */
typedef struct
{
    uint8_t      numOfChannels;
    uint8_t      version;
    uint16_t     dummy;
    config_t     config;
} config_str_t;

typedef struct
{
    bool temperature : 1;
    bool voltage     : 1;
} limiting_t;

/* function prototypes -------------------------------------------------------*/
static void bmiHandler(bmi160drv_event_t const* pEvent);
static void reorientOnboard(int16_t* pAccel, int16_t* pGyro);
static void reorientModule(int16_t* pAccel, int16_t* pGyro);

/* Private read only variables -----------------------------------------------*/
static const cht_types_t channelTypes[] =
{
    CHT_TYPE_CURRENT,   // left(flood) current regulator on helena base driver
    CHT_TYPE_CURRENT,   // right(spot) current regulator on helena base driver
    CHT_TYPE_CURRENT,   // onboard current regulator
    CHT_TYPE_PWM,       // onboard PWM output
    CHT_TYPE_PWM,       // PWM signal on com pin
};

static const bmi160drv_init_t bmiInit[] =
{
    {   // configuration for onboard BMI160
        .addr        = BMI160_I2C_ADDR,
        .int1Pin     = 26,
        .reorient    = reorientOnboard,
        .handler     = bmiHandler,
    },
    {   // configuration for BMI160 on external module
        .addr        = BMI160_I2C_ADDR + 1,
        .int1Pin     = BMI160DRV_NOINT,
        .reorient    = reorientModule,
        .handler     = bmiHandler,
    },
};

/* Private variables ---------------------------------------------------------*/
static channels_t           channels = CHANNEL_DEFAULTS;
static channels_str_t       newChannels __ALIGN(4) = CHANNEL_STR_DEFAULTS;

static config_t             config = CONFIG_DEFAULTS;
static config_str_t         newConfig __ALIGN(4) = CONFIG_STR_DEFAULTS;

static uint8_t              modesBuffer[SIZE_OF_MODES];
static brd_features_t       features;
static ble_hps_modes_init_t modes;
static uint8_t              qwrBuffer[SIZE_OF_QWR_BUFFER];

static hbd_inst_t           helenaInst;

static lim_channelDef_t     limiter[CHANNEL_CNT];
static limiting_t           limActive[CHANNEL_CNT];
static uint16_t             updatePrescale;
static uint8_t              batteryCellCnt;
FIL_LOWPASS_DEF(filterVolt, 3000);
FIL_LOWPASS_DEF(filterTemp, 25);

static brd_powerMode_t      currentPowerMode, pendingPowerMode;
static uint8_t              currentLightMode = MM_MODE_OFF, pendingLightMode = MM_MODE_OFF;
static bool                 isMultiOpticMode;   // indicating if the current mode is a pitch compensated mode with both 15 and 30° optic

static KD2_target_t         currentOutputPower[CHANNEL_CNT];
static hbd_samplingData_t   lastHelenaResults;
static KD2_adc_result_t     lastAdcResults;
FIL_MOVAVG_DEF(filterPitch, 8);
static q15_t                pitch;

static uint16_t             messagePrescale;

/* Board inforamtion --------------------------------------------------------*/
static const btle_info_t bleInfo =
{
    .pDevicename = (char const*)&config.deviceName,
    .pManufacturer = NULL,
    .pModelNumber = "KD2",
    .pBoardHWVersion = "1.0",
    .deviceAppearance = BLE_APPEARANCE_GENERIC_CYCLING,
};

static const brd_info_t     brdInfo =
{
    .pFeatures = &features,
    .pInfo = &bleInfo,
    .pModes = &modes,
    .qwrBuffer.p_mem = qwrBuffer,
    .qwrBuffer.len   = sizeof(qwrBuffer),
};

/* Private functions ---------------------------------------------------------*/
/** @brief helper to check if a given channel configuration is valid
 */
static bool isChannelConfigValid(channels_t const* pChannels)
{
    for (uint_fast8_t i = 0; i < MM_NUM_OF_MODES; i++)
    {
        // intensity is only valid from 0 .. 100 and special feature requires IMU
        if (pChannels->current[i].intensity > CHT_PERCENT_TO_53(100) ||
            (pChannels->current[i].mode == CHMODE_PITCH && config.IMUAddr == 0))
            return false;

        // ignore helena settings if board is not available
        if (config.helenaBaseAddr)
        {   // intensity needs to be within limits, and special feature requires IMU
            if (pChannels->helenaFlood[i].intensity > CHT_PERCENT_TO_53(100) ||
                (pChannels->helenaFlood[i].mode == CHMODE_PITCH && config.IMUAddr == 0))
                return false;
            if (pChannels->helenaSpot[i].intensity > CHT_PERCENT_TO_53(100) ||
                (pChannels->helenaSpot[i].mode == CHMODE_PITCH && config.IMUAddr == 0))
                return false;
        }

        // if board is not fully assembled ignore configuration for PWM channels
        if (config.backsideAssembled)
        {
            // PWM channel intensity needs to be within limits
            if (pChannels->pwm[i].intensity > CHT_PERCENT_TO_53(100))
                return false;

            // second PWM channel needs to be withing limits if activated
            if (config.comPinMode == KD2_COMPIN_PWM && pChannels->comPwm[i].intensity > CHT_PERCENT_TO_53(100))
                return false;
        }
    }
    return true;
}

/** @brief function to load configuration from flash
 * @param[in] set  reserved for future use
 */
static ret_code_t loadChannels(uint8_t set)
{
    ret_code_t            errCode;
    channels_str_t const* pChannels;
    uint16_t              lengthWords;

    errCode = ds_Read(CHANNELS_FILE_ID, CHANNELS_RECORD_BASE + set, (void const**)&pChannels, &lengthWords);
    if (errCode == NRF_SUCCESS)
    {
        if (lengthWords == BYTES_TO_WORDS(sizeof(channels_str_t)) &&
            isChannelConfigValid(&pChannels->channels))
            memcpy(&channels, &pChannels->channels, sizeof(channels_t));
    }
    else if (errCode != FDS_ERR_NOT_FOUND)
    {
        NRF_LOG_ERROR("[BRD]: record find error %d", errCode);
    }

    return errCode;
}

/** @brief function to store configuration to flash
 * @param[in] set     reserved for future use
 * @param[in] handler the handler to be relayed to the write function
 */
static ret_code_t storeChannels(uint8_t set, ds_reportHandler_t handler)
{
    ret_code_t errCode;
    void const* pData      = (void const*)&newChannels;
    uint16_t lenghtInWords = BYTES_TO_WORDS(sizeof(channels_str_t));

    errCode = ds_Write(CHANNELS_FILE_ID, CHANNELS_RECORD_BASE + set, pData, lenghtInWords, handler);

    LOG_ERROR_CHECK("[BRD]: error %d storing channels", errCode);
    return errCode;
}

/** @brief function to delete configuration in flash
 * @param[in] set  reserved for future use
 */
/*static ret_code_t deleteChannels(uint8_t set)
{
    /// HINT: if more sets are used and they should be deleted all use fds_file_delete()

    ret_code_t errCode;
    fds_record_desc_t desc;
    fds_find_token_t  tok = {0};

    errCode = fds_record_find(CHANNELS_FILE_ID, CHANNELS_RECORD_BASE + set, &desc, &tok);
    if (errCode == NRF_SUCCESS)
        errCode = fds_record_delete(&desc);
    else if(errCode == FDS_ERR_NOT_FOUND)
        errCode = NRF_SUCCESS;

    return errCode;
}*/

/** @brief helper to determine if channel setup is valid
 */
static bool isChannelSetupValid(KD2_channelSetup_t const* pSetup)
{
    // output power is not limited yet

    // output limit needs to be between 0 and 100%
    if (pSetup->outputLimit > KD2_TARGET_MAX || pSetup->outputPower < KD2_TARGET_MIN)
        return false;

    // optic needs to be a valid type
    if (pSetup->optic.type >= PRG_TYPE_CNT && pSetup->optic.type != PRG_TYPE_NA)
        return false;

    // optic offset need to be between +-45°
    if (pSetup->optic.offset > KD2_PITCH_OFF_MAX || pSetup->optic.offset < KD2_PITCH_OFF_MIN)
        return false;

    return true;
}

/** @brief function to load compensation data from UICR
 *
 * @param[out] pComp
 * @return NRF_SUCCESS
 *         NRF_ERROR_INVALID_STATE if no data stored in UICR
 */
static ret_code_t loadCompFromUICR(KD2_adcCompensation_t* pComp)
{
    KD2_adcCompensation_t compare;
    memset(&compare, 0xFF, sizeof(KD2_adcCompensation_t));

    if (memcmp((uint32_t*)COMP_ADDR_UICR, &compare, sizeof(KD2_adcCompensation_t)) != 0)
    {
        memcpy(pComp, (uint32_t*)COMP_ADDR_UICR, sizeof(KD2_adcCompensation_t));
        return NRF_SUCCESS;
    }
    else
        return NRF_ERROR_INVALID_STATE;
}

/** @brief function to save compensation to UICR
 *
 * @param[in] pComp the compensation to store
 * @return    NRF_SUCCESS if stored successful
 *            NRF_ERROR_INVALID_DATA if compensation data represents the defaults
 *            NRF_ERROR_INVALID_STATE if UICR is not empty
 */
static ret_code_t saveCompToUICR(KD2_adcCompensation_t const* pComp)
{
    // compare settings with default settings, to prevent saving defaults
    KD2_adcCompensation_t compare = CONFIG_COMP_DEFAULTS;
    if (memcmp(pComp, &compare, sizeof(KD2_adcCompensation_t)) == 0)
        return NRF_ERROR_INVALID_DATA;

    // check if there are already settings saved
    memset(&compare, 0xFF, sizeof(KD2_adcCompensation_t));
    if (memcmp((uint32_t*)COMP_ADDR_UICR, &compare, sizeof(KD2_adcCompensation_t)) != 0)
        return NRF_ERROR_INVALID_STATE;

    nrfx_nvmc_bytes_write((uint32_t)COMP_ADDR_UICR, pComp, sizeof(KD2_adcCompensation_t));

    return NRF_SUCCESS;
}

static bool isCompensationValid(KD2_adcCompensation_t const* pComp)
{
    KD2_adcCompensation_t compUCIR;

    if (loadCompFromUICR(&compUCIR) == NRF_SUCCESS)
    {
        // for safety reasons the new current gain factor cannot be lower than
        // 93.75% of the stored value in UICR
        if (pComp->current.gain < compUCIR.current.gain - (compUCIR.current.gain >> 4))
            return false;
    }

    return true;
}

static bool isComPinModeValid(KD2_comPinMode_t mode)
{
    if (mode > KD2_COMPIN_PWM)
        return false;
    else
        return true;
}

static bool isDeviceNameValid(char const* pName)
{
    return strlen(pName) <= DEVICENAME_LENGHT;
}

/** @brief function to load configuration from flash
 */
static ret_code_t loadConfig()
{
    ret_code_t          errCode;
    config_str_t const* pConfig;
    uint16_t            lengthWords;

    (void)loadCompFromUICR(&config.comp);

    errCode = ds_Read(CHANNELS_FILE_ID, CONFIG_RECORD_BASE, (void const**)&pConfig, &lengthWords);
    if (errCode == NRF_SUCCESS)
    {
        if (lengthWords == BYTES_TO_WORDS(sizeof(config_str_t)) && pConfig->numOfChannels == CHANNEL_CNT)
        {
            bool channelSetupValid = true;
            for (uint8_t i = 0; i < pConfig->numOfChannels; i++)
            {
                channelSetupValid = channelSetupValid && isChannelSetupValid(&pConfig->config.channelSetup[i]);
            }
            if (channelSetupValid)
                memcpy(config.channelSetup, pConfig->config.channelSetup, sizeof(config.channelSetup));
            if (isCompensationValid(&pConfig->config.comp))
            {
                memcpy(&config.comp, &pConfig->config.comp, sizeof(config.comp));
                if (saveCompToUICR(&config.comp) == NRF_SUCCESS)
                {
                    NRF_LOG_INFO("[BRD]: adc compensation stored to UICR")
                }
            }
            if (isComPinModeValid(pConfig->config.comPinMode))
                config.comPinMode = pConfig->config.comPinMode;
            /// TODO: validity check for hardware setup and I2C device addresses?
            config.backsideAssembled = pConfig->config.backsideAssembled;
            config.helenaBaseAddr    = pConfig->config.helenaBaseAddr;
            config.IMUAddr           = pConfig->config.IMUAddr;

            if (isDeviceNameValid(pConfig->config.deviceName))
                strcpy(config.deviceName, pConfig->config.deviceName);
        }
    }
    else if (errCode != FDS_ERR_NOT_FOUND)
    {
        NRF_LOG_ERROR("[BRD]: record find error %d", errCode);
    }

    return errCode;
}

/** @brief function to store configuration to flash
 * @param[in] handler the handler to be relayed to the write function
 */
static ret_code_t storeConfig(ds_reportHandler_t handler)
{
    ret_code_t errCode;
    void const* pData      = (void const*)&newConfig;
    uint16_t lenghtInWords = BYTES_TO_WORDS(sizeof(config_str_t));

    errCode = ds_Write(CHANNELS_FILE_ID, CONFIG_RECORD_BASE, pData, lenghtInWords, handler);

    LOG_ERROR_CHECK("[BRD]: error %d storing config", errCode);
    return errCode;
}


/** @brief adc result handler
 *
 * @note In ON mode this handler has the highest possible IRQ priority, so no bongabonga
 */
static void adcResultHander(KD2_adc_result_t const* pResult)
{
    if (messagePrescale)
        messagePrescale--;

    if (updatePrescale)
        updatePrescale--;

    // store results
    lastAdcResults.inputVoltage = pResult->inputVoltage;
    lastAdcResults.ledCurrent   = pResult->ledCurrent;
    if (pResult->temperature)
    {
        lastAdcResults.temperature = pResult->temperature;
        fil_LowPassFeed(&filterTemp, (int32_t)pResult->temperature << 9);
    }

    if (currentLightMode != MM_MODE_OFF)
    {
        // relay current to regulator
        int32_t actual = lastAdcResults.ledCurrent;
        actual *= (100 << 8);
        actual /= FULL_CURRENT;
        KD2_Curr_ReportCurrent(actual);

        // low pass filter for use with limiter
        fil_LowPassFeed(&filterVolt, (int32_t)pResult->inputVoltage << 5);
    }
    else
    {
        // no voltage filtering in idle mode
        fil_LowPassReset(&filterVolt, (int32_t)pResult->inputVoltage << 5);
    }
}

static bool isHelenaModeUsed(uint8_t lightMode)
{
    if (lightMode >= ARRAY_SIZE(channels.helenaFlood))
        return false;

    /// TODO: what about special feature?
    return config.helenaBaseAddr != 0 &&
           (channels.helenaFlood[lightMode].intensity ||
            channels.helenaSpot[lightMode].intensity);
}

static bool isCurrentModeUsed(uint8_t lightMode)
{
    if (lightMode >= ARRAY_SIZE(channels.current))
        return false;

    /// TODO: what about special feature?
    return (bool)channels.current[lightMode].intensity;
}

static bool isPWMModeUsed(uint8_t lightMode)
{
    if (lightMode >= ARRAY_SIZE(channels.pwm))
        return false;

    return config.backsideAssembled &&
           (channels.pwm[lightMode].intensity ||
            (config.comPinMode == KD2_COMPIN_PWM && channels.comPwm[lightMode].intensity));
}

/** @brief helper to determine if any channel is active for this mode
 */
static bool isModeUsed(uint8_t lightMode)
{
    // if (lightMode == MM_MODE_SOS)
    // return true;

    if (lightMode >= MM_NUM_OF_MODES)
        return false;

    return isHelenaModeUsed(lightMode) || isCurrentModeUsed(lightMode) || isPWMModeUsed(lightMode);
}

/** @brief helper to check if the light mode is a mode which uses pitch
 *  compensation on channels with 15° and 30°
 */
static bool isMultiOptic(uint8_t lightMode)
{
    bool is15 = false, is30 = false;

    if (lightMode >= MM_NUM_OF_MODES)
        return false;

    for (uint8_t i = 0; i < CHANNEL_CNT; i++)
    {
        cht_5int3mode_t* pCh = &channels.helenaFlood[lightMode];                 // set pointer to this mode in the first channel
        pCh += i * MM_NUM_OF_MODES;                                         // and increase pointer to desired channel

        if ((config.helenaBaseAddr == 0 && i <= CHANNEL_HELENASPOT) ||      // ignore helena driver channels if not available
            (!config.backsideAssembled && i >= CHANNEL_PWM) ||              // ignore PWM channels if backside not assembled
            (config.comPinMode != KD2_COMPIN_PWM && i >= CHANNEL_COMPWM) || // ignore com PWM if not enabled
            (pCh->mode != CHMODE_PITCH))                                    // or ignore if pitch compensation not used for this mode and channel
            continue;

        if (config.channelSetup[i].optic.type == PRG_TYPE_15)
            is15 = true;
        else if (config.channelSetup[i].optic.type == PRG_TYPE_30)
            is30 = true;
    }

    return is15 && is30;
}

/** @brief function to check for power mode changes and the necessary actions
 *
 * @param[in] powerMode the new power mode (as defined in board.h)
 * @param[in] lightMode the new light mode
 */
static void setMode(brd_powerMode_t powerMode, uint8_t lightMode)
{
    /// TODO: error checking

    static KD2_powerMode_t adcPM, currPM, pwmPM;
    static bmi160drv_powerMode_t imuPM;
    static bool helenaPM;

    KD2_powerMode_t newPM;
    bmi160drv_powerMode_t newImuPM;
    bool newHelenaPM;

    // adc is on whenever any of the channels is enabled (for limiter)
    newPM = powerMode == BRD_PM_IDLE && isModeUsed(lightMode) ? KD2_PWR_ON : (KD2_powerMode_t)powerMode;
    if (adcPM != newPM)
    {
        (void)KD2_Adc_SetPowerMode(newPM);
        // reset message prescaler when switching between ON and IDLE mode
        if (powerMode == BRD_PM_IDLE)
        {
            messagePrescale = 0;
            updatePrescale = 0;
        }

        if (config.comPinMode != KD2_COMPIN_COM)    // com pin needs clock all the time
        {   // request or release the crystal clock for accurate temperature measurements
            /// TODO: for for idle power reduction, clock can be enabled just for measurement period,
            ///       but be aware, that it needs to be enabled in ON mode to prevent the PWM frequency jitter
            if (newPM >= KD2_PWR_IDLE && adcPM < KD2_PWR_IDLE)
                (void)sd_clock_hfclk_request();
            else if (newPM < KD2_PWR_IDLE && adcPM >= KD2_PWR_IDLE)
                (void)sd_clock_hfclk_release();
        }
        /// TODO: maybe it's better to source clock request and release into
        ///       cumulative functions and let every module (com, adc, PWM,
        ///       current, what else?) request clock independently.

        // request or release the crystal clock, otherwise the PWM frequency will jitter
        // com need hfclock all the time, so no need to request in that case
        // clock only necessary for onboard current and PWM, but not for helena
        // board channels, but this is not evaluated yet
        /*if (config.comPinMode != KD2_COMPIN_COM)
        {
            if (newPM == KD2_PWR_ON && adcPM != KD2_PWR_ON)
                (void)sd_clock_hfclk_request();
            else if (adcPM == KD2_PWR_ON && newPM != KD2_PWR_ON)
                (void)sd_clock_hfclk_release();
        }*/

        adcPM = newPM;
    }

    if (config.helenaBaseAddr)
    {
        // no need to differentiate between IDLE and ON, driver board does this on its own
        newHelenaPM = powerMode == BRD_PM_IDLE;
        if (helenaPM != newHelenaPM)
        {
            hbd_config_t hlCfg =
            {
                .sampleRate = HBD_SAMPLERATE_1SPS,
                .sleepMode = newHelenaPM ? HBD_SLEEP_MODE_ON : HBD_SLEEP_MODE_OFF,
            };
            (void)hbd_SetConfig(&helenaInst, &hlCfg);
            helenaPM = newHelenaPM;
        }
    }

    newPM = powerMode == BRD_PM_IDLE && isCurrentModeUsed(lightMode) ? KD2_PWR_ON : (KD2_powerMode_t)powerMode;
    if (currPM != newPM)
    {
        (void)KD2_Curr_SetPowerMode(newPM);
        currPM = newPM;
    }

    newPM = powerMode == BRD_PM_IDLE && isPWMModeUsed(lightMode) ? KD2_PWR_ON : (KD2_powerMode_t)powerMode;
    if (pwmPM != newPM)
    {
        (void)KD2_Pwm_SetPowerMode(newPM);
        pwmPM = newPM;
    }

    /// TODO: when low power mode is implemented in BMI160 driver a simple cast will not be sufficient anymore
    if (config.IMUAddr)
    {
        newImuPM = (bmi160drv_powerMode_t)powerMode;
        if (imuPM != newImuPM)
        {
            (void)bmi_SetPowerMode(newImuPM);
            imuPM = newImuPM;
        }
    }

    if (lightMode != currentLightMode)
    {
        isMultiOpticMode = isMultiOptic(lightMode);
    }

    currentPowerMode = powerMode;
    currentLightMode = lightMode;
}

/** @brief temporary helper function to convert the current helen channel configuration to LCS mode
 *
 * @param[out] pLcsMode
 */
static void getLcsMode(ble_lcs_hlmt_mode_t* pLcsMode)
{
    if (pLcsMode == NULL)
        return;

    memset(pLcsMode, 0, sizeof(ble_lcs_hlmt_mode_t));

    uint8_t currentMode = mm_GetCurrentMode();
    if (currentMode == MM_MODE_OFF)
        currentMode = mm_GetOffMode();
    if (currentMode == MM_MODE_OFF)
        return;

    cht_5int3mode_t const* pChannel = &channels.current[currentMode];

    pLcsMode->intensity = CHT_53_TO_PERCENT(pChannel->intensity);
    pLcsMode->setup.spot = pChannel->intensity ? 1 : 0;
    pLcsMode->setup.pitchCompensation = pChannel->mode == CHMODE_PITCH ? 1 : 0;
}

/** @brief temporary function to generate the LCS status flags
 *
 * @param[out] pLcsStatus
 * @return void
 */
static void getLcsStatus(ble_lcs_lm_status_flags_t* pLcsStatus)
{
    if (pLcsStatus == NULL)
        return;

    memset(pLcsStatus, 0, sizeof(ble_lcs_lm_status_flags_t));

    pLcsStatus->temperature = limActive[CHANNEL_CURRENT].temperature ? 1 : 0;
    pLcsStatus->voltage = limActive[CHANNEL_CURRENT].voltage ? 1 : 0;
}

static void sendHpsMessage()
{
    if (currentPowerMode != BRD_PM_IDLE)
        return;

    ret_code_t errCode;
    btle_hpsMeasurement_t message;
    uint32_t power;

    message.mode = mm_GetCurrentMode();//currentLightMode;
    message.outputPower = 0;
    /// TODO: read helena driver to calculate their power
    if (isModeUsed(currentLightMode))
    {
        for (uint8_t i = 0; i < CHANNEL_CNT; i++)
        {
            power = config.channelSetup[i].outputPower;
            power *= currentOutputPower[i];
            power /= KD2_TARGET_MAX;
            power *= 1000;
            power >>= 10;
            message.outputPower += power;
        }
    }
    message.inputVoltage = (1000ul * lastAdcResults.inputVoltage) >> 11;
    message.temperature = (int16_t)(lastAdcResults.temperature >> 7) - 273;

    errCode = btle_ReportHpsMeasurements(&message);
    if (errCode != NRF_SUCCESS &&
        errCode != NRF_ERROR_INVALID_STATE && // ignore error if notifications are not enabled
        errCode != NRF_ERROR_RESOURCES)       // ignore error if message buffer is full
    {
        NRF_LOG_ERROR("[BRD]: error %d sending message", errCode);
    }
}

/** @brief function to send a ble notification with the current data
 */
static void sendLcsMessage()
{
    /// TODO: include PWM channels and helena driver

    if (currentPowerMode != BRD_PM_IDLE)
        return;

    ret_code_t errCode;
    btle_lcsMeasurement_t message;
    int32_t power, angle;
    q3_13_t tailPower;
    q7_9_t soc;

    power = config.channelSetup[CHANNEL_CURRENT].outputPower;
    power *= lastAdcResults.ledCurrent;
    power /= FULL_CURRENT;
    power *= 1000;  // convert from q6_10_t to mW
    power >>= 10;

    getLcsMode(&message.mode);
    getLcsStatus(&message.statusSpot);
    if (currentLightMode != MM_MODE_OFF)
        message.powerSpot = power;
    else
        message.powerSpot = 0;
    message.inputVoltage = (1000ul * lastAdcResults.inputVoltage) >> 11;
    message.temperature = (int16_t)(lastAdcResults.temperature >> 7) - 273;

    angle = pitch;
    if (angle >= (1 << 14))
        angle -= (1l << 15);
    angle = (angle * 360) >> 15;
    if (angle <= 90 && angle >= -90)
        message.pitch = angle;
    else
        message.pitch = 91;     // out of valid range, won't be included

    if (cmh_GetBatterySOC(&soc) == NRF_SUCCESS)
        message.batterySoc = soc >> 9;
    else
        message.batterySoc = 101;   // out of valid range, won't be included

    if (cmh_GetTaillightPower(&tailPower) == NRF_SUCCESS)
        message.powerTaillight = (tailPower * 1000ul) >> 13;
    else
        message.powerTaillight = 0; // out of valid range, won't be included

    errCode = btle_ReportLcsMeasurements(&message);
    if (errCode != NRF_SUCCESS &&
        errCode != NRF_ERROR_INVALID_STATE && // ignore error if notifications are not enabled
        errCode != NRF_ERROR_RESOURCES)       // ignore error if message buffer is full
    {
        NRF_LOG_ERROR("[BRD]: error %d sending message", errCode);
    }
}

/** @brief function to set the emergency current limit
 */
static ret_code_t setCurrentLimit(KD2_target_t limit)
{
    int32_t curr;
    ret_code_t errCode;

    // convert the relative input to adc current format
    curr = (int32_t)limit * FULL_CURRENT;
    curr /= KD2_TARGET_MAX;

    errCode = KD2_Adc_SetCurrentLimit(curr, KD2_Curr_GetAbortTask());
    LOG_ERROR_CHECK("[BRD]: error %d setting current limit", errCode);
    return errCode;
}

static ret_code_t applyCompensation(KD2_adcCompensation_t const* comp)
{
    int32_t gain, offset;

    gain = comp->current.gain;
    gain *= KD2_ADC_CURRGAIN_DEFAULT;
    gain >>= 15;

    offset =  KD2_ADC_TEMPOFF_DEFAULT;
    offset += comp->temperature.offset;

    return KD2_Adc_SetCompensation(gain, offset);
}

static void limiterInit(KD2_adc_result_t const* pAdc)
{
    q15_16_t fullPower = 0;

    for (channelTypes_t i = 0; i < CHANNEL_CNT; i++)
    {
        limiter[i].fullPower = (int32_t)config.channelSetup[i].outputPower << 6;
        fullPower += limiter[i].fullPower;
    }

    for (channelTypes_t i = 0; i < CHANNEL_CNT; i++)
    {
        // a channel with less than 5% of the total output power gets the
        // highest priority regardless of the configured optic
        if (config.channelSetup[i].outputPower * 20 > fullPower)
            limiter[i].priority = LIM_PRIO_HIGH;
        // spot drivers get mid priority
        else if (config.channelSetup[i].optic.type == PRG_TYPE_15)
            limiter[i].priority = LIM_PRIO_MID;
        // all others get low priority
        else
            limiter[i].priority = LIM_PRIO_LOW;
    }

    batteryCellCnt = lim_CalcLiionCellCnt((int32_t)pAdc->inputVoltage << 5);
}

static void getTargets(KD2_target_t* pTargets, uint8_t lightMode)
{
    int32_t target;

    memset(pTargets, 0, sizeof(KD2_target_t) * CHANNEL_CNT);

    if (lightMode >= MM_NUM_OF_MODES)
        return;

    for (uint8_t i = 0; i < CHANNEL_CNT; i++)
    {
        cht_5int3mode_t* pCh = &channels.helenaFlood[lightMode];                 // set pointer to this mode in the first channel
        pCh += i * MM_NUM_OF_MODES;                                         // and increase pointer to desired channel

        if ((config.helenaBaseAddr == 0 && i <= CHANNEL_HELENASPOT) ||      // ignore helena driver channels if not available
            (!config.backsideAssembled && i >= CHANNEL_PWM) ||              // ignore PWM channels if backside not assembled
            (config.comPinMode != KD2_COMPIN_PWM && i >= CHANNEL_COMPWM))   // ignore com PWM if not enabled
            continue;

        target = CHT_53_TO_PERCENT(pCh->intensity) << 8;

        if (pCh->mode == CHMODE_PITCH)
        {
            (void)prg_GetComp(pitch, &config.channelSetup[i].optic, &target);
            if (isMultiOpticMode)
                (void)prg_MultiOptic(pitch, &config.channelSetup[i].optic, &target);
        }

        if (target > KD2_TARGET_MAX)
            pTargets[i] = KD2_TARGET_MAX;
        else if (target < KD2_TARGET_MIN)
            pTargets[i] = KD2_TARGET_MIN;
        else
            pTargets[i] = target;
    }
}

/** @brief function to update channels
 */
static void updateExecute(uint8_t lightMode)
{
    ret_code_t errCode;
    hbd_retVal_t helenaCode;
    lim_channelReq_t targetsLim[CHANNEL_CNT];
    KD2_target_t     targetsKD2[CHANNEL_CNT] = {0};
    q15_16_t power, tempLimit, voltLimit;

    if (currentPowerMode != BRD_PM_IDLE || !isModeUsed(lightMode))
        return;

    // generate targets
    getTargets(targetsKD2, lightMode);

    // limit to hardware / settings
    for (uint_fast8_t i = 0; i < CHANNEL_CNT; i++)
    {
        if (targetsKD2[i] > config.channelSetup[i].outputLimit)
            targetsKD2[i] = config.channelSetup[i].outputLimit;
    }

    // get the voltage and temperature limits
    power = 0;
    for (uint_fast8_t i = 0; i < CHANNEL_CNT; i++)
    {
        power += limiter[i].fullPower;
    }
    voltLimit = lim_CalcLiionVoltageLimit(fil_LowPassGet(&filterVolt), batteryCellCnt);
    tempLimit = lim_CalcTemperatureLimit(fil_LowPassGet(&filterTemp));

    if (voltLimit < tempLimit)
        power = ((int64_t)power * voltLimit) >> 16;
    else
        power = ((int64_t)power * tempLimit) >> 16;

    // run targets through limiter
    for (uint_fast8_t i = 0; i < CHANNEL_CNT; i++)
    {
        targetsLim[i].request = (((q15_16_t)targetsKD2[i] << 8) + 50) / 100l;
        targetsLim[i].limiting = false;
    }

    errCode = lim_LimitChannels(power, limiter, (lim_channelReq_t*)targetsLim, CHANNEL_CNT);
    LOG_ERROR_CHECK("[BRD]: limiter calculation error %d", errCode);

    for (uint_fast8_t i = 0; i < CHANNEL_CNT; i++)
    {
        targetsKD2[i] = ((targetsLim[i].request * 100l) + 128) >> 8;
        if (voltLimit < tempLimit)
            limActive[i].voltage = targetsLim[i].limiting;
        else
            limActive[i].temperature = targetsLim[i].limiting;
    }

    // send targets
    if (isCurrentModeUsed(lightMode))
    {
        errCode = KD2_Curr_SetCurrent(targetsKD2[CHANNEL_CURRENT]);
        LOG_ERROR_CHECK("[BRD]: error %d setting current", errCode);
    }

    if (isPWMModeUsed(lightMode))
    {
        KD2_Pwm_dc_t dc = {.channels = {targetsKD2[CHANNEL_PWM], targetsKD2[CHANNEL_COMPWM]}};
        errCode = KD2_Pwm_SetDC(&dc);
        LOG_ERROR_CHECK("[BRD]: error %d setting DC", errCode);
    }

    if (config.helenaBaseAddr)
    {
        q8_t flood, spot;
        flood = targetsKD2[CHANNEL_HELENAFLOOD] >= KD2_TARGET_MAX ? 255 : targetsKD2[CHANNEL_HELENAFLOOD] / 100;
        spot  = targetsKD2[CHANNEL_HELENASPOT] >= KD2_TARGET_MAX ? 255 : targetsKD2[CHANNEL_HELENASPOT] / 100;
        helenaCode = hbd_SetTargetCurrent(&helenaInst, lastAdcResults.inputVoltage, flood, spot);
        if (helenaCode != HBD_SUCCESS && helenaCode != HBD_ERROR_FORBIDDEN)
        {
            NRF_LOG_ERROR("[BRD]: error %d setting helena current", helenaCode);
        }
    }

    // calculate current output power
    /// TODO: read helena board for actual currents
    for (channelTypes_t i = 0; i <= CHANNEL_CURRENT; i++)
    {
        int32_t power;
        if (i == CHANNEL_HELENAFLOOD)
            power = lastHelenaResults.currentLeft.current << 3;
        else if (i == CHANNEL_HELENASPOT)
            power = lastHelenaResults.currentRight.current << 3;
        else // if (i == CHANNEL_CURRENT)
            power = lastAdcResults.ledCurrent;
        power *= KD2_TARGET_MAX;
        power /= FULL_CURRENT;  // all driver have the same max output current
        currentOutputPower[i] = power;
    }
    for (channelTypes_t i = CHANNEL_PWM; i < CHANNEL_CNT; i++)
    {
        currentOutputPower[i] = targetsKD2[i];
    }
}

static ret_code_t initI2c()
{
    return i2c_Init(SDA_PIN, SCL_PIN, NRF_TWIM_FREQ_400K);
}

static void bmiHandler(bmi160drv_event_t const* pEvent)
{
    if (pEvent->type == BMI160_EVT_TYPE_MOTION)
        main_ResetIdleTimer();

    else if (pEvent->type == BMI160_EVT_TYPE_DATA)
    {
        // convert from 0..360° to -180..180° to prevent trumpery at 0°
        int32_t rawPitch = pEvent->data.euler[0];
        if (rawPitch >= (1 << 14))
            rawPitch -= (1 << 15);
        // convert back to 0..360°
        int32_t filteredPitch = fil_MovAvg(&filterPitch, rawPitch);
        if (filteredPitch < 0)
            filteredPitch += (1 << 15);
        pitch = filteredPitch;
    }
}

/**< remaps data for onboard sensor
 *   x_enu = y_raw, y_enu = -z_raw, z_enu = -x_raw
 */
static void reorientOnboard(int16_t* pAccel, int16_t* pGyro)
{
    int16_t i;

    i = pAccel[0];
    pAccel[0] =      pAccel[1];
    pAccel[1] = -1 * pAccel[2];
    pAccel[2] = -1 * i;

    i = pGyro[0];
    pGyro[0] =      pGyro[1];
    pGyro[1] = -1 * pGyro[2];
    pGyro[2] = -1 * i;
}

/**< remaps data for onboard sensor
 *   x_enu = -y_raw, y_enu = z_raw, z_enu = -x_raw
 */
static void reorientModule(int16_t* pAccel, int16_t* pGyro)
{
    /// TODO: test!

    int16_t i;

    i = pAccel[0];
    pAccel[0] = -1 * pAccel[1];
    pAccel[1] =      pAccel[2];
    pAccel[2] = -1 * i;

    i = pGyro[0];
    pGyro[0] = -1 * pGyro[1];
    pGyro[1] =      pGyro[2];
    pGyro[2] = -1 * i;
}

/** @brief function to initialize the bmi160 driver
 *
 * @param[out] pAddrs  the address used, 0 if no sensor found
 * @return NRF_SUCCESS, NRF_ERROR_NOT_FOUND or propagated error
 */
static ret_code_t initBmi160(uint8_t* pAddrs)
{
    ret_code_t errCode;
    *pAddrs = 0;

    for (uint_fast8_t i = 0; i < ARRAY_SIZE(bmiInit); i++)
    {
        errCode = bmi_Init(&bmiInit[i]);
        if (errCode == NRF_SUCCESS)
        {
            *pAddrs = bmiInit[i].addr;
            return NRF_SUCCESS;
        }
        else if (errCode != NRF_ERROR_NOT_FOUND)
            return errCode;
    }

    return NRF_ERROR_NOT_FOUND;
}

/** @brief function to initialize the helena base driver
 *
 * @param[out] pAddrs  if present, the device address, otherwise 0
 * @return NRF_SUCCESS or NRF_ERROR_NOT_FOUND
 */
static ret_code_t initHelenaDriver(uint8_t* pAddrs)
{
    hbd_retVal_t errCode;
    hbd_init_t init;
    uint8_t cnt = 1;    // only one instance available

    init.i2cRead = i2c_Read;
    init.i2cWrite = i2c_Write;

    errCode = hbd_Init(&init, &helenaInst, &cnt);
    if (errCode == HBD_SUCCESS)
    {
        *pAddrs = helenaInst.address;
        return NRF_SUCCESS;
    }
    else
    {
        *pAddrs = 0;
        return NRF_ERROR_NOT_FOUND;
    }
}

/** @brief helper function to check if the backside of the board is assembled
 * @note Checks if the 4k7 pull-up resistor on COM pin is available
 */
static bool isBacksideAssembled()
{
    bool assembled;
    // check the state of the COM pin, when configured as input with pull down.
    // if the external 4k7 pull up is assembled the state should be high
    nrf_gpio_cfg_input(COM_PIN, NRF_GPIO_PIN_PULLDOWN);
    nrf_delay_ms(10);
    assembled = (bool)nrf_gpio_pin_read(COM_PIN);
    nrf_gpio_cfg_default(COM_PIN);

    return assembled;
}

/** @brief function sets the board feature structure depending whats available
 */
static void setFeatures(brd_features_t* pFtr, bool isAssembled, uint8_t bmiAddr, uint8_t helenaAddr)
{
    // minimal setup
    pFtr->maxNameLenght      = DEVICENAME_LENGHT;
    pFtr->channelCount       = 1;
    pFtr->pChannelTypes      = &channelTypes[CHANNEL_CURRENT];
    pFtr->comSupported.rx    = COM_PIN_NOT_USED;
    pFtr->comSupported.tx    = COM_PIN_NOT_USED;
    pFtr->comSupported.dummy = COM_PIN_NOT_USED;
    pFtr->isIMUPresent       = (bool)bmiAddr;

    // two additional channels if helena driver board is present
    if (helenaAddr)
    {
        pFtr->channelCount += 2;
        pFtr->pChannelTypes = &channelTypes[CHANNEL_HELENAFLOOD];;
    }

    // if assembled, PWM channel is available
    if (isAssembled)
    {
        pFtr->channelCount++;
        //depending on com pin configuration another channel or com is available
        if (config.comPinMode == KD2_COMPIN_PWM)
            pFtr->channelCount++;
        else if (config.comPinMode == KD2_COMPIN_COM)
        {
            pFtr->comSupported.rx    = COM_PIN;
            pFtr->comSupported.tx    = COM_PIN;
            pFtr->comSupported.dummy = COM_DUMMY;
        }
    }
}

static void setModes(ble_hps_modes_init_t* pModes, uint16_t channelCount, cht_5int3mode_t const* pFirstChannel)
{
    mm_modeConfig_t const* pModesConfig;
    uint16_t sizeModes = MM_NUM_OF_MODES * sizeof(mm_modeConfig_t);
    uint16_t sizeChannels = channelCount * MM_NUM_OF_MODES * sizeof(cht_5int3mode_t);

    (void)mm_GetModeConfigs(&pModesConfig); // cannot fail unless fed with null-pointer
    memcpy(modesBuffer, pModesConfig, sizeModes);
    memcpy(&modesBuffer[sizeModes], pFirstChannel, sizeChannels);

    pModes->p_mode_config = (ble_hps_mode_config_t*)modesBuffer;
    pModes->total_size = sizeModes + sizeChannels;
}

/** @brief function to compare the results of init function with the saved config setup
 */
static bool hasHardwareSetupChanged(bool isAssembled, uint8_t bmiAddr, uint8_t helenaAddr)
{
    if (config.backsideAssembled != isAssembled ||
        config.IMUAddr           != bmiAddr ||
        config.helenaBaseAddr    != helenaAddr)
        return true;
    else
        return false;
}

/* Public functions ----------------------------------------------------------*/
ret_code_t brd_Init(brd_info_t const* *pInfo)
{
    ret_code_t       errCode;
    KD2_adc_init_t   adcInit = {0};
    bool             backsideAssembled;
    uint32_t         secondaryPin;
    uint8_t          bmiAddr, helenaAddr = 0;

    if (pInfo == NULL)
        return NRF_ERROR_NULL;

    *pInfo = &brdInfo;

    backsideAssembled = isBacksideAssembled();

    errCode = loadConfig();
    if (errCode != NRF_SUCCESS && errCode != FDS_ERR_NOT_FOUND)
        return errCode;

    if (backsideAssembled && config.comPinMode == KD2_COMPIN_BUTTON)
        secondaryPin = COM_PIN;
    else
        secondaryPin = BUTTON_NOT_USED;
    errCode = KD2_Hmi_Init(BUTTON_PIN, secondaryPin);
    if (errCode != NRF_SUCCESS)
        return errCode;

    errCode = applyCompensation(&config.comp);
    if (errCode != NRF_SUCCESS)
        return errCode;

    adcInit.resultHandler = adcResultHander;
    errCode = KD2_Adc_Init(&adcInit, &lastAdcResults);
    if (errCode != NRF_SUCCESS)
        return errCode;

    errCode = initI2c();
    if (errCode != NRF_SUCCESS)
        return errCode;

    errCode = initBmi160(&bmiAddr);
    if (errCode != NRF_SUCCESS && errCode != NRF_ERROR_NOT_FOUND)
        return errCode;

    errCode = initHelenaDriver(&helenaAddr);
    if (errCode != NRF_SUCCESS && errCode != NRF_ERROR_NOT_FOUND)
        return errCode;

    KD2_Pwm_init_t pwmInit = KD2_PWM_INIT_ALL_DISABLED;
    if (backsideAssembled)
    {
        pwmInit.channels[0].pinNo = PWM_PIN;
        pwmInit.channels[0].type = KD2_PWM_PUSHPULL;
        if (config.comPinMode == KD2_COMPIN_PWM)
        {
            pwmInit.channels[0].pinNo = COM_PIN;
            pwmInit.channels[0].type = KD2_PWM_OPENDRAININVERTED;
        }
    }
    errCode = KD2_Pwm_Init(&pwmInit);
    if (errCode != NRF_SUCCESS)
        return errCode;

    errCode = KD2_Curr_Init();      /// TODO: change api from internal fixed pin to variable pin through init
    if (errCode != NRF_SUCCESS)
        return errCode;

    // set emergency shutdown current to 125% of configured max. current
    KD2_target_t limit = config.channelSetup[CHANNEL_CURRENT].outputLimit;
    limit += limit >> 2;
    errCode = setCurrentLimit(limit);
    if (errCode != NRF_SUCCESS)
        return errCode;

    limiterInit(&lastAdcResults);

    if (hasHardwareSetupChanged(backsideAssembled, bmiAddr, helenaAddr))
    {
        /// TODO: save setup and send characteristic changed indication
        config.backsideAssembled = backsideAssembled;
        config.IMUAddr = bmiAddr;
        config.helenaBaseAddr = helenaAddr;
    }

    errCode = loadChannels(DEFAULT_SET);
    if (errCode != NRF_SUCCESS && errCode != FDS_ERR_NOT_FOUND)
        return errCode;

    setFeatures(&features, backsideAssembled, bmiAddr, helenaAddr);

    setModes(&modes, features.channelCount, helenaAddr ? channels.helenaFlood : channels.current);

    errCode = KD2_btle_Init(&features, (bool)bmiAddr);
    if (errCode != NRF_SUCCESS)
        return errCode;

    return NRF_SUCCESS;
}

ret_code_t brd_SetPowerMode(brd_powerMode_t newMode)
{
    pendingPowerMode = newMode;

    return NRF_SUCCESS;
}

ret_code_t brd_SetLightMode(uint8_t newMode)
{
    pendingLightMode = newMode;

    return NRF_SUCCESS;
}

#include "SEGGER_RTT.h"
bool brd_Execute(void)
{
    KD2_Hmi_Execute();
    KD2_Adc_Execute();
    KD2_Curr_Execute();
    bmi_Execute();

    if (pendingPowerMode != currentPowerMode || pendingLightMode != currentLightMode)
        setMode(pendingPowerMode, pendingLightMode);

    if (updatePrescale == 0)
    {
        updateExecute(currentLightMode);
        updatePrescale = isModeUsed(currentLightMode) ? UPD_PRESCALE_ON : UPD_PRESCALE_IDLE;
    }

    if (messagePrescale == 0)
    {
        sendLcsMessage();
        sendHpsMessage();
        messagePrescale = isModeUsed(currentLightMode) ? MSG_PRESCALE_ON : MSG_PRESCALE_IDLE;

        // without acceleration sensor idle timer is reset when light is used (and not in an active OFF mode)
        if (mm_GetCurrentMode() != MM_MODE_OFF && isModeUsed(currentLightMode))
            main_ResetIdleTimer();
    }

    // check if limiting is active
    if (isModeUsed(currentLightMode))    // limiting can only be active in on mode
    {
        for (uint_fast8_t i = 0; i < CHANNEL_CNT; i++)
        {
            if (limActive[i].temperature || limActive[i].voltage)
                return true;
        }
    }

    return false;
}

ret_code_t brd_UpdateChannelConfig(ble_lcs_ctrlpt_mode_cnfg_t const* pLcs, ds_reportHandler_t resultHandler)
{
    if (resultHandler == NULL)
        return NRF_ERROR_NULL;

    if (pLcs->mode_number_start >= ARRAY_SIZE(newChannels.channels.current) ||
        pLcs->mode_number_start + pLcs->mode_entries > ARRAY_SIZE(newChannels.channels.current))
        return NRF_ERROR_INVALID_PARAM;

    memcpy(&newChannels.channels, &channels, sizeof(channels));

    for (uint8_t i = 0; i < pLcs->mode_entries; i++)
    {
        newChannels.channels.current[i + pLcs->mode_number_start].intensity =
            CHT_PERCENT_TO_53(pLcs->config_hlmt[i].intensity);
        if (pLcs->config_hlmt[i].setup.pitchCompensation)
            newChannels.channels.current[i + pLcs->mode_number_start].mode = CHMODE_PITCH;
        else
            newChannels.channels.current[i + pLcs->mode_number_start].mode = CHMODE_CONST;
    }

    if (!isChannelConfigValid(&newChannels.channels))
        return NRF_ERROR_INVALID_PARAM;

    memcpy(&channels, &newChannels.channels, sizeof(channels));

    return storeChannels(DEFAULT_SET, resultHandler);
}

ret_code_t brd_GetChannelConfig(void const** ppData, uint16_t* pSize)
{
    if (ppData == NULL || pSize == NULL)
        return NRF_ERROR_NULL;

    if (config.helenaBaseAddr)
        *ppData = &channels.helenaFlood;
    else
        *ppData = &channels.current;
    *pSize = sizeof(channels.current) * features.channelCount;  /// TODO: as long as all channels use the same size channel type this works

    return NRF_SUCCESS;
}

ret_code_t brd_SetChannelConfig(void const* pData, uint16_t size, ds_reportHandler_t resultHandler)
{
    if (pData == NULL)
        return NRF_ERROR_NULL;

    if (size != sizeof(channels.current) * features.channelCount)
        return NRF_ERROR_INVALID_LENGTH;

    cht_5int3mode_t* pFirstChannel = config.helenaBaseAddr ? newChannels.channels.helenaFlood : newChannels.channels.current;

    memcpy(pFirstChannel, pData, size);

    if (!isChannelConfigValid(&newChannels.channels))
        return NRF_ERROR_INVALID_PARAM;

    memcpy(&channels, &newChannels.channels, sizeof(channels));

    setMode(currentPowerMode, currentLightMode);    // the current channel might have been changed, update mode

    return storeChannels(DEFAULT_SET, resultHandler);
}

ret_code_t brd_SetDeviceName(char const* pNewName, ds_reportHandler_t resultHandler)
{
    if (pNewName == NULL)
        return NRF_ERROR_NULL;

    if (strlen(pNewName) > DEVICENAME_LENGHT)
        return NRF_ERROR_INVALID_LENGTH;

    strcpy(config.deviceName, pNewName);

    memcpy(&newConfig.config, &config, sizeof(config));

    return storeConfig(resultHandler);
}

ret_code_t brd_FactoryReset(ds_reportHandler_t resultHandler)
{
    if (resultHandler == NULL)
        return NRF_ERROR_NULL;

    // set defaults setting
    channels_t chDefaults = CHANNEL_DEFAULTS;
    memcpy(&channels, &chDefaults, sizeof(channels_t));
    config_t cfgDefaults = CONFIG_DEFAULTS;
    memcpy(&config, &cfgDefaults, sizeof(config_t));

    // initiate deletion
    return ds_Reset(CHANNELS_FILE_ID, resultHandler);
}

ret_code_t KD2_GetChannelSetup(KD2_channelSetup_t* pSetup, uint8_t channel)
{
    if (pSetup == NULL)
        return NRF_ERROR_NULL;

    if (channel >= features.channelCount)
        return NRF_ERROR_INVALID_PARAM;

    KD2_channelSetup_t const* pFirstChannel =
        config.helenaBaseAddr ? &config.channelSetup[CHANNEL_HELENAFLOOD] : &config.channelSetup[CHANNEL_CURRENT];

    memcpy(pSetup, &pFirstChannel[channel], sizeof(KD2_channelSetup_t));

    return NRF_SUCCESS;
}

ret_code_t KD2_SetChannelSetup(KD2_channelSetup_t const* pSetup, uint8_t channel, ds_reportHandler_t resultHandler)
{
    if (pSetup == NULL)
        return NRF_ERROR_NULL;

    if (channel >= features.channelCount || !isChannelSetupValid(pSetup))
        return NRF_ERROR_INVALID_PARAM;

    KD2_channelSetup_t* pFirstChannel =
        config.helenaBaseAddr ? &config.channelSetup[CHANNEL_HELENAFLOOD] : &config.channelSetup[CHANNEL_CURRENT];

    memcpy(&pFirstChannel[channel], pSetup, sizeof(KD2_channelSetup_t));

    memcpy(&newConfig.config, &config, sizeof(config));

    return storeConfig(resultHandler);
}

ret_code_t KD2_GetCompensation(KD2_adcCompensation_t* pComp)
{
    if (pComp == NULL)
        return NRF_ERROR_NULL;

    memcpy(pComp, &config.comp, sizeof(KD2_adcCompensation_t));

    return NRF_SUCCESS;
}

ret_code_t KD2_SetCompensation(KD2_adcCompensation_t const* pComp, ds_reportHandler_t resultHandler)
{
    if (pComp == NULL)
        return NRF_ERROR_NULL;

    if (!isCompensationValid(pComp))
        return NRF_ERROR_INVALID_PARAM;

    if (applyCompensation(pComp) != NRF_SUCCESS)
    {
        (void)applyCompensation(&config.comp);
        return NRF_ERROR_INVALID_PARAM;
    }

    memcpy(&config.comp, pComp, sizeof(KD2_adcCompensation_t));

    memcpy(&newConfig.config, &config, sizeof(config));

    return storeConfig(resultHandler);
}

ret_code_t KD2_GetComPinMode(KD2_comPinMode_t* pMode)
{
    if (pMode == NULL)
        return NRF_ERROR_NULL;

    *pMode = config.comPinMode;

    return NRF_SUCCESS;
}

ret_code_t KD2_SetComPinMode(KD2_comPinMode_t mode, ds_reportHandler_t resultHandler)
{
    if (!isComPinModeValid(mode))
        return NRF_ERROR_INVALID_PARAM;

    config.comPinMode = mode;

    memcpy(&newConfig.config, &config, sizeof(config));

    return storeConfig(resultHandler);
}

ret_code_t KD2_GetHelenaCompensation(hbd_calibData_t* pComp)
{
    if (pComp == NULL)
        return NRF_ERROR_NULL;

    if (config.helenaBaseAddr == 0)
        return NRF_ERROR_NOT_FOUND;

    // helena error codes are mapped to sdk error codes, no conversion necessary
    return hbd_GetCalibrationData(&helenaInst, pComp);
}

ret_code_t KD2_SetHelenaCompensation(hbd_calibData_t* pComp)
{
    if (pComp == NULL)
        return NRF_ERROR_NULL;

    if (config.helenaBaseAddr == 0)
        return NRF_ERROR_NOT_FOUND;

    // helena error codes are mapped to sdk error codes, no conversion necessary
    return hbd_SetCalibrationData(&helenaInst, pComp);
}

/**END OF FILE*****************************************************************/
