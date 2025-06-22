/**
  ******************************************************************************
  * @file    Drake_10.c
  * @author  Thomas Reisnecker
  * @brief   board managment module for Drake Hardware 1.0
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
#include "board.h"
#include "mode_management.h"
#include "btle.h"
#include "debug.h"
#include "Drake_hmi.h"
#include "Drake_light.h"
#include "i2c.h"
#include "lis2dh12drv.h"
#include "vl53l0xdrv.h"
#include "drakedrv.h"
#include "Drake_adc.h"
#include "charge_control.h"
#include "battery_gauge.h"
#include "main.h"
#include "motor_control.h"
//#include "plunger_control_i2cm.h"
//#include "plunger_control_i2cs.h"
#include "Drake_btle.h"
#include "Drake_i2cs.h"
#include "Drake_10.h"
#include "nrf_pwr_mgmt.h"
#include "feature_tools.h"
#include "brake2d.h"
#include "app_timer.h"
#include "link_management.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrfx_gpiote.h"
#include "nrf_power.h"

/* External variables --------------------------------------------------------*/
extern uint32_t __start_cell_data;

/* Private defines -----------------------------------------------------------*/
#define BUTTON_LEFT             11
#define BUTTON_RIGHT            16
#define LED_TIMEOUT             APP_TIMER_TICKS(180000)
#define BAT_CAP_TIMEOUT_THRESH  ((int32_t)(0.5 * 3600l) << 16)

#define I2CM_SDA_PIN            13
#define I2CM_SCL_PIN            14
#define ACC_INT_PIN             15

#define I2CS_SDA_PIN            20
#define I2CS_SCL_PIN            21

#define MODE_BRAKE_BIT          0
#define MODE_BRAKE_MASK         (1 << MODE_BRAKE_BIT)
#define MODE_INDIC_BIT          1
#define MODE_INDIC_MASK         (1 << MODE_INDIC_BIT)

#define DEVICENAME_LENGHT       BLE_GAP_DEVNAME_DEFAULT_LEN

#define BAT_CAP_MAX             ((5 * 3600l) << 16)
#define BAT_CAP_MIN             ((int32_t)(0.15 * 3600l) << 16)
#define BAT_CAP_FULL_CHARGE     ((q15_16_t)(0.5 / 0.7 * 3600) << 16)

#define PL_TIMEOUT_MIN          APP_TIMER_TICKS(100)
#define PL_TIMEOUT_MAX          APP_TIMER_TICKS(4000)

#define SOC_TIMEOUT_MAX         APP_TIMER_TICKS(5000)

#define LOW_BATTERY             64  // 25%

//#define NOINIT_VALID            0xDA4EA72D

#define BRAKELIGHT_THRESH       ((q3_12_t)(0.3 * 4096))
#define BRAKEFLASH_THRESH       ((q3_12_t)(0.6 * 4096))

#define PATTERN_LEN             9

#define DEFAULT_SET             0           // the default config set, different sets not implemented yet, so 0
#define CHANNELS_VERSION        0           // channel version to identify changes for future extensions
#define SETUP_VERSION           0
#define DRAKE_FILE_ID           0x0A13      // the file id for this module
#define CHANNELS_RECORD_BASE    0x0101      // the base for channel configuration
#define SETUP_RECORD_BASE       0x0111      // the base for config (should be renamed to setup as in the app)
#define NAME_RECORD_BASE        0x0121      // the base for the device name

#define CHANNELS_DEFAULTS       \
{                              \
    { 64, 64, 192, 192, 0, 0, 0, 0}, \
    { 0, 0, 0, 0, 0, 0, 0, 0}, \
    { 0, 0, 0, 0, 0, 0, 0, 0}, \
}

#define CHANNELS_STR_DEFAULTS                \
{                                           \
    .numOfModesPerChannel = MM_NUM_OF_MODES,\
    .version      = CHANNELS_VERSION,       \
}

#define MOTOR_CONFIG_DEFAULT                \
{                                           \
    .openTimeout = APP_TIMER_TICKS(500),    \
    .closeTimeout = APP_TIMER_TICKS(500),   \
    .openDutyCycle = 256,                   \
    .closeDutyCycle = 128                   \
}

#define SETUP_DEFAULTS                                          \
{                                                               \
    .deviceName = "Drake",                                      \
    .usage = DRAKE_USAGE_DROPPER_ACTUATOR,                      \
    .battInfo = {DRAKE_BAT_TYPE_LCO, 1080l << 16, 1080l << 16}, \
    .tailPattern = {0xF1, 0x16, 0x0C, 0xC3, 0x75, 0x00},        \
    .frontPattern = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00},       \
    .brakePattern = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},       \
    .leftPattern = {0x00, 0x00, 0x00, 0xC0, 0x75, 0x00},        \
    .rightPattern = {0xF0, 0x16, 0x00, 0x00, 0x00, 0x00},       \
    .motorConfig = MOTOR_CONFIG_DEFAULT,                        \
}

#define SETUP_STR_DEFAULTS          \
{                                   \
    .numOfChannels = CHANNEL_CNT,   \
    .version       = SETUP_VERSION, \
}

#define STATE_DEFAULTS              \
{                                   \
    .powerMode = BRD_PM_OFF,        \
    .lightMode = MM_MODE_OFF        \
}

#define SIZE_OF_MODES           (sizeof(channels) + MM_NUM_OF_MODES * sizeof(mm_modeConfig_t))
#define SIZE_OF_QWR_BUFFER      (((SIZE_OF_MODES / 18) + 1) * 24)

/* Private typedef -----------------------------------------------------------*/
typedef enum
{
    CHANNEL_POS,
    CHANNEL_BRAKE,
    CHANNEL_INDIC,
    CHANNEL_CNT
} channel_t;

/**< the channel configurations storage format*/
typedef struct
{
    uint8_t  numOfModesPerChannel;
    uint8_t  version;
    uint16_t dummy;
    uint8_t  channels[CHANNEL_CNT][MM_NUM_OF_MODES];
} channels_str_t;

/**< device setup */
typedef struct
{
    char                        deviceName[DEVICENAME_LENGHT + 1];// the device name if changed
    drake_usage_t               usage;                     // device usage
    drake_batteryInfo_t         battInfo;                  // information about the battery
    uint32_t                    socTimeout;                // length of soc led indication or reftime for morse
    uint8_t                     tailPattern[PATTERN_LEN];  // light pattern for tail light (red)
    uint8_t                     frontPattern[PATTERN_LEN]; // light pattern for front light (white)
    uint8_t                     posPattern[PATTERN_LEN];   // light pattern for position light (amber)
    uint8_t                     brakePattern[PATTERN_LEN]; // light pattern for brake light (red)
    uint8_t                     leftPattern[PATTERN_LEN];  // light pattern for left indicator light (amber)
    uint8_t                     rightPattern[PATTERN_LEN]; // light pattern for right indicator light (amber)
    mtCtrl_motorConfig_t        motorConfig;               // motor configuration data (timeouts for opening and closing)
    mtCtrl_sensorConfig_t       sensorConfig;              // motor sensor calibration data
    drake_travelSensorData_t    travelSensor;              // travel sensor offset data
    drake_indexedTravelConfig_t indexedConfig;             // the indexed travel configuration
} setup_t;

/**< device setup storage format */
typedef struct
{
    uint8_t         numOfChannels;
    uint8_t         version;
    uint16_t        dummy;
    setup_t         setup;
} setup_str_t;

typedef struct
{
    brd_powerMode_t powerMode;
    bool            actuatorSlaveMode;
    bool            sysOff;
    uint8_t         lightMode;
    uint8_t         overrideConfig[CHANNEL_CNT];
} state_t;

typedef enum
{
    COUNTDOWN_BRAKE,
    COUNTDOWN_LEFT_IND,
    COUNTDOWN_RIGHT_IND,
    COUNTDOWN_CNT
} countdown_t;

typedef struct
{
    uint32_t isValid;
    bg_inst_t bgInst;
    uint16_t activationCounter;
} noinit_t;

/* function prototypes -------------------------------------------------------*/

/* Private read only variables -----------------------------------------------*/
/*static const cht_types_t channelTypes[] =
{
    CHT_TYPE_PWM,
    CHT_TYPE_PWM,
    CHT_TYPE_PWM
};*/

static const ble_hps_f_ch_size_t channelSizes[] =
{
    { 8, 0, BLE_HPS_CH_DESC_PWM },
    { 8, 0, BLE_HPS_CH_DESC_PWM },
    { 8, 0, BLE_HPS_CH_DESC_PWM }
};

static brd_features_t features =
{
    .maxNameLenght = DEVICENAME_LENGHT,
    .advType       = BTLE_ADV_TYPE_ALWAYS_OPEN, //BTLE_ADV_TYPE_AFTER_SEARCH,
    .channelCount  = 0,                                     // light channel is added if light is supported
    //.pChannelTypes = channelTypes,
    .pChannelSize  = channelSizes,
    .comSupported  = {.rx = 0xFFFFFFFF, .tx = 0xFFFFFFFF},  // no com supported
    .isIMUPresent  = false                                  // this filed is deprecated, only used for LCS
};

/* Private variables ---------------------------------------------------------*/
static uint8_t              channels[CHANNEL_CNT][MM_NUM_OF_MODES] = CHANNELS_DEFAULTS;
static channels_str_t       newChannels __ALIGN(4) = CHANNELS_STR_DEFAULTS;

static setup_t              setup = SETUP_DEFAULTS;
static setup_str_t          newSetup __ALIGN(4) = SETUP_STR_DEFAULTS;

static state_t              currentState = STATE_DEFAULTS;
static state_t              pendingState = STATE_DEFAULTS;

static ftools_inst_t        countdown[COUNTDOWN_CNT];

static uint8_t              modesBuffer[SIZE_OF_MODES];
static ble_hps_modes_init_t modes;
static uint8_t              qwrBuffer[SIZE_OF_QWR_BUFFER];

static Drake_adc_result_t   adcResults;
static bool                 oneHzFlag;      // will be set in the adc handler, sampling with 1Hz, only in IDLE mode

static noinit_t             noinit __attribute__((section(".noinit"))); // contains not initialized data which should survive reset (but not sysoff)
static chctrl_state_t       chargeState;
static q8_t                 batterySoc;
static q16_t                ledCurrent;
static q3_13_t              ledPower;
static q3_12_t              deceleration;
static q15_t                inclination;
static int16_t              travel;
static q8_t                 motorPosition;

static ds_reportHandler_t   pendingResultHandler;   // used to buffer resulthandler when starting calibration routines

/* Board information --------------------------------------------------------*/
static const btle_info_t bleInfo =
{
    .pDevicename = (char const*)&setup.deviceName,
    .pManufacturer = NULL,
    .pModelNumber = "Drake",
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
 *
 * @param[in] pChannels   the configuration to check
 * @param[in] numOfModes  the number of modes for each channel
 * @return true if valid, otherwise false
 */
static bool isChannelConfigValid(uint8_t const* pChannels, uint16_t numOfModes)
{
    for (uint8_t i = 0; i < CHANNEL_CNT; i++)
    {
        for (uint16_t j = 0; j < numOfModes; j++)
        {
            // for now there are no invalid settings possible
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

    errCode = ds_Read(DRAKE_FILE_ID, CHANNELS_RECORD_BASE + set, (void const**)&pChannels, &lengthWords);
    if (errCode == NRF_SUCCESS)
    {
        if (lengthWords == BYTES_TO_WORDS(sizeof(channels_str_t)) &&
            pChannels->numOfModesPerChannel == MM_NUM_OF_MODES &&
            isChannelConfigValid(&pChannels->channels[0][0], pChannels->numOfModesPerChannel))
            memcpy(&channels, &pChannels->channels, sizeof(channels));
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

    errCode = ds_Write(DRAKE_FILE_ID, CHANNELS_RECORD_BASE + set, pData, lenghtInWords, handler);

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

    errCode = fds_record_find(DRAKE_FILE_ID, CHANNELS_RECORD_BASE + set, &desc, &tok);
    if (errCode == NRF_SUCCESS)
        errCode = fds_record_delete(&desc);
    else if(errCode == FDS_ERR_NOT_FOUND)
        errCode = NRF_SUCCESS;

    return errCode;
}*/

static bool isDeviceNameValid(char const* pName)
{
    return strlen(pName) <= DEVICENAME_LENGHT;
}

static bool isDeviceUsageValid(drake_usage_t usage)
{
    return usage == DRAKE_USAGE_DROPPER_ACTUATOR ||
           usage == DRAKE_USAGE_TAILLIGHT_16LED  ||
           usage == DRAKE_USAGE_TAILLIGHT_44LED  ||
           usage == DRAKE_USAGE_BARLIGHTS_36LED;
}

static bool isBatteryInfoValid(drake_batteryInfo_t const* pBattInfo)
{
    if (pBattInfo->type != DRAKE_BAT_TYPE_LCO && pBattInfo->type != DRAKE_BAT_TYPE_NMC)
        return false;
    if (pBattInfo->capNom > BAT_CAP_MAX || pBattInfo->capNom < BAT_CAP_MIN)
        return false;
    // also check actual capacity? what if capacity exceeds limits due to aging?

    return true;
}

static bool isSocTimeoutValid(uint32_t timeout)
{
    return timeout <= SOC_TIMEOUT_MAX;
}

static bool isMotorConfigValid(mtCtrl_motorConfig_t const* pConfig)
{
    if (pConfig->openTimeout > PL_TIMEOUT_MAX || pConfig->openTimeout < PL_TIMEOUT_MIN)
        return false;
    if (pConfig->closeTimeout > PL_TIMEOUT_MAX || pConfig->closeTimeout < PL_TIMEOUT_MIN)
        return false;
    if (pConfig->openDutyCycle > 256 || pConfig->closeDutyCycle > 256)
        return false;

    return true;
}

static bool isMotorSensorConfigValid(mtCtrl_sensorConfig_t const* pData)
{
    // no checking for thresholds and open count necessary

    return true;
}

static bool isTravelSensorDataValid(drake_travelSensorData_t const* pData)
{
    return true;
}

static bool isIndexedTravelConfigValid(drake_indexedTravelConfig_t const* pConfig)
{
    return true;
}

/** @brief function to load setup from flash
 */
static ret_code_t loadSetup()
{
    ret_code_t         errCode;
    setup_str_t const* pSetup;
    uint16_t           lengthWords;

    errCode = ds_Read(DRAKE_FILE_ID, SETUP_RECORD_BASE, (void const**)&pSetup, &lengthWords);
    if (errCode == NRF_SUCCESS)
    {
        if (lengthWords == BYTES_TO_WORDS(sizeof(setup_str_t)))
        {
            if (isDeviceNameValid(pSetup->setup.deviceName))
                strcpy(setup.deviceName, pSetup->setup.deviceName);
            if (isDeviceUsageValid(pSetup->setup.usage))
                setup.usage = pSetup->setup.usage;
            if (isBatteryInfoValid(&pSetup->setup.battInfo))
                setup.battInfo = pSetup->setup.battInfo;
            if (isSocTimeoutValid(pSetup->setup.socTimeout))
                setup.socTimeout = pSetup->setup.socTimeout;

            //no validation necessary for pattern
            memcpy(setup.tailPattern, pSetup->setup.tailPattern, PATTERN_LEN);
            memcpy(setup.frontPattern, pSetup->setup.frontPattern, PATTERN_LEN);
            memcpy(setup.posPattern, pSetup->setup.posPattern, PATTERN_LEN);
            memcpy(setup.brakePattern, pSetup->setup.brakePattern, PATTERN_LEN);
            memcpy(setup.leftPattern, pSetup->setup.leftPattern, PATTERN_LEN);
            memcpy(setup.rightPattern, pSetup->setup.rightPattern, PATTERN_LEN);

            if (isMotorConfigValid(&pSetup->setup.motorConfig))
                setup.motorConfig = pSetup->setup.motorConfig;

            if (isMotorSensorConfigValid(&pSetup->setup.sensorConfig))
                setup.sensorConfig = pSetup->setup.sensorConfig;

            if (isTravelSensorDataValid(&pSetup->setup.travelSensor))
                setup.travelSensor = pSetup->setup.travelSensor;

            if (isIndexedTravelConfigValid(&pSetup->setup.indexedConfig))
                setup.indexedConfig = pSetup->setup.indexedConfig;
        }
    }
    else if (errCode != FDS_ERR_NOT_FOUND)
    {
        NRF_LOG_ERROR("[BRD]: record find error %d", errCode);
    }

    return errCode;
}

/** @brief function to store setup to flash
 * @param[in] handler the handler to be relayed to the write function
 */
static ret_code_t storeSetup(ds_reportHandler_t handler)
{
    ret_code_t errCode;
    void const* pData      = (void const*)&newSetup;
    uint16_t lenghtInWords = BYTES_TO_WORDS(sizeof(setup_str_t));

    errCode = ds_Write(DRAKE_FILE_ID, SETUP_RECORD_BASE, pData, lenghtInWords, handler);

    LOG_ERROR_CHECK("[BRD]: error %d storing setup", errCode);
    return errCode;
}

static bool isModeLightUsed(uint8_t const* pChannels)
{
    return pChannels != NULL && pChannels[CHANNEL_POS] != 0;
}

static bool isModeBrakeUsed(uint8_t const* pChannels)
{
    return pChannels != NULL && pChannels[CHANNEL_BRAKE] != 0;
}

/*static bool isModeIndicUsed(uint8_t const* pChannels)
{
    return pChannels != NULL && pChannels[CHANNEL_INIDC] != 0;
}*/

static void getChannelConfig(state_t const* pState, uint8_t* pConfig)
{
    uint16_t size = sizeof(uint8_t) * CHANNEL_CNT;
    memset(pConfig, 0, size);

    if (features.channelCount == 0)
        return;

    // if override is active use override config
    if (memcmp(pConfig, &pState->overrideConfig, size) != 0)
    {
        memcpy(pConfig, &pState->overrideConfig, size);
        return;
    }

    // otherwise use light mode config
    uint8_t lightMode = pState->lightMode;
    if (lightMode == MM_MODE_OFF)
        lightMode = mm_GetOffMode();
    if (lightMode != MM_MODE_OFF)
    {
        for (uint8_t i = 0; i < CHANNEL_CNT; i++)
        {
            pConfig[i] = channels[i][lightMode];
        }
    }
}

static void setAccelPowerMode(state_t const* pPendingState, uint8_t const* pConfig)
{
    ret_code_t errCode;
    lis2dh12drv_powerMode_t accelMode;

    if (setup.usage == DRAKE_USAGE_DROPPER_ACTUATOR)
    {
        if (pPendingState->actuatorSlaveMode || pPendingState->powerMode == BRD_PM_OFF)
            accelMode = LIS2DH12_PWR_OFF;
        else
            accelMode = LIS2DH12_PWR_MOTION;
    }
    else
    {
        if (pPendingState->powerMode == BRD_PM_OFF)
            accelMode = LIS2DH12_PWR_OFF;
        else if (isModeBrakeUsed(pConfig))
            accelMode = LIS2DH12_PWR_DATA;
        else
            accelMode = LIS2DH12_PWR_MOTION;
    }

    errCode = lis2dh12drv_SetPowerMode(accelMode);  // will return success if already in requested mode
    if (errCode != NRF_SUCCESS)
        NRF_LOG_ERROR("error %d setting accel power mode", errCode);
}

static void setAdcMode(state_t const* pPendingState)
{
    ret_code_t errCode;
    bool enable;

    // adc is not needed in plunger slave mode
    if (pPendingState->actuatorSlaveMode)
        enable = false;
    // otherwise it is needed for either detecting battery relaxing or usb power detection
    else
        enable = true;

    errCode = Drake_Adc_SetMode(enable ? DRAKE_ADC_1Hz : DRAKE_ADC_OFF);
    LOG_ERROR_CHECK("error %d setting adc mode", errCode);
}

static void setChargeControl(state_t const* pPendingState)
{
    chctrl_Enable(pPendingState->powerMode >= BRD_PM_STANDBY && !pPendingState->actuatorSlaveMode);
}

static void setPlungerSlaveMode(bool oldMode, state_t const* pPendingState)
{
    if (oldMode == pendingState.actuatorSlaveMode)
        return;

    // switching to plunger slave mode
    if (pendingState.actuatorSlaveMode)
    {
        NRF_LOG_INFO("switching to plunger slave mode");

        ret_code_t errCode = lm_SetExposureMode(LM_EXP_OFF);
        LOG_ERROR_CHECK("error %d disabling bluetooth", errCode);
    }
    // leaving plunger slave mode
    else
    {
        NRF_LOG_INFO("leaving plunger slave mode");

        ret_code_t errCode = main_ResetIdleTimer();
        LOG_ERROR_CHECK("error %d resetting idle Timer", errCode);

        // enabling only necessary if already in idle, otherwise power mode change will trigger
        if (pendingState.powerMode == BRD_PM_IDLE)
        {
            errCode = lm_SetExposureMode(LM_EXP_LOW_LATENCY);
            LOG_ERROR_CHECK("error %d re-enabling bluetooth", errCode);
        }
    }
}

/** @brief function to update the state
 */
static void updateState()
{
    ret_code_t errCode;
    uint8_t oldLightModeConfig[CHANNEL_CNT], newLightModeConfig[CHANNEL_CNT];
    getChannelConfig(&currentState, oldLightModeConfig);
    getChannelConfig(&pendingState, newLightModeConfig);

    // update accel sensor
    setAccelPowerMode(&pendingState, features.channelCount ? newLightModeConfig : NULL);

    // update adc
    setAdcMode(&pendingState);

    // update charger
    setChargeControl(&pendingState);

    // checking plunger slave mode
    setPlungerSlaveMode(currentState.actuatorSlaveMode, &pendingState);

    // in plunger slave mode a power mode change will re-enable bluetooth, so disable again
    if (pendingState.actuatorSlaveMode && pendingState.powerMode != currentState.powerMode)
    {
        errCode = lm_SetExposureMode(LM_EXP_OFF);
        LOG_ERROR_CHECK("error %d disabling bluetooth", errCode);
    }

    if (pendingState.sysOff && !currentState.sysOff)
    {
        if (!pendingState.actuatorSlaveMode)
        {
            // send shutdown request to remote actuator
            errCode = drakedrv_ReqSysOff();
            if (errCode != NRF_SUCCESS && errCode != NRF_ERROR_NOT_FOUND)
                NRF_LOG_ERROR("error %d sending Sysoff Request to i2c slave", errCode);
        }
        else // activate sense on scl pin to wake up again
            nrf_gpio_cfg_sense_set(I2CS_SCL_PIN, NRF_GPIO_PIN_SENSE_LOW);
        nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
    }

    currentState = pendingState;
}

static bool goToSysOff()
{
    // in plunger slave mode sysOff is only requested from master side
    if (currentState.actuatorSlaveMode || pendingState.actuatorSlaveMode)
        return false;

    // sysoff only possible in standby or off mode
    if (currentState.powerMode == BRD_PM_IDLE || pendingState.powerMode == BRD_PM_IDLE)
        return false;

    // sysoff not available if battery is used or relaxing
    if (noinit.bgInst.loadTimeout || noinit.bgInst.colCnt)
        return false;

    // and no sysoff as long as USB is connected
    if (chargeState != CHCTRL_STATE_DISCONNECTED)
        return false;

    return true;
}

/** @brief function to prepare and send a helen project measurement notification
 */
static void sendHpsMessage()
{
    if (currentState.powerMode != BRD_PM_IDLE)
        return;

    ret_code_t errCode;
    btle_hpsMeasurement_t message;

    message.mode = mm_GetCurrentMode();
    message.inputVoltage = (adcResults.inputVoltage * 1000l) >> 13;
    message.temperature = ((int32_t)adcResults.temperature - (273 << 7)) >> 7;
    message.outputPower = (ledPower * 1000l) >> 13;

    errCode = btle_ReportHpsMeasurements(&message);
    if (errCode != NRF_SUCCESS &&
        errCode != NRF_ERROR_INVALID_STATE && // ignore error if notifications are not enabled
        errCode != NRF_ERROR_RESOURCES)       // ignore error if message buffer is full
    {
        NRF_LOG_ERROR("[BRD]: error %d sending message", errCode);
    }
}

static void feedChargeControl()
{
    chctrl_state_t newChargeState = chctrl_Feed(adcResults.cc1Voltage, adcResults.cc2Voltage, adcResults.chargeCurrent, adcResults.inputVoltage);

    if (newChargeState != chargeState)
    {
        NRF_LOG_INFO("new charge state %d", newChargeState);

        chargeState = newChargeState;
    }
}

static void feedBatteryGauge()
{
    ret_code_t errCode;
    int32_t batteryCurrent;

    batteryCurrent = adcResults.chargeCurrent - ledCurrent;
    batterySoc = bg_Feed(&noinit.bgInst, batteryCurrent, adcResults.inputVoltage);
    errCode = Drake_btle_BatteryLevelUpdate(batterySoc);
    if (errCode != NRF_SUCCESS && errCode != NRF_ERROR_INVALID_STATE)
        NRF_LOG_ERROR("battery level update error %d", errCode);
}

static void generateLightData(uint8_t const* pLightModeConfig, Drake_light_t* pLight)
{
    q6_10_t dummyCD;

    pLight[DRAKE_LIGHT_FRONT].intensity = pLightModeConfig[CHANNEL_POS];
    pLight[DRAKE_LIGHT_FRONT].activeClear = false;
    pLight[DRAKE_LIGHT_FRONT].pattern.pPattern = setup.frontPattern;
    pLight[DRAKE_LIGHT_FRONT].pattern.len = PATTERN_LEN;

    pLight[DRAKE_LIGHT_REAR].intensity = pLightModeConfig[CHANNEL_POS];
    pLight[DRAKE_LIGHT_REAR].activeClear = false;
    pLight[DRAKE_LIGHT_REAR].pattern.pPattern = setup.tailPattern;
    pLight[DRAKE_LIGHT_REAR].pattern.len = PATTERN_LEN;

    pLight[DRAKE_LIGHT_POS].intensity = pLightModeConfig[CHANNEL_POS];
    pLight[DRAKE_LIGHT_POS].activeClear = false;
    pLight[DRAKE_LIGHT_POS].pattern.pPattern = setup.posPattern;
    pLight[DRAKE_LIGHT_POS].pattern.len = PATTERN_LEN;

    pLight[DRAKE_LIGHT_BRAKE].intensity = ftools_IsOn(&countdown[COUNTDOWN_BRAKE]) ? pLightModeConfig[CHANNEL_BRAKE] : 0;
    pLight[DRAKE_LIGHT_BRAKE].activeClear = false;
    pLight[DRAKE_LIGHT_BRAKE].pattern.pPattern = setup.brakePattern;
    pLight[DRAKE_LIGHT_BRAKE].pattern.len = PATTERN_LEN;

    pLight[DRAKE_LIGHT_INDIC_LEFT].intensity = ftools_IsOn(&countdown[COUNTDOWN_LEFT_IND]) ? pLightModeConfig[CHANNEL_INDIC] : 0;
    pLight[DRAKE_LIGHT_INDIC_LEFT].activeClear = ftools_GetCountdown(&countdown[COUNTDOWN_LEFT_IND], &dummyCD) == NRF_SUCCESS;
    pLight[DRAKE_LIGHT_INDIC_LEFT].pattern.pPattern = setup.leftPattern;
    pLight[DRAKE_LIGHT_INDIC_LEFT].pattern.len = PATTERN_LEN;

    pLight[DRAKE_LIGHT_INDIC_RIGHT].intensity = ftools_IsOn(&countdown[COUNTDOWN_RIGHT_IND]) ? pLightModeConfig[CHANNEL_INDIC] : 0;
    pLight[DRAKE_LIGHT_INDIC_RIGHT].activeClear = ftools_GetCountdown(&countdown[COUNTDOWN_RIGHT_IND], &dummyCD) == NRF_SUCCESS;
    pLight[DRAKE_LIGHT_INDIC_RIGHT].pattern.pPattern = setup.rightPattern;
    pLight[DRAKE_LIGHT_INDIC_RIGHT].pattern.len = PATTERN_LEN;
}

static void limitLightData(Drake_light_t* pLight)
{
    if (batterySoc)
        return;

    for (Drake_lightType_t i = 0; i < DRAKE_LIGHT_CNT; i++)
    {
        pLight[i].intensity = 0;
    }
}

/** @brief function to update channels
 */
static void updateTarget()
{
    ret_code_t    errCode;
    uint8_t       lightModeConfig[CHANNEL_CNT];
    Drake_light_t lights[DRAKE_LIGHT_CNT];
    q2_14_t       current;

    if (features.channelCount == 0)
        return;

    // get channel config first
    getChannelConfig(&currentState, lightModeConfig);

    // send targets
    generateLightData(lightModeConfig, lights);
    limitLightData(lights);
    errCode = Drake_Light_Update(lights, setup.usage, &current, &ledPower);
    LOG_ERROR_CHECK("Drake_light_Update error %d", errCode);
    ledCurrent = current << 2;
}

/** @brief function to set the buffer which is used for the Helen Modes Characteristic
 *
 * @param[out] pModes         the ble service configuration
 * @param[in]  channelCount   number of active channels
 * @param[in]  pFirstChannel  pointer to beginning of the channel configuration
 * @return void
 *
 */
static void setModes(ble_hps_modes_init_t* pModes, uint16_t channelCount, uint8_t const* pChannels)
{
    mm_modeConfig_t const* pModesConfig;
    uint16_t sizeModes = MM_NUM_OF_MODES * sizeof(mm_modeConfig_t);
    uint16_t sizeChannels = channelCount * MM_NUM_OF_MODES * sizeof(uint8_t);

    (void)mm_GetModeConfigs(&pModesConfig); // cannot fail unless fed with null-pointer
    memcpy(modesBuffer, pModesConfig, sizeModes);
    memcpy(&modesBuffer[sizeModes], pChannels, sizeChannels);

    pModes->p_mode_config = (ble_hps_mode_config_t*)modesBuffer;
    pModes->total_size = sizeModes + sizeChannels;
}

static void accelHandler(lis2dh12drv_event_t const* pEvt)
{
    if (pEvt->type == LIS2DH12_EVT_MOTION)
    {
        NRF_LOG_INFO("motion detected");

        main_ResetIdleTimer();
    }
    else if (pEvt->type == LIS2DH12_EVT_DATA)
    {
        ret_code_t errCode;
        q6_10_t cntdown;
        q3_12_t accel[2] = {pEvt->data.accel[0], pEvt->data.accel[2]};

        errCode = brake2d_Feed(accel, &deceleration, &inclination);
        LOG_ERROR_CHECK("brake2d_feed() error %d", errCode);

        if (deceleration > BRAKEFLASH_THRESH)
            cntdown = FTOOLS_BRAKEFLASH;
        else if (deceleration > BRAKELIGHT_THRESH)
            cntdown = FTOOLS_BRAKELIGHT;
        else
            return;

        errCode = ftools_UpdateCountdown(&countdown[COUNTDOWN_BRAKE], cntdown);
        LOG_ERROR_CHECK("ftools_UpdateCountdown() error %d", errCode);

        //SEGGER_RTT_printf(0, "%d, %d, %d\r\n", pEvt->data.accel[0], pEvt->data.accel[1], pEvt->data.accel[2]);
        //NRF_LOG_INFO("%d, %d, %d", pEvt->data.accel[0], pEvt->data.accel[1], pEvt->data.accel[2]);
    }
}

static ret_code_t initAccel()
{
    static lis2dh12drv_init_t const accelInit =
    {
        .addr =    LIS2DH12_ADDR1,
        .int1Pin = ACC_INT_PIN,
        .handler = accelHandler
    };

    return lis2dh12drv_Init(&accelInit);
}

static void adcResultHandler(Drake_adc_result_t const* pResult)
{
    adcResults = *pResult;

    // when usb is connected and battery is full (charging disabled)
    // charge current pin is not floating and therefore value is incorrect
    if (chargeState == CHCTRL_STATE_FULL)
        adcResults.chargeCurrent = 0;

    // charge current can only be measured up to 200mA, if 500mA charging is
    // selected, current needs to be manually adjusted to 500mA.
    else if (setup.battInfo.capNom >= BAT_CAP_FULL_CHARGE)
    {
        // if current is higher than 180mA, we assume that charging is still
        // in CC mode. This will produce minor error if the real charging
        // current has already dropped between 200 and 500mA but we have to
        // live with that
        if (adcResults.chargeCurrent > ((180l << 16) / 1000))
            adcResults.chargeCurrent = ((500l << 16) / 1000);
    }

    oneHzFlag = true;

    if (chargeState != CHCTRL_STATE_DISCONNECTED)
        NRF_LOG_INFO("charge current (mA) %d", (adcResults.chargeCurrent * 1000l) / 65536);
}

void calibStoreResultHandler(ret_code_t errCode)
{
    // relay result to pending handler
    if (pendingResultHandler != NULL)
    {
        pendingResultHandler(errCode);
        pendingResultHandler = NULL;
    }

    NRF_LOG_INFO("calibration data stored result %d", errCode);
}

static void motorControlHandler(mtCtrl_event_t const* pEvt)
{
    ret_code_t errCode;

    switch (pEvt->type)
    {
    case MTCTRL_EVT_TYPE_POWER_FAIL:
        return;

    case MTCTRL_EVT_TYPE_NEW_POSITION:
        motorPosition = pEvt->position;
        if (pEvt->position == 0)                // travel sensor can be disabled
        {
            errCode = vl53l0xdrv_SetPowerMode(VL53L0X_PWR_OFF);
            if (errCode != NRF_SUCCESS && errCode != NRF_ERROR_INVALID_STATE)
                NRF_LOG_ERROR("error %d disabling tof sensor", errCode);
        }
        Drake_I2cs_UpdateMotPos(motorPosition); // update i2cs value
        /// TODO: update ble service            // update ble service value
        return;

    case MTCTRL_EVT_TYPE_CALIB_SENSOR:
        setup.sensorConfig.compThresholds = *pEvt->pThresholds;
        NRF_LOG_INFO("new thresholds received: %d and %d", pEvt->pThresholds->upper, pEvt->pThresholds->lower);
        break;

    case MTCTRL_EVT_TYPE_CALIB_FULL_OPEN:
        setup.sensorConfig.fullOpenCount = pEvt->fullyOpenCount;
        NRF_LOG_INFO("new sensor position calib data received %d", pEvt->fullyOpenCount);
        break;
    }

    memcpy(&newSetup.setup, &setup, sizeof(setup));
    errCode = storeSetup(calibStoreResultHandler);
    if (errCode != NRF_SUCCESS && pendingResultHandler != NULL)
    {
        pendingResultHandler(errCode);
        pendingResultHandler = NULL;
    }

    NRF_LOG_INFO("plunger calibration data received");
}

static ret_code_t motorControlInit(bool* pIsMotSensPresent)
{
    mtCtrl_init_t mclInit =
    {
        .pMotorConfig = &setup.motorConfig,
        .pSensorConfig = &setup.sensorConfig,
        .eventHandler = motorControlHandler
    };

    return mtCtrl_Init(&mclInit, pIsMotSensPresent);
}

static void travelHandler(uint16_t rawTravel)
{
    NRF_LOG_INFO("raw travel: %d", rawTravel);

    if (setup.travelSensor.travelInmm == 0)
        return; // not calibrated yet

    travel = rawTravel - setup.travelSensor.travelInmm;

    Drake_I2cs_UpdateTravel(UINT8_MAX * travel / setup.travelSensor.travelInmm);
}

ret_code_t initTravelSensor(bool* isPresent)
{
    vl53l0xdrv_init_t trSensInit =
    {
        .handler = travelHandler,
        .offsetInMicrom = setup.travelSensor.offsetInMikrom
    };

    ret_code_t errCode = vl53l0xdrv_Init(&trSensInit);
    if (errCode == NRF_ERROR_NOT_FOUND)
    {
        *isPresent = false;
        errCode = NRF_SUCCESS;
    }
    else if (errCode == NRF_SUCCESS)
        *isPresent = true;

    return errCode;
}

static ret_code_t setMotorConfiguration(mtCtrl_motorConfig_t const* pMotorConfig, ds_reportHandler_t resultHandler)
{
    setup.motorConfig = *pMotorConfig;
    memcpy(&newSetup.setup, &setup, sizeof(setup));
    mtCtrl_UpdateMotorConfig(&setup.motorConfig);
    return storeSetup(resultHandler);
}

static ret_code_t clearMotorCalibrationData(drake_motorSensCalibType_t type, ds_reportHandler_t resultHandler)
{
    switch (type)
        {
        case DRAKE_MOTOR_SENS_CALIB_THRESHOLD:
            setup.sensorConfig.compThresholds.upper = 0;
            setup.sensorConfig.compThresholds.lower = 0;
            break;

        case DRAKE_MOTOR_SENS_CALIN_OPEN_CNT:
            setup.sensorConfig.fullOpenCount = 0;
            break;

        default:
            return NRF_ERROR_INVALID_PARAM;
        }

        /// TODO: local motor control needs to be updated
        memcpy(&newSetup.setup, &setup, sizeof(setup));
        return storeSetup(resultHandler);
}

static ret_code_t startMotorCalibration(drake_motorSensCalibType_t type, ds_reportHandler_t resultHandler)
{
    switch (type)
    {
    case DRAKE_MOTOR_SENS_CALIB_THRESHOLD:
    {
        mtSens_thresholds_t thresholds;
        ret_code_t errCode = mtCtrl_CalibrateSensor(&thresholds);
        if (errCode != NRF_SUCCESS)
            return errCode;

        setup.sensorConfig.compThresholds = thresholds;
        memcpy(&newSetup.setup, &setup, sizeof(setup));
        return storeSetup(resultHandler);
    }

    case DRAKE_MOTOR_SENS_CALIN_OPEN_CNT:
    {
        ret_code_t errCode = mtCtrl_CalibrateFullOpen();
        if (errCode == NRF_SUCCESS)
            pendingResultHandler = resultHandler;   // new values are assigned in event header

        return errCode;
    }

    default:
        return NRF_ERROR_INVALID_PARAM;
    }
}

static ret_code_t clearTravelSensorData(ds_reportHandler_t resultHandler)
{
    ret_code_t errCode = cl53l0xdrv_ClearOffsetCalibration();
    if (errCode != NRF_SUCCESS)
        return errCode;

    setup.travelSensor.travelInmm = 0;
    setup.travelSensor.offsetInMikrom = 0;

    memcpy(&newSetup.setup, &setup, sizeof(setup));

    return storeSetup(resultHandler);
}

static ret_code_t startTravelSensorCalibration(uint16_t travelInmm, ds_reportHandler_t resultHandler)
{
    drake_travelSensorData_t newData;
    newData.travelInmm = travelInmm;
    ret_code_t errCode = vl53l0xdrv_CalibrateOffset(travelInmm, &newData.offsetInMikrom);
    if (errCode != NRF_SUCCESS)
        return errCode;

    setup.travelSensor = newData;
    memcpy(&newSetup.setup, &setup, sizeof(setup));
    return storeSetup(resultHandler);
}

static ret_code_t setIndexedTravelConfig(drake_indexedTravelConfig_t const* pConfig, ds_reportHandler_t resultHandler)
{
    setup.indexedConfig = *pConfig;
    memcpy(&newSetup.setup, &setup, sizeof(setup));
    return storeSetup(resultHandler);
}

void i2csHandler(Drake_I2cs_event_t const* pEvt)
{
    ret_code_t errCode = NRF_SUCCESS;

    pendingState.actuatorSlaveMode = true;

    switch (pEvt->type)
    {
    case DRAKE_I2CS_SYS_OFF_REQ:
        pendingState.sysOff = true;
        break;

    case DRAKE_I2CS_MOT_POS_RCVD:
        errCode = mtCtrl_SetPosition(pEvt->motorPosition);
        Drake_I2cs_UpdateMotPos(pEvt->motorPosition);
        break;

    case DRAKE_I2CS_MOT_CFG_RCVD:
    {
        mtCtrl_motorConfig_t cfg;
        cfg.openDutyCycle = pEvt->pMotorCfg->dutyCycleOpen == UINT8_MAX ? 256 : pEvt->pMotorCfg->dutyCycleOpen;
        cfg.closeDutyCycle = pEvt->pMotorCfg->dutyCycleClose == UINT8_MAX ? 256 : pEvt->pMotorCfg->dutyCycleOpen;
        cfg.openTimeout = pEvt->pMotorCfg->timeoutOpen << 3;
        cfg.closeTimeout = pEvt->pMotorCfg->timeoutClose << 3;
        if (isMotorConfigValid(&cfg))
            errCode = setMotorConfiguration(&cfg, NULL);
    }   break;

    case DRAKE_I2CS_MOT_CAL_THR_CLEAR:
        errCode = clearMotorCalibrationData(DRAKE_MOTOR_SENS_CALIB_THRESHOLD, NULL);
        break;

    case DRAKE_I2CS_MOT_CAL_THR_START:
        errCode = startMotorCalibration(DRAKE_MOTOR_SENS_CALIB_THRESHOLD, NULL);
        break;

    case DRAKE_I2CS_MOT_CAL_FO_CLEAR:
        errCode = clearMotorCalibrationData(DRAKE_MOTOR_SENS_CALIN_OPEN_CNT, NULL);
        break;

    case DRAKE_I2CS_MOT_CAL_FO_START:
        errCode = startMotorCalibration(DRAKE_MOTOR_SENS_CALIN_OPEN_CNT, NULL);
        break;

    case DRAKE_I2CS_TRV_CAL_OFF_CLEAR:
        errCode = clearTravelSensorData(NULL);
        break;

    case DRAKE_I2CS_TRV_CAL_OFF_START:
        errCode = startTravelSensorCalibration(pEvt->travel, NULL);
        break;

    case DRAKE_I2CS_IND_CFG_RCVD:
        if (setup.travelSensor.travelInmm)
        {
            drake_indexedTravelConfig_t cfg;
            cfg.blockingTime = pEvt->pIndexedCfg->openBlocking << 8;
            cfg.timeout = pEvt->pIndexedCfg->closeTimeout << 8;
            for (uint8_t i = 0; i < ARRAY_SIZE(cfg.positions); i++)
            {
                if (i < ARRAY_SIZE(pEvt->pIndexedCfg->positions))
                    cfg.positions[i] = (int32_t)pEvt->pIndexedCfg->positions[i] * -setup.travelSensor.travelInmm / UINT8_MAX;
                else
                    cfg.positions[i] = 0;
            }
            errCode = setIndexedTravelConfig(&cfg, NULL);
        }
        break;

    default:
        break;
    }

    if (errCode != NRF_SUCCESS)
        NRF_LOG_ERROR("error %d on reaction to i2cs event", errCode);
}

ret_code_t initI2cs(bool isMtSensPresent, bool isTrvSensPresent, q8_t mtPos, int16_t travel)
{
    Drake_I2cs_motorConfig_t mtCfg =
    {
        .dutyCycleOpen = setup.motorConfig.openDutyCycle,
        .dutyCycleClose = setup.motorConfig.closeDutyCycle,
        .timeoutOpen = setup.motorConfig.openTimeout,
        .timeoutClose = setup.motorConfig.closeTimeout
    };

    Drake_I2cs_sensorConfig_t sensCfg =
    {
        .threshLow = setup.sensorConfig.compThresholds.lower,
        .threshUp = setup.sensorConfig.compThresholds.upper,
        .fullOpenCnt = setup.sensorConfig.fullOpenCount
    };

    Drake_I2cs_travelSensor_t trvSens =
    {
        .travel = setup.travelSensor.travelInmm,
        .offset = setup.travelSensor.offsetInMikrom & 0xFFFF
    };

    Drake_I2cs_indexedConfig_t indCfg =
    {
        .openBlocking = setup.indexedConfig.blockingTime >> 8,
        .closeTimeout = setup.indexedConfig.timeout >> 8
    };
    for (uint8_t i = 0; i < DRK_NUM_OF_IND_POS; i++)
    {
        if (i >= ARRAY_SIZE(setup.indexedConfig.positions) || setup.travelSensor.travelInmm == 0)
            indCfg.positions[i] = 0;
        else
            indCfg.positions[i] = UINT8_MAX * abs(setup.indexedConfig.positions[i]) / setup.travelSensor.travelInmm;
    }

    Drake_I2cs_init_t init =
    {
        .handler = i2csHandler,
        .isMotorSensorPresent = isMtSensPresent,
        .isTravelSensorPresent = isTrvSensPresent,
        .motorPosition = mtPos,
        .travel = setup.travelSensor.travelInmm == 0 ? 0 : UINT8_MAX * abs(travel) / setup.travelSensor.travelInmm,
        .pMotorCfg = &mtCfg,
        .pSensorCfg = &sensCfg,
        .pTravelSens = &trvSens,
        .pIndexedCfg = &indCfg
    };

    return Drake_I2cs_Init(&init);
}

static void setFeatures(brd_features_t* pFtr, bool isLightPresent)
{
    pFtr->channelCount = isLightPresent ? CHANNEL_CNT : 0;
}

static void batteryGaugeHandler(bg_evt_t const* pEvt)
{
    ret_code_t errCode;

    switch (pEvt->type)
    {
    case BG_EVT_TYPE_CAP_RECALC:
        NRF_LOG_INFO("battery capacity updated: %d", pEvt->capActual);
        setup.battInfo.capAct = pEvt->capActual;
        memcpy(&newSetup.setup, &setup, sizeof(setup));
        errCode = storeSetup(NULL);
        LOG_ERROR_CHECK("error %d storing new battery capacity", errCode);
        break;

    default:
        break;
    }
}

static bool isNoInitValid()
{
    return noinit.isValid == (uint32_t)&(__start_cell_data);
}

static uint16_t const* getOcvLut(drake_batteryType_t type)
{
    switch (type)
    {
    case DRAKE_BAT_TYPE_LCO:
        return bg_ocvLco;
    case DRAKE_BAT_TYPE_NMC:
        return bg_ocvNmc;
    default:
        return NULL;
    }
}

static ret_code_t initBatteryGauge()
{
    if (isNoInitValid())
        return NRF_SUCCESS;

    bg_init_t bgInit =
    {
        .pOCV       = getOcvLut(setup.battInfo.type),
        .capNominal = setup.battInfo.capNom,
        .capActual  = setup.battInfo.capAct,
        .handler    = batteryGaugeHandler,
    };

    return bg_Init(&noinit.bgInst, &bgInit);
}

static ret_code_t initFTools()
{
    countdown[COUNTDOWN_BRAKE].type = FTOOL_BRAKE_INDICATOR;
    countdown[COUNTDOWN_LEFT_IND].type = FTOOL_DIR_INDICATOR;
    countdown[COUNTDOWN_RIGHT_IND].type = FTOOL_DIR_INDICATOR;

    return ftools_Init(countdown, sizeof(countdown));
}

/*static void plctrli2csEvtHandler(plctrli2cs_evtType_t evtType)
{
    switch (evtType)
    {
    case PCI2CS_EVT_TYPE_MASTER_PRESENT:
        pendingState.actuatorSlaveMode = true;
        break;

    case PCI2CS_EVT_TYPE_SYSOFF_REQ:
        pendingState.sysOff = true;
        break;
    }
}*/

static bool hasStateChanged()
{
    return memcmp(&currentState, &pendingState, sizeof(currentState));
}

static bool hasTargetChanged()
{
    // if any countdown is active, light targets might have changed
    static uint32_t lastCheck;
    static bool wasActive;
    uint32_t timestamp = app_timer_cnt_get();
    bool isActive = false;

    if (app_timer_cnt_diff_compute(timestamp, lastCheck) < APP_TIMER_TICKS(10))
        return false;

    lastCheck = timestamp;

    for (uint8_t i = 0; i < ARRAY_SIZE(countdown); i++)
    {
        if (countdown[i].active)
        {
            isActive = true;
            break;
        }
    }

    bool changed = isActive || wasActive;

    wasActive = isActive;

    return changed;
}

/* Public functions ----------------------------------------------------------*/
ret_code_t brd_Init(brd_info_t const* *pInfo)
{
    ret_code_t       errCode;
    if (pInfo == NULL)
        return NRF_ERROR_NULL;

    *pInfo = &brdInfo;
    bool isActuatorPresent = false;
    bool isMotSensPresent = false;
    bool isLightPresent = false;
    bool isBrightnessPresent = true;    /// TODO: change when implemented
    bool isTravelSensorPresent = false;

    nrf_power_rampower_mask_on(7, NRF_POWER_RAMPOWER_S1RETENTION_MASK);

    errCode = loadSetup();
    if (errCode != NRF_SUCCESS && errCode != FDS_ERR_NOT_FOUND)
        return errCode;

    errCode = loadChannels(DEFAULT_SET);
    if (errCode != NRF_SUCCESS && errCode != FDS_ERR_NOT_FOUND)
        return errCode;

    errCode = i2c_Init(I2CM_SDA_PIN, I2CM_SCL_PIN, NRF_TWIM_FREQ_400K);
    if (errCode != NRF_SUCCESS)
        return errCode;

    errCode = initAccel();  // initialize accel first, so that a remote board can wake up until remote plunger control is initialized
    if (errCode != NRF_SUCCESS)
        return errCode;

    if (setup.usage == DRAKE_USAGE_DROPPER_ACTUATOR)
    {
        isActuatorPresent = true;

        //errCode = plctrli2cs_Init(plctrli2csEvtHandler);
        //if (errCode != NRF_SUCCESS && errCode != NRF_ERROR_NOT_FOUND)
        //    return errCode;

        errCode = motorControlInit(&isMotSensPresent);
        if (errCode != NRF_SUCCESS)
            return errCode;

        errCode = initTravelSensor(&isTravelSensorPresent);
        if (errCode != NRF_SUCCESS)
            return errCode;

        errCode = initI2cs(isMotSensPresent, isTravelSensorPresent, mtCtrl_GetCurrentPosition(), 0);   /// TODO: travel sensor value
        if (errCode != NRF_SUCCESS  && errCode != NRF_ERROR_NOT_FOUND)
            return errCode;
    }
    else // if Taillight or barlight
    {
        // brake detection and remote plunger control is only useful for taillight
        if (setup.usage == DRAKE_USAGE_TAILLIGHT_16LED || setup.usage == DRAKE_USAGE_TAILLIGHT_44LED)
        {
            /*errCode = brightnessInit()            /// TODO
            if (errCode == NRF_SUCCESS)
                isBrightnessPresent = true;
            else if (errCode != NRF_ERROR_NOT_FOUND)
                return errCode;*/

            errCode = brake2d_Init();
            if (errCode != NRF_SUCCESS)
                return errCode;

            // if a remote plunger is connected, it just woke up (when accel is initilized)
            // and needs about 8ms until the i2c slave interface is initialized. (currently a
            // 5ms delay is used to detect if a remote bus is present. When this is reduced
            // the following delay can be reduced, too.
            /// TODO: times have changed with travel sensor, remeasure!
            nrf_delay_ms(25);   // not working with smaller delay, why?

            errCode = drakedrv_Init(&isMotSensPresent, &isTravelSensorPresent);
            if (errCode == NRF_SUCCESS)
                isActuatorPresent = true;
            else if (errCode != NRF_ERROR_NOT_FOUND)
                return errCode;
        }

        errCode = Drake_Light_Init(!isBrightnessPresent);
        if (errCode == NRF_SUCCESS)
            isLightPresent = true;
        else if (errCode != NRF_ERROR_NOT_FOUND)
            return errCode;
    }

    // with small batteries hmi leds are shut off after start to reduce current consumption
    uint32_t timeout = setup.battInfo.capNom >= BAT_CAP_TIMEOUT_THRESH ? 0 : LED_TIMEOUT;
    errCode = Drake_Hmi_Init(BUTTON_LEFT, BUTTON_RIGHT, timeout);
    if (errCode != NRF_SUCCESS)
        return errCode;

    // charge current depends on battery size
    chctrl_current_t maxChargeCurrent = setup.battInfo.capNom >= BAT_CAP_FULL_CHARGE ? CHCTRL_CURRENT_500MA : CHCTRL_CURRENT_200MA;
    errCode = chctrl_Init(maxChargeCurrent);
    if (errCode != NRF_SUCCESS)
        return errCode;

    Drake_adc_init_t adcInit = {.resultHandler = adcResultHandler}; /// TODO: use reflex coupler as brightness sensor?
    errCode = Drake_Adc_Init(&adcInit, &adcResults);
    if (errCode != NRF_SUCCESS)
        return errCode;

    errCode = initBatteryGauge();
    if (errCode != NRF_SUCCESS)
        return errCode;
    batterySoc = bg_Feed(&noinit.bgInst, 0, adcResults.inputVoltage);

    errCode = initFTools();
    if (errCode != NRF_SUCCESS)
        return errCode;

    setFeatures(&features, isLightPresent); /// TODO: if light presence has change a service changed indication is necessary

    setModes(&modes, features.channelCount, &channels[0][0]);

    errCode = Drake_btle_Init(&features, batterySoc, isActuatorPresent, isMotSensPresent, isTravelSensorPresent);
    if (errCode != NRF_SUCCESS)
        return errCode;

    if (noinit.isValid != (uint32_t)&(__start_cell_data))
    {
        noinit.isValid = (uint32_t)&(__start_cell_data);
        noinit.activationCounter = 0;
    }

    return NRF_SUCCESS;
}

ret_code_t brd_SetPowerMode(brd_powerMode_t newMode)
{
    pendingState.powerMode = newMode;

    memset(&pendingState.overrideConfig, 0, sizeof(pendingState.overrideConfig));

    return NRF_SUCCESS;
}

ret_code_t brd_SetLightMode(uint8_t newMode)
{
    pendingState.lightMode = newMode;

    memset(&pendingState.overrideConfig, 0, sizeof(pendingState.overrideConfig));

    return NRF_SUCCESS;
}

ret_code_t brd_OverrideMode(ble_hps_cp_channel_config_t const* pConfig)
{
    if (pConfig->size != features.channelCount * sizeof(uint8_t))
        return NRF_ERROR_INVALID_LENGTH;

    memcpy(&pendingState.overrideConfig, pConfig->p_config, sizeof(pendingState.overrideConfig));

    return NRF_SUCCESS;
}

static void indicateBatteryState()
{
    hmi_ledState_t ledState = HMI_LS_OFF;

    if (chargeState == CHCTRL_STATE_CHARGING)
        ledState = HMI_LS_ON;
    else if (batterySoc < LOW_BATTERY)
        ledState = HMI_LS_BLINKSLOW;

    hmi_SetLed(HMI_LT_RED, ledState);
}

bool brd_Execute(void)
{
    static bool firstCall = true;
    bool resetLedTimeout = ((pendingState.powerMode == BRD_PM_IDLE && currentState.powerMode != pendingState.powerMode) ||
                            chargeState != CHCTRL_STATE_DISCONNECTED);

    if (firstCall)             /// TODO: better solution to s
    {
        main_ResetIdleTimer();

        if (setup.socTimeout)
        {
            if (setup.usage == DRAKE_USAGE_DROPPER_ACTUATOR)
                mtCtrl_MorseSOC(batterySoc, setup.socTimeout);
            else
            {
                Drake_Light_ShowSOC(batterySoc, setup.usage, setup.socTimeout, NULL, NULL);
                nrf_delay_ms(250);
            }
        }

        firstCall = false;
    }

    lis2dh12drv_Execute();
    vl53l0xdrv_Execute(adcResults.temperature);
    Drake_Hmi_Execute(resetLedTimeout);
    Drake_Adc_Execute();

    // check if we can enter sysoff
    if (goToSysOff())
        pendingState.sysOff = true; // no direct assignment to prevent overwriting in plunger slave mode

    bool stateChanged = hasStateChanged();
    bool targetChanged = hasTargetChanged();

    if (stateChanged)
        updateState();

    if (stateChanged || targetChanged)
        updateTarget();

    if (oneHzFlag)
    {
        uint8_t lightModeConfig[CHANNEL_CNT];

        getChannelConfig(&currentState, lightModeConfig);

        // reset idle timer if light is on
        if (mm_GetCurrentMode() != MM_MODE_OFF && isModeLightUsed(lightModeConfig)) // cannot be 0 while charging
            main_ResetIdleTimer();

        // feed battery gauge and update ble service
        feedBatteryGauge();

        // feed charge control
        feedChargeControl();

        // update message data
        sendHpsMessage();

        // indicate charging/lo bat
        indicateBatteryState();

        oneHzFlag = false;
    }

    return false;
}

ret_code_t brd_GetChannelConfig(void const** ppData, uint16_t* pSize)
{
    if (ppData == NULL || pSize == NULL)
        return NRF_ERROR_NULL;

    *ppData = channels;

    return NRF_SUCCESS;
}

ret_code_t brd_SetChannelConfig(void const* pData, uint16_t size, ds_reportHandler_t resultHandler)
{
    if (pData == NULL)
        return NRF_ERROR_NULL;

    if (size != sizeof(uint8_t) * MM_NUM_OF_MODES * features.channelCount)
        return NRF_ERROR_INVALID_LENGTH;

    memcpy(newChannels.channels, pData, size);

    if (!isChannelConfigValid(pData, ARRAY_SIZE(newChannels.channels)))
        return NRF_ERROR_INVALID_PARAM;

    memcpy(&channels, &newChannels.channels, sizeof(channels));

    updateState();    // the current channel might have been changed, update mode

    return storeChannels(DEFAULT_SET, resultHandler);
}

ret_code_t brd_SetDeviceName(char const* pNewName, ds_reportHandler_t resultHandler)
{
    if (pNewName == NULL)
        return NRF_ERROR_NULL;

    if (strlen(pNewName) > DEVICENAME_LENGHT)
        return NRF_ERROR_INVALID_LENGTH;

    strcpy(setup.deviceName, pNewName);

    memcpy(&newSetup.setup, &setup, sizeof(setup));

    return storeSetup(resultHandler);

    /// TODO: is advertising name isn't updated until next start
}

ret_code_t brd_FactoryReset(ds_reportHandler_t resultHandler)
{
    if (resultHandler == NULL)
        return NRF_ERROR_NULL;

    // set defaults setting
    uint8_t chDefaults[CHANNEL_CNT][MM_NUM_OF_MODES] = CHANNELS_DEFAULTS;
    memcpy(&channels, &chDefaults, sizeof(channels));
    setup_t cfgDefaults = SETUP_DEFAULTS;
    memcpy(&setup, &cfgDefaults, sizeof(setup_t));

    ret_code_t errCode = ds_Reset(DRAKE_FILE_ID, resultHandler);

    // since device role will change a device reset is necessary
    /// TODO: a delay is necessary to allow other modules perform factory reset
    //nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_RESET); /// TODO: service change indication!

    return errCode;
}

ret_code_t brd_SupportNotif(ble_hps_c_s_t const* pSupportData)
{
    if (pSupportData == NULL)
        return NRF_ERROR_NULL;

    /// TODO: synchronization necessary?

    if (pSupportData->flags.brake_ind_present)
    {
        q6_10_t cd = pSupportData->brake_ind - pSupportData->time_ref;
        return ftools_SetCountdown(&countdown[COUNTDOWN_BRAKE], cd);
    }

    if (pSupportData->flags.left_ind_present)
    {
        q6_10_t cd = pSupportData->left_ind - pSupportData->time_ref;
        return ftools_SetCountdown(&countdown[COUNTDOWN_LEFT_IND], cd);
    }

    if (pSupportData->flags.right_ind_present)
    {
        q6_10_t cd = pSupportData->right_ind - pSupportData->time_ref;
        return ftools_SetCountdown(&countdown[COUNTDOWN_RIGHT_IND], cd);
    }

    return NRF_SUCCESS;
}

drake_usage_t Drake_GetDeviceUsage(void)
{
    return setup.usage;
}

ret_code_t Drake_SetDeviceUsage(drake_usage_t usage, ds_reportHandler_t resultHandler)
{
    if (!isDeviceUsageValid(usage))
        return NRF_ERROR_INVALID_PARAM;

    setup.usage = usage;

    memcpy(&newSetup.setup, &setup, sizeof(setup));

    ret_code_t errCode = storeSetup(resultHandler);

    nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_RESET);

    return errCode;
}

ret_code_t Drake_GetBatteryInfo(drake_batteryInfo_t* pInfo)
{
    if (pInfo == NULL)
        return NRF_ERROR_NULL;

    *pInfo = setup.battInfo;

    return NRF_SUCCESS;
}

ret_code_t Drake_SetBatteryInfo(drake_batteryInfo_t const* pInfo, ds_reportHandler_t resultHandler)
{
    if (pInfo == NULL)
        return NRF_ERROR_NULL;

    if (!isBatteryInfoValid(pInfo))
        return NRF_ERROR_INVALID_PARAM;

    // reinitialize charge module to adjust charge current
    chctrl_current_t maxChargeCurrent = pInfo->capNom >= BAT_CAP_FULL_CHARGE ? CHCTRL_CURRENT_500MA : CHCTRL_CURRENT_200MA;
    ret_code_t errCode = chctrl_Init(maxChargeCurrent);
    if (errCode != NRF_SUCCESS)
        return errCode;

    setup.battInfo = *pInfo;

    memcpy(&newSetup.setup, &setup, sizeof(setup));

    return storeSetup(resultHandler);
}

ret_code_t Drake_GetSocTimeout(uint32_t* pTimeout)
{
    if (pTimeout == NULL)
        return NRF_ERROR_NULL;

    *pTimeout = setup.socTimeout;

    return NRF_SUCCESS;
}

ret_code_t Drake_SetSocTimeout(uint32_t timeout, ds_reportHandler_t resultHandler)
{
    if (!isSocTimeoutValid(timeout))
        return NRF_ERROR_INVALID_PARAM;

    setup.socTimeout = timeout;

    memcpy(&newSetup.setup, &setup, sizeof(setup));

    return storeSetup(resultHandler);
}

ret_code_t Drake_GetLightPattern(Drake_lightType_t type, Drake_lightPattern_t* pPattern)
{
    if (pPattern == NULL)
        return NRF_ERROR_NULL;

    pPattern->len = PATTERN_LEN;

    switch (type)
    {
    case DRAKE_LIGHT_REAR:
        pPattern->pPattern = setup.tailPattern;
        break;
    case DRAKE_LIGHT_FRONT:
        pPattern->pPattern = setup.frontPattern;
        break;
    case DRAKE_LIGHT_POS:
        pPattern->pPattern = setup.posPattern;
        break;
    case DRAKE_LIGHT_BRAKE:
        pPattern->pPattern = setup.brakePattern;
        break;
    case DRAKE_LIGHT_INDIC_LEFT:
        pPattern->pPattern = setup.leftPattern;
        break;
    case DRAKE_LIGHT_INDIC_RIGHT:
        pPattern->pPattern = setup.rightPattern;
        break;
    default:
        return NRF_ERROR_INVALID_PARAM;
    }

    return NRF_SUCCESS;
}

ret_code_t Drake_SetLightPattern(Drake_lightType_t type, Drake_lightPattern_t const* pPattern, ds_reportHandler_t resultHandler)
{
    if (pPattern == NULL || pPattern->pPattern == NULL)
        return NRF_ERROR_NULL;

    if (pPattern->len > PATTERN_LEN)
        return NRF_ERROR_INVALID_LENGTH;

    switch (type)
    {
    case DRAKE_LIGHT_REAR:
        memset(setup.tailPattern, 0, PATTERN_LEN);
        memcpy(setup.tailPattern, pPattern->pPattern, pPattern->len);
        break;
    case DRAKE_LIGHT_FRONT:
        memset(setup.frontPattern, 0, PATTERN_LEN);
        memcpy(setup.frontPattern, pPattern->pPattern, pPattern->len);
        break;
    case DRAKE_LIGHT_POS:
        memset(setup.posPattern, 0, PATTERN_LEN);
        memcpy(setup.posPattern, pPattern->pPattern, pPattern->len);
        break;
    case DRAKE_LIGHT_BRAKE:
        memset(setup.brakePattern, 0, PATTERN_LEN);
        memcpy(setup.brakePattern, pPattern->pPattern, pPattern->len);
        break;
    case DRAKE_LIGHT_INDIC_LEFT:
        memset(setup.leftPattern, 0, PATTERN_LEN);
        memcpy(setup.leftPattern, pPattern->pPattern, pPattern->len);
        break;
    case DRAKE_LIGHT_INDIC_RIGHT:
        memset(setup.rightPattern, 0, PATTERN_LEN);
        memcpy(setup.rightPattern, pPattern->pPattern, pPattern->len);
        break;
    default:
        return NRF_ERROR_INVALID_PARAM;
    }

    memcpy(&newSetup.setup, &setup, sizeof(setup));

    return storeSetup(resultHandler);
}

ret_code_t Drake_SetPlungerPosition(q8_t newPosition, bool remote)
{
    if (newPosition)
        noinit.activationCounter++;

    if (setup.usage == DRAKE_USAGE_DROPPER_ACTUATOR)
    {
        if (!remote && currentState.actuatorSlaveMode)
            pendingState.actuatorSlaveMode = false;

        ret_code_t errCode = mtCtrl_SetPosition(newPosition);
        if (errCode != NRF_SUCCESS)
            return errCode;

        if (newPosition)    // enable travel sensor, if actuator is opening, it is disabled when the motor reports closed position
        {
            errCode = vl53l0xdrv_SetPowerMode(VL53L0X_PWR_DATA);
            if (errCode == NRF_ERROR_INVALID_STATE) // if no sensor was found at startup
                errCode = NRF_SUCCESS;
        }

        return errCode;

    }
    else if (setup.usage == DRAKE_USAGE_TAILLIGHT_16LED)
        return drakedrv_SetMotorPosition(newPosition);
    else
        return NRF_ERROR_NOT_SUPPORTED;
}

ret_code_t Drake_GetPlungerPosition(q8_t* pPosition)
{
    if (pPosition == NULL)
        return NRF_ERROR_NULL;

    if (setup.usage == DRAKE_USAGE_DROPPER_ACTUATOR)
    {
        *pPosition =  mtCtrl_GetCurrentPosition();
        return NRF_SUCCESS;
    }
    else if (setup.usage == DRAKE_USAGE_TAILLIGHT_16LED)
        return drakedrv_GetMotorPosition(pPosition);

    return NRF_ERROR_NOT_SUPPORTED;
}

ret_code_t Drake_GetMotorConfig(mtCtrl_motorConfig_t* pMotorConfig)
{
    if (pMotorConfig == NULL)
        return NRF_ERROR_NULL;

    if (setup.usage == DRAKE_USAGE_DROPPER_ACTUATOR)
        *pMotorConfig = setup.motorConfig;
    else if (setup.usage == DRAKE_USAGE_TAILLIGHT_16LED)
    {
        drakedrv_motorConfig_t cfg;
        ret_code_t errCode = drakedrv_GetMotorConfig(&cfg); /// TODO: working? same interrupt level!
        if (errCode != NRF_SUCCESS)
            return errCode;

        pMotorConfig->openDutyCycle = cfg.dutyCycleOpen;
        pMotorConfig->closeDutyCycle = cfg.dutyCycleClose;
        pMotorConfig->openTimeout = cfg.timeoutOpen << 7;
        pMotorConfig->closeTimeout = cfg.timeoutClose << 7;
    }
    else
        return NRF_ERROR_NOT_SUPPORTED;

    return NRF_SUCCESS;
}

ret_code_t Drake_SetMotorConfiguration(mtCtrl_motorConfig_t const* pMotorConfig, ds_reportHandler_t resultHandler)
{
    if (pMotorConfig == NULL)
        return NRF_ERROR_NULL;

    if (!isMotorConfigValid(pMotorConfig))
        return NRF_ERROR_INVALID_PARAM;

    if (setup.usage == DRAKE_USAGE_DROPPER_ACTUATOR)
        return setMotorConfiguration(pMotorConfig, resultHandler);
    else if (setup.usage == DRAKE_USAGE_TAILLIGHT_16LED)
    {
        drakedrv_motorConfig_t cfg;
        cfg.dutyCycleOpen = pMotorConfig->openDutyCycle > UINT8_MAX ? UINT8_MAX : pMotorConfig->openDutyCycle;
        cfg.dutyCycleClose = pMotorConfig->closeDutyCycle > UINT8_MAX ? UINT8_MAX : pMotorConfig->closeDutyCycle;
        cfg.timeoutOpen = pMotorConfig->openTimeout >> 7;
        cfg.timeoutClose = pMotorConfig->closeTimeout >> 7;
        return drakedrv_SetMotorConfig(&cfg);       /// TODO: working? same interrupt level
    }
    else
        return NRF_ERROR_NOT_SUPPORTED;
}

ret_code_t Drake_GetMotorCalibrationData(drake_motorSensCalibType_t type, drake_motorSensCalibData_t* pCalibData)
{
    if (pCalibData == NULL)
        return NRF_ERROR_NULL;

    if (setup.usage == DRAKE_USAGE_DROPPER_ACTUATOR)
    {
        switch (type)
        {
        case DRAKE_MOTOR_SENS_CALIB_THRESHOLD:
            pCalibData->threshold = setup.sensorConfig.compThresholds;
            return NRF_SUCCESS;

        case DRAKE_MOTOR_SENS_CALIN_OPEN_CNT:
            pCalibData->openCnt = setup.sensorConfig.fullOpenCount;
            return NRF_SUCCESS;

        default:
            return NRF_ERROR_NOT_SUPPORTED;
        }
    }
    else if (setup.usage == DRAKE_USAGE_TAILLIGHT_16LED)
    {
        drakedrv_motorSensorData_t data;
        ret_code_t errCode = drakedrv_GetMotorSensorData(&data);
        switch (type)
        {
        case DRAKE_MOTOR_SENS_CALIB_THRESHOLD:
            pCalibData->threshold.lower = data.threshLow;
            pCalibData->threshold.upper = data.threshUp;
            return errCode;

        case DRAKE_MOTOR_SENS_CALIN_OPEN_CNT:
            pCalibData->openCnt = data.fullOpenCnt;
            return errCode;

        default:
            return NRF_ERROR_INVALID_PARAM;
        }
    }
    else
        return NRF_ERROR_NOT_SUPPORTED;
}

ret_code_t Drake_ClearMotorCalibrationData(drake_motorSensCalibType_t type, ds_reportHandler_t resultHandler)
{
    if (setup.usage == DRAKE_USAGE_DROPPER_ACTUATOR)
        return clearMotorCalibrationData(type, resultHandler);
    else if (setup.usage == DRAKE_USAGE_TAILLIGHT_16LED)
    {
        switch (type)
        {
        case DRAKE_MOTOR_SENS_CALIB_THRESHOLD:
            return drakedrv_ClearThresholdCalibrationData();

        case DRAKE_MOTOR_SENS_CALIN_OPEN_CNT:
            return drakedrv_ClearFullOpenThresholdData();

        default:
            return NRF_ERROR_INVALID_PARAM;
        }
    }
    else
        return NRF_ERROR_NOT_SUPPORTED;
}

ret_code_t Drake_StartMotorCalibration(drake_motorSensCalibType_t type, ds_reportHandler_t resultHandler)
{
    if (setup.usage == DRAKE_USAGE_DROPPER_ACTUATOR)
        return startMotorCalibration(type, resultHandler);
    else if (setup.usage == DRAKE_USAGE_TAILLIGHT_16LED)
    {
        switch (type)
        {
        case DRAKE_MOTOR_SENS_CALIB_THRESHOLD:
            return drakedrv_StartThresholdCalibration();

        case DRAKE_MOTOR_SENS_CALIN_OPEN_CNT:
            return drakedrv_StartFullOpenCalibration();

        default:
            return NRF_ERROR_INVALID_PARAM;
        }
    }
    else
        return NRF_ERROR_NOT_SUPPORTED;
}

ret_code_t Drake_GetTravelSensorData(drake_travelSensorData_t* pData)
{
    if (pData == NULL)
        return NRF_ERROR_NULL;

    if (setup.usage == DRAKE_USAGE_DROPPER_ACTUATOR)
    {
        *pData = setup.travelSensor;
        return NRF_SUCCESS;
    }
    else if (setup.usage == DRAKE_USAGE_TAILLIGHT_16LED)
    {
        drakedrv_travelSensorData_t data;
        ret_code_t errCode = drakedrv_GetTravelSensorData(&data);
        pData->travelInmm = data.travel;
        pData->offsetInMikrom = data.offset;
        return errCode;
    }
    else
        return NRF_ERROR_NOT_SUPPORTED;

}

ret_code_t Drake_ClearTravelSensorData(ds_reportHandler_t resultHandler)
{
    if (setup.usage == DRAKE_USAGE_DROPPER_ACTUATOR)
        return clearTravelSensorData(resultHandler);
    else if (setup.usage == DRAKE_USAGE_TAILLIGHT_16LED)
        return drakedrv_ClearTravelSensorOffsetCalibrationData();
    else
        return NRF_ERROR_NOT_SUPPORTED;
}

ret_code_t Drake_StartTravelSensorCalibration(uint16_t travelInmm, ds_reportHandler_t resultHandler)
{
    if (setup.usage == DRAKE_USAGE_DROPPER_ACTUATOR)
        return startTravelSensorCalibration(travelInmm, resultHandler);
    else if (setup.usage == DRAKE_USAGE_TAILLIGHT_16LED)
        return drakedrv_StartTravelSensorOffsetCalibration(travelInmm);
    else
        return NRF_ERROR_NOT_SUPPORTED;
}

ret_code_t Drake_GetIndexedTravelConfig(drake_indexedTravelConfig_t* pConfig)
{
    if (pConfig == NULL)
        return NRF_ERROR_NULL;

    if (setup.usage == DRAKE_USAGE_DROPPER_ACTUATOR)
    {
        if (setup.travelSensor.travelInmm == 0)
            return NRF_ERROR_NOT_SUPPORTED;
        *pConfig = setup.indexedConfig;
        return NRF_SUCCESS;
    }
    else if (setup.usage == DRAKE_USAGE_TAILLIGHT_16LED)
    {
        memset(pConfig, 0, sizeof(drake_indexedTravelConfig_t));

        drakedrv_travelSensorData_t data;
        ret_code_t errCode = drakedrv_GetTravelSensorData(&data);
        if (errCode != NRF_SUCCESS)
            return errCode;
        if (data.travel == 0)   // not calibrated yet or no sensor available
            return NRF_ERROR_NOT_SUPPORTED;

        drakedrv_indexedConfig_t cfg;
        errCode = drakedrv_GetIndexedTravelConfig(&cfg);
        if (errCode != NRF_SUCCESS)
            return errCode;

        pConfig->blockingTime = cfg.openBlocking << 8;
        pConfig->timeout = cfg.closeTimeout << 8;
        for (uint8_t i = 0; i < ARRAY_SIZE(pConfig->positions); i++)
        {
            if (i < ARRAY_SIZE(cfg.positions))
                pConfig->positions[i] = (int32_t)cfg.positions[i] * -data.travel / UINT8_MAX;
        }
        return NRF_SUCCESS;
    }
    else
        return NRF_ERROR_NOT_SUPPORTED;



    return NRF_SUCCESS;
}

ret_code_t Drake_SetIndexedTravelConfig(drake_indexedTravelConfig_t const* pConfig, ds_reportHandler_t resultHandler)
{
    if (pConfig == NULL)
        return NRF_ERROR_NULL;

    if (!isIndexedTravelConfigValid(pConfig))
        return NRF_ERROR_INVALID_PARAM;

    if (setup.usage == DRAKE_USAGE_DROPPER_ACTUATOR)
        return setIndexedTravelConfig(pConfig, resultHandler);
    else if (setup.usage == DRAKE_USAGE_TAILLIGHT_16LED)
    {
        drakedrv_travelSensorData_t data;
        ret_code_t errCode = drakedrv_GetTravelSensorData(&data);
        if (errCode != NRF_SUCCESS)
            return errCode;
        if (data.travel == 0)   // not calibrated yet or no sensor available
            return NRF_ERROR_NOT_SUPPORTED;

        drakedrv_indexedConfig_t cfg;
        cfg.openBlocking = pConfig->blockingTime >> 8;
        cfg.closeTimeout = pConfig->timeout >> 8;
        for (uint8_t i = 0; i < ARRAY_SIZE(cfg.positions); i++)
        {
            if (i < ARRAY_SIZE(pConfig->positions))
                cfg.positions[i] = abs(pConfig->positions[i]) * UINT8_MAX / data.travel;
            else
                cfg.positions[i] = 0;
        }
        return drakedrv_SetIndexedTravelConfig(&cfg);
    }
    else
        return NRF_ERROR_NOT_SUPPORTED;
}

void Drake_Brake()
{
    ftools_UpdateCountdown(&countdown[COUNTDOWN_BRAKE], FTOOLS_BRAKEFLASH);
}

void Drake_IndicLeft()
{
    ftools_UpdateCountdown(&countdown[COUNTDOWN_LEFT_IND], FTOOLS_INDICATOR);
}

void Drake_IndicRight()
{
    ftools_UpdateCountdown(&countdown[COUNTDOWN_RIGHT_IND], FTOOLS_INDICATOR);
}

uint16_t Drake_GetActivationCounter(void)
{
    return noinit.activationCounter;
}

void Drake_SetActivationCounter(uint16_t newCounterValue)
{
    noinit.activationCounter = newCounterValue;
}

/**END OF FILE*****************************************************************/
