/**
  ******************************************************************************
  * @file    motor_control.c
  * @author  Thomas Reisnecker
  * @brief   local motor control for Drake Hardware 1.0
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

/* Includes ------------------------------------------------------------------*/
#include "motor_control.h"
#include "motor_drv.h"
#include "app_timer.h"
#include "debug.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_power.h"

/* External variables --------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
    uint32_t isValid;
    int16_t count;
} currentOpenCount_t;

/* Private macros ------------------------------------------------------------*/

/* Private defines -----------------------------------------------------------*/
#define VALID_PATTERN           0x976EA72D

#define DC_MAX                  256
#define DC_MORSE                32

#define MOTOR_TIMEOUT_MAX       APP_TIMER_TICKS(2000)
//#define MOTOR_TIMEOUT_BOOST     APP_TIMER_TICKS(50)     // boost time for starting reverse
#define MOTOR_TIMEOUT_NEXT      APP_TIMER_TICKS(100)    // max timeout to next sensor irq

#define DEFAULT_FULL_OPEN_CNT   INT16_MAX

/* Private variables ---------------------------------------------------------*/
static mtCtrl_motorConfig_t motorConfig;   // the configuration used for motor (timeouts and DCs)
static mtdrv_mode_t      motorMode;     // the current motor mode
APP_TIMER_DEF(motorTimer);              // timer for controlling the motor

static mtCtrl_eventHandler_t eventHandler;

static int16_t fullOpenCount;           // sensor count value for full open
static int16_t targetOpenCount;         // the current target value
static currentOpenCount_t currentOpenCount __attribute__((section(".noinit"))); // the actual value (stored in a noinit section to survive system off)

static bool powerWarningEnabled;    // indicator if power warning has been enabled (can't be enabled at init, because softdevice isn't enabled yet)
static bool fullOpenCalibration;    // indicator if positionCalibration is ongoing (determining the fullOpenCount value)
//static bool reverseBoost;           // indicator if reverse is running with max DC or lowered DC
static uint64_t morseCode;          // indicator that the motor is used to indicate the state of charge
static uint32_t timebaseMorse;      // the timebase for a dot when morseing is enabled

/* Private read only variables -----------------------------------------------*/
static const uint64_t digitInMorse[] =
{
    0b1110111011101110111,                      // 0
    0b11101110111011101,                        // 1
    0b111011101110101,                          // 2
    0b1110111010101,                            // 3
    0b11101010101,                              // 4
    0b101010101,                                // 5
    0b10101010111,                              // 6
    0b1010101110111,                            // 7
    0b101011101110111,                          // 8
    0b10111011101110111,                        // 9
    0b111011101110111011100011101110111011101   // 10
};

/* Private functions ---------------------------------------------------------*/
static q8_t calculateCurrentPosition(uint16_t rawCnt)
{
    if (fullOpenCount == DEFAULT_FULL_OPEN_CNT)
        return 0;

    uint32_t pos = rawCnt * 256;
    pos /= fullOpenCount;
    if (pos > UINT8_MAX)
        pos = UINT8_MAX;

    return (q8_t)pos;
}

static void motSensHandler(mtSens_evt_t const* pEvt)
{
    if (pEvt->type == MTSENS_EVTTYPE_POS_CHANGE)
    {
        currentOpenCount.count = pEvt->position;

        // position sensor only works for 100% DC, otherwise PWM noise is picked up
        // so no position regulation if dc isn't 100%
        if ((motorMode == MTDRV_REVERSE && motorConfig.closeDutyCycle != DC_MAX) ||
            (motorMode == MTDRV_FORWARD && motorConfig.openDutyCycle != DC_MAX))
            return;

        NRF_LOG_INFO("motor pos.: %d", pEvt->position);

        // send event with new position
        if (eventHandler)
        {
            mtCtrl_event_t evt;
            evt.type = MTCTRL_EVT_TYPE_NEW_POSITION;
            evt.position = calculateCurrentPosition(currentOpenCount.count);
            eventHandler(&evt);
        }

        if (fullOpenCount == DEFAULT_FULL_OPEN_CNT) // no calibration data available, fall back to timer mode
            return;

        (void)app_timer_stop(motorTimer);   // timer can be disabled, it is either restarted or not needed anymore

        if ((motorMode == MTDRV_FORWARD && currentOpenCount.count >= targetOpenCount) ||    // target reached, stopping motor
            (motorMode == MTDRV_REVERSE && currentOpenCount.count <= targetOpenCount))
        {
            motorMode = MTDRV_OFF;
            mtdrv_Enable(motorMode, 0);
            mtSens_Enable(false);
            return; // job done, no need for timer restart
        }

        (void)app_timer_start(motorTimer, MOTOR_TIMEOUT_NEXT, NULL);    // restart timer for the case that target cannot be reached
    }
}

static void motorTimerCallback(void * pContext)
{
    (void) pContext;

    if (fullOpenCalibration)
    {
        // close plunger
        motorMode = MTDRV_REVERSE;
        mtdrv_Enable(motorMode, motorConfig.closeDutyCycle);
        mtSens_SetDirection(MTSENS_DIR_REVERSE);
        (void)app_timer_start(motorTimer, motorConfig.closeTimeout, NULL);

        // end calibration
        fullOpenCalibration = false;

        // update count, use max value (which results in timeout mode) as fail save if no count was detected
        fullOpenCount = currentOpenCount.count ? currentOpenCount.count : DEFAULT_FULL_OPEN_CNT;

        // send event
        if (eventHandler)
        {
            mtCtrl_event_t evt =
            {
                .type = MTCTRL_EVT_TYPE_CALIB_FULL_OPEN,
                .fullyOpenCount = currentOpenCount.count
            };
            eventHandler(&evt);
        }
    }
    else if (morseCode)
    {
        // send next morse "bit"
        mtdrv_Enable(motorMode, morseCode & 1 ? DC_MORSE : 0);
        morseCode >>= 1;
        (void)app_timer_start(motorTimer, timebaseMorse, NULL);
    }
    else // normal operation, shutting off motor
    {
        if (motorMode == MTDRV_REVERSE)
        {
            /*if (reverseBoost)
            {
                // start boost is over, reducing motor power
                mtdrv_Enable(motorMode, motorConfig.closeDutyCycle);
                (void)app_timer_start(motorTimer, motorConfig.closeTimeout, NULL);
                reverseBoost = false;
                return; // job done for this case
            }
            else/*/
            {
                // plunger should be closed now, resetting position Sensor
                mtSens_ResetCounter();
                currentOpenCount.count = 0;
            }
        }

        // send event with new position, assuming full opened or closed
        if (eventHandler)
        {
            mtCtrl_event_t evt;
            evt.type = MTCTRL_EVT_TYPE_NEW_POSITION;
            evt.position = motorMode == MTDRV_FORWARD ? UINT8_MAX : 0;
            eventHandler(&evt);
        }

        motorMode = MTDRV_OFF;
        mtdrv_Enable(motorMode, 0);
        mtSens_Enable(false);
    }
}

static void powerWarningHandler(uint32_t evtId, void* pContext)
{
    (void)pContext;

    if (evtId == NRF_EVT_POWER_FAILURE_WARNING)
    {
        // battery voltage dropped below 2.8V, shutting down to prevent reset or battery protection cut off
        motorMode = MTDRV_OFF;
        mtdrv_Enable(motorMode, 0);
        mtSens_Enable(false);
        (void)app_timer_stop(motorTimer);

        NRF_LOG_INFO("low voltage, shutting down motor");

        if (eventHandler)
        {
            mtCtrl_event_t evt = {.type = MTCTRL_EVT_TYPE_POWER_FAIL};
            eventHandler(&evt);
        }
    }
}

NRF_SDH_SOC_OBSERVER(powerWarningObserver, 0, powerWarningHandler, NULL);

static ret_code_t powerFailureEnable()
{
    ret_code_t errCode;

    // enabling power failure comparator
    errCode = sd_power_pof_threshold_set(NRF_POWER_THRESHOLD_V28);
    if (errCode != NRF_SUCCESS)
        return errCode;

    errCode = sd_power_pof_enable(true);
    if (errCode != NRF_SUCCESS)
        return errCode;

    return NRF_SUCCESS;
}

static uint64_t getMorsePatternFromSOC(q8_t soc)
{
    int16_t i = (soc * 1000) / 256; // convert from q8_t to permille
    i += 50;                        // add rounding
    i /= 100;                       // reduce to per tens
    // final result 0..5% -> 0, 5..15% -> 1, 15..25% -> 2 ...

    return (i < 0 || i > 10) ? 0 : digitInMorse[i];
}

/* Public functions ----------------------------------------------------------*/
ret_code_t mtCtrl_Init(mtCtrl_init_t const* pInit, bool* pSensorAvailable)
{
    ret_code_t errCode;

    if (pInit == NULL || pInit->pMotorConfig == NULL || pInit->pSensorConfig == NULL)
        return NRF_ERROR_NULL;

    mtSens_Init_t msInit;
    msInit.evtHandler = motSensHandler;
    msInit.pThresholds = &pInit->pSensorConfig->compThresholds;
    errCode = mtSens_Init(&msInit);
    if (errCode == NRF_ERROR_NOT_FOUND)
        *pSensorAvailable = false;
    else if (errCode == NRF_SUCCESS)
        *pSensorAvailable = true;
    else
        return errCode;

    errCode = mtdrv_Init();
    if (errCode != NRF_SUCCESS)
        return errCode;

    errCode = app_timer_create(&motorTimer, APP_TIMER_MODE_SINGLE_SHOT, &motorTimerCallback);
    if (errCode != NRF_SUCCESS)
        return errCode;

    motorConfig = *pInit->pMotorConfig;

    fullOpenCount = pInit->pSensorConfig->fullOpenCount ? pInit->pSensorConfig->fullOpenCount : DEFAULT_FULL_OPEN_CNT;

    if (currentOpenCount.isValid != VALID_PATTERN)  // in noinit section, only needs initialization at power on
    {
        currentOpenCount.count = INT16_MAX;         // let's assume that motor is fully opend
        currentOpenCount.isValid = VALID_PATTERN;
    }

    eventHandler = pInit->eventHandler;

    return NRF_SUCCESS;
}

ret_code_t mtCtrl_SetPosition(q8_t position)
{
    if (fullOpenCalibration)
        return NRF_ERROR_INVALID_STATE;

    if (position != 0 && position != 255)
        return NRF_ERROR_NOT_SUPPORTED;

    if (!powerWarningEnabled)   // enable power warning on first use (cannot be done in init because softdevice needs to be enabled)
    {
        powerFailureEnable();
        powerWarningEnabled = true;
    }

    int32_t newTarget;
    mtdrv_mode_t newMode = MTDRV_OFF;
    ret_code_t errCode = NRF_SUCCESS;

    // calculate counter target value
    if (position == 0)
        newTarget = 0;
    else
        newTarget = fullOpenCount - 1; // do not open completely to prevent plunger jamming

    // determine direction
    if (newTarget <= 0 || newTarget < currentOpenCount.count)
        newMode = MTDRV_REVERSE;
    else if (newTarget > currentOpenCount.count)
        newMode = MTDRV_FORWARD;

    // enable motor, in case of closing use max DC for boost phase
    if (newMode == MTDRV_FORWARD)
    {
        mtSens_Enable(true);
        mtSens_SetDirection(MTSENS_DIR_FORWARD);

        mtdrv_Enable(MTDRV_FORWARD, motorConfig.openDutyCycle);

        (void)app_timer_stop(motorTimer);
        errCode = app_timer_start(motorTimer, motorConfig.openTimeout, NULL);
        LOG_ERROR_CHECK("error %d starting motor timer", errCode);
    }
    else if (newMode == MTDRV_REVERSE)
    {
        mtSens_Enable(true);
        mtSens_SetDirection(MTSENS_DIR_REVERSE);

        mtdrv_Enable(MTDRV_REVERSE, motorConfig.closeDutyCycle);

        (void)app_timer_stop(motorTimer);
        errCode = app_timer_start(motorTimer, motorConfig.closeTimeout, NULL);
        LOG_ERROR_CHECK("error %d starting motor timer", errCode);

        //reverseBoost = true;
    }

    targetOpenCount = newTarget;
    motorMode = newMode;

    NRF_LOG_INFO("target: %d %s", newTarget, newMode == MTDRV_FORWARD ? "forward" : "reverse");

    return errCode;
}

q8_t mtCtrl_GetCurrentPosition()
{
    return targetOpenCount ? UINT8_MAX : 0;

    /*if (fullyOpenCount == DEFAULT_FULLY_OPEN_CNT)
        return targetOpenCount ? UINT8_MAX : 0;
    else if (currentOpenCount.count < 0)
        return 0;
    else
        return ((int32_t)currentOpenCount.count << 8) / fullyOpenCount;*/
}

ret_code_t mtCtrl_CalibrateSensor(mtSens_thresholds_t* pTh)
{
    if (eventHandler == NULL)
        return NRF_ERROR_NOT_SUPPORTED;

    mtdrv_Enable(MTDRV_FORWARD, DC_MAX);
    mtSens_Calibrate(pTh); /// TODO: this function is blocking for a long time, rewrite!
    mtdrv_Enable(MTDRV_OFF, 0);

    return NRF_SUCCESS;
}

ret_code_t mtCtrl_CalibrateFullOpen(void)
{
    if (eventHandler == NULL)
        return NRF_ERROR_NOT_SUPPORTED;

    if (motorMode != MTDRV_OFF || currentOpenCount.count > 0 || fullOpenCalibration)
        return NRF_ERROR_INVALID_STATE;

    if (motorConfig.openDutyCycle != DC_MAX)    // sensor not working unless DC is 100%
        return NRF_ERROR_INVALID_PARAM;

    targetOpenCount = INT16_MAX;    // set to max (which cannot be reached) and wait for timeout
    fullOpenCalibration = true;
    motorMode = MTDRV_FORWARD;
    (void)mtSens_Enable(true);
    mtSens_ResetCounter();
    (void)mtSens_SetDirection(MTSENS_DIR_FORWARD);
    mtdrv_Enable(motorMode, motorConfig.openDutyCycle);
    return app_timer_start(motorTimer, motorConfig.openTimeout, NULL);
}

ret_code_t mtCtrl_UpdateMotorConfig(mtCtrl_motorConfig_t const* pConfig)
{
    if (pConfig == NULL)
        return NRF_ERROR_NULL;

    if (pConfig->openDutyCycle > DC_MAX || pConfig->closeDutyCycle > DC_MAX)
        return NRF_ERROR_INVALID_PARAM;

    if (pConfig->openTimeout > MOTOR_TIMEOUT_MAX || pConfig->closeTimeout > MOTOR_TIMEOUT_MAX)
        return NRF_ERROR_INVALID_PARAM;

    motorConfig = *pConfig;

    return NRF_SUCCESS;
}

ret_code_t mtCtrl_MorseSOC(q8_t soc, uint32_t timebase)
 {
     if (motorMode != MTDRV_OFF || currentOpenCount.count > 0 || fullOpenCalibration)
        return NRF_ERROR_INVALID_STATE;

    morseCode = getMorsePatternFromSOC(soc);

    if (morseCode == 0)
        return NRF_ERROR_INVALID_PARAM;

    timebaseMorse = timebase;

    motorMode = MTDRV_REVERSE;
    mtdrv_Enable(motorMode, morseCode & 1 ? DC_MORSE : 0);
    morseCode >>= 1;
    return app_timer_start(motorTimer, timebaseMorse, NULL);
 }

/**END OF FILE*****************************************************************/
