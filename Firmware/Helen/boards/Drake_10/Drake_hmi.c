/**
  ******************************************************************************
  * @file    Drake_hmi.c
  * @author  Thomas Reisnecker
  * @brief   button and status led handling for Drake Hardware 1.0
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
#include "nrf_gpio.h"
#include "SEGGER_RTT.h"

#include "Drake_hmi.h"
#include "mode_management.h"
#include "link_management.h"
#include "board.h"
#include "button.h"
#include "debug.h"
#include "app_timer.h"

#include "Drake_10.h"

/* External variables --------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/
typedef enum
{
    BUTEVAL_NONE = 0,   // no event
    BUTEVAL_PRESSED,    // button changed from open to pressed
    BUTEVAL_CLICKSHORT, // button released after short period
    BUTEVAL_CLICKLONG,  // button released after long period
    BUTEVAL_RELEASED,   // button released after a hold event
    BUTEVAL_HOLD2SEC,   // button is hold for 2sec (a BUTEVAL_RELEASED event will follow)
    BUTEVAL_HOLD10SEC,  // button is hold for 10sec (a BUTEVAL_RELEASED event will follow)
    BUTEVAL_ABORT,      // button is hold too long, evaluation is aborted
} buttonEvalEvents_t;

/* Private macros ------------------------------------------------------------*/

/* Private defines -----------------------------------------------------------*/
#define LED_RED                 27
#define LED_BLUE                25
#define LED_GREEN               26

#define MAIN_BUTTON             0
#define SECONDARY_BUTTON        1

#define BUTTONPRESS_LONG        MSEC_TO_UNITS(500, UNIT_10_MS)
#define BUTTONPRESS_HOLD2SEC    MSEC_TO_UNITS(2000, UNIT_10_MS)
#define BUTTONPRESS_HOLD10SEC   MSEC_TO_UNITS(10000, UNIT_10_MS)
#define BUTTONPRESS_HOLDABORT   MSEC_TO_UNITS(30000, UNIT_10_MS)

#undef  NRF_LOG_LEVEL
#define NRF_LOG_LEVEL 2         // override debug level to only show error and warnings

/* Private read-only variables -----------------------------------------------*/
static uint32_t const         ledPins[HMI_LT_CNT] =
{
    27, // red
    26, // green
    25  // blue
};

/* Private variables ---------------------------------------------------------*/
static uint8_t            buttonIds[2] = {BUTTON_EVALUATE_CNT, BUTTON_EVALUATE_CNT};
static buttonEvalEvents_t buttonEvents[2];  // the last occured button event
static bool               ledsEnabled;
static bool               ledsState[HMI_LT_CNT];
static uint32_t           timeout;
APP_TIMER_DEF(drakeHmiTimer);

/* Private functions ---------------------------------------------------------*/
/** @brief button evaluation handler, just stores the received values, the hmi
 *         events are sent in the main loop
 */
static void buttonEval(uint8_t evalId, but_evalEvent_t evt, uint16_t stillPressCnt)
{
    buttonEvalEvents_t* pButtonEvent = NULL;

    for (uint_fast8_t i = 0; i < ARRAY_SIZE(buttonIds); i++)
    {
        if (evalId == buttonIds[i])
            pButtonEvent = &buttonEvents[i];
    }
    if (pButtonEvent == NULL)
        return;

    switch (evt)
    {
    case BUTTON_CLICKSHORT:
        *pButtonEvent = BUTEVAL_CLICKSHORT;
        break;
    case BUTTON_CLICKLONG:
        *pButtonEvent = BUTEVAL_CLICKLONG;
        break;
    case BUTTON_IS_STILL_PRESSED:
        if (stillPressCnt == BUTTONPRESS_HOLD2SEC)
            *pButtonEvent = BUTEVAL_HOLD2SEC;
        else if (stillPressCnt == BUTTONPRESS_HOLD10SEC)
            *pButtonEvent = BUTEVAL_HOLD10SEC;
        break;
    default:
        break;
    }
}

/**< initialization of the led pins
 */
static void ledsInit()
{
    // green and red led pins are also used for usb detection, just use them active high, passive low for the moment
    nrf_gpio_cfg_default(LED_RED);
    nrf_gpio_cfg_default(LED_GREEN);
    nrf_gpio_cfg_default(LED_BLUE);
}

/**< initialization of the button debouncing and evaluation
 */
static ret_code_t buttonInit(uint32_t mainButton, uint32_t secondaryButton)
{
    ret_code_t errCode;

    if (mainButton == BUTTON_NOT_USED)
        return NRF_ERROR_INVALID_PARAM;

    but_evaluateInit_t evalInit = {0};
    evalInit.longClickCnt = BUTTONPRESS_LONG;
    evalInit.StillPressedPeriod = BUTTONPRESS_HOLD2SEC;
    evalInit.abortCnt = BUTTONPRESS_HOLDABORT;
    evalInit.reportHandler = buttonEval;
    buttonIds[MAIN_BUTTON] = but_EvaluateInit(&evalInit);

    but_debounceInit_t debInit = {0};
    debInit.pinNo = mainButton;
    debInit.polarity = BUTTON_ACTIVE_LOW;
    debInit.evalId = buttonIds[MAIN_BUTTON];
    errCode = but_DebounceInit(&debInit);
    LOG_ERROR_CHECK("debounce init error %d", errCode);

    if (secondaryButton == BUTTON_NOT_USED)
        return errCode;

    buttonIds[SECONDARY_BUTTON] = but_EvaluateInit(&evalInit);
    debInit.pinNo = secondaryButton;
    debInit.evalId = buttonIds[SECONDARY_BUTTON];
    if (errCode == NRF_SUCCESS)
    {
        errCode = but_DebounceInit(&debInit);
        LOG_ERROR_CHECK("debounce init error %d", errCode);
    }
    else
        (void)but_DebounceInit(&debInit);

    return errCode;
}

#ifdef DEBUG_EXT
/**< bonus function in debug mode. uses RTT to fake button events
 */
static void fakeButtonOverRTT()
{
    char segin[1];
    unsigned numOfBytes = SEGGER_RTT_Read(0, segin, sizeof(segin));
    if (numOfBytes == 1)
    {
        if (segin[0] == '1')            // simulate short click
            buttonEvents[MAIN_BUTTON] = BUTEVAL_CLICKSHORT;
        else if (segin[0] == '2')       // simulate long click
            buttonEvents[MAIN_BUTTON] = BUTEVAL_CLICKLONG;
        else if (segin[0] == '3')       // simulate 2s hold
            buttonEvents[MAIN_BUTTON] = BUTEVAL_HOLD2SEC;
        else if (segin[0] == '4')       // simulate 10s hold
            buttonEvents[MAIN_BUTTON] = BUTEVAL_HOLD10SEC;
        else if (segin[0] == '6')            // simulate short click
            buttonEvents[SECONDARY_BUTTON] = BUTEVAL_CLICKSHORT;
        else if (segin[0] == '7')       // simulate long click
            buttonEvents[SECONDARY_BUTTON] = BUTEVAL_CLICKLONG;
        else if (segin[0] == '8')       // simulate 2s hold
            buttonEvents[SECONDARY_BUTTON] = BUTEVAL_HOLD2SEC;
        else if (segin[0] == '9')       // simulate 10s hold
            buttonEvents[SECONDARY_BUTTON] = BUTEVAL_HOLD10SEC;

        else if (segin[0] == 'b')       // simulate brake event
            Drake_Brake();
        else if (segin[0] == 'l')       // simulate left event
            Drake_IndicLeft();
        else if (segin[0] == 'r')       // simulate right event
            Drake_IndicRight();
    }
}
#endif

static void enableLed(hmi_ledType_t color, bool enable)
{
    uint32_t ledPin = ledPins[color];

    if (enable)
    {
        nrf_gpio_cfg_output(ledPin);
        nrf_gpio_pin_set(ledPin);
    }
    else
        nrf_gpio_cfg_default(ledPin);
}

static void timerCallback(void *p_context)
{
    ledsEnabled = false;

    for (hmi_ledType_t i = 0; i < HMI_LT_CNT; i++)
    {
        enableLed(i, false);
    }
}

static void resetTimeout()
{
    if (timeout == 0)
        return ;

    (void)app_timer_stop(drakeHmiTimer);
    ret_code_t errCode = app_timer_start(drakeHmiTimer, timeout, NULL);
    LOG_ERROR_CHECK("error %d starting drakeHmiTimer", errCode);

    if (ledsEnabled)
        return;

    for (hmi_ledType_t i = 0; i < HMI_LT_CNT; i++)
    {
        enableLed(i, ledsState[i]);
    }

    ledsEnabled = true;
}

/* Public functions ----------------------------------------------------------*/
ret_code_t Drake_Hmi_Init(uint32_t mainButton, uint32_t secondaryButton, uint32_t ledsTimeout)
{
    ret_code_t errCode;

    ledsInit();
    errCode = buttonInit(mainButton, secondaryButton);
    if (errCode != NRF_SUCCESS)
        return errCode;

    ledsEnabled = true;
    timeout = ledsTimeout;
    if (ledsTimeout == 0)
        return NRF_SUCCESS;

    errCode = app_timer_create(&drakeHmiTimer, APP_TIMER_MODE_SINGLE_SHOT, &timerCallback);
    if (errCode != NRF_SUCCESS)
        return errCode;

    errCode = app_timer_start(drakeHmiTimer, timeout, NULL);
    if (errCode != NRF_SUCCESS)
        return errCode;

    return NRF_SUCCESS;
}

/*void Drake_Hmi_LED_Enable(bool enable)
{
    if (enable == ledsEnabled)
        return;

    for (hmi_ledType_t i = 0; i < HMI_LT_CNT; i++)
    {
        enableLed(i, enable ? ledsState[i] : false);
    }
}*/

#include "motor_drv.h"
//#include "position_sensor.h"

void Drake_Hmi_Execute(bool resetLedTimeout)
{
    if (resetLedTimeout)
        resetTimeout();
#ifdef DEBUG_EXT
    fakeButtonOverRTT();
#endif // DEBUG_EXT

    if (buttonEvents[MAIN_BUTTON] != BUTEVAL_NONE)
    {
        uint32_t errCode = NRF_SUCCESS;
        q8_t plungerPos;
        lm_scanState_t currentScanState = lm_GetScanningState();

        switch (buttonEvents[MAIN_BUTTON])
        {
        case BUTEVAL_CLICKSHORT:
            errCode = Drake_GetPlungerPosition(&plungerPos);
            if (errCode != NRF_SUCCESS)
                break;
            errCode = Drake_SetPlungerPosition(plungerPos ? 0 : 255, false);        // activate or deactivate plunger
            break;

        case BUTEVAL_HOLD2SEC:
            //plctrl_CalibrateSensor();
            if (currentScanState != LM_SCAN_SEARCH)
                errCode = hmi_RequestEvent(HMI_EVT_SEARCHREMOTE, NULL); // search for remote
            else
                errCode = hmi_RequestEvent(HMI_EVT_DELETEBONDS, NULL);  // or delete bonds if already searching
            break;

        case BUTEVAL_HOLD10SEC:
            errCode = hmi_RequestEvent(HMI_EVT_FACTORYRESET, NULL);
            break;
        default:
            break;
        }

        resetTimeout();

        buttonEvents[MAIN_BUTTON] = BUTEVAL_NONE;

        LOG_WARNING_CHECK("[BRD:] error %d on main button event", errCode);
    }

    if (buttonEvents[SECONDARY_BUTTON] != BUTEVAL_NONE)
    {
        ret_code_t errCode = NRF_SUCCESS;

        switch (buttonEvents[SECONDARY_BUTTON])
        {
        case BUTEVAL_CLICKSHORT:
            errCode = hmi_RequestEvent(HMI_EVT_NEXTMODE, NULL);
            break;
        case BUTEVAL_CLICKLONG:
            errCode = hmi_RequestEvent(HMI_EVT_NEXTGROUP, NULL);
            break;
        case BUTEVAL_HOLD2SEC:
            errCode = hmi_RequestEvent(HMI_EVT_MODEPREF, NULL);
            break;
        case BUTEVAL_HOLD10SEC:
            errCode = hmi_RequestEvent(HMI_EVT_MODESOS, NULL);
            break;
        default:
            break;
        }

        resetTimeout();

        buttonEvents[SECONDARY_BUTTON] = BUTEVAL_NONE;

        LOG_WARNING_CHECK("[BRD:] error %d on secondary button event", errCode);
    }
}

void brd_EnableLed(hmi_ledType_t color, bool enable)
{
    if (color >= HMI_LT_CNT)
        return;

    ledsState[color] = enable;

    if (!ledsEnabled)
        return;

    enableLed(color, enable);

    /*uint32_t ledPin;

    if (color == HMI_LT_RED)
        ledPin = LED_RED;
    else if (color == HMI_LT_GREEN)
        ledPin = LED_GREEN;
    else if (color == HMI_LT_BLUE)
        ledPin = LED_BLUE;
    else
        return;

    if (enable)
    {
        nrf_gpio_cfg_output(ledPin);
        nrf_gpio_pin_set(ledPin);
    }
    else
        nrf_gpio_cfg_default(ledPin);*/
}

#undef  NRF_LOG_LEVEL

/**END OF FILE*****************************************************************/
