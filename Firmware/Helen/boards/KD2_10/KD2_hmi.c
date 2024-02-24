/**
  ******************************************************************************
  * @file    KD2_hmi.c
  * @author  Thomas Reisnecker
  * @brief   button and status led handling for KD2 Hardware 1.0
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

#include "KD2_hmi.h"
#include "mode_management.h"
#include "link_management.h"
#include "board.h"
#include "button.h"
#include "debug.h"

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
#define LED_RED                 18
#define LED_BLUE                20

#define MAIN_BUTTON             0
#define SECONDARY_BUTTON        1

//#define BUTTON                  21
//#define BUTTON_COM              27    // maybe in future?

#define BUTTONPRESS_LONG        MSEC_TO_UNITS(500, UNIT_10_MS)
#define BUTTONPRESS_HOLD2SEC    MSEC_TO_UNITS(2000, UNIT_10_MS)
#define BUTTONPRESS_HOLD10SEC   MSEC_TO_UNITS(10000, UNIT_10_MS)
#define BUTTONPRESS_HOLDABORT   MSEC_TO_UNITS(30000, UNIT_10_MS)

#undef  NRF_LOG_LEVEL
#define NRF_LOG_LEVEL 2         // override debug level to only show error and warnings

/* Private variables ---------------------------------------------------------*/
static uint8_t            buttonIds[2] = {BUTTON_EVALUATE_CNT, BUTTON_EVALUATE_CNT};
static buttonEvalEvents_t buttonEvents[2];  // the last occured button event

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
    // configure LED pins and clear LEDs
    nrf_gpio_cfg_output(LED_RED);
    nrf_gpio_pin_clear(LED_RED);
    nrf_gpio_cfg_output(LED_BLUE);
    nrf_gpio_pin_clear(LED_BLUE);
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
    }
}
#endif

/* Public functions ----------------------------------------------------------*/
ret_code_t KD2_Hmi_Init(uint32_t mainButton, uint32_t secondaryButton)
{
    ret_code_t errCode;

    ledsInit();
    errCode = buttonInit(mainButton, secondaryButton);
    if (errCode != NRF_SUCCESS)
        return errCode;

    return NRF_SUCCESS;
}

void KD2_Hmi_Execute()
{
#ifdef DEBUG_EXT
    fakeButtonOverRTT();
#endif // DEBUG_EXT

    if (buttonEvents[MAIN_BUTTON] != BUTEVAL_NONE)
    {
        uint32_t errCode = NRF_SUCCESS;
        bool isOff = mm_GetCurrentMode() == MM_MODE_OFF;
        //uint8_t currentMode = mm_GetCurrentMode();
        lm_scanState_t currentScanState = lm_GetScanningState();

        switch (buttonEvents[MAIN_BUTTON])
        {
        case BUTEVAL_CLICKSHORT:
            errCode = hmi_RequestEvent(HMI_EVT_NEXTMODE, NULL);
            break;
        case BUTEVAL_CLICKLONG:
            errCode = hmi_RequestEvent(HMI_EVT_NEXTGROUP, NULL);
            break;
        case BUTEVAL_HOLD2SEC:
            if (!isOff)
                errCode = hmi_RequestEvent(HMI_EVT_MODEPREF, NULL);     // preferred mode if on
            else if (currentScanState != LM_SCAN_SEARCH)
                errCode = hmi_RequestEvent(HMI_EVT_SEARCHREMOTE, NULL); // search if off
            else
                errCode = hmi_RequestEvent(HMI_EVT_DELETEBONDS, NULL);  // or delete bonds if already searching
            break;
        case BUTEVAL_HOLD10SEC:
            if (isOff)
                errCode = hmi_RequestEvent(HMI_EVT_FACTORYRESET, NULL);
            else
                errCode = hmi_RequestEvent(HMI_EVT_MODESOS, NULL);
            break;
        default:
            break;
        }

        buttonEvents[MAIN_BUTTON] = BUTEVAL_NONE;

        LOG_WARNING_CHECK("[BRD:] request event error %d", errCode);
    }

    if (buttonEvents[SECONDARY_BUTTON] != BUTEVAL_NONE)
    {
        uint32_t errCode = NRF_SUCCESS;

        switch (buttonEvents[SECONDARY_BUTTON])
        {
        case BUTEVAL_CLICKSHORT:
            errCode = hmi_RequestEvent(HMI_EVT_NEXTMODE, NULL);
            break;
        case BUTEVAL_CLICKLONG:
            errCode = hmi_RequestEvent(HMI_EVT_NEXTGROUP, NULL);
            break;
        case BUTEVAL_HOLD2SEC:
            errCode = hmi_RequestEvent(HMI_EVT_MODEPREF, NULL);     // preferred mode
            break;
        default:
            break;
        }

        buttonEvents[SECONDARY_BUTTON] = BUTEVAL_NONE;

        LOG_WARNING_CHECK("[BRD:] request event error %d", errCode);
    }
}

void brd_EnableLed(hmi_ledType_t color, bool enable)
{
    uint32_t ledPin;

    if (color == HMI_LT_RED)
        ledPin = LED_RED;
    else if (color == HMI_LT_BLUE)
        ledPin = LED_BLUE;
    else
        return;

    if (enable)
        nrf_gpio_pin_set(ledPin);
    else
        nrf_gpio_pin_clear(ledPin);
}

#undef  NRF_LOG_LEVEL

/**END OF FILE*****************************************************************/
