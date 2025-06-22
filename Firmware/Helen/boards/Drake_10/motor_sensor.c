/**
  ******************************************************************************
  * @file    motor_sensor.c
  * @author  Thomas Reisnecker
  * @brief   motor sensor for Drake Hardware 1.0
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
#include "motor_sensor.h"
#include "nrfx_comp.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "watchdog.h"

/* External variables --------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/

/* Private defines -----------------------------------------------------------*/
#define DRIVE_PIN   19
#define SENSE_PIN   5
#define SENSE_COMP  NRF_COMP_INPUT_3

//#define PSENS_DIR_OFF   (MTSENS_DIR_REVERSE + 1)

/* Private variables ---------------------------------------------------------*/
static mtSens_eventHandler_t handler;
static bool isPresent;
static int16_t counterStep;
static int16_t counter;

/* Private read only variables -----------------------------------------------*/
static nrf_comp_th_t const thresholdsDefauts =
{
    .th_up = 2,
    .th_down = 0,
};

/* Private functions ---------------------------------------------------------*/
static void compEventHandler(nrf_comp_event_t event)
{
    if (event == NRF_COMP_EVENT_UP || event == NRF_COMP_EVENT_DOWN)
    {
        counter += counterStep;

        if (handler != NULL)
        {
            mtSens_evt_t evt =
            {
                .type = MTSENS_EVTTYPE_POS_CHANGE,
                .position = counter
            };
            handler(&evt);
        }
    }
}

static void calculateThresholds(mtSens_thresholds_t const* pThIn, nrf_comp_th_t* pThOut)
{
    if (pThIn->lower == 0 && pThIn->upper == 0)
    {
        *pThOut = thresholdsDefauts;
        return;
    }

    uint8_t step = (pThIn->lower + pThIn->upper) >> 3;

    pThOut->th_up = step + 1;
    pThOut->th_down = step - 1;
}

static ret_code_t initComp(mtSens_thresholds_t const* pTh)
{
    ret_code_t errCode;

    nrfx_comp_config_t config =
    {
        .reference = NRF_COMP_REF_VDD,
        .main_mode = NRF_COMP_MAIN_MODE_SE,
        .speed_mode = NRF_COMP_SP_MODE_Normal,
        .isource = NRF_COMP_ISOURCE_Off,
        .input = SENSE_COMP,
        .interrupt_priority = APP_IRQ_PRIORITY_LOW
    };

    calculateThresholds(pTh, &config.threshold);

    errCode = nrfx_comp_init(&config, compEventHandler);

    return errCode;
}

static void enableIR(bool enable)
{
    if (enable)
    {
        //nrf_gpio_cfg_output(DRIVE_PIN);
        nrf_gpio_cfg(DRIVE_PIN,
                 NRF_GPIO_PIN_DIR_OUTPUT,
                 NRF_GPIO_PIN_INPUT_DISCONNECT,
                 NRF_GPIO_PIN_NOPULL,
                 NRF_GPIO_PIN_S0S1,
                 NRF_GPIO_PIN_NOSENSE);
        nrf_gpio_pin_set(DRIVE_PIN);
    }
    else
        nrf_gpio_cfg_default(DRIVE_PIN);
}

/*static void enablePullup(bool enable)
{
    nrf_gpio_cfg(SENSE_PIN,
                 NRF_GPIO_PIN_DIR_INPUT,
                 NRF_GPIO_PIN_INPUT_DISCONNECT,
                 enable ? NRF_GPIO_PIN_PULLUP : NRF_GPIO_PIN_NOPULL,
                 NRF_GPIO_PIN_S0S1,
                 NRF_GPIO_PIN_NOSENSE);
}*/

/** @brief helper to check if sensor is present by checking is the pull
 *         up resistor on the sense pin is present
 */
static bool isSensorPresent()
{
    // set drive pin to low level and sense pin to high level
    nrf_gpio_cfg_output(DRIVE_PIN);
    nrf_gpio_pin_clear(DRIVE_PIN);
    nrf_gpio_cfg_output(SENSE_PIN);
    nrf_gpio_pin_set(SENSE_PIN);
    nrf_delay_ms(1);

    // set sense pin to input
    nrf_gpio_cfg_input(SENSE_PIN, NRF_GPIO_PIN_NOPULL);
    nrf_delay_ms(10);
    isPresent = !(bool)nrf_gpio_pin_read(SENSE_PIN); // if no sensor connected, pin should still have high level, otherwise pin level should be low

    // restore default state
    nrf_gpio_cfg_default(DRIVE_PIN);
    nrf_gpio_cfg_default(SENSE_PIN);

    return isPresent;
}

/* Public functions ----------------------------------------------------------*/
ret_code_t mtSens_Init(mtSens_Init_t const* pInit)
{
    ret_code_t errCode;

    if (pInit == NULL || pInit->evtHandler == NULL)
        return NRF_ERROR_NULL;

    if (isSensorPresent() == false)
        return NRF_ERROR_NOT_FOUND;

    errCode = initComp(pInit->pThresholds);  // error?

    handler = pInit->evtHandler;

    return errCode;
}

ret_code_t mtSens_Enable(bool enable)
{
    if (handler == NULL)
        return NRF_ERROR_INVALID_STATE;

    if (isPresent == false)
        return NRF_ERROR_NOT_FOUND;

    enableIR(enable);

    if (enable)
        nrfx_comp_start(NRFX_COMP_EVT_EN_UP_MASK | NRFX_COMP_EVT_EN_DOWN_MASK, 0);
    else
        nrfx_comp_stop();

    return NRF_SUCCESS;
}

ret_code_t mtSens_SetDirection(mtSens_direction_t direction)
{
    if (handler == NULL)
        return NRF_ERROR_INVALID_STATE;

    if (isPresent == false)
        return NRF_ERROR_NOT_FOUND;

    if (direction == MTSENS_DIR_FORWARD)
        counterStep = 1;
    else if (direction == MTSENS_DIR_REVERSE)
        counterStep = -1;
    else
        return NRF_ERROR_INVALID_PARAM;

    return NRF_SUCCESS;
}

void mtSens_ResetCounter()
{
    counter = 0;
}
#include "SEGGER_RTT.h"
ret_code_t mtSens_Calibrate(mtSens_thresholds_t* pTh)
{
    if (handler == NULL)
        return NRF_ERROR_INVALID_STATE;

    if (isPresent == false)
        return NRF_ERROR_NOT_FOUND;

    nrf_comp_th_t thresholds = {0};
    uint8_t lastAllHigh = 0, firstAllLow = 0;

    enableIR(true);

    // stepping through comparator values from top to bottom
    for (int8_t i = 0; i < 64; i++)
    {
        thresholds.th_up = i;
        thresholds.th_down = i;
        nrf_comp_th_set(thresholds);

        nrfx_comp_start(0, 0);

        uint8_t j;
        uint32_t oldResult;

        SEGGER_RTT_printf(0, "\r\nstep %d: ", i);

        for (j = 0; j < 16; j++)
        {
            uint32_t result = nrfx_comp_sample();

            SEGGER_RTT_printf(0, "%d, ", result);

            if (j != 0 && result != oldResult)
                break;  // change detected, further checks can be skipped

            oldResult = result;
            nrf_delay_ms(1);
        }
        nrfx_comp_stop();

        if (oldResult != 0 && j == 16)  // saving last threshold above active region
            lastAllHigh = i;

        if (oldResult == 0 && j == 16)  // saving first threshold below active region
        {
            firstAllLow = i;
            break;          // no need to check further
        }

        wdg_Feed();     // feed watchdog to prevent reset
    }

    enableIR(false);

    NRF_LOG_INFO("thresholds %d and %d", firstAllLow, lastAllHigh);

    pTh->lower = firstAllLow << 2;
    pTh->upper = lastAllHigh << 2;

    calculateThresholds(pTh, &thresholds);
    nrf_comp_th_set(thresholds);

    return NRF_SUCCESS;
}

/**END OF FILE*****************************************************************/
