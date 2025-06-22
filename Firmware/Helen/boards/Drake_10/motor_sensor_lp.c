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
//#include "nrfx_pwm.h"
#include "nrfx_lpcomp.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "watchdog.h"

/* External variables --------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/

/* Private defines -----------------------------------------------------------*/
#define DRIVE_PIN   19
#define SENSE_PIN   5
#define SENSE_COMP  NRF_LPCOMP_INPUT_3

//#define PSENS_DIR_OFF   (MTSENS_DIR_REVERSE + 1)

/* Private variables ---------------------------------------------------------*/
static mtSens_eventHandler_t handler;
static bool isPresent;
static int16_t counterStep;
static int16_t counter;

/* Private read only variables -----------------------------------------------*/
/*static nrf_comp_th_t const thresholdsDefauts =
{
    .th_up = 2,
    .th_down = 0,
};*/

/* Private functions ---------------------------------------------------------*/
/*static bool areThresholdsValid(nrf_comp_th_t const* pTh)
{
    if (pTh->th_up <= pTh->th_down)
        return false;
    if (pTh->th_up >= 64 || pTh->th_down >= 64)
        return false;

    return true;
}*/

static void compEventHandler(nrf_lpcomp_event_t event)
{
    if (event == NRF_LPCOMP_EVENT_CROSS)
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

static nrf_lpcomp_ref_t getRefsel(uint8_t step)
{
    if (step >= 15)
        return NRF_LPCOMP_REF_SUPPLY_1_8;   // default value for invalid step

    return 8 - ((step % 2) * 8) + (step / 2);
}

static ret_code_t initComp(mtSens_thresholds_t const* pTh)
{
    ret_code_t errCode;

    nrfx_lpcomp_config_t config =
    {
        .hal.detection = NRF_LPCOMP_DETECT_CROSS,
        .hal.hyst = LPCOMP_HYST_HYST_Hyst50mV,
        .input = SENSE_COMP,
        .interrupt_priority = APP_IRQ_PRIORITY_LOW
    };

    uint8_t ref = (pTh->upper + pTh->lower) >> 5;

    config.hal.reference = getRefsel(ref);

    errCode = nrfx_lpcomp_init(&config, compEventHandler);

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
    nrf_delay_ms(10);      /// TODO: not working with shorter delay, but this is way too high -> check hardware
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

    errCode = initComp(pInit->pThresholds);

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
        nrfx_lpcomp_enable();
    else
        nrfx_lpcomp_disable();

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

    uint8_t lastAllHigh = 0, firstAllLow = 0;
    nrf_lpcomp_config_t config =
    {
        .detection = NRF_LPCOMP_DETECT_CROSS,
        .hyst = LPCOMP_HYST_HYST_Hyst50mV
    };

    nrfx_lpcomp_uninit();
    enableIR(true);

    // stepping through comparator values from top to bottom
    for (int8_t i = 0; i < 15; i++)
    {
        config.reference = getRefsel(i);

        nrf_lpcomp_configure(&config);
        nrf_lpcomp_input_select(SENSE_COMP);
        nrf_lpcomp_enable();
        nrf_lpcomp_task_trigger(NRF_LPCOMP_TASK_START);
        while(!nrf_lpcomp_event_check(NRF_LPCOMP_EVENT_READY));

        uint8_t j;
        uint32_t oldResult;

        SEGGER_RTT_printf(0, "\r\nstep %d: ", config.reference);

        for (j = 0; j < 16; j++)
        {
            nrf_lpcomp_task_trigger(NRF_LPCOMP_TASK_SAMPLE);
            uint32_t result = nrf_lpcomp_result_get();

            SEGGER_RTT_printf(0, "%d, ", result);

            if (j != 0 && result != oldResult)
                break;  // change detected, further checks can be skipped

            oldResult = result;

            nrf_delay_ms(1);
        }

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

    pTh->upper = firstAllLow << 4;
    pTh->lower = lastAllHigh << 4;

    initComp(pTh);

    return NRF_SUCCESS;
}

/**END OF FILE*****************************************************************/
