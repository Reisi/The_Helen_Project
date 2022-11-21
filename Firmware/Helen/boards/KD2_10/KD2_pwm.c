/**
  ******************************************************************************
  * @file    KD2_pwm.c
  * @author  Thomas Reisnecker
  * @brief   tbd.
  ******************************************************************************
  */

/// TODO: test dual/quad channel pwm and set polarity of channels to optimize
///       current load

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
#include "nrfx_pwm.h"
#include "app_util_platform.h"

#include "KD2_pwm.h"

/* External variables --------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/

/* Private defines -----------------------------------------------------------*/
// 100 is the maximum value we get from the settings
// multiplying it with 256 gives more than enough
// room for smooth non visible steps due to the
// limiter and will result in a frequency of 312.5Hz
#define PWM_CONFIG_DEFAULT                  \
{                                           \
    .irq_priority = APP_IRQ_PRIORITY_LOW,   \
    .base_clock = NRF_PWM_CLK_8MHz,         \
    .count_mode = NRF_PWM_MODE_UP,          \
    .top_value = (100 << 8),                \
    .load_mode = NRF_PWM_LOAD_INDIVIDUAL,   \
    .step_mode = NRF_PWM_STEP_AUTO,         \
}

/* Private variables ---------------------------------------------------------*/
static KD2_powerMode_t powerMode;
static nrfx_pwm_t pwmInst = NRFX_PWM_INSTANCE(0);
static nrf_pwm_values_individual_t pwmDCs;
static uint8_t outputPins[] = {NRFX_PWM_PIN_NOT_USED, NRFX_PWM_PIN_NOT_USED,
                               NRFX_PWM_PIN_NOT_USED, NRFX_PWM_PIN_NOT_USED};

/* Private read only variables -----------------------------------------------*/
static nrf_pwm_sequence_t const pwmSeq =
{
    .values.p_individual = &pwmDCs,
    .length          = NRF_PWM_VALUES_LENGTH(pwmDCs),
    .repeats         = 0,
    .end_delay       = 0
};

/* Private functions ---------------------------------------------------------*/
static void initGpio(KD2_Pwm_init_t const* pInit)
{
    for (uint_fast8_t i = 0; i < KD2_PWM_CHANNEL_CNT; i++)
    {
        // ignore unused pins
        if (pInit->channels[i].pinNo == KD2_PWM_PIN_NOT_USED)
            continue;

        if (pInit->channels[i].type == KD2_PWM_PUSHPULL ||
            pInit->channels[i].type == KD2_PWM_PUSHPULLINVERTED)
        {
            nrf_gpio_cfg(pInit->channels[i].pinNo, NRF_GPIO_PIN_DIR_OUTPUT,
                     NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL,
                     NRF_GPIO_PIN_H0H1, NRF_GPIO_PIN_NOSENSE);
        }

        else if (pInit->channels[i].type == KD2_PWM_OPENDRAIN ||
            pInit->channels[i].type == KD2_PWM_OPENDRAININVERTED)
        {
            nrf_gpio_cfg(pInit->channels[i].pinNo, NRF_GPIO_PIN_DIR_OUTPUT,
                     NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL,
                     NRF_GPIO_PIN_H0D1, NRF_GPIO_PIN_NOSENSE);
        }
    }
}

static void updateDutyCycle()
{
    uint16_t const* raw = (uint16_t const*)&pwmDCs;

    for (uint_fast8_t i = 0; i < NRF_PWM_CHANNEL_COUNT; i++)
    {
        if (raw[i] & 0x7FFF)   // start PWM if at least one output is enabled, MSB is polarity
        {
            (void)nrfx_pwm_simple_playback(&pwmInst, &pwmSeq, 1, 0);
            return;
        }
    }
    (void)nrfx_pwm_stop(&pwmInst, false);
}

static void setDutyCycle(uint8_t channel, uint16_t dutyCycle)
{
    uint16_t* raw = (uint16_t*)&pwmDCs;
    if (outputPins[channel] != NRFX_PWM_PIN_NOT_USED)
    {
        if (outputPins[channel] & NRFX_PWM_PIN_INVERTED)
            raw[channel] = dutyCycle;
        else
            raw[channel] = dutyCycle | 0x8000;
    }

    //updateDutyCycle();
}

/* Public functions ----------------------------------------------------------*/
ret_code_t KD2_Pwm_Init(KD2_Pwm_init_t const* pInit)
{
    //return NRF_SUCCESS;

    if (pInit == NULL)
        return NRF_ERROR_NULL;

    ret_code_t errCode;
    nrfx_pwm_config_t pwmConfig = PWM_CONFIG_DEFAULT;

    for (uint_fast8_t i = 0; i < KD2_PWM_CHANNEL_CNT; i++)
    {
        if (pInit->channels[i].pinNo == KD2_PWM_PIN_NOT_USED)
            outputPins[i] = NRFX_PWM_PIN_NOT_USED;
        else
        {
            outputPins[i] = pInit->channels[i].pinNo;
            if (pInit->channels[i].type == KD2_PWM_PUSHPULLINVERTED ||
                pInit->channels[i].type == KD2_PWM_OPENDRAININVERTED)
                outputPins[i] |= NRFX_PWM_PIN_INVERTED;
        }
        pwmConfig.output_pins[i] = outputPins[i];
    }

    errCode = nrfx_pwm_init(&pwmInst, &pwmConfig, NULL);

    initGpio(pInit);

    return errCode;
}

ret_code_t KD2_Pwm_SetPowerMode(KD2_powerMode_t newMode)
{
    if (newMode == powerMode)
        return NRF_SUCCESS;

    // leaving ON mode, shut down
    if (powerMode == KD2_PWR_ON)
    {
        (void)nrfx_pwm_stop(&pwmInst, false);
        pwmDCs.channel_0 = 0;
        pwmDCs.channel_1 = 0;
        pwmDCs.channel_2 = 0;
        pwmDCs.channel_3 = 0;
    }

    powerMode = newMode;

    return NRF_SUCCESS;
}

ret_code_t KD2_Pwm_SetDC(KD2_Pwm_dc_t const* pDC)
{
    if (pDC == NULL)
        return NRF_ERROR_NULL;

    if (powerMode != KD2_PWR_ON)
        return NRF_ERROR_INVALID_STATE;

    for (uint_fast8_t i = 0; i < KD2_PWM_CHANNEL_CNT; i++)
    {
        // ignore unused pins
        if (outputPins[i] == NRFX_PWM_PIN_NOT_USED)
            continue;

        if (pDC->channels[i] > KD2_TARGET_MAX || pDC->channels[i] < KD2_TARGET_MIN)
            return NRF_ERROR_INVALID_PARAM;
    }

    for (uint_fast8_t i = 0; i < KD2_PWM_CHANNEL_CNT; i++)
    {
        setDutyCycle(i, pDC->channels[i]);   // no conversion necessary
    }

    updateDutyCycle();

    return NRF_SUCCESS;
}

/**END OF FILE*****************************************************************/
