/**
  ******************************************************************************
  * @file    motor_drv.c
  * @author  Thomas Reisnecker
  * @brief   motor driver for Drake Hardware 1.0
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
#include "motor_drv.h"
#include "nrfx_pwm.h"

/* External variables --------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/

/* Private defines -----------------------------------------------------------*/
#define FORWARD_PIN 18
#define REVERSE_PIN 17

#define OFF         0
#define ON          1

/* Private variables ---------------------------------------------------------*/
static nrfx_pwm_t pwmInst = NRFX_PWM_INSTANCE(0);
static nrf_pwm_values_individual_t pwmDCs;

/* Private read only variables -----------------------------------------------*/
static nrfx_pwm_config_t const pwmConfig =
{
    .output_pins =
    {
        FORWARD_PIN,
        REVERSE_PIN,
        NRFX_PWM_PIN_NOT_USED,
        NRFX_PWM_PIN_NOT_USED
    },
    .irq_priority = APP_IRQ_PRIORITY_LOW,
    .base_clock = NRF_PWM_CLK_1MHz,
    .count_mode = NRF_PWM_MODE_UP,
    .top_value = 256,
    .load_mode = NRF_PWM_LOAD_INDIVIDUAL,
    .step_mode = NRF_PWM_STEP_AUTO,
};

static nrf_pwm_sequence_t const pwmSeq =
{
    .values.p_individual = &pwmDCs,
    .length          = NRF_PWM_VALUES_LENGTH(pwmDCs),
    .repeats         = 0,
    .end_delay       = 0
};

static uint32_t const outputPins[2][4] =
{
    {NRF_PWM_PIN_NOT_CONNECTED, NRF_PWM_PIN_NOT_CONNECTED, NRF_PWM_PIN_NOT_CONNECTED, NRF_PWM_PIN_NOT_CONNECTED},
    {FORWARD_PIN, REVERSE_PIN, NRF_PWM_PIN_NOT_CONNECTED, NRF_PWM_PIN_NOT_CONNECTED}
};

/* Private functions ---------------------------------------------------------*/

/* Public functions ----------------------------------------------------------*/
ret_code_t mtdrv_Init(void)
{
    return nrfx_pwm_init(&pwmInst, &pwmConfig, NULL);
}

void mtdrv_Enable(mtdrv_mode_t mode, uint16_t dutyCycle)
{
    switch (mode)
    {
    case MTDRV_OFF:
        // disconnect pins to switch off immediatelly
        nrf_pwm_pins_set(pwmInst.p_registers, (uint32_t*)outputPins[OFF]);
        pwmDCs.channel_0 = 0;
        pwmDCs.channel_1 = 0;
        (void)nrfx_pwm_stop(&pwmInst, false);
        return;

    case MTDRV_FORWARD:
        pwmDCs.channel_0 = dutyCycle;
        pwmDCs.channel_1 = 0;
        break;

    case MTDRV_REVERSE:
        pwmDCs.channel_0 = 0;
        pwmDCs.channel_1 = dutyCycle;
        break;

    case MTDRV_BRAKE:
        pwmDCs.channel_0 = dutyCycle;
        pwmDCs.channel_1 = dutyCycle;
        break;
    }

    nrf_pwm_pins_set(pwmInst.p_registers, (uint32_t*)outputPins[ON]);
    (void)nrfx_pwm_simple_playback(&pwmInst, &pwmSeq, 1, 0);
}

/**END OF FILE*****************************************************************/
