/**
  ******************************************************************************
  * @file    KD2_curr.c
  * @author  Thomas Reisnecker
  * @brief   tbd.
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
#include "nrfx_pwm.h"
#include "app_util_platform.h"
#include "nrf_log.h"

#include "KD2_curr.h"
#include "regulator.h"
#include "filter.h"

/* External variables --------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/

/* Configuration -------------------------------------------------------------*/
// use (pinNo) if pin is non inverted (idle state 0)
// use (pinNo | NRFX_PWM_PIN_INVERTED) if pin is inverted (idle state 1)
#define PWM_CH0_PIN     19
#define PWM_CH1_PIN     NRFX_PWM_PIN_NOT_USED

#define DITHERING_BITS  2           // enhance pwm resolution with additional 2bit trough dithering

#define MIN_REG_LIMIT   (4 << 8)    // minimal possible regulation limit, below this limit the
                                    // quiescent current of the current measurement circuit results in flickering

/* Private defines -----------------------------------------------------------*/
#if (PWM_CH0_PIN != NRFX_PWM_PIN_NOT_USED) && (PWM_CH0_PIN != NRFX_PWM_PIN_NOT_USED)
#define NUM_OF_CHANNELS 2
#elif (PWM_CH0_PIN == NRFX_PWM_PIN_NOT_USED) && (PWM_CH0_PIN == NRFX_PWM_PIN_NOT_USED)
#error "You have to use at least one channel"
#else
#define NUM_OF_CHANNELS 1
#endif

#define DITH_SIZE       (1<<DITHERING_BITS)
#define DITH_MASK       (DITH_SIZE-1)

/* Private variables ---------------------------------------------------------*/
static KD2_powerMode_t          powerMode;
static nrfx_pwm_t               pwmInst = NRFX_PWM_INSTANCE(1);
static nrf_pwm_values_grouped_t pwmDCs[2][DITH_SIZE];
static KD2_target_t             targets[NUM_OF_CHANNELS];
static reg_piInstance_t         regulators[NUM_OF_CHANNELS];
FIL_MOVAVG_DEF(filter, 2);

/* Private read only variables -----------------------------------------------*/
#if DITHERING_BITS == 1
static uint16_t const ditheringTable[DITH_SIZE][DITH_SIZE] =
{
    {0x0, 0x0},
    {0x0, 0x1},
};
#elif DITHERING_BITS == 2
static uint16_t const ditheringTable[DITH_SIZE][DITH_SIZE] =
{
    {0x0, 0x0, 0x0, 0x0},
    {0x0, 0x0, 0x0, 0x1},
    {0x0, 0x1, 0x0, 0x1},
    {0x0, 0x1, 0x1, 0x1},
};
#endif // if

static nrfx_pwm_config_t const pwmConfig =
{
    .output_pins =
    {
        PWM_CH0_PIN,
        PWM_CH1_PIN,
        NRFX_PWM_PIN_NOT_USED,
        NRFX_PWM_PIN_NOT_USED
    },
    .irq_priority = APP_IRQ_PRIORITY_LOW,
    .base_clock = NRF_PWM_CLK_16MHz,
    .count_mode = NRF_PWM_MODE_UP,
    .top_value = 64,
    .load_mode = NRF_PWM_LOAD_GROUPED,
    .step_mode = NRF_PWM_STEP_AUTO,
};

/* Private functions ---------------------------------------------------------*/
/** @brief function to override the gpio output drive settings
 */
static void initGpio()
{
    uint32_t pin;

    for (uint_fast8_t i = 0; i < ARRAY_SIZE(pwmConfig.output_pins); i++)
    {
        if (pwmConfig.output_pins[i] == NRFX_PWM_PIN_NOT_USED)
            continue;

        pin = pwmConfig.output_pins[i] & (~NRFX_PWM_PIN_INVERTED);
        nrf_gpio_cfg(pin, NRF_GPIO_PIN_DIR_OUTPUT,
                     NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL,
                     NRF_GPIO_PIN_H0H1, NRF_GPIO_PIN_NOSENSE);
    }
}

/** @brief function to update the duty cyle
 * @param pVaules  the DC values to use
 */
static void updateDutyCycle(nrf_pwm_values_grouped_t const* pValues, bool start)
{
    nrf_pwm_sequence_t pwmSeq =
    {
        .values.p_grouped = pValues,
        .length          = DITH_SIZE,
        .repeats         = 0,
        .end_delay       = 0
    };

    // only update value if already running or start is requested
    /// TODO: maybe there is a possible race condition if the actual stop
    /// happens between these two function calls, but a period is only 256
    /// clocks long and the program sequence between adc event and setting
    /// the new dutycycle will probably need more time
    if (!nrfx_pwm_is_stopped(&pwmInst) || start)
        (void)nrfx_pwm_simple_playback(&pwmInst, &pwmSeq, 1, NRFX_PWM_FLAG_LOOP);
}

/** @brief function to set a new duty cycle buffer
 *
 * @param dutyCycleChN the desired duty cycle including dithering bits, use 0 for unused/disabled channels
 * @return pointer to the buffer
 */
static nrf_pwm_values_grouped_t const* setDutyCycle(uint16_t dutyCycleCh0, uint16_t dutyCycleCh1)
{
    static uint8_t index;   // the buffer index, 0 or 1

    uint16_t baseDC0, baseDC1;
    uint16_t row0, row1;

    // update buffer index
    index ^= 1;

    // limit duty cycle to valid values
    if (dutyCycleCh0 > (pwmConfig.top_value << DITHERING_BITS))
        dutyCycleCh0 = pwmConfig.top_value << DITHERING_BITS;
    if (dutyCycleCh1 > (pwmConfig.top_value << DITHERING_BITS))
        dutyCycleCh1 = pwmConfig.top_value << DITHERING_BITS;

    // convert o base DC and dithering table index
    baseDC0 = dutyCycleCh0 >> DITHERING_BITS;
    row0 = dutyCycleCh0 & DITH_MASK;
    baseDC1 = dutyCycleCh1 >> DITHERING_BITS;
    row1 = dutyCycleCh1 & DITH_MASK;

    // set polarity
    if (!(pwmConfig.output_pins[0] & NRFX_PWM_PIN_INVERTED))
        baseDC0 |= 0x8000;
    if (!(pwmConfig.output_pins[1] & NRFX_PWM_PIN_INVERTED))
        baseDC1 |= 0x8000;

    // set up buffer
    for (uint_fast8_t i = 0; i < DITH_SIZE; i++)
    {
        pwmDCs[index][i].group_0 = baseDC0 + ditheringTable[row0][i];
        pwmDCs[index][i].group_1 = baseDC1 + ditheringTable[row1][i];
    }

    return &pwmDCs[index][0];
}

static void initRegulators()
{
    reg_piInit_t piInit =
    {
        .gainProp      = 268435456,   // 0.125
        .gainInt       = 134217728,   // 0.0625
        .intUpperLimit = 1048576,     // 65535 / 0.0625
        .intLowerLimit = 0,
    };

    for (uint_fast8_t i = 0; i < ARRAY_SIZE(regulators); i++)
    {
        reg_PiInit(&regulators[i], &piInit);
    }
}

/* Public functions ----------------------------------------------------------*/
ret_code_t KD2_Curr_Init()
{
    ret_code_t errCode;

    errCode = nrfx_pwm_init(&pwmInst, &pwmConfig, NULL);

    // init function set output pins to standard drive, let's reconfigure to use high drive
    initGpio();

    initRegulators();

    return errCode;
}

ret_code_t KD2_Curr_SetPowerMode(KD2_powerMode_t newMode)
{
    nrf_pwm_values_grouped_t const* pDcBuffer;

    if (newMode == powerMode)
        return NRF_SUCCESS;

    if (newMode == KD2_PWR_ON)
    {
        fil_MovAvgReset(&filter, 0);        // reset filter
        reg_PiReset(&regulators[0]);        // reset regulator
        pDcBuffer = setDutyCycle(0, 0);     // reset dutycycle to start with zero
        updateDutyCycle(pDcBuffer, true);   // this is the only place to start the duty cycle
    }
    else if (powerMode == KD2_PWR_ON)
    {
        (void)nrfx_pwm_stop(&pwmInst, false);
    }

    powerMode = newMode;

    return NRF_SUCCESS;
}

ret_code_t KD2_Curr_SetCurrent(KD2_target_t desired)
{
    if (desired > KD2_TARGET_MAX || desired < KD2_TARGET_MIN)
        return NRF_ERROR_INVALID_PARAM;

    if (powerMode != KD2_PWR_ON)
        return NRF_ERROR_INVALID_STATE;

    if (desired != 0 && desired < MIN_REG_LIMIT)
        targets[0] = MIN_REG_LIMIT;
    else
        targets[0] = desired;

    //NRF_LOG_INFO("target %d", desired >> 8);

    return NRF_SUCCESS;
}

void KD2_Curr_ReportCurrent(KD2_target_t actual)
{
    int32_t dc;
    nrf_pwm_values_grouped_t const* pDcBuffer;

    if (powerMode != KD2_PWR_ON)
        return;

    // filter current
    actual = fil_MovAvg(&filter, actual);

    // get new duty cylcle
    dc = reg_PiRegulate(&regulators[0], targets[0], actual);

    // limit dutycycle
    if (dc > (256 << 8))
        dc = 256 << 8;
    else if (dc < 0)
        dc = 0;

    // set DC buffer
    pDcBuffer = setDutyCycle(dc >> 8, 0);

    // and finally update duty cycle, but don't restart for the case it has been stopped through ppi
    updateDutyCycle(pDcBuffer, false);
}

uint32_t KD2_Curr_GetAbortTask()
{
    return nrfx_pwm_task_address_get(&pwmInst, NRF_PWM_TASK_STOP);
}

/**END OF FILE*****************************************************************/
