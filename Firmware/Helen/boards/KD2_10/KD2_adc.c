/**
  ******************************************************************************
  * @file    KD2_adc.c
  * @author  Thomas Reisnecker
  * @brief   adc module for for KD2 Hardware 1.0
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
//#include "nrf_rtc.h"
#include "nrfx_rtc.h"
#include "nrfx_ppi.h"
#include "nrfx_saadc.h"
#include "nrfx_temp.h"
#include "nrf_timer.h"

#include "KD2_adc.h"
#include "debug.h"

/* External variables --------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
    uint16_t prescale;
    q9_7_t lastResult;
} temperature_t;

typedef struct
{
    q4_12_t gainVolt;
    q4_12_t gainCurr;
    q9_7_t  offsetTemp;
} compensation_t;

/* Private macros ------------------------------------------------------------*/

/* Private defines -----------------------------------------------------------*/
#define COMP_DEFAULT                        \
{                                           \
    .gainVolt   = KD2_ADC_VOLTGAIN_DEFAULT, \
    .gainCurr   = KD2_ADC_CURRGAIN_DEFAULT, \
    .offsetTemp = KD2_ADC_TEMPOFF_DEFAULT   \
}
#define CURRGAIN_MIN        27034   // guaranties, that the current can be at least measured up to 3.3A

// desired sample rates
// the SR for temperature cannot be bigger than the SR for adc!
// timer is configured as 16bit timer, check if this is sufficient when changing
// on state sample rate
#define SR_ADC_IDLE         1
#define SR_ADC_ON           7692 //  (exactly fsw/32.5)
#define SR_TEMP_IDLE        1
#define SR_TEMP_ON          8

// the indices for the adc channel configuration structures
#define CHN_VOLTAGE         0
#define CHN_CURRENT         1
#define CHN_CNT_IDLE        1
#define CHN_CNT_ON          2
#define CHN_MASK_IDLE       (1<<CHN_VOLTAGE)
#define CHN_MASK_ON         ((1<<CHN_VOLTAGE)|(1<<CHN_CURRENT))

// the pins used for adc
#define AIN_VOLTAGE         NRF_SAADC_INPUT_AIN4
#define AIN_CURRENT         NRF_SAADC_INPUT_AIN5

// the rtc timer used in idle mode
#define RTC_FREQ            RTC_INPUT_FREQ          // no prescaler
#define RTC_COMP_VALUE      (RTC_FREQ/SR_ADC_IDLE)

// the timer used in on mode
#define TIMER_INST          NRF_TIMER1
#define TIMER_FREQ          NRF_TIMER_FREQ_1MHz
#define TIMER_COMP_VALUE    (1000000ul/SR_ADC_ON)

#define TEMP_PRESCALE_IDLE  (SR_ADC_IDLE/SR_TEMP_IDLE)
#define TEMP_PRESCALE_ON    (SR_ADC_ON/SR_TEMP_ON)


/* Private read only variables -----------------------------------------------*/
static nrfx_saadc_channel_t const adcChannels[] =
{
    {   // configuration for input voltage measurement
        .channel_config.resistor_p = NRF_SAADC_RESISTOR_DISABLED,
        .channel_config.resistor_n = NRF_SAADC_RESISTOR_DISABLED,
        .channel_config.gain       = NRF_SAADC_GAIN1,
        .channel_config.reference  = NRF_SAADC_REFERENCE_INTERNAL,
        .channel_config.acq_time   = NRF_SAADC_ACQTIME_10US,
        .channel_config.mode       = NRF_SAADC_MODE_SINGLE_ENDED,
        .channel_config.burst      = NRF_SAADC_BURST_DISABLED,
        .pin_p             = AIN_VOLTAGE,
        .pin_n             = NRF_SAADC_INPUT_DISABLED,
        .channel_index     = 0,
    },
    {   // configuration for led current measurement
        .channel_config.resistor_p = NRF_SAADC_RESISTOR_DISABLED,
        .channel_config.resistor_n = NRF_SAADC_RESISTOR_DISABLED,
        .channel_config.gain       = NRF_SAADC_GAIN1,
        .channel_config.reference  = NRF_SAADC_REFERENCE_INTERNAL,
        .channel_config.acq_time   = NRF_SAADC_ACQTIME_3US,  // low impedance, no need for long acquisition time
        .channel_config.mode       = NRF_SAADC_MODE_SINGLE_ENDED,
        .channel_config.burst      = NRF_SAADC_BURST_DISABLED,
        .pin_p             = AIN_CURRENT,
        .pin_n             = NRF_SAADC_INPUT_DISABLED,
        .channel_index     = 1,
    }
};

static nrfx_saadc_adv_config_t const adcAdvConfig =
{
    .oversampling      = NRF_SAADC_OVERSAMPLE_DISABLED,
    .burst             = NRF_SAADC_BURST_DISABLED,
    .internal_timer_cc = 0,
    .start_on_end      = false,
};

static nrfx_rtc_config_t const rtcConfig =
{
    .prescaler          = RTC_FREQ_TO_PRESCALER(RTC_FREQ),
    .interrupt_priority = APP_IRQ_PRIORITY_LOW,
    .tick_latency       = NRFX_RTC_US_TO_TICKS(2000, RTC_FREQ),
    .reliable           = false,
};

/* Private variables ---------------------------------------------------------*/
static KD2_powerMode_t          powerMode;                  // the current power mode

static nrf_ppi_channel_t        ppiPwmStop;                 // the ppi channel used to trigger the pwm stop task
static nrf_ppi_channel_t        ppiAdcTrigger;              // the ppi channel used to trigger adc measurement
static nrf_ppi_channel_t        ppiAdcEndStart;             // the ppi channel used to trigger the start task with the end event
static nrf_saadc_value_t        adcBuffer[2][CHN_CNT_ON];   // the buffers for adc results
static KD2_adc_resultHandler_t  resultHandler;              // the handler to report the result

static nrfx_rtc_t               rtcInst = NRFX_RTC_INSTANCE(2);
static temperature_t            temperature;
static q3_13_t                  lastCurrent;
static compensation_t           compensation = COMP_DEFAULT;

/* Private functions ---------------------------------------------------------*/
static q5_11_t convertVoltage(int16_t rawValue)
{
    return ((int32_t)rawValue * compensation.gainVolt)>>12;
}

static q3_13_t convertCurrent(int16_t rawValue)
{
    int32_t comp = ((int32_t)rawValue * compensation.gainCurr);
    if (comp > 0)
        return comp >> 12;
    else
        return 0;
}

static int16_t reconvertCurrent(q3_13_t current)
{
    // current / 2¹³ * 0.021 * 0.01 * 820 / 0.6 * 2¹²
    //          q3_13   R_L    gain    R6   rev   res
    int32_t raw = (int32_t)current << 12;
    raw /= compensation.gainCurr;
    if (raw >= 1<<12)       // maximum adc value
        return (1<<12) - 1;
    else
        return raw;
}

static q9_7_t convertTemp(int32_t rawValue)
{
    rawValue <<= 5;
    return (q9_7_t)(rawValue + (int32_t)compensation.offsetTemp);
}

/** @brief rtc handler for triggering adc in idle mode
 */
static void rtcHandler(nrfx_rtc_int_type_t type)
{
    (void)type; // only using cc ch0 interrupt
    ret_code_t errCode;

    if (powerMode != KD2_PWR_IDLE)
        return;

    nrfx_rtc_counter_clear(&rtcInst);                       // clear timer
    nrfx_rtc_int_enable(&rtcInst, RTC_CHANNEL_INT_MASK(0)); // and enable irq again

    // trigger the adc to measure the input voltage
    errCode = nrfx_saadc_mode_trigger();
    LOG_ERROR_CHECK("error %d triggering adc", errCode);
}

/** @brief power mode setting for RTC
 */
static void setRtc(KD2_powerMode_t newMode)
{
    nrfx_rtc_disable(&rtcInst);

    // only used in idle mode
    if (newMode == KD2_PWR_IDLE)
    {
        nrfx_rtc_counter_clear(&rtcInst);
        nrfx_rtc_int_enable(&rtcInst, RTC_CHANNEL_INT_MASK(0));
        nrfx_rtc_enable(&rtcInst);
    }
}

static void initRtc()
{
    ret_code_t errCode;

    errCode = nrfx_rtc_init(&rtcInst, &rtcConfig, rtcHandler);
    APP_ERROR_CHECK(errCode);

    errCode = nrfx_rtc_cc_set(&rtcInst, 0, RTC_COMP_VALUE, true);
    APP_ERROR_CHECK(errCode);
}

//#include "SEGGER_RTT.h"
//#include "debug.h"
static void onAdcDone(nrfx_saadc_done_evt_t const* pDone)
{
    //dbg_ResetCycleCnt();
    //dbg_StartCycleCnt();

    KD2_adc_result_t result = {0};

    // reduce prescaler, at zero a measurement is initiated
    if (temperature.prescale)
        temperature.prescale--;

    result.inputVoltage = convertVoltage(pDone->p_buffer[0]);
    if (pDone->size == 2)
    {
        result.ledCurrent = convertCurrent(pDone->p_buffer[1]);
        lastCurrent = result.ledCurrent;
    }
    if (temperature.lastResult != 0)
    {
        result.temperature = temperature.lastResult;
        temperature.lastResult = 0;
    }

    resultHandler(&result);

    //dbg_StopCycleCnt();
    //SEGGER_RTT_printf(0, "%d\r\n", dbg_GetCycleCnt());
}

/** @brief helper function to provide the two adc buffer alternately
 */
static uint8_t nextBuffer()
{
    static uint8_t index = 0;
    return (++index) % 2;
}

static void adcHandler(nrfx_saadc_evt_t const* pEvent)
{
    ret_code_t errCode;

    switch (pEvent->type)
    {
    case NRFX_SAADC_EVT_DONE:
        onAdcDone(&pEvent->data.done);
        break;
    case NRFX_SAADC_EVT_READY:
        nrf_timer_task_trigger(TIMER_INST, NRF_TIMER_TASK_START);
        break;
    case NRFX_SAADC_EVT_FINISHED:
        // this event should not occur in on mode. If it does it means that the interrupts has been blocked
        if (powerMode == KD2_PWR_ON)
        {
            NRF_LOG_WARNING("{BRDadc]: ahhhh!");
        }
        nrf_timer_task_trigger(TIMER_INST, NRF_TIMER_TASK_STOP);
        break;
    case NRFX_SAADC_EVT_BUF_REQ:
        errCode = nrfx_saadc_buffer_set(adcBuffer[nextBuffer()], CHN_CNT_ON);
        APP_ERROR_CHECK(errCode);
        break;
    default:
        break;
    }
}

static void setAdc(KD2_powerMode_t newMode)
{
    ret_code_t errCode;

    //nrfx_saadc_uninit();

    // In idle mode the conversion is triggered manually in the rtc handler.
    if (newMode == KD2_PWR_IDLE)
    {
        nrfx_saadc_uninit();

        errCode = nrfx_saadc_init(APP_IRQ_PRIORITY_LOW);
        APP_ERROR_CHECK(errCode);

        errCode = nrfx_saadc_channels_config(adcChannels, CHN_CNT_IDLE);
        APP_ERROR_CHECK(errCode);

        errCode = nrfx_saadc_simple_mode_set(CHN_MASK_IDLE, NRF_SAADC_RESOLUTION_12BIT, NRF_SAADC_OVERSAMPLE_DISABLED, adcHandler);
        APP_ERROR_CHECK(errCode);

        errCode = nrfx_saadc_buffer_set(adcBuffer[0], CHN_CNT_IDLE);
        APP_ERROR_CHECK(errCode);
    }
    // In on mode the adc is triggered with the timer through a ppi channel
    else if (newMode == KD2_PWR_ON)
    {   // the current measurement is used to regulate the led driver, so use high irq priority
        nrfx_saadc_uninit();

        errCode = nrfx_saadc_init(APP_IRQ_PRIORITY_HIGH);
        APP_ERROR_CHECK(errCode);

        errCode = nrfx_saadc_channels_config(adcChannels, CHN_CNT_ON);
        APP_ERROR_CHECK(errCode);

        errCode = nrfx_saadc_advanced_mode_set(CHN_MASK_ON, NRF_SAADC_RESOLUTION_12BIT, &adcAdvConfig, adcHandler);
        APP_ERROR_CHECK(errCode);

        errCode = nrfx_saadc_buffer_set(adcBuffer[nextBuffer()], CHN_CNT_ON);
        APP_ERROR_CHECK(errCode);

        errCode = nrfx_saadc_buffer_set(adcBuffer[nextBuffer()], CHN_CNT_ON);
        APP_ERROR_CHECK(errCode);

        errCode = nrfx_saadc_mode_trigger();               // enable conversions
        APP_ERROR_CHECK(errCode);
    }
}

static q5_11_t initAdc()
{
    // read the initial value of the input voltage
    ret_code_t errCode;
    nrf_saadc_value_t voltage;

    errCode = nrfx_saadc_init(APP_IRQ_PRIORITY_LOW);
    APP_ERROR_CHECK(errCode);

    errCode = nrfx_saadc_channels_config(adcChannels, CHN_CNT_IDLE);
    APP_ERROR_CHECK(errCode);

    errCode = nrfx_saadc_simple_mode_set(CHN_MASK_IDLE, NRF_SAADC_RESOLUTION_12BIT, NRF_SAADC_OVERSAMPLE_DISABLED, NULL);
    APP_ERROR_CHECK(errCode);

    errCode = nrfx_saadc_buffer_set(&voltage, 1);
    APP_ERROR_CHECK(errCode);

    errCode = nrfx_saadc_mode_trigger();
    APP_ERROR_CHECK(errCode);

    nrfx_saadc_uninit();

    return convertVoltage(voltage);
}

static void setTemp(KD2_powerMode_t newMode)
{
    // if changing from any off to a active mode the temperature value
    // might be pretty old, so reset and initiate new measurement
    if (newMode == KD2_PWR_IDLE || newMode == KD2_PWR_ON)
    {
        temperature.prescale = 0;
        temperature.lastResult = 0;
    }
}

static void executeTemp()
{
    if (temperature.prescale == 0)
    {
        ret_code_t errCode;
        int32_t rawTemp;

        errCode = sd_temp_get(&rawTemp);
        APP_ERROR_CHECK(errCode);

        temperature.lastResult = convertTemp(rawTemp);

        temperature.prescale = powerMode == KD2_PWR_ON ? TEMP_PRESCALE_ON : TEMP_PRESCALE_IDLE;
    }
}

static q9_7_t initTemp()
{
    ret_code_t errCode;
    int32_t rawTemp;
    errCode = sd_temp_get(&rawTemp);
    // if softdevice is not enabled yet, get the tamperature manually
    if (errCode == NRF_ERROR_SOFTDEVICE_NOT_ENABLED)
    {
        nrfx_temp_config_t cfg = {.interrupt_priority = APP_IRQ_PRIORITY_LOW};
        errCode = nrfx_temp_init(&cfg, NULL);
        APP_ERROR_CHECK(errCode);

        errCode = nrfx_temp_measure();
        APP_ERROR_CHECK(errCode);

        rawTemp = nrfx_temp_result_get();

        nrfx_temp_uninit();
    }

    APP_ERROR_CHECK(errCode);

    temperature.lastResult = convertTemp(rawTemp);
    return temperature.lastResult;
}

static void setTimer(KD2_powerMode_t newMode)
{
    // timer is just stopped and cleared here, it is started in the adc irq
    nrf_timer_task_trigger(TIMER_INST, NRF_TIMER_TASK_STOP);
    nrf_timer_task_trigger(TIMER_INST, NRF_TIMER_TASK_CLEAR);
}

static void initTimer()
{
    nrf_timer_frequency_set(TIMER_INST, TIMER_FREQ);
    nrf_timer_cc_write(TIMER_INST, 0, TIMER_COMP_VALUE);
    nrf_timer_shorts_enable(TIMER_INST, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK);
    nrf_timer_mode_set(TIMER_INST, NRF_TIMER_MODE_TIMER);
}

static void setPpi(KD2_powerMode_t newMode)
{
    ret_code_t errCode;

    if (newMode == KD2_PWR_ON)
    {
        errCode = nrfx_ppi_channel_enable(ppiAdcEndStart); // connecting end event with start task for flawless conversions
        APP_ERROR_CHECK(errCode);

        errCode = nrfx_ppi_channel_enable(ppiAdcTrigger);  // trigger adc through timer
        APP_ERROR_CHECK(errCode);

        nrf_saadc_event_clear(NRF_SAADC_INT_CH1LIMITH);
        errCode = nrfx_ppi_channel_enable(ppiPwmStop);  // trigger adc through timer
        APP_ERROR_CHECK(errCode);
    }
    else
    {
        errCode = nrfx_ppi_channel_disable(ppiAdcEndStart);
        APP_ERROR_CHECK(errCode);

        errCode = nrfx_ppi_channel_disable(ppiAdcTrigger);
        APP_ERROR_CHECK(errCode);

        errCode = nrfx_ppi_channel_disable(ppiPwmStop);
        APP_ERROR_CHECK(errCode);
    }
}

static void initPpi()
{
    ret_code_t errCode;

    // configure ppi to reset the counter on compare
    errCode = nrfx_ppi_channel_alloc(&ppiAdcTrigger);
    APP_ERROR_CHECK(errCode);

    errCode = nrfx_ppi_channel_assign(ppiAdcTrigger,
                                      (uint32_t)nrf_timer_event_address_get(TIMER_INST, NRF_TIMER_EVENT_COMPARE0),
                                      nrf_saadc_task_address_get(NRF_SAADC_TASK_SAMPLE));
    APP_ERROR_CHECK(errCode);

    // connect the end event with the start task
    errCode = nrfx_ppi_channel_alloc(&ppiAdcEndStart);
    APP_ERROR_CHECK(errCode);

    errCode = nrfx_ppi_channel_assign(ppiAdcEndStart,
                                      nrf_saadc_event_address_get(NRF_SAADC_EVENT_END),
                                      nrf_saadc_task_address_get(NRF_SAADC_TASK_START));
    APP_ERROR_CHECK(errCode);

    // allocate channel for connecting channel limit event with the pwm stop task
    // the channel assignment is done in the limit setting function
    errCode = nrfx_ppi_channel_alloc(&ppiPwmStop);
    APP_ERROR_CHECK(errCode);
}

/* Public functions ----------------------------------------------------------*/
ret_code_t KD2_Adc_Init(KD2_adc_init_t const* pInit, KD2_adc_result_t* pInitResult)
{
    q5_11_t inputVoltage;
    q9_7_t  temp;

    if (pInit == NULL || pInit->resultHandler == NULL)
        return NRF_ERROR_NULL;

    resultHandler = pInit->resultHandler;

    initRtc();
    initTimer();
    initPpi();
    inputVoltage = initAdc();
    temp = initTemp();

    if (pInitResult != NULL)
    {
        pInitResult->inputVoltage = inputVoltage;
        pInitResult->ledCurrent = 0;
        pInitResult->temperature = temp;
    }

    return NRF_SUCCESS;
}

ret_code_t KD2_Adc_SetPowerMode(KD2_powerMode_t newMode)
{
    if (powerMode == newMode)
        return NRF_SUCCESS;

    /*KD2_powerMode_t testPM;
    if (newMode == KD2_PWR_STANDBY)
        testPM = KD2_PWR_IDLE;
    else
        testPM = newMode;*/

    setRtc(newMode);
    setTemp(newMode);
    setTimer(newMode);
    setPpi(newMode);

    powerMode = newMode;    // stopping adc will generate NRFX_SAADC_EVT_FINISHED event
                            //and falsely generate warning if new mode is not assigned prior
    setAdc(newMode);

    return NRF_SUCCESS;
}

void KD2_Adc_Execute()
{
    executeTemp();
}

ret_code_t KD2_Adc_SetCurrentLimit(q3_13_t limit, uint32_t taskToTrigger)
{
    return NRF_SUCCESS;

    /// TODO: activating limits seems to trigger a stop event, but why?

    int16_t rawLimit = reconvertCurrent(limit);

    nrf_saadc_channel_limits_set(CHN_CURRENT, INT16_MIN, rawLimit);

    return nrfx_ppi_channel_assign(ppiPwmStop,
                                    nrf_saadc_event_address_get(NRF_SAADC_INT_CH1LIMITH),
                                    taskToTrigger);
}

ret_code_t KD2_Adc_SetCompensation(q4_12_t gainCurr, q9_7_t offTemp)
{
    compensation.offsetTemp = offTemp;

    if (gainCurr > CURRGAIN_MIN)
        compensation.gainCurr = gainCurr;
    else
        return NRF_ERROR_INVALID_PARAM;

    return NRF_SUCCESS;
}

/**END OF FILE*****************************************************************/
