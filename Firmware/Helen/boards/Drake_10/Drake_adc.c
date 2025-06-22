/**
  ******************************************************************************
  * @file    Drake_adc.c
  * @author  Thomas Reisnecker
  * @brief   adc module for for Drake Hardware 1.0
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
#include "nrfx_saadc.h"
#include "nrfx_temp.h"

#include "app_timer.h"

#include "Drake_adc.h"
#include "debug.h"
#include "charge_control.h"

/* External variables --------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/
typedef enum
{
    CHN_INPUTVOLTAGE = 0,
    CHN_CC1VOLTAGE,
    CHN_CC2VOLTAGE,
    CHN_CHARGECURRENT,
    CHN_CNT
} channel_index_t;

/* Private macros ------------------------------------------------------------*/

/* Private defines -----------------------------------------------------------*/
#define CHN_MASK            ((1<<CHN_INPUTVOLTAGE)|(1<<CHN_CC1VOLTAGE)|(1<<CHN_CC2VOLTAGE)|(1<<CHN_CHARGECURRENT))

// the pins used for adc
#define AIN_INPUTVOLTAGE    NRF_SAADC_INPUT_AIN1
#define AIN_CC1VOLTAGE      NRF_SAADC_INPUT_AIN7
#define AIN_CC2VOLTAGE      NRF_SAADC_INPUT_AIN5
#define AIN_CHARGECURRENT   NRF_SAADC_INPUT_AIN2

#define SAMPLE_1HZ          APP_TIMER_TICKS(1000)

/* Private read only variables -----------------------------------------------*/
static nrfx_saadc_channel_t const adcChannels[] =
{
    {   // configuration for input voltage measurement
        .channel_config.resistor_p = NRF_SAADC_RESISTOR_DISABLED,
        .channel_config.resistor_n = NRF_SAADC_RESISTOR_DISABLED,
        .channel_config.gain       = NRF_SAADC_GAIN1_4,
        .channel_config.reference  = NRF_SAADC_REFERENCE_INTERNAL,
        .channel_config.acq_time   = NRF_SAADC_ACQTIME_10US,    /// TODO: sufficient?
        .channel_config.mode       = NRF_SAADC_MODE_SINGLE_ENDED,
        .channel_config.burst      = NRF_SAADC_BURST_DISABLED,
        .pin_p             = AIN_INPUTVOLTAGE,
        .pin_n             = NRF_SAADC_INPUT_DISABLED,
        .channel_index     = CHN_INPUTVOLTAGE,
    },
    {   // configuration for CC1 pin voltage measurement
        .channel_config.resistor_p = NRF_SAADC_RESISTOR_DISABLED,
        .channel_config.resistor_n = NRF_SAADC_RESISTOR_DISABLED,
        .channel_config.gain       = NRF_SAADC_GAIN1_4,
        .channel_config.reference  = NRF_SAADC_REFERENCE_INTERNAL,
        .channel_config.acq_time   = NRF_SAADC_ACQTIME_3US,  // low impedance, no need for long acquisition time
        .channel_config.mode       = NRF_SAADC_MODE_SINGLE_ENDED,
        .channel_config.burst      = NRF_SAADC_BURST_DISABLED,
        .pin_p             = AIN_CC1VOLTAGE,
        .pin_n             = NRF_SAADC_INPUT_DISABLED,
        .channel_index     = CHN_CC1VOLTAGE,
    },
    {   // configuration for CC2 pin voltage measurement
        .channel_config.resistor_p = NRF_SAADC_RESISTOR_DISABLED,
        .channel_config.resistor_n = NRF_SAADC_RESISTOR_DISABLED,
        .channel_config.gain       = NRF_SAADC_GAIN1_4,
        .channel_config.reference  = NRF_SAADC_REFERENCE_INTERNAL,
        .channel_config.acq_time   = NRF_SAADC_ACQTIME_3US,  // low impedance, no need for long acquisition time
        .channel_config.mode       = NRF_SAADC_MODE_SINGLE_ENDED,
        .channel_config.burst      = NRF_SAADC_BURST_DISABLED,
        .pin_p             = AIN_CC2VOLTAGE,
        .pin_n             = NRF_SAADC_INPUT_DISABLED,
        .channel_index     = CHN_CC2VOLTAGE,
    },
    {   // configuration for charge current measurement
        .channel_config.resistor_p = NRF_SAADC_RESISTOR_DISABLED,
        .channel_config.resistor_n = NRF_SAADC_RESISTOR_DISABLED,
        .channel_config.gain       = NRF_SAADC_GAIN1_2,
        .channel_config.reference  = NRF_SAADC_REFERENCE_INTERNAL,
        .channel_config.acq_time   = NRF_SAADC_ACQTIME_40US,
        .channel_config.mode       = NRF_SAADC_MODE_SINGLE_ENDED,
        .channel_config.burst      = NRF_SAADC_BURST_DISABLED,
        .pin_p             = AIN_CHARGECURRENT,
        .pin_n             = NRF_SAADC_INPUT_DISABLED,
        .channel_index     = CHN_CHARGECURRENT,
    }
};

/* Private variables ---------------------------------------------------------*/
static Drake_adc_mode_t          adcMode;                    // the current adc mode

static nrf_saadc_value_t         adcBuffer[1][CHN_CNT];      // the buffers for adc results
static Drake_adc_resultHandler_t resultHandler;              // the handler to report the result

APP_TIMER_DEF(adcTimer);
static bool timerExpired;
static q9_7_t lastTemperature;

/* Private functions ---------------------------------------------------------*/
static q3_13_t convertInputVoltage(nrf_saadc_value_t rawValue)
{
    // result /    4096  * 0,6V  * 4   *      2       * 8192    = 9.6 = 157286 / 2^14
    //          12bit res   ref   gain voltage divider q3_13_t

    if (rawValue < 0)
        rawValue = 0;

    return (rawValue * 157286l) >> 14;
}

static q3_13_t convertCCVoltage(nrf_saadc_value_t rawValue)
{
    // result /    4096  * 0,6V  * 4   * 8192    = 9.6 = 157286 / 2^15
    //          12bit res   ref   gain  q3_13_t

    if (rawValue < 0)
        rawValue = 0;

    return (rawValue * 157286l) >> 15;
}

static q16_t convertCurrent(int16_t rawValue)
{
    // result /   4096 * 0,6V  * 2  / 5100 * 1000  * 2^16    = 3.76471 ~= 123362 / 2^15
    //         12bit res  ref  gain  r_set  ic_gain  q16_t

    if (rawValue < 0)
        rawValue = 0;

    return (rawValue * 123362l) >> 15;
}

static q9_7_t convertTemp(int32_t rawValue)
{
    rawValue <<= 5;
    return (q9_7_t)(rawValue + (273 << 7));
}

static void convertResults(Drake_adc_result_t* pResults,
                           nrf_saadc_value_t rawInputVoltage,
                           nrf_saadc_value_t rawCC1Voltage,
                           nrf_saadc_value_t rawCC2Voltage,
                           nrf_saadc_value_t rawChargeCurrent)
{
    pResults->inputVoltage = convertInputVoltage(rawInputVoltage);
    pResults->cc1Voltage = convertCCVoltage(rawCC1Voltage);
    pResults->cc2Voltage = convertCCVoltage(rawCC2Voltage);
    pResults->chargeCurrent = convertCurrent(rawChargeCurrent);
}

//#include "SEGGER_RTT.h"
//#include "debug.h"
static void onAdcDone(nrfx_saadc_done_evt_t const* pDone)
{
    //dbg_ResetCycleCnt();
    //dbg_StartCycleCnt();

    Drake_adc_result_t result = {0};

    convertResults(&result, pDone->p_buffer[CHN_INPUTVOLTAGE], pDone->p_buffer[CHN_CC1VOLTAGE],
                   pDone->p_buffer[CHN_CC2VOLTAGE], pDone->p_buffer[CHN_CHARGECURRENT]);

    result.temperature = lastTemperature;

    (void)chctrl_MeasurementMode(false);

    resultHandler(&result);

    nrfx_saadc_abort();

    //dbg_StopCycleCnt();
    //SEGGER_RTT_printf(0, "%d\r\n", dbg_GetCycleCnt());
}

static void adcHandler(nrfx_saadc_evt_t const* pEvent)
{
    //ret_code_t errCode;

    //NRF_LOG_INFO("adc event %d", pEvent->type)

    switch (pEvent->type)
    {
    case NRFX_SAADC_EVT_DONE:
        onAdcDone(&pEvent->data.done);
        break;
    /*case NRFX_SAADC_EVT_READY:
        //nrf_timer_task_trigger(TIMER_INST, NRF_TIMER_TASK_START);
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
        errCode = nrfx_saadc_buffer_set(adcBuffer[0], CHN_CNT);
        APP_ERROR_CHECK(errCode);
        break;*/
    default:
        break;
    }
}

static ret_code_t initAdc(Drake_adc_result_t* pResult)
{
    ret_code_t errCode;

    if (pResult != NULL)
    {
        errCode = nrfx_saadc_init(APP_IRQ_PRIORITY_LOW);
        if (errCode != NRF_SUCCESS)
            return errCode;

        errCode = nrfx_saadc_channels_config(adcChannels, CHN_CNT);
        if (errCode != NRF_SUCCESS)
            return errCode;

        errCode = nrfx_saadc_simple_mode_set(CHN_MASK, NRF_SAADC_RESOLUTION_12BIT, NRF_SAADC_OVERSAMPLE_DISABLED, NULL);
        if (errCode != NRF_SUCCESS)
            return errCode;

        errCode = nrfx_saadc_buffer_set(adcBuffer[0], CHN_CNT);
        if (errCode != NRF_SUCCESS)
            return errCode;

        errCode = nrfx_saadc_mode_trigger();
        if (errCode != NRF_SUCCESS)
            return errCode;

        convertResults(pResult, adcBuffer[0][CHN_INPUTVOLTAGE], adcBuffer[0][CHN_CC1VOLTAGE],
                       adcBuffer[0][CHN_CC2VOLTAGE], adcBuffer[0][CHN_CHARGECURRENT]);

        nrfx_saadc_uninit();
    }

    errCode = nrfx_saadc_init(APP_IRQ_PRIORITY_LOW);
    if (errCode != NRF_SUCCESS)
        return errCode;

    errCode = nrfx_saadc_channels_config(adcChannels, CHN_CNT);
    if (errCode != NRF_SUCCESS)
        return errCode;

    return NRF_SUCCESS;

    //return nrfx_saadc_simple_mode_set(CHN_MASK, NRF_SAADC_RESOLUTION_12BIT, NRF_SAADC_OVERSAMPLE_DISABLED, adcHandler);
}

static void executeTemp()
{
    if (timerExpired)
    {
        ret_code_t errCode;
        int32_t rawTemp;

        errCode = sd_temp_get(&rawTemp);
        LOG_ERROR_CHECK("error %d getting temperature", errCode);

        if (errCode != NRF_SUCCESS)
            lastTemperature = 0;
        else
            lastTemperature = convertTemp(rawTemp);

        timerExpired = false;
    }
}

static ret_code_t initTemp(Drake_adc_result_t* pResult)
{
    if (pResult == NULL)
        return NRF_SUCCESS;

    ret_code_t errCode;
    int32_t rawTemp;
    errCode = sd_temp_get(&rawTemp);
    // if softdevice is not enabled yet, get the temperature manually
    if (errCode == NRF_ERROR_SOFTDEVICE_NOT_ENABLED)
    {
        nrfx_temp_config_t cfg = {.interrupt_priority = APP_IRQ_PRIORITY_LOW};
        errCode = nrfx_temp_init(&cfg, NULL);
        if (errCode != NRF_SUCCESS)
            return errCode;

        errCode = nrfx_temp_measure();
        if (errCode != NRF_SUCCESS)
            return errCode;

        rawTemp = nrfx_temp_result_get();

        nrfx_temp_uninit();
    }

    lastTemperature = convertTemp(rawTemp);
    pResult->temperature = lastTemperature;

    return errCode;
}

void adcTimerCallback(void * pContext)
{
    (void)pContext;

    ret_code_t errCode;

    // set flag to initiate temperature measurement
    timerExpired = true;

    (void)chctrl_MeasurementMode(true);

    errCode = nrfx_saadc_simple_mode_set(CHN_MASK, NRF_SAADC_RESOLUTION_12BIT, NRF_SAADC_OVERSAMPLE_DISABLED, adcHandler);
    if (errCode == NRF_ERROR_BUSY)  // happens when timer interrupts pile up during debugging
        return;
    LOG_ERROR_CHECK("error %d setting adc simple mode", errCode);

    // set buffer and start adc cycle
    errCode = nrfx_saadc_buffer_set(adcBuffer[0], CHN_CNT);
    LOG_ERROR_CHECK("error %d setting adc buffer", errCode);

    errCode = nrfx_saadc_mode_trigger();
    LOG_ERROR_CHECK("error %d starting adc", errCode);
}

static ret_code_t initTimer()
{
    return app_timer_create(&adcTimer, APP_TIMER_MODE_REPEATED, &adcTimerCallback);
}

/* Public functions ----------------------------------------------------------*/
ret_code_t Drake_Adc_Init(Drake_adc_init_t const* pInit, Drake_adc_result_t* pInitResult)
{
    if (pInit == NULL || pInit->resultHandler == NULL)
        return NRF_ERROR_NULL;

    resultHandler = pInit->resultHandler;

    initAdc(pInitResult);   /// TODO: with initial measurement current draw of about 400-450µ

    initTemp(pInitResult);

    initTimer();

    return NRF_SUCCESS;
}

ret_code_t Drake_Adc_SetMode(Drake_adc_mode_t newMode)
{
    ret_code_t errCode;

    if (adcMode == newMode)
        return NRF_SUCCESS;

    if (newMode == DRAKE_ADC_OFF)
    {
        errCode = app_timer_stop(adcTimer);
        LOG_ERROR_CHECK("stopping adc timer error %d", errCode);
    }
    else
    {
        errCode = app_timer_start(adcTimer, SAMPLE_1HZ, NULL);
        LOG_ERROR_CHECK("starting adc timer error %d", errCode);
    }

    adcMode = newMode;

    return NRF_SUCCESS;
}

void Drake_Adc_Execute()
{
    executeTemp();
}

/**END OF FILE*****************************************************************/
