/**
  ******************************************************************************
  * @file    feature_tools.c
  * @author  Thomas Reisnecker
  * @brief   tbd.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "feature_tools.h"
#include "app_timer.h"

/* External variables --------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/

/* Private defines -----------------------------------------------------------*/
#if APP_TIMER_CONFIG_RTC_FREQUENCY != 1
#error "feature tools only working with APP_TIMER_CONFIG_RTC_FREQUENCY == 1"
#endif // APP_TIMER_CONFIG_RTC_FREQUENCY

#define TIMER_SHIFT 4   // RTC1 running with 2^14Hz, so 4 bit shift to get 2^10
#define TIMEBASE    160 // = 9.765625ms = 10/1024s

#define BRAKE_STEP  256
#define DIR_STEP    1024

/* Private read only variables -----------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static ftools_inst_t* pInsts;
static uint8_t        numOfInsts;
static q6_10_t        refOffset;
APP_TIMER_DEF(ftoolsTimer);

/* Private functions ---------------------------------------------------------*/
static bool isBrakeLightOn(q6_10_t countdown)
{
    // in case of overrun which hasn't been disabled yet
    if (countdown & 0x8000)
        return false;

    // braklight on below 1 sec timeout
    if (countdown <= (1 << 10))
        return true;

    // above 1 sec timeout, brakelight is blinking with 4Hz
    if (countdown & (1 << 7))
        return true;
    else
        return false;
}

static bool isDirectionLightOn(q6_10_t countdown)
{
    // in case of overrun which hasn't been disabled yet
    if (countdown & 0x8000)
        return false;

    // indicator light blinking with 1Hz
    if (countdown & (1 << 9))
        return true;
    else
        return false;
}

static ret_code_t checkTimer()
{
    for (uint8_t i = 0; i < numOfInsts; i++)
    {
        if (pInsts[i].active)
            return app_timer_start(ftoolsTimer, TIMEBASE, NULL);
    }

    return app_timer_stop(ftoolsTimer);
}

static void timerCallback(void* pContext)
{
    q6_10_t currentTS = ftools_GetReference();

    for (uint8_t i = 0; i < numOfInsts; i++)
    {
        if (pInsts[i].active && ((pInsts[i].endTS - currentTS) & 0x8000))
            pInsts[i].active = false;
    }

    (void)checkTimer();
}

static void setCountdown(ftools_inst_t* pInst, q6_10_t countdown, q6_10_t currentTS, q6_10_t step)
{
    if (pInst->active)
    {
        q6_10_t currentCD = pInst->endTS - currentTS;

        while (currentCD + step < countdown)
        {
            currentCD += step;
        }

        pInst->endTS = currentTS + currentCD;
    }
    else
    {
        pInst->endTS = currentTS + countdown;
    }
}

/* Public functions ----------------------------------------------------------*/
ret_code_t ftools_Init(ftools_inst_t* pInst, uint8_t numOfInst)
{
    if (pInst == NULL)
        return NRF_ERROR_NULL;

    if (numOfInst == 0)
        return NRF_ERROR_INVALID_PARAM;

    pInsts = pInst;
    numOfInsts = numOfInst;

    for (uint8_t i = 0; i < numOfInst; i++)
    {
        pInst[i].active = false;
    }

    return app_timer_create(&ftoolsTimer, APP_TIMER_MODE_REPEATED, &timerCallback);
}

void ftools_UpdateReference(q6_10_t newReference)
{
    uint32_t cnt = app_timer_cnt_get();
    cnt >>= TIMER_SHIFT;

    q6_10_t newOffset = newReference - (q6_10_t)cnt;

    /// TODO: critical region in case of timer interrupt fires while in this loop?
    for (uint8_t i = 0; i < numOfInsts; i++)
    {
        if (pInsts[i].active)
            pInsts[i].endTS += newOffset - refOffset;
    }

    refOffset = newOffset;
}

q6_10_t ftools_GetReference(void)
{
    uint32_t cnt = app_timer_cnt_get();
    cnt >>= TIMER_SHIFT;

    return (q6_10_t)cnt + refOffset;
}

ret_code_t ftools_SetCountdown(ftools_inst_t* pInst, q6_10_t countdown)
{
    if (pInst == NULL)
        return NRF_ERROR_NULL;

    q6_10_t currentTS = ftools_GetReference();
    if (pInst->active)
    {
        q6_10_t currentCD = pInst->endTS - currentTS;
        if (countdown > currentCD)
            pInst->endTS = currentTS + countdown;
    }
    else
    {
        pInst->endTS = currentTS + countdown;
        pInst->active = true;
    }

    return checkTimer();
}

ret_code_t ftools_UpdateCountdown(ftools_inst_t* pInst, q6_10_t countdown)
{
    if (pInst == NULL)
        return NRF_ERROR_NULL;

    if (countdown == 0)
        return NRF_SUCCESS;

    switch (pInst->type)
    {
    case FTOOL_BRAKE_INDICATOR:
        setCountdown(pInst, countdown, ftools_GetReference(), BRAKE_STEP);
        break;

    case FTOOL_DIR_INDICATOR:
        setCountdown(pInst, countdown, ftools_GetReference(), DIR_STEP);
        break;

    default:
        return NRF_ERROR_INVALID_PARAM;
    }

    pInst->active = true;

    return checkTimer();
}

ret_code_t ftools_GetCountdown(ftools_inst_t const* pInst, q6_10_t* pCountdown)
{
    if (pInst == NULL || pCountdown == NULL)
        return NRF_ERROR_NULL;

    if (!pInst->active)
        return NRF_ERROR_INVALID_STATE;

    *pCountdown = pInst->endTS - ftools_GetReference();

    return NRF_SUCCESS;
}

bool ftools_IsOn(ftools_inst_t const* pInst)
{
    if (pInst == NULL || !pInst->active)
        return false;

    switch (pInst->type)
    {
    case FTOOL_BRAKE_INDICATOR:
        return isBrakeLightOn(pInst->endTS - ftools_GetReference());

    case FTOOL_DIR_INDICATOR:
        return isDirectionLightOn(pInst->endTS - ftools_GetReference());

    default:
        return false;
    }
}

/**END OF FILE*****************************************************************/
