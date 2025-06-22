/**
  ******************************************************************************
  * @file    charge_control.c
  * @author  Thomas Reisnecker
  * @brief   charge control module for for Drake Hardware 1.0
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
#include "charge_control.h"
#include "nrf_gpio.h"

/* External variables --------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/

/* Private defines -----------------------------------------------------------*/
#define CHCTRL_PIN          4

#define CCVOLTAGECONNECT    1638    // 0,2V threshold for connect
#define CCVOLTAGE1_5        5407   // 0,66V threshold for at least 1.5A

#define FULLCURRENT200      1310    // 20mA (10% of 200mA)
#define FULLCURRENT500      3276    // 50mA (10% of 500mA)

#define FULLREPEATCNT       116       // charging will be terminated after n times below full current

#define FULLVOLTAGE         33177   // 4.05V

/* Private read only variables -----------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static chctrl_current_t      maxCurrent;
//static chctrl_eventHandler_t evtHandler;
static bool                  enabled;
static chctrl_state_t        state = CHCTRL_STATE_DISCONNECTED;

/* Private functions ---------------------------------------------------------*/
static void shutDown()
{
    nrf_gpio_cfg_default(CHCTRL_PIN);
}

static void notCharging()
{
    nrf_gpio_cfg_output(CHCTRL_PIN);
    nrf_gpio_pin_set(CHCTRL_PIN);
}

static void charging(chctrl_current_t current)
{
    if (current == CHCTRL_CURRENT_200MA)
        nrf_gpio_cfg_default(CHCTRL_PIN);
    else
    {
        nrf_gpio_cfg_output(CHCTRL_PIN);
        nrf_gpio_pin_clear(CHCTRL_PIN);
    }
}

static void setState(chctrl_state_t newState)
{
    switch (newState)
    {
    case CHCTRL_STATE_DISCONNECTED:
        shutDown();
        break;

    case CHCTRL_STATE_NOT_CHARGING:
        notCharging();
        break;

    case CHCTRL_STATE_CHARGING:
        charging(maxCurrent);
        break;

    case CHCTRL_STATE_FULL:
        notCharging();
        break;

    default:
        break;
    }
}

/* Public functions ----------------------------------------------------------*/
ret_code_t chctrl_Init(chctrl_current_t maxChargeCurrent)
{
  if (maxChargeCurrent > CHCTRL_CURRENT_500MA || maxChargeCurrent < CHCTRL_CURRENT_200MA)
        return NRF_ERROR_INVALID_PARAM;

    maxCurrent = maxChargeCurrent;

    shutDown();
    state = CHCTRL_STATE_DISCONNECTED;  // in case of reinitialization

    return NRF_SUCCESS;
}

void chctrl_Enable(bool enable)
{
    if (enable == enabled)
        return;

    if (!enable)
        shutDown();

    enabled = enable;
}

chctrl_state_t chctrl_Feed(q1_15_t cc1Voltage, q1_15_t cc2Voltage, q16_t chargeCurrent, q3_13_t batteryVoltage)
{
    if (!enabled)
        return cc1Voltage > CCVOLTAGECONNECT || cc2Voltage > CCVOLTAGECONNECT ? CHCTRL_STATE_CHARGING : CHCTRL_STATE_DISCONNECTED;

    chctrl_state_t newState = state;
    q16_t currentLimit = maxCurrent == CHCTRL_CURRENT_200MA ? FULLCURRENT200 : FULLCURRENT500;
    static uint8_t belowLimitCnt;

    switch (state)
    {
    case CHCTRL_STATE_DISCONNECTED:
        if (cc1Voltage > CCVOLTAGECONNECT || cc2Voltage > CCVOLTAGECONNECT)
            newState = CHCTRL_STATE_CHARGING;
        break;

    case CHCTRL_STATE_CHARGING:
        if (cc1Voltage <= CCVOLTAGECONNECT && cc2Voltage <= CCVOLTAGECONNECT)
            newState = CHCTRL_STATE_DISCONNECTED;
        else if (chargeCurrent <= currentLimit)
        {
            if (++belowLimitCnt >= FULLREPEATCNT)
                newState = CHCTRL_STATE_FULL;
        }
        else
            belowLimitCnt = 0;
        break;

    case CHCTRL_STATE_FULL:
        if (cc1Voltage <= CCVOLTAGECONNECT && cc2Voltage <= CCVOLTAGECONNECT)
            newState = CHCTRL_STATE_DISCONNECTED;
        else if (batteryVoltage < FULLVOLTAGE)
            newState = CHCTRL_STATE_CHARGING;
        break;

    case CHCTRL_STATE_NOT_CHARGING:
    default:
        break;
    }

    if (newState != state)
    {
        if (state == CHCTRL_STATE_CHARGING)
            belowLimitCnt = 0;

        setState(newState);
        state = newState;
    }

    return newState;

    /*if (newState != state)
    {
        setState(newState);

        if (evtHandler)
        {
            chctrl_evt_t evt;
            evt.type = CHCTRL_EVT_STATE_CHANGE;
            evt.newState = newState;

            evtHandler(&evt);
        }

        state = newState;
    }*/
}

ret_code_t chctrl_MeasurementMode(bool enable)
{
    if (state != CHCTRL_STATE_CHARGING)
        return NRF_ERROR_INVALID_STATE;

    if (maxCurrent == CHCTRL_CURRENT_200MA)
        return NRF_SUCCESS;

    if (enable)
    {
        charging(CHCTRL_CURRENT_200MA);
    }
    else
    {
        charging(CHCTRL_CURRENT_500MA);
    }

    return NRF_SUCCESS;
}

/**END OF FILE*****************************************************************/
