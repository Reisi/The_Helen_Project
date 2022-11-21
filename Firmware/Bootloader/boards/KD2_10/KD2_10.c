/**
  ******************************************************************************
  * @file    KD2_10.c
  * @author  Thomas Reisnecker
  * @brief   status led driver for helen bootloader
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "nrf_gpio.h"

#include "board.h"

/* External variables --------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/
#define ISINIT()    \
do                  \
{                   \
    if (!isInit)    \
        init();     \
} while(0)

/* Private defines -----------------------------------------------------------*/
#define LED_RED                 18
#define LED_BLUE                20

/* Private variables ---------------------------------------------------------*/
static bool isInit;

/* Private functions ---------------------------------------------------------*/
static void init()
{
    // configure LED pins and clear LEDs
    nrf_gpio_cfg_output(LED_RED);
    nrf_gpio_pin_clear(LED_RED);
    nrf_gpio_cfg_output(LED_BLUE);
    nrf_gpio_pin_clear(LED_BLUE);

    isInit = true;
}

/* Public functions ----------------------------------------------------------*/
void brd_EnableLed(brd_ledType_t color, bool enable)
{
    ISINIT();

    uint32_t pin;

    if (color == BRD_LT_BLUE)
        pin = LED_BLUE;
    else if (color == BRD_LT_RED)
        pin = LED_RED;
    else
        return;

    if (enable)
        nrf_gpio_pin_set(pin);
    else
        nrf_gpio_pin_clear(pin);
}

/**END OF FILE*****************************************************************/
