/**
  ******************************************************************************
  * @file    watchdog.c
  *          https://devzone.nordicsemi.com/f/nordic-q-a/30075/saving-the-location-of-watchdog-timeout
  ******************************************************************************
  */

/* logger configuration ----------------------------------------------_-------*/
#define DBG_CONFIG_LOG_ENABLED 1 /// TODO: just temporary until project specific sdk_config
#define DBG_CONFIG_LOG_LEVEL   3

#define NRF_LOG_MODULE_NAME dbg
#if DBG_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL DBG_CONFIG_LOG_LEVEL
#else //BASE_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL 0
#endif //BASE_CONFIG_LOG_ENABLED
#include "nrf_log.h"

/* Includes ------------------------------------------------------------------*/
#include "nrfx_wdt.h"
#include "nrf_power.h"
#include "crc16.h"
#include "nrf_bootloader_info.h"

#include "debug.h"

/* External variables --------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
    uint32_t r0;  ///< R0 register.
    uint32_t r1;  ///< R1 register.
    uint32_t r2;  ///< R2 register.
    uint32_t r3;  ///< R3 register.
    uint32_t r12; ///< R12 register.
    uint32_t lr;  ///< Link register.
    uint32_t pc;  ///< Program counter.
    uint32_t psr; ///< Program status register.
} stack_t;

typedef struct
{
    stack_t stack;
    //uint32_t programCounter;
    uint16_t crc;
} savedPC_t;

/* Private macros ------------------------------------------------------------*/

/* Private defines -----------------------------------------------------------*/

/* Private function prototype ------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static nrfx_wdt_config_t const config =
{
    .behaviour          = NRF_WDT_BEHAVIOUR_PAUSE_SLEEP_HALT,
    .reload_value       = 2000,
    .interrupt_priority = APP_IRQ_PRIORITY_HIGH
};

static nrfx_wdt_channel_id watchdogChannel;

static savedPC_t savedPC __attribute__((section(".noinit")));

/* Private functions ---------------------------------------------------------*/
void dumpStack(stack_t const * pStack)
{
    //savedPC.programCounter = pStack->pc;
    memcpy(&savedPC.stack, pStack, sizeof(stack_t));
    /*Note: RAM is not guaranteed to be retained retained through WD reset:
      http://infocenter.nordicsemi.com/topic/com.nordic.infocenter.nrf52832.ps.v1.1/power.html?cp=2_1_0_17_7#unique_832471788.
      CRC may be used to verify the data integrity*/
    savedPC.crc = crc16_compute((const uint8_t *)&savedPC.stack, sizeof(stack_t), NULL);
}

static void watchdogHandler()
{
    // dump stack
    __ASM volatile(
                     "   mrs r0, msp                             \n"
                     "   ldr r3, =dumpStack                      \n"
                     "   bx r3                                   \n"
                  );
}

static void checkResetReason()
{
    uint32_t resetReason;
    uint16_t crc;

    resetReason = nrf_power_resetreas_get();
    nrf_power_resetreas_clear(resetReason);

    // check if reset was caused by watchdog
    if (resetReason & NRF_POWER_RESETREAS_DOG_MASK)
    {
        crc = crc16_compute((const uint8_t *)&savedPC.stack, sizeof(stack_t), NULL);
        if (crc == savedPC.crc)
        {
            NRF_LOG_ERROR("watchdog reset! R3 0x%08X, PC 0x%08X", savedPC.stack.r3, savedPC.stack.pc);
        }
        else
        {
            NRF_LOG_ERROR("watchdog reset!");
        }
    }
    memset(&savedPC, 0, sizeof(savedPC_t));
}

/* Public functions ----------------------------------------------------------*/
ret_code_t wdg_Init()
{
    ret_code_t errCode;

    checkResetReason();

    errCode = nrfx_wdt_init(&config, watchdogHandler);
    LOG_ERROR_RETURN("watchdog init error %d", errCode);

    errCode = nrfx_wdt_channel_alloc(&watchdogChannel);
    LOG_ERROR_RETURN("watchdog channel allocation error %d", errCode);

    nrfx_wdt_enable();
    nrfx_wdt_channel_feed(watchdogChannel);

    return NRF_SUCCESS;
}

void wdg_Feed()
{
    nrfx_wdt_channel_feed(watchdogChannel);
}

/**END OF FILE*****************************************************************/
