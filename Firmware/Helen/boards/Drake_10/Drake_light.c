/**
  ******************************************************************************
  * @file    Drake_light.c
  * @author  Thomas Reisnecker
  * @brief   light module for Drake Hardware 1.0
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
#include "Drake_hmi.h"
#include "ws2812drv.h"
#include "Drake_light.h"
#include "nrf_gpio.h"
#include "debug.h"
#include "nrf_delay.h"
#include "app_timer.h"

/* External variables --------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/
typedef void (*setColor_t)(q8_t intensity, uint8_t* pGRB);

/* Private macros ------------------------------------------------------------*/

/* Private defines -----------------------------------------------------------*/
#define EN_PIN          18

#define SCK_PIN         1

#define WS_PIN_PRI      0
#define LED_CNT_PRI     45

#define WS_PIN_SEC      5
#define LED_CNT_SEC     27

#define LED_CNT         (LED_CNT_PRI + LED_CNT_SEC)

#define PRI             0
#define SEC             1

#define VF_RED          2
#define VF_GREEN        3
#define VF_BLUE         3

#define QUIESCENT_CURR  9   // ~0.55mA

/* Private variables ---------------------------------------------------------*/
static ws2812_inst_t wsInst[2];
static uint32_t      wsBufferPri[WS2812_BUFFER_SIZE_WORDS(LED_CNT_PRI)];
static uint32_t      wsBufferSec[WS2812_BUFFER_SIZE_WORDS(LED_CNT_SEC)];

APP_TIMER_DEF(socTimer);
static uint32_t      socTimeout;

static const uint8_t socPattern16[][2] =
{
    {0b00000000, 0b00000000},
    {0b00000000, 0b00001000},
    {0b00000000, 0b00011000},
    {0b00000000, 0b01011000},
    {0b00000000, 0b11011000},
    {0b00000100, 0b11011000},
    {0b00010100, 0b11011000},
    {0b00110100, 0b11011000},
    {0b01110100, 0b11011000},
    {0b11110100, 0b11011000},
    {0b11110100, 0b11011010},
    {0b11110100, 0b11011110}
};

static const uint8_t socPattern44[][6] =
{
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000},
    {0b00000000, 0b00000100, 0b00000000, 0b00000000, 0b00000000, 0b00000000},
    {0b00000000, 0b00010100, 0b00000000, 0b00000000, 0b00000000, 0b00000000},
    {0b00000000, 0b00110100, 0b00000000, 0b00000000, 0b00000000, 0b00000000},
    {0b00000000, 0b00110100, 0b00000000, 0b00000001, 0b00000000, 0b00000000},
    {0b00000000, 0b00110100, 0b10000000, 0b00000001, 0b00000000, 0b00000000},
    {0b00000000, 0b00110100, 0b10000000, 0b00000011, 0b00000000, 0b00000000},
    {0b00000000, 0b00110100, 0b10000000, 0b00010011, 0b00000000, 0b00000000},
    {0b00000000, 0b00110100, 0b10000000, 0b01010011, 0b00000000, 0b00000000},
    {0b00000000, 0b00110100, 0b10000000, 0b11010011, 0b00000000, 0b00000000},
    {0b00000000, 0b00110100, 0b10000000, 0b11010011, 0b00000001, 0b00000000},
    {0b00000000, 0b00110100, 0b10000000, 0b11010011, 0b00000101, 0b00000000},
    {0b00000000, 0b00110100, 0b10000000, 0b11010011, 0b00010101, 0b00000000},
    {0b00000000, 0b00110100, 0b10000000, 0b11010011, 0b00110101, 0b00000000},
    {0b00000000, 0b00110100, 0b10000000, 0b11010011, 0b01110101, 0b00000000},
    {0b00000000, 0b00110100, 0b10001000, 0b11010011, 0b11110101, 0b00000000},
    {0b00000000, 0b00110100, 0b10011000, 0b11010011, 0b11110101, 0b00000000},
    {0b00000000, 0b00110100, 0b10011100, 0b11010011, 0b11110101, 0b00000000},
    {0b00000100, 0b00110100, 0b10011100, 0b11010011, 0b11110101, 0b00000000},
    {0b00010100, 0b00110100, 0b10011100, 0b11010011, 0b11110101, 0b00000000},
    {0b00110100, 0b00110100, 0b10011100, 0b11010011, 0b11110101, 0b00000000},
    {0b01110100, 0b00110100, 0b10011100, 0b11010011, 0b11110101, 0b00000000},
    {0b11110100, 0b00110100, 0b10011100, 0b11010011, 0b11110101, 0b00000000},
    {0b11110100, 0b00110110, 0b10011100, 0b11010011, 0b11110101, 0b00000000}
};

/* Private functions ---------------------------------------------------------*/
static bool isLightAvailable()
{
    // Checking the presence of the first ws2182 led by enabling the pullup for
    // a short time and the deactivating it again. A LED is present it will
    // discharge the pin, otherwise the stray capacitance will keep the pin
    // level high

    nrf_gpio_cfg_input(WS_PIN_PRI, NRF_GPIO_PIN_PULLUP);
    nrf_delay_ms(10);

    nrf_gpio_cfg_input(WS_PIN_PRI, NRF_GPIO_PIN_NOPULL);
    nrf_delay_ms(10);

    bool pinState = nrf_gpio_pin_read(WS_PIN_PRI);

    nrf_gpio_cfg_default(WS_PIN_PRI);

    return !pinState;
}

// red:   typ 400 mCd @ 100%
// green: typ 937.5 mCd @ 100%
// blue:  typ 250 mCd @ 100%

static void setRed(q8_t intensity, uint8_t* pGRB)   // ~400 mCd @ 100%
{
    pGRB[0] = 0;
    pGRB[1] = intensity;
    pGRB[2] = 0;
}

static void setAmber(q8_t intensity, uint8_t* pGRB) // ~700 mCd @ 100%
{
    pGRB[0] = (intensity * 32) / 100;
    pGRB[1] = intensity;
    pGRB[2] = 0;
}

static void setWhite(q8_t intensity, uint8_t* pGRB) // ~750 mCd @ 100%
{
    pGRB[0] = (intensity * 4) / 15;
    pGRB[1] = (intensity * 5) / 8;
    pGRB[2] = intensity;
}

static void setGreen(q8_t intensity, uint8_t* pGRB)      // ~937,5 mCd @ 100%
{
    pGRB[0] = intensity;
    pGRB[1] = 0;
    pGRB[2] = 0;
}

static void setYellow(q8_t intensity, uint8_t* pGRB) // ~800 mCd @ 100%
{
    pGRB[0] = (intensity * 43) / 100;
    pGRB[1] = intensity;
    pGRB[2] = 0;
}

/** @brief function to generate the datastream for i2s interface out of the GRB buffer
 */
static void generateStream(Drake_light_t const* pLight, uint8_t* pGRB)
{
    for (Drake_lightType_t i = DRAKE_LIGHT_FRONT; i < DRAKE_LIGHT_CNT; i++)
    {
        for (uint8_t j = 0; j < LED_CNT && (j/8) < pLight->pattern.len; j++)
        {
            uint8_t bit  = j % 8;
            uint8_t byte = j / 8;

            bool inPattern = pLight[i].pattern.pPattern[byte] & (1 << bit);
            bool ignore    = !pLight[i].activeClear && pLight[i].intensity == 0;

            if (inPattern && !ignore)
            {
                switch (i)
                {
                case DRAKE_LIGHT_FRONT:
                    setWhite(pLight[i].intensity, &pGRB[3 * j]);
                    break;

                case DRAKE_LIGHT_REAR:
                case DRAKE_LIGHT_BRAKE:
                    setRed(pLight[i].intensity, &pGRB[3 * j]);
                    break;

                case DRAKE_LIGHT_POS:
                case DRAKE_LIGHT_INDIC_LEFT:
                case DRAKE_LIGHT_INDIC_RIGHT:
                    setAmber(pLight[i].intensity, &pGRB[3 * j]);
                    break;

                default:
                    break;
                }
            }
        }
    }
}

static void generateSOCStream(q8_t soc, uint16_t ledCnt, uint8_t* pGRB)
{
    uint8_t const* pPattern;
    setColor_t setColor;

    if (soc < 64)
        setColor = setRed;
    else if (soc < 192)
        setColor = setYellow;
    else
        setColor = setGreen;

    if (ledCnt == 16)
    {
        soc = ((soc * 11) + 128) / 256;
        if (soc > 11)
            return;

        pPattern = socPattern16[soc];
    }
    else if (ledCnt == 44)
    {
        soc = ((soc * 23) + 128) / 256;
        if (soc > 23)
            return;

        pPattern = socPattern44[soc];
    }
    else
        return;

    for (uint8_t i = 0; i < ledCnt && i < LED_CNT; i++)
    {
        uint8_t bit  = i % 8;
        uint8_t byte = i / 8;

        if (pPattern[byte] & (1 << bit))
            setColor(128, &pGRB[3 * i]);
    }
}

static void gpioInit()
{
    nrf_gpio_cfg_output(EN_PIN);
    nrf_gpio_pin_clear(EN_PIN);
}

/** @brief helper to enable the power supply for the leds
 *
 * @param[in] enable  true to enable, false to disable
 * @return true if power has been enabled, otherwise false
 */
static bool pwrEnable(bool enable)
{
    if (nrf_gpio_pin_out_read(EN_PIN) == enable)
        return false;

    if (enable)
    {
        nrf_gpio_pin_set(EN_PIN);
        return true;
    }
    else
    {
        nrf_gpio_pin_clear(EN_PIN);
        return false;
    }
}

/** @brief handler called after data has been sent, used to send the data of the second instance
 */
static void dataSentHandler(uint8_t dataPin)
{
    if (dataPin == WS_PIN_PRI && wsInst[SEC].cnt)
    {
        ret_code_t errCode = ws2812_SendDataStream(&wsInst[SEC], NULL);
        LOG_ERROR_CHECK("error %d sending second ws2812 data stream", errCode);
    }
}

static void socTimerCallback(void * pContext)
{
    socTimeout = 0;

    // check if a stream has been prepared or power off
    for (uint16_t i = 0; i < ARRAY_SIZE(wsBufferPri); i++)
    {
        if (wsBufferPri[i] != 0 && wsBufferPri[i] != 0x88888888)
        {
            if (ws2812_SendDataStream(&wsInst[PRI], dataSentHandler) == NRF_SUCCESS)
                return;
        }
    }

    (void)pwrEnable(false);
}

/* Public functions ----------------------------------------------------------*/
ret_code_t Drake_Light_Init(bool addSecondInst)
{
    if (!isLightAvailable())
        return NRF_ERROR_NOT_FOUND;

    ret_code_t errCode;

    gpioInit();

    ws2812_init_t wsInit = {.sckPin = SCK_PIN};

    wsInit.dataPin = WS_PIN_PRI;
    wsInit.cnt = LED_CNT_PRI;
    wsInit.pBuffer = wsBufferPri;
    wsInit.bufferSize = WS2812_BUFFER_SIZE_WORDS(LED_CNT_PRI);

    errCode = ws2812_Init(&wsInst[PRI], &wsInit);
    if (errCode != NRF_SUCCESS)
        return errCode;

    if (addSecondInst)
    {
        wsInit.dataPin = WS_PIN_SEC;
        wsInit.cnt = LED_CNT_SEC;
        wsInit.pBuffer = wsBufferSec;
        wsInit.bufferSize = WS2812_BUFFER_SIZE_WORDS(LED_CNT_SEC);

        errCode = ws2812_Init(&wsInst[SEC], &wsInit);
        if (errCode != NRF_SUCCESS)
            return errCode;
    }

    errCode = app_timer_create(&socTimer, APP_TIMER_MODE_SINGLE_SHOT, &socTimerCallback);
    if (errCode != NRF_SUCCESS)
        return errCode;

    return NRF_SUCCESS;
}

q2_14_t calculateCurrent(uint8_t* pGRB, uint16_t size, uint16_t ledCnt)
{
    q2_14_t current = ledCnt * QUIESCENT_CURR;

    for (uint16_t i = 0; i < size; i++)
    {
        current += pGRB[i];
    }

    return current;
}

q3_13_t calculatePower(uint8_t* pGRB, uint16_t size)
{
    q3_13_t power = 0;

    for (uint16_t i = 0; i < size; i += 3)
    {
        power += pGRB[i + 0] * VF_GREEN;
        power += pGRB[i + 1] * VF_RED;
        power += pGRB[i + 2] * VF_BLUE;
    }

    return power;
}

ret_code_t Drake_Light_Update(Drake_light_t const* pLight, uint16_t ledCnt, q2_14_t* pCurrent, q3_13_t* pPower)
{
    if (pLight == NULL)
        return NRF_ERROR_NULL;

    //if (socTimeout)
    //    return NRF_ERROR_INVALID_STATE;

    ret_code_t errCode;
    uint8_t grb[3 * LED_CNT];
    memset((uint8_t*)grb, 0, sizeof(grb));
    ws2812_update_t wsUpdate =  {.start = 0};

    generateStream(pLight, (uint8_t*)grb);

    // prepare data
    wsUpdate.len = LED_CNT_PRI;
    wsUpdate.pGRB = (uint8_t*)&grb[0];
    errCode = ws2812_UpdateDataStream(&wsInst[PRI], &wsUpdate);
    if (errCode != NRF_SUCCESS)
        return errCode;

    if (wsInst[SEC].cnt)
    {
        wsUpdate.len = LED_CNT_SEC;
        wsUpdate.pGRB = (uint8_t*)&grb[3 * LED_CNT_PRI];
        errCode = ws2812_UpdateDataStream(&wsInst[SEC], &wsUpdate);
        if (errCode != NRF_SUCCESS)
            return errCode;
    }

    if (pCurrent != NULL)
        *pCurrent = 0;

    if (pPower != NULL)
        *pPower = 0;

    // if state of charge is shown, exit. light will be shown when timer expires
    if (socTimeout)
        return NRF_SUCCESS;

    // only update data if at least one led is active, otherwise power off
    for (uint16_t i = 0; i < sizeof(wsBufferPri); i++)
    {
        if (wsBufferPri[i] != 0 && wsBufferPri[i] != 0x88888888) // 0 when buffer hasn't been set yet, or 0x88888888 is encoded 0
        {
            if (pCurrent != NULL)
            *pCurrent = calculateCurrent((uint8_t*)grb, sizeof(grb), ledCnt);

            if (pPower != NULL)
                *pPower = calculatePower((uint8_t*)grb, sizeof(grb));

            if (pwrEnable(true))
                nrf_delay_ms(5);    // after enabling power needs to stabilize before sending data

            NRF_LOG_INFO("sending light data");
            return ws2812_SendDataStream(&wsInst[PRI], dataSentHandler);
        }
    }

    (void)pwrEnable(false);

    return NRF_SUCCESS;
}

ret_code_t Drake_Light_ShowSOC(q8_t soc, uint16_t ledCnt, uint32_t timeout, q2_14_t* pCurrent, q3_13_t* pPower)
{
    uint8_t grb[3 * LED_CNT];
    memset((uint8_t*)grb, 0, sizeof(grb));

    generateSOCStream(soc, ledCnt, (uint8_t*)grb);

    if (pCurrent != NULL)
        *pCurrent = calculateCurrent((uint8_t*)grb, sizeof(grb), ledCnt);

    if (pPower != NULL)
        *pPower = calculatePower((uint8_t*)grb, sizeof(grb));

    // only update data if at least one led is active, otherwise power off
    for (uint16_t i = 0; i < sizeof(grb); i++)
    {
        if (grb[i] != 0)
        {
            ret_code_t errCode;
            ws2812_update_t wsUpdate =  {.start = 0};

            wsUpdate.len = LED_CNT_PRI;
            wsUpdate.pGRB = (uint8_t*)&grb[0];
            errCode = ws2812_UpdateDataStream(&wsInst[PRI], &wsUpdate);
            if (errCode != NRF_SUCCESS)
                return errCode;

            /*if (wsInst[SEC].cnt)
            {
                wsUpdate.len = LED_CNT_SEC;
                wsUpdate.pGRB = (uint8_t*)&grb[3 * LED_CNT_PRI];
                errCode = ws2812_UpdateDataStream(&wsInst[SEC], &wsUpdate);
                if (errCode != NRF_SUCCESS)
                    return errCode;
            }*/

            if (pwrEnable(true))
                nrf_delay_ms(5);    // after enabling power needs to stabilize before sending data

            errCode = app_timer_start(socTimer, timeout, NULL);
            if (errCode == NRF_SUCCESS)
                socTimeout = timeout;
            else
                return errCode;

            NRF_LOG_INFO("sending light data");
            errCode = ws2812_SendDataStream(&wsInst[PRI], dataSentHandler);
            return errCode;
        }
    }

    (void)pwrEnable(false);

    if (pCurrent != NULL)
        *pCurrent = 0;

    if (pPower != NULL)
        *pPower = 0;

    return NRF_SUCCESS;
}

#undef  NRF_LOG_LEVEL

/**END OF FILE*****************************************************************/
