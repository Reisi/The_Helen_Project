/**
  ******************************************************************************
  * @file    ws2812drv.c
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
#include "ws2812drv.h"
#include "nrfx_i2s.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"

/* Private typedef -----------------------------------------------------------*/
typedef enum
{
    STATE_OFF,
    STATE_IDLE,
    STATE_TRANSFERING,
} state_t;

/* Private macros ------------------------------------------------------------*/

/* Private defines -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static state_t state;
static ws2812_dataSentHandler_t handler;

/* Private read only variables -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
static void i2sHandler(nrfx_i2s_buffers_t const* pReleased, uint32_t status)
{
    //NRF_LOG_INFO("i2sHandler %d, %d", pReleased, status);

    if (!(status & NRFX_I2S_STATUS_NEXT_BUFFERS_NEEDED))
        return;

    if (pReleased == NULL)  // data has been sent, i2s can be stopped
    {
        nrfx_i2s_stop();
        state = STATE_IDLE;

        if (handler != NULL)
            handler(NRF_I2S->PSEL.SDOUT);
    }
}

static ret_code_t initI2s(uint8_t dataPin, uint8_t sckPin)
{
    nrfx_i2s_config_t cfg =
    {
        .sck_pin = sckPin,
        .lrck_pin = NRFX_I2S_PIN_NOT_USED,
        .mck_pin = NRFX_I2S_PIN_NOT_USED,
        .sdout_pin = dataPin,
        .sdin_pin = NRFX_I2S_PIN_NOT_USED,
        .irq_priority = APP_IRQ_PRIORITY_LOW,
        .mode = NRF_I2S_MODE_MASTER,
        .format = NRF_I2S_FORMAT_ALIGNED,
        .alignment = NRF_I2S_ALIGN_LEFT,
        .sample_width = NRF_I2S_SWIDTH_16BIT,
        .channels = NRF_I2S_CHANNELS_STEREO,
        .mck_setup = NRF_I2S_MCK_32MDIV10,
        .ratio = NRF_I2S_RATIO_32X
    };

    return nrfx_i2s_init(&cfg, i2sHandler);
}

static void dataConvert(uint8_t const* pData, uint16_t len, uint32_t* pBuffer)
{
    for (uint16_t i = 0; i < len; i++)
    {
        uint8_t in = pData[i];
        uint32_t out = 0;
        for (uint8_t j = 0; j < 8; j++)
        {
            out <<= 4;
            out |= (in & 0x80) ? 0xe : 0x8;
            in <<= 1;
        }
        pBuffer[i] = (out >> 16) | (out << 16);
    }
}

/* Public functions ----------------------------------------------------------*/
ret_code_t ws2812_Init(ws2812_inst_t* pInst, ws2812_init_t const* pInit)
{
    ret_code_t errCode;

    if (pInst == NULL || pInit == NULL)
        return NRF_ERROR_NULL;

    if (pInit->bufferSize < WS2812_BUFFER_SIZE_WORDS(pInit->cnt))
        return NRF_ERROR_NO_MEM;

    if (state == STATE_TRANSFERING)
        return NRF_ERROR_INVALID_STATE;

    pInst->dataPin = pInit->dataPin;
    pInst->cnt = pInit->cnt;
    pInst->pBuffer = pInit->pBuffer;

    errCode = initI2s(pInit->dataPin, pInit->sckPin);
    if (errCode == NRF_ERROR_INVALID_STATE) // i2s already initialized, set pin to output manually
        nrf_gpio_cfg_output(pInit->dataPin);    /// TODO: is it necessary to set to zero?
    else if (errCode != NRF_SUCCESS)
        return errCode;

    state = STATE_IDLE;

    return NRF_SUCCESS;
}

ret_code_t ws2812_UpdateDataStream(ws2812_inst_t const* pInst, ws2812_update_t const* pData)
{
    if (pInst == NULL || pData == NULL || pData->pGRB == NULL)
        return NRF_ERROR_NULL;

    if (pData->len == 0 || (pData->start + pData->len) > pInst->cnt)
        return NRF_ERROR_INVALID_PARAM;

    dataConvert(pData->pGRB, pData->len * 3, &pInst->pBuffer[pData->start * 3]);

    return NRF_SUCCESS;
}

ret_code_t ws2812_SendDataStream(ws2812_inst_t const* pInst, ws2812_dataSentHandler_t pHandler)
{
    if (pInst == NULL)
        return NRF_ERROR_NULL;

    if (state == STATE_TRANSFERING)
        return NRF_ERROR_INVALID_STATE;

    nrfx_i2s_buffers_t buffer =
    {
        .p_tx_buffer = pInst->pBuffer
    };

    NRF_I2S->PSEL.SDOUT = pInst->dataPin;

    ret_code_t errCode = nrfx_i2s_start(&buffer, WS2812_BUFFER_SIZE_WORDS(pInst->cnt), 0);
    if (errCode == NRF_SUCCESS)
    {
        state = STATE_TRANSFERING;
        handler = pHandler;
    }

    return errCode;
}

/*ret_code_t ws2812_Update(ws2812_inst_t const* pInst, ws2812_update_t const* pData)
{
    /// TODO: maybe just resend old data if pData is NULL?

    if (pInst == NULL)
        return NRF_ERROR_NULL;

    if (pData != NULL)
    {
        if (pData->pGRB == NULL)
            return NRF_ERROR_NULL;

        if (pData->start >= pInst->cnt || (pData->start + pData->len) > pInst->cnt)
            return NRF_ERROR_INVALID_PARAM;
    }

    if (state != STATE_IDLE)
        return NRF_ERROR_INVALID_STATE;

    if (pData != NULL)
        dataConvert(pData->pGRB, pData->len * 3, &pInst->pBuffer[pData->start * 3]);

    nrfx_i2s_buffers_t buffer =
    {
        .p_tx_buffer = pInst->pBuffer
    };

    NRF_I2S->PSEL.SDOUT = pInst->dataPin;

    ret_code_t errCode = nrfx_i2s_start(&buffer, WS2812_BUFFER_SIZE_WORDS(pInst->cnt), 0);
    if (errCode == NRF_SUCCESS)
        state = STATE_TRANSFERING;

    return errCode;
}*/

/**END OF FILE*****************************************************************/
