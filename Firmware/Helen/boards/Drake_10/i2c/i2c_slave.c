/**
  ******************************************************************************
  * @file    i2c_slave.c
  * @author  Thomas Reisnecker
  * @brief   tbd.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "i2c_slave.h"
#include "nrfx_twis.h"
#include "nrf_gpio.h"
#include "nrfx_gpiote.h"
#include "nrf_log.h"

/* External variables --------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/

/* Private defines -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private read only variables -----------------------------------------------*/
static nrfx_twis_t const twisInst = NRFX_TWIS_INSTANCE(1);
static uint8_t const* pRead;
static uint8_t* pWrite;
static uint8_t bufferSize;
static uint8_t addrOffset;
static i2cs_handler_t eventHandler;

/* Private functions ---------------------------------------------------------*/
static void onReadReq(bool prepareBuffer)
{
    if (prepareBuffer)
    {
        // set address offset to last byte of buffer if out of boundaries
        if (addrOffset >= bufferSize)
            addrOffset = bufferSize - 1;

        uint8_t readLen = bufferSize - addrOffset;

        (void)nrfx_twis_tx_prepare(&twisInst, &pRead[addrOffset], readLen);
    }

    if (eventHandler)
    {
        i2cs_evt_t evt = {.type = I2CS_EVT_TYPE_READ};
        eventHandler(&evt);
    }
}

static void onReadDone(size_t cnt)
{
    if (addrOffset + cnt >= bufferSize)
        addrOffset = bufferSize - 1;
    else
        addrOffset += cnt;
}

static void onWriteReq()
{
    (void)nrfx_twis_rx_prepare(&twisInst, pWrite, bufferSize + 1);
}

static void onWriteDone(size_t cnt)
{
    addrOffset = pWrite[0];
    if (cnt > 1)
        addrOffset += cnt;
    if (addrOffset >= bufferSize)
        addrOffset = bufferSize - 1;

    if (cnt > 1 && eventHandler)
    {
        i2cs_evt_t evt =
        {
            .type = I2CS_EVT_TYPE_WRITE,
            .write =
            {
                .start = pWrite[0],
                .len = cnt - 1,
                .pData = &pWrite[1]
            }
        };
        eventHandler(&evt);
    }
}

static void twiHandler(nrfx_twis_evt_t const* const pEvt)
{
    switch (pEvt->type)
    {
    case NRFX_TWIS_EVT_READ_REQ:
        onReadReq(pEvt->data.buf_req);
        break;

    case NRFX_TWIS_EVT_READ_DONE:
        onReadDone(pEvt->data.tx_amount);
        break;

    case NRFX_TWIS_EVT_WRITE_REQ:
        if (pEvt->data.buf_req)
            onWriteReq();
        break;

    case NRFX_TWIS_EVT_WRITE_DONE:
        onWriteDone(pEvt->data.rx_amount);
        break;

    case NRFX_TWIS_EVT_READ_ERROR:
    case NRFX_TWIS_EVT_WRITE_ERROR:
    case NRFX_TWIS_EVT_GENERAL_ERROR:
        /// TODO: the obious
        break;

    default:
        break;
    }
}

/* Public functions ----------------------------------------------------------*/
ret_code_t i2cs_Init(i2cs_init_t const* pInit)
{
    ret_code_t errCode;

    if (pInit == NULL || pInit->pRead == NULL || pInit->pWrite == NULL)
        return NRF_ERROR_NULL;

    if (pInit->size == 0)
        return NRF_ERROR_INVALID_PARAM;

    nrfx_twis_config_t const cfg =
    {
        .addr               = {pInit->slaveAddr, 0},
        .scl                = pInit->scl,
        .sda                = pInit->sda,
        .scl_pull           = NRF_GPIO_PIN_PULLUP,
        .sda_pull           = NRF_GPIO_PIN_PULLUP,
        .interrupt_priority = APP_IRQ_PRIORITY_LOW
    };

    errCode = nrfx_twis_init(&twisInst, &cfg, twiHandler);
    if (errCode != NRF_SUCCESS)
        return NRF_SUCCESS;

    nrfx_twis_enable(&twisInst);    /// TODO: current draw of about 400-450µ

    pRead = pInit->pRead;
    pWrite = pInit->pWrite;
    bufferSize = pInit->size;
    eventHandler = pInit->evtHandler;

    return errCode;
}

/**END OF FILE*****************************************************************/

