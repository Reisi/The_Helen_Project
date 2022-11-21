/**
  ******************************************************************************
  * @file    com.c
  * @author  Thomas Reisnecker
  * @brief   timer based 1-wire communication module
  ******************************************************************************
  */

/* logger configuration ----------------------------------------------_-------*/
#define COM_CONFIG_LOG_ENABLED 1 /// TODO: just temporary until project specific sdk_config
#define COM_CONFIG_LOG_LEVEL   3

#define NRF_LOG_MODULE_NAME com
#if COM_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL COM_CONFIG_LOG_LEVEL
#else //BASE_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL 0
#endif //BASE_CONFIG_LOG_ENABLED
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

/* Includes ------------------------------------------------------------------*/
#include "nrfx_timer.h"
#include "nrfx_spim.h"
#include "nrf_gpio.h"
#include "nrfx_gpiote.h"

#include "debug.h"
#include "comreloaded.h"

#include "SEGGER_RTT.h"

/* External variables --------------------------------------------------------*/

/* Private defines -----------------------------------------------------------*/
#define MAX_BYTES       18              // up to 15 data bytes + id, ctrl and crc byte
#define MAX_BITLENGTH   (MAX_BYTES * 8) // max bitlength without stuff bits
#define MAX_STUFFLENGTH 4               // stuff bit necessary after 5 bits (0..4)
#define END_OF_FRAME    5               // 5 bits of high level indicates end of frame
#define IDLE_LENGTH     8               // 8 bits of high level indicates bus idle state

#define SPIM_BASE_CONFIG                                     \
{                                                            \
    .miso_pin       = NRFX_SPIM_PIN_NOT_USED,                \
    .mosi_pin       = NRFX_SPIM_PIN_NOT_USED,                \
    .ss_pin         = NRFX_SPIM_PIN_NOT_USED,                \
    .ss_active_high = false,                                 \
    .irq_priority   = APP_IRQ_PRIORITY_LOWEST,               \
    .orc            = 0xFF,                                  \
    .frequency      = NRF_SPIM_FREQ_125K,                    \
    .mode           = NRF_SPIM_MODE_0,                       \
    .bit_order      = NRF_SPIM_BIT_ORDER_MSB_FIRST,          \
}

/* Private typedef -----------------------------------------------------------*/
/** @brief reception structure */
typedef struct
{
    uint8_t dataByte;   // the current databyte which is received
    uint8_t stuffCnt;   // counter to determine if stuffbyte is necessary
    uint8_t bitCnt;     // bit counter
    bool lastBit : 1;   // state of last bit
    uint8_t data[MAX_BYTES];
} receive_t;

/** @brief transmission structure used in timer mode */
typedef struct
{
    uint8_t dataByte;   // the current databyte which is sent or received
    uint8_t stuffCnt;   // counter to determine if stuffbyte is necessary
    uint8_t bitCnt;     // bit counter
    uint8_t bitSize;    // total bit size
    bool lastBit : 1;   // state of last bit
    uint8_t data[MAX_BYTES];
} transmitTim_t;

/** @brief transmission structure used in spi mode */
typedef struct
{
    uint16_t len;       // length of spi message
    uint8_t data[348];  // spi frequency is 16 times higher than com frequency,
                        // so each com bit needs 2 spi bytes
                        // 2 * (   1    +   18*8  ) * 6/5
                        //     start bit  databits  stuffbits (worst case)
} transmitSpi_t;

typedef union
{
    transmitTim_t tim;
    transmitSpi_t spi;
} transmit_t;

/* Private macros ------------------------------------------------------------*/

/* Private function prototype ------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static nrfx_timer_config_t const timerConfig =
{
    .frequency =          NRF_TIMER_FREQ_2MHz,  // 2mhz with 8 bits width leads to com frequency of 7812.5Hz
    .mode =               NRF_TIMER_MODE_TIMER,
    .bit_width =          NRF_TIMER_BIT_WIDTH_8,
    .interrupt_priority = APP_IRQ_PRIORITY_MID, // higher than sd low, but lower than current regulator
    .p_context =          NULL
};

static nrfx_timer_t timerInst = NRFX_TIMER_INSTANCE(2);
static nrfx_spim_t spiInst = NRFX_SPIM_INSTANCE(2);
static com_init_t   config;
static receive_t rcv;
static transmit_t trsm;

/* Private functions ---------------------------------------------------------*/
static bool isEnabled()
{
    return nrfx_timer_is_enabled(&timerInst);
}

static void enable()
{
    nrfx_timer_enable(&timerInst);
}

static void transmitStart()
{
    if (config.txPin == COM_PIN_NOT_USED)
        return;

    if (isEnabled())
        nrfx_timer_clear(&timerInst);   // reset timer for correct length of start bit in timer mode
    else
        enable();

    if (config.dummyPin == COM_PIN_NOT_USED)
    {
        nrfx_timer_compare_int_enable(&timerInst, NRF_TIMER_CC_CHANNEL0);
        nrf_gpio_pin_clear(config.txPin);   // generate start bit
    }
    else
    {
        ret_code_t errCode;
        nrfx_spim_xfer_desc_t xfer;
        xfer.p_tx_buffer = trsm.spi.data;
        xfer.tx_length = trsm.spi.len;
        xfer.p_rx_buffer = NULL;
        xfer.rx_length = 0;
        errCode = nrfx_spim_xfer(&spiInst, &xfer, 0);
        LOG_ERROR_CHECK("spi xfer failed, error: %d", errCode);
    }
}

static void transmitTimEnd()
{
    nrf_gpio_pin_set(config.txPin); // set output to idle state
    nrfx_timer_compare_int_disable(&timerInst, NRF_TIMER_CC_CHANNEL0);
}

static void resetReceiveBuffer(receive_t* pBuf)
{
    pBuf->dataByte = 0;
    pBuf->bitCnt = 0;
    pBuf->stuffCnt = 4;     // to ensure ignoring of startbit
    pBuf->lastBit = true;   // idle state of bus
    memset(pBuf->data, 0, MAX_BYTES);
}

static void disable()
{
    if (config.txPin != COM_PIN_NOT_USED)
        nrf_gpio_pin_set(config.txPin); // set to idle state

    nrfx_timer_disable(&timerInst);
    nrfx_timer_clear(&timerInst);
    nrfx_timer_clear(&timerInst);
    nrfx_timer_compare(&timerInst, NRF_TIMER_CC_CHANNEL0, 255, false);
    nrfx_timer_compare(&timerInst, NRF_TIMER_CC_CHANNEL1, 95, true);

    resetReceiveBuffer(&rcv);
}

/** @brief function to calculate the crc byte */
static uint8_t crc8Compute(uint8_t const* pData, uint16_t size)
{
    uint8_t crc = 0xFF;
    while (size)
    {
        crc = crc ^ *pData;
        for (uint8_t i = 0; i < 8; i++)
        {
            if (crc & 0x01)
                crc = (crc >> 1) ^ 0x8C;
            else
                crc >>= 1;
        }
        pData++;
        size--;
    }
    return crc;
}

/** @brief function to decode the incoming bit stream
 *
 * @param[in/out] pBuf   the receive buffer
 * @param[in]     state  the current com pin state
 * @return        NRF_ERROR_BUSY if message is not complete yet
 *                NRF_SUCCESS for correct received messages
 *                NRF_ERROR_INVALID_DATA for complete messages, but crc error
 *                NRF_ERROR_INVALID_LENGTH for incorrect length or buffer overflow
 *                NRF_ERROR_TIMEOUT if stuff bit is missing
 */
static ret_code_t receiveBit(receive_t* pBuf, bool state)
{
    ret_code_t retVal = NRF_ERROR_BUSY;

    if (pBuf == NULL)
        return NRF_ERROR_NULL;

    // databit
    if (pBuf->stuffCnt < MAX_STUFFLENGTH)
    {
        uint_fast16_t dataByte = pBuf->dataByte;
        dataByte <<= 1;             // add bit
        if (state)
            dataByte += 1;
        pBuf->dataByte = dataByte;  // and save

        if ((pBuf->bitCnt & 0x07) == 0x07)  // byte complete?
        {
            //SEGGER_RTT_printf(0, "0x%02x ", pBuf->dataByte);
            pBuf->data[pBuf->bitCnt >> 3] = pBuf->dataByte;
        }

        if (pBuf->bitCnt < MAX_BITLENGTH)
            pBuf->bitCnt++;
        else
        {
            //SEGGER_RTT_printf(0, "BL ");
            retVal = NRF_ERROR_INVALID_LENGTH;  // the received message is too long
        }
    }
    // EOF
    else if (state && pBuf->lastBit && pBuf->stuffCnt == END_OF_FRAME)
    {
        uint16_t len = (pBuf->data[1] & 0x0F) + 3;
        if ((pBuf->bitCnt >> 3) == len) // correct length?
        {
            if (crc8Compute(pBuf->data, len - 1) == pBuf->data[len - 1])
            {
                //SEGGER_RTT_printf(0, "OK ");
                retVal = NRF_SUCCESS;             // valid message received
            }
            else
            {
                //SEGGER_RTT_printf(0, "CRC ");
                retVal = NRF_ERROR_INVALID_DATA;  // crc error
            }
        }
        else
        {
            //SEGGER_RTT_printf(0, "LEN ");
            retVal = NRF_ERROR_INVALID_LENGTH;    // received message has not reported length
        }
    }

    // stuff bit handling
    if (state == pBuf->lastBit)
    {
        if (pBuf->stuffCnt < UINT8_MAX)
            pBuf->stuffCnt++;
        if (pBuf->stuffCnt > IDLE_LENGTH)
        {
            //SEGGER_RTT_printf(0, "TO ");
            retVal = NRF_ERROR_TIMEOUT;
        }
    }
    else
    {
        pBuf->stuffCnt = 0;
        pBuf->lastBit = state;
    }

    return retVal;
}

/** @brief function to prepare the timer transmission struct */
static void prepareTimTransmission(transmitTim_t* pBuf, com_message_t const* pMsg)
{
    pBuf->bitCnt = 0;
    pBuf->stuffCnt = 0;                 // start bit changes the level, so definitely 0
    pBuf->lastBit = false;              // state of start bit
    pBuf->bitSize = (pMsg->len + 3) * 8;
    pBuf->dataByte = pMsg->id;          // preload first byte
    pBuf->data[0] = pMsg->id;
    pBuf->data[1] = (pMsg->type << 4) + pMsg->len;
    memcpy(&pBuf->data[2], pMsg->data, pMsg->len);
    pBuf->data[2 + pMsg->len] = crc8Compute(pBuf->data, 2 + pMsg->len);
}

/** @brief function to generate the transmission bit stream
 * @note this function is not just used in timer mode, but also to generate the spi bitstream
 * @return true if next bit is high, false if low
 */
static bool transmitTimBit(transmitTim_t* pBuf)
{
    uint16_t dataByte = pBuf->dataByte;

    /*if (pBuf->bitCnt == 0)
    {
        SEGGER_RTT_printf(0, "0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\r\n",
                          pBuf->data[0], pBuf->data[1], pBuf->data[2], pBuf->data[3], pBuf->data[4], pBuf->data[5]);
    }*/
    if (pBuf->bitCnt < pBuf->bitSize)
    {
        // data bit
        if (pBuf->stuffCnt < MAX_STUFFLENGTH)
        {
            dataByte <<= 1;

            if ((pBuf->bitCnt++ & 0x07) == 0x07)    // load next byte?
                pBuf->dataByte = pBuf->data[pBuf->bitCnt >> 3];
            else
                pBuf->dataByte = (uint8_t)dataByte;

            dataByte = dataByte & (1 << 8);         // dataByte holds the state of the next bit now
            //SEGGER_RTT_printf(0, "%d", dataByte ? 1 : 0);
        }
        // stuff bit
        else
        {
            dataByte = pBuf->lastBit ? 0 : 1;
            //SEGGER_RTT_printf(0, "S");
        }

        if (dataByte == pBuf->lastBit)
            pBuf->stuffCnt++;
        else
        {
            pBuf->stuffCnt = 0;
            pBuf->lastBit = (bool)dataByte;
        }

        return (bool)dataByte;
    }
    else
    {
        pBuf->bitSize = 0;
        //SEGGER_RTT_printf(0, "\r\n");
        return true;    // output is high in idle state
    }
}

/** @brief function to prepare the spi transmission structure
 * @note spi frequency is 16 times higher than com frequency, therfore each com
 *       bits uses 2 spi bytes
 */
static void prepareSpiTransmission(transmitSpi_t* pBuf, com_message_t const* pMsg)
{
    transmitTim_t trans;            // timer transmission structure to generate bitstream

    prepareTimTransmission(&trans, pMsg);

    pBuf->data[pBuf->len++] = 0;    // start bit
    pBuf->data[pBuf->len++] = 0;

    while (trans.bitSize && pBuf->len < sizeof(pBuf->data))
    {
        if (transmitTimBit(&trans))
        {
            pBuf->data[pBuf->len++] = 0xFF; // high stee
            pBuf->data[pBuf->len++] = 0xFF;
        }
        else
        {
            pBuf->data[pBuf->len++] = 0;    // low state
            pBuf->data[pBuf->len++] = 0;
        }
    }

    if (pBuf->len >= sizeof(pBuf->data))    // buffer too small, which means I miscalculated
        return; //NRF_ERROR_INVALID_LENGTH
}

/** @brief function to check if com line is idle
 * @note call this function only once per com clock cycle with the current com line state
 */
static bool isLineIdle(bool state)
{
    static uint8_t idleCnt;

    if (state && idleCnt >= IDLE_LENGTH)
        return true;

    if (state)
        idleCnt++;
    else
        idleCnt = 0;

    return false;
}

/** @brief function to convert message in buffer to message structure and send an event
 */
static inline void convertAndReportSuccess(receive_t const* pBuf)
{
    com_message_t msg;
    com_event_t evt;

    msg.id = pBuf->data[0];
    msg.type = pBuf->data[1] >> 4;
    msg.len = pBuf->data[1] & 0x0F;
    memcpy(msg.data, &pBuf->data[2], msg.len);

    evt.type = COM_EVT_MESSAGE_RECEIVED;
    evt.pMessage = &msg;

    config.pHandler(&evt);
}

/** @brief function to send an event in case of an crc error
 */
static inline void reportCRCError()
{
    com_event_t evt = {.type = COM_EVT_CRC_ERROR};
    config.pHandler(&evt);
}

/** @brief the reception interrupt handler
 */
static void receive()
{
    ret_code_t errCode;
    bool inputState;

    //if (config.rxPin == COM_PIN_NOT_USED || config.pHandler == NULL)
    //    return;

    inputState = nrf_gpio_pin_read(config.rxPin);

    /// TODO: collision check if also transmitting?

    errCode = receiveBit(&rcv, inputState);     // decode bitstream
    if (errCode == NRF_SUCCESS)                 // message complete
        convertAndReportSuccess(&rcv);
    else if (errCode == NRF_ERROR_INVALID_DATA) // message complete but crc error
        reportCRCError();
    else if (errCode != NRF_ERROR_BUSY)         // reception errors
    {
        /// TODO: report length error?
    }

    if (isLineIdle(inputState))
    {
        if ((config.dummyPin == COM_PIN_NOT_USED && trsm.tim.bitSize) ||
            (config.dummyPin != COM_PIN_NOT_USED && trsm.spi.len))
            transmitStart();    // start sending
        else
            disable();          // or disable
    }
}

/** @brief the transmitting interrupt handler in timer mode
 */
static void transmitTim()
{
    /// TODO: this check should not be necessary
    if (config.txPin == COM_PIN_NOT_USED || config.dummyPin != COM_PIN_NOT_USED)
        return;

    bool nextBit = transmitTimBit(&trsm.tim);   // encode bitstream

    if (nextBit)                                // high bit
        nrf_gpio_pin_set(config.txPin);
    else                                        // low bit
        nrf_gpio_pin_clear(config.txPin);

    if (trsm.tim.bitSize == 0)                  // all sent?
    {
        transmitTimEnd();
    }
}

static void timerHandler(nrf_timer_event_t event, void* pContext)
{
    (void)pContext;

    if (event == NRF_TIMER_EVENT_COMPARE0)  // used for transmitting
        transmitTim();
    if (event == NRF_TIMER_EVENT_COMPARE1)  // used for receiving
        receive();
}

/** @brief pin change interrupt handler used to start reception and syncing timer *
 */
static void pinChangeSync(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    (void)pin;
    (void)action;

    if (isEnabled())
    {
        uint32_t capture;
        capture = nrfx_timer_capture_get(&timerInst, NRF_TIMER_CC_CHANNEL3);
        capture += 95;  /// TODO: determine time from real pin change to this point
        capture &= 0xFF;
        nrf_timer_cc_write(timerInst.p_reg, NRF_TIMER_CC_CHANNEL1, capture);
    }
    else
    {
        enable();
    }
}

/** @brief spi done event handler
 */
static void spiEventHandler(nrfx_spim_evt_t const *pEvent, void *pContext)
{
    (void)pEvent;
    (void)pContext;

    trsm.spi.len = 0;   // just mark buffer as free
}

/* Public functions -----------------------------------------------------------*/
ret_code_t com_Init(com_init_t const* pInit)
{
    if (pInit == NULL || pInit->pHandler == NULL)
        return NRF_ERROR_NULL;

    if (pInit->rxPin == COM_PIN_NOT_USED && pInit->txPin != COM_PIN_NOT_USED)
        return NRF_ERROR_INVALID_PARAM;

    config = *pInit;

    // if com is not used, initialization is already finished
    if (pInit->rxPin == COM_PIN_NOT_USED && pInit->txPin == COM_PIN_NOT_USED)
        return NRF_SUCCESS;

    ret_code_t errCode;

    // request XTAL
    sd_clock_hfclk_request();

    // timer configuration
    errCode = nrfx_timer_init(&timerInst, &timerConfig, &timerHandler);
    LOG_ERROR_RETURN("timer init failed, error: %d", errCode);
    nrfx_timer_clear(&timerInst);
    nrfx_timer_compare(&timerInst, NRF_TIMER_CC_CHANNEL0, 255, false);   // used to transmit
    nrfx_timer_compare(&timerInst, NRF_TIMER_CC_CHANNEL1, 95, true);     // used to receive

    // spi configuration
    if (pInit->dummyPin != COM_PIN_NOT_USED)
    {
        nrfx_spim_config_t spiConfig = SPIM_BASE_CONFIG;
        spiConfig.sck_pin = pInit->dummyPin;
        errCode = nrfx_spim_init(&spiInst, &spiConfig, spiEventHandler, NULL);
        LOG_ERROR_RETURN("spi init failed, error: %d", errCode);
    }

    // initialize gpiote if not already done
    if (!nrfx_gpiote_is_init())
    {
        errCode = nrfx_gpiote_init();
        LOG_ERROR_RETURN("gpiote init failed, error: %d", errCode);
    }

    // gpiote configuration
    nrfx_gpiote_in_config_t rxConfig = NRFX_GPIOTE_CONFIG_IN_SENSE_TOGGLE(false);
    rxConfig.skip_gpio_setup = true;
    if (pInit->rxPin == pInit->txPin)
        rxConfig.is_watcher = true;
    errCode = nrfx_gpiote_in_init(pInit->rxPin, &rxConfig, &pinChangeSync);
    LOG_ERROR_RETURN("gpiote pin init failed, error: %d", errCode);

    // pin configuration for shared pin
    if (pInit->rxPin == pInit->txPin)
    {
        nrf_gpio_cfg(pInit->txPin, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_CONNECT,
                     NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_H0D1, NRF_GPIO_PIN_NOSENSE);
        nrf_gpio_pin_set(pInit->rxPin);
    }
    // pin configuration for separate pins
    else
    {
         if (pInit->txPin != COM_PIN_NOT_USED)
         {
             nrf_gpio_cfg(pInit->txPin, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT,
                          NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_H0D1, NRF_GPIO_PIN_NOSENSE);
            nrf_gpio_pin_set(pInit->txPin);
         }
        nrf_gpio_cfg_input(pInit->rxPin, NRF_GPIO_PIN_NOPULL);
    }

    // spi init function sets mosi pin to low, therefore init function is
    // called with diconnected mosi pin and pins are assigned manually after
    // gpio configuration to prevent low pulse on com line
    if (pInit->dummyPin != COM_PIN_NOT_USED)
        nrf_spim_pins_set(spiInst.p_reg, pInit->dummyPin, pInit->txPin, NRF_SPIM_PIN_NOT_CONNECTED);

    nrfx_gpiote_in_event_enable(pInit->rxPin, true);

    return NRF_SUCCESS;
}

ret_code_t com_Put(com_message_t const* pMsg)
{
    // not initialized
    if (config.pHandler == NULL)
        return NRF_ERROR_INVALID_STATE;

    // initialized, but disabled
    if (config.txPin == COM_PIN_NOT_USED && config.rxPin == COM_PIN_NOT_USED)
        return NRF_SUCCESS;

    // passive mode, no sending
    if (config.txPin == COM_PIN_NOT_USED)
        return NRF_ERROR_NOT_SUPPORTED;

    if (pMsg == NULL)
        return NRF_ERROR_NULL;

    if (config.dummyPin == COM_PIN_NOT_USED)
    {
        if (trsm.tim.bitSize)
            return NRF_ERROR_NO_MEM;
        prepareTimTransmission(&trsm.tim, pMsg);
    }
    else
    {
        if (trsm.spi.len)
            return NRF_ERROR_NO_MEM;
        prepareSpiTransmission(&trsm.spi, pMsg);
    }

    if (!isEnabled())
    {
        enable();
        transmitStart();
    }

    return NRF_SUCCESS;
}

/**END OF FILE*****************************************************************/
