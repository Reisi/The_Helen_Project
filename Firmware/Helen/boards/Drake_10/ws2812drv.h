/**
  ******************************************************************************
  * @file    ws2812drv.h
  * @author  Thomas Reisnecker
  * @brief   driver to control a chain of ws2812 leds.
  *          This driver uses the i2s interface to send the data to the leds.
  *          You must provide a buffer of reasonable size (use makro
  *          WS2812_BUFFER_SIZE_WORDS() for correct size). In addition to the
  *          data pin a clock pin is necessary for the i2s peripheral to work.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef WS2812DRV_H_INCLUDED
#define WS2812DRV_H_INCLUDED

/* Includes ------------------------------------------------------------------*/
#include "sdk_errors.h"

/* Exported types ------------------------------------------------------------*/
typedef struct
{
    uint8_t   dataPin;      // the pin number of data pin
    uint16_t  cnt;          // number of leds
    uint32_t* pBuffer;      // pointer to buffer used for sending
} ws2812_inst_t;

typedef void (*ws2812_dataSentHandler_t)(uint8_t dataPin);

typedef struct
{
    uint8_t                  dataPin;      // the pin number of data pin
    uint8_t                  sckPin;       // the pin number for sck pin, not used for transmission,
                                           // but i2s peripheral needs an active clock to work.
                                           // Can be any unused pin (avoid pin near radio!)
    uint16_t                 cnt;          // number of leds
    //ws2812_dataSentHandler_t dataSentHandler;
    uint32_t*                pBuffer;      // pointer to buffer used for sending
    uint16_t                 bufferSize;   // size of buffer in words
} ws2812_init_t;

typedef struct
{
    uint16_t       start;   // the led to start the update
    uint16_t       len;     // number of leds to update
    uint8_t const* pGRB;    // the array of led data (len * 3 bytes (green, red, blue))
} ws2812_update_t;

/* Exported constants --------------------------------------------------------*/
#define WS2812_RESET_BUF_CNT_WORDS          28 /// TODO: waste of memory, maybe try other way to generate reset pulse

/* Exported macros -----------------------------------------------------------*/
#define WS2812_BUFFER_SIZE_WORDS(ledCnt)    (ledCnt * 3 + WS2812_RESET_BUF_CNT_WORDS)

/* Exported functions ------------------------------------------------------- */
/** @brief function to initialize the ws2812 driver
 *
 * @param[in/out] pInst  ws2812 instance to initialize
 * @param[in]     pInit  init structure
 * @return NRF_SUCCESS, NRF_ERROR_NULL, NRF_ERROR_NO_MEM if buffer too small or
 *         a propagated error from nrfx_i2s_init()
 */
ret_code_t ws2812_Init(ws2812_inst_t* pInst, ws2812_init_t const* pInit);

ret_code_t ws2812_UpdateDataStream(ws2812_inst_t const* pInst, ws2812_update_t const* pData);

ret_code_t ws2812_SendDataStream(ws2812_inst_t const* pInst, ws2812_dataSentHandler_t pHandler);

/** @brief function to update the led chain
 *
 * @param[in] pInst  ws2812 instance
 * @param[in] pData  led data, NULL to resend last data
 * @return NRF_SUCCESS, NRF_ERROR_NULL, NRF_ERROR_INVALID_PARAM,
 *         NRF_ERROR_INVALID_STATE if already transmitting or a propagated error
 *         from nrfx_i2s_start()
 */
//ret_code_t ws2812_Update(ws2812_inst_t const* pInst, ws2812_update_t const* pData);

#endif // WS2812DRV_H_INCLUDED

/**END OF FILE*****************************************************************/

