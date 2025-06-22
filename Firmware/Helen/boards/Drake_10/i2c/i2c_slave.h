/**
  ******************************************************************************
  * @file    i2c_slave.h
  * @author  Thomas Reisnecker
  * @brief   tbd.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef I2C_SLAVE_H_INCLUDED
#define I2C_SLAVE_H_INCLUDED

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

#include "sdk_errors.h"
#include "nrf_twim.h"

/* Exported types ------------------------------------------------------------*/
typedef enum
{
    I2CS_EVT_TYPE_READ, // read operation has been initiated (time to update data)
    I2CS_EVT_TYPE_WRITE // data has been written
} i2cs_evt_type_t;

typedef struct
{
    i2cs_evt_type_t type;
    struct
    {
        uint8_t start;         // start address of write operation
        uint8_t len;           // length of write operation
        uint8_t const* pData;   // data
    } write;
} i2cs_evt_t;

typedef void (*i2cs_handler_t)(i2cs_evt_t const* pEvt);

typedef struct
{
    uint8_t slaveAddr;          // the slave address to react to
    uint8_t scl;                // scl pin number
    uint8_t sda;                // sda pin number
    uint8_t const* pRead;       // pointer to data to read
    uint8_t* pWrite;            // pointer to write data (
    uint8_t size;               // size of read data, write data must be one byte bigger (for address byte)
    i2cs_handler_t evtHandler;  // event handler, can be NULL if only read operations are sufficient)
} i2cs_init_t;

/* Exported constants --------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
/** @brief function to initialize the i2c slave module
 *
 * @param[in] pInit
 * @return NRF_SUCCESS, NRF_ERROR_NULL, NRF_ERROR_NOT_FOUND
 *
 */
ret_code_t i2cs_Init(i2cs_init_t const* pInit);

#endif // I2C_SLAVE_H_INCLUDED

/**END OF FILE*****************************************************************/

