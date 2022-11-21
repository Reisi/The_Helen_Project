/**
  ******************************************************************************
  * @file    i2c.c
  * @author  Thomas Reisnecker
  * @brief   tbd.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <string.h>

#include "nrfx_twim.h"
#include "app_util_platform.h"

#include "i2c.h"

/* External variables --------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/

/* Private defines -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static nrfx_twim_t twiInst = NRFX_TWIM_INSTANCE(0);

/* Private read only variables -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/* Public functions ----------------------------------------------------------*/
ret_code_t i2c_Init(uint32_t pinSDA, uint32_t pinSCL, nrf_twim_frequency_t frequency)
{
    nrfx_twim_config_t cfg = {0};

    cfg.scl = pinSCL;
    cfg.sda = pinSDA;
    cfg.frequency = frequency;
    cfg.interrupt_priority = APP_IRQ_PRIORITY_LOW;

    return nrfx_twim_init(&twiInst, &cfg, NULL, NULL);
}

int8_t i2c_Read(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint16_t len)
{
    ret_code_t errCode;
    nrfx_twim_xfer_desc_t xfer =
    {
        .address = devAddr,
    };

    nrfx_twim_enable(&twiInst);

    xfer.type           = NRFX_TWIM_XFER_TX;
    xfer.primary_length = 1;
    xfer.p_primary_buf  = &regAddr;
    errCode = nrfx_twim_xfer(&twiInst, &xfer, NRFX_TWIM_FLAG_TX_NO_STOP);
    if (errCode != NRF_SUCCESS)
        return -2;  // represents BMI160_E_COM_FAIL

    xfer.type           = NRFX_TWIM_XFER_RX;
    xfer.primary_length = len;
    xfer.p_primary_buf  = data;
    errCode = nrfx_twim_xfer(&twiInst, &xfer, 0);
    if (errCode != NRF_SUCCESS)
        return -2;  // represents BMI160_E_COM_FAIL

    nrfx_twim_disable(&twiInst);

    return 0;
}

int8_t i2c_Write(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint16_t len)
{
    ret_code_t errCode;
    uint8_t buffer[len+1];
    nrfx_twim_xfer_desc_t xfer =
    {
        .type           = NRFX_TWIM_XFER_TX,
        .address        = devAddr,
        .primary_length = len + 1,
        .p_primary_buf  = buffer,
    };

    buffer[0] = regAddr;
    memcpy(&buffer[1], data, len);

    nrfx_twim_enable(&twiInst);

    /*xfer.primary_length = 1;
    xfer.p_primary_buf  = &regAddr;
    errCode = nrfx_twim_xfer(&twiInst, &xfer, NRFX_TWIM_FLAG_TX_NO_STOP);
    if (errCode != NRF_SUCCESS)
        return -2;  // represents BMI160_E_COM_FAIL

    xfer.primary_length = len;
    xfer.p_primary_buf  = data;*/
    errCode = nrfx_twim_xfer(&twiInst, &xfer, 0);
    if (errCode != NRF_SUCCESS)
        return -2;  // represents BMI160_E_COM_FAIL

    nrfx_twim_disable(&twiInst);

    return 0;
}

/**END OF FILE*****************************************************************/

