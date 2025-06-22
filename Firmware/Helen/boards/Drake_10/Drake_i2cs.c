/**
  ******************************************************************************
  * @file    Drake_i2cs.c
  * @author  Thomas Reisnecker
  * @brief   i2c slave interface for Drake Hardware 1.0
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
#include "Drake_i2cs.h"
#include "i2c_slave.h"
#include "drake_regs.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"

/* External variables --------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/
#if APP_TIMER_CONFIG_RTC_FREQUENCY != 1
#error "motor timeout values are stored in q18_14_t and need to be converted if APP_TIMER_CONFIG_RTC_FREQUENCY!== 1"
#endif // APP_TIMER_CONFIG_RTC_FREQUENCY

/* Private defines -----------------------------------------------------------*/
#define I2CS_SDA_PIN            20
#define I2CS_SCL_PIN            21

/* Private variables ---------------------------------------------------------*/
static uint8_t readBuffer[DRK_REG_CNT];
static uint8_t writeBuffer[sizeof(readBuffer) + 1];
static Drake_I2cs_eventHandler_t evtHandler;

/* Private read only variables -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
static void i2cSlaveEventHandler(i2cs_evt_t const* pEvt)
{
    if (evtHandler == NULL)
        return;

    Drake_I2cs_event_t evt;

    switch (pEvt->type)
    {
    case I2CS_EVT_TYPE_READ:
        evt.type = DRAKE_I2CS_READ_OPERATION;
        evtHandler(&evt);
        break;

    case I2CS_EVT_TYPE_WRITE:
        // extract sys off request
        if (pEvt->write.start <= DRK_REG_PWR && (pEvt->write.start + pEvt->write.len) > DRK_REG_PWR)
        {
            if (pEvt->write.pData[DRK_REG_PWR - pEvt->write.start] & DRK_PWR_SYS_OFF_MASK)
            {
                evt.type = DRAKE_I2CS_SYS_OFF_REQ;
                evtHandler(&evt);
            }
        }
        // extract motor position data
        if (pEvt->write.start <= DRK_REG_MDATA && (pEvt->write.start + pEvt->write.len) > DRK_REG_MDATA)
        {
            evt.type = DRAKE_I2CS_MOT_POS_RCVD;
            evt.motorPosition = pEvt->write.pData[DRK_REG_MDATA - pEvt->write.start];
            evtHandler(&evt);
        }
        // extract motor config
        if (pEvt->write.start <= DRK_REG_MTCFG3 || (pEvt->write.start + pEvt->write.len) > DRK_REG_MTCFG0)
        {
            Drake_I2cs_motorConfig_t cfg;

            evt.type = DRAKE_I2CS_MOT_CFG_RCVD;
            evt.pMotorCfg = &cfg;

            if (pEvt->write.start <= DRK_REG_MTCFG0 && (pEvt->write.start + pEvt->write.len) > DRK_REG_MTCFG0)
                cfg.dutyCycleOpen = pEvt->write.pData[DRK_REG_MTCFG0 - pEvt->write.start];
            else
                cfg.dutyCycleOpen = readBuffer[DRK_REG_MTCFG0];

            if (pEvt->write.start <= DRK_REG_MTCFG1 && (pEvt->write.start + pEvt->write.len) > DRK_REG_MTCFG1)
                cfg.dutyCycleClose = pEvt->write.pData[DRK_REG_MTCFG1 - pEvt->write.start];
            else
                cfg.dutyCycleClose = readBuffer[DRK_REG_MTCFG1];

            if (pEvt->write.start <= DRK_REG_MTCFG2 && (pEvt->write.start + pEvt->write.len) > DRK_REG_MTCFG1)
                cfg.timeoutOpen = pEvt->write.pData[DRK_REG_MTCFG2 - pEvt->write.start];
            else
                cfg.timeoutOpen = readBuffer[DRK_REG_MTCFG2];

            if (pEvt->write.start <= DRK_REG_MTCFG3 && (pEvt->write.start + pEvt->write.len) > DRK_REG_MTCFG3)
                cfg.timeoutClose = pEvt->write.pData[DRK_REG_MTCFG3 - pEvt->write.start];
            else
                cfg.timeoutClose = readBuffer[DRK_REG_MTCFG3];

            evtHandler(&evt);
        }
        // extract clear/start calibration
        if (pEvt->write.start <= DRK_REG_MTCAL0 && (pEvt->write.start + pEvt->write.len) > DRK_REG_MTCAL0)
        {
            if (pEvt->write.pData[DRK_REG_MTCAL0 - pEvt->write.start] & DRK_CALIB_THC_MASK)
                evt.type = DRAKE_I2CS_MOT_CAL_THR_CLEAR;
            else if (pEvt->write.pData[DRK_REG_MTCAL0 - pEvt->write.start] & DRK_CALIB_FOC_MASK)
                evt.type = DRAKE_I2CS_MOT_CAL_FO_CLEAR;
            else if (pEvt->write.pData[DRK_REG_MTCAL0 - pEvt->write.start] & DRK_CALIB_THS_MASK)
                evt.type = DRAKE_I2CS_MOT_CAL_THR_START;
            else if (pEvt->write.pData[DRK_REG_MTCAL0 - pEvt->write.start] & DRK_CALIB_FOS_MASK)
                evt.type = DRAKE_I2CS_MOT_CAL_FO_START;
            else
                break;

            evtHandler(&evt);
        }
        // extract travel offset clear
        if (pEvt->write.start <= DRK_REG_TRVCAL0 && (pEvt->write.start + pEvt->write.len) > DRK_REG_TRVCAL0)
        {
            if (pEvt->write.pData[DRK_REG_TRVCAL0 - pEvt->write.start] & DRK_CALIB_TOC_MASK)
                evt.type = DRAKE_I2CS_TRV_CAL_OFF_CLEAR;
            else
                break;

            evtHandler(&evt);
        }
        // extract travel offset start
        if (pEvt->write.start <= DRK_REG_TRVCAL1 && (pEvt->write.start + pEvt->write.len) > DRK_REG_TRVCAL2)
        {
            evt.type = DRAKE_I2CS_TRV_CAL_OFF_START;
            evt.travel = pEvt->write.pData[DRK_REG_TRVCAL1 - pEvt->write.start] + (pEvt->write.pData[DRK_REG_TRVCAL2 - pEvt->write.start] << 8);

            evtHandler(&evt);
        }
        // extract indexed config
        if (pEvt->write.start <= DRK_REG_INDPOS3 || (pEvt->write.start + pEvt->write.len) > DRK_REG_INDCFG0)
        {
            Drake_I2cs_indexedConfig_t cfg;

            evt.type = DRAKE_I2CS_IND_CFG_RCVD;
            evt.pIndexedCfg = &cfg;

            if (pEvt->write.start <= DRK_REG_INDCFG0 && (pEvt->write.start + pEvt->write.len) > DRK_REG_INDCFG0)
                cfg.openBlocking = pEvt->write.pData[DRK_REG_INDCFG0 - pEvt->write.start];
            else
                cfg.openBlocking = readBuffer[DRK_REG_INDCFG0];

            if (pEvt->write.start <= DRK_REG_INDCFG1 && (pEvt->write.start + pEvt->write.len) > DRK_REG_INDCFG1)
                cfg.closeTimeout = pEvt->write.pData[DRK_REG_INDCFG1 - pEvt->write.start];
            else
                cfg.closeTimeout = readBuffer[DRK_REG_INDCFG1];

            for (uint8_t i = 0; i < DRK_NUM_OF_IND_POS; i++)
            {
                if (pEvt->write.start <= (DRK_REG_INDPOS0 + i) && (pEvt->write.start + pEvt->write.len) > (DRK_REG_INDPOS0 + i))
                    cfg.positions[i] = pEvt->write.pData[DRK_REG_INDPOS0 + i - pEvt->write.start];
                else
                    cfg.positions[i] = readBuffer[DRK_REG_INDPOS0 + i];
            }

            evtHandler(&evt);
        }
        break;
    }
}

static bool isSlaveBusPresent()
{
    bool isPresent;

    nrf_gpio_cfg_input(I2CS_SCL_PIN, NRF_GPIO_PIN_PULLDOWN);
    nrf_gpio_cfg_input(I2CS_SDA_PIN, NRF_GPIO_PIN_PULLDOWN);
    nrf_delay_ms(5);

    // if connected the 3k3 hardware pullups will win against the 13k internal pulldowns
    isPresent = (bool)nrf_gpio_pin_read(I2CS_SCL_PIN) && (bool)nrf_gpio_pin_read(I2CS_SDA_PIN);

    nrf_gpio_cfg_default(I2CS_SCL_PIN);
    nrf_gpio_cfg_default(I2CS_SDA_PIN);

    return isPresent;
}

static void initData(Drake_I2cs_init_t const* pInit)
{
    if (pInit->isMotorSensorPresent)
        readBuffer[DRK_REG_STATUS0] |= DRK_STATUS_MPR_MASK;

    if (pInit->isTravelSensorPresent)
        readBuffer[DRK_REG_STATUS0] |= DRK_STATUS_TPR_MASK;

    readBuffer[DRK_REG_MDATA] = pInit->motorPosition;
    readBuffer[DRK_REG_TDATA] = pInit->travel;

    readBuffer[DRK_REG_WHOAMI] = DRK_WHO_AM_I;

    Drake_I2cs_UpdateMotorConfig(pInit->pMotorCfg);
    Drake_I2cs_UpdateSensorConfig(pInit->pSensorCfg);
    Drake_I2cs_UpdateTravelSensor(pInit->pTravelSens);
    Drake_I2cs_UpdateIndexedConfig(pInit->pIndexedCfg);
}

/* Public functions ----------------------------------------------------------*/
ret_code_t Drake_I2cs_Init(Drake_I2cs_init_t const* pInit)
{
    ret_code_t errCode;

    if (pInit == NULL || pInit->handler == NULL)
        return NRF_ERROR_NULL;

    if (!isSlaveBusPresent())
        return NRF_ERROR_NOT_FOUND;

    static i2cs_init_t const i2csInit =
    {
        .slaveAddr  = DRK_ADDR,
        .scl        = I2CS_SCL_PIN,
        .sda        = I2CS_SDA_PIN,
        .pRead      = readBuffer,
        .pWrite     = writeBuffer,
        .size       = DRK_REG_CNT,
        .evtHandler = i2cSlaveEventHandler
    };

    errCode = i2cs_Init(&i2csInit);
    if (errCode != NRF_SUCCESS)
        return errCode;

    initData(pInit);

    evtHandler = pInit->handler;

    return NRF_SUCCESS;
}

void Drake_I2cs_UpdateMotPos(q8_t motorPosition)
{
    if (readBuffer[DRK_REG_STATUS1] & DRK_STATUS_MDR_MASK)
        readBuffer[DRK_REG_STATUS1] |= DRK_STATUS_MOR_MASK;
    readBuffer[DRK_REG_STATUS1] |= DRK_STATUS_MDR_MASK;
    readBuffer[DRK_REG_MDATA] = motorPosition;
}

void Drake_I2cs_UpdateTravel(q8_t travel)
{
    if (readBuffer[DRK_REG_STATUS1] & DRK_STATUS_TDR_MASK)
        readBuffer[DRK_REG_STATUS1] |= DRK_STATUS_TOR_MASK;
    readBuffer[DRK_REG_STATUS1] |= DRK_STATUS_TDR_MASK;
    readBuffer[DRK_REG_TDATA] = travel;
}

void Drake_I2cs_UpdateMotorConfig(Drake_I2cs_motorConfig_t const* pMotorCfg)
{
    readBuffer[DRK_REG_MTCFG0] = pMotorCfg->dutyCycleOpen;
    readBuffer[DRK_REG_MTCFG1] = pMotorCfg->dutyCycleClose;
    readBuffer[DRK_REG_MTCFG2] = pMotorCfg->timeoutOpen;
    readBuffer[DRK_REG_MTCFG3] = pMotorCfg->timeoutClose;
}

void Drake_I2cs_UpdateSensorConfig(Drake_I2cs_sensorConfig_t const* pSensorCfg)
{
    readBuffer[DRK_REG_MTCAL1] = pSensorCfg->threshLow;
    readBuffer[DRK_REG_MTCAL2] = pSensorCfg->threshUp;
    readBuffer[DRK_REG_MTCAL3] = pSensorCfg->fullOpenCnt & 0xFF;
    readBuffer[DRK_REG_MTCAL4] = pSensorCfg->fullOpenCnt >> 8;
}

void Drake_I2cs_UpdateTravelSensor(Drake_I2cs_travelSensor_t const* pTravelSens)
{
    readBuffer[DRK_REG_TRVCAL1] = pTravelSens->travel & 0xFF;
    readBuffer[DRK_REG_TRVCAL2] = pTravelSens->travel >> 8;
    readBuffer[DRK_REG_TRVCAL3] = pTravelSens->offset & 0xFF;
    readBuffer[DRK_REG_TRVCAL4] = pTravelSens->offset >> 8;
}

void Drake_I2cs_UpdateIndexedConfig(Drake_I2cs_indexedConfig_t const* pIndexedCfg)
{
    readBuffer[DRK_REG_INDCFG0] = pIndexedCfg->openBlocking;
    readBuffer[DRK_REG_INDCFG1] = pIndexedCfg->closeTimeout;

    for (uint8_t i = 0; i < DRK_NUM_OF_IND_POS; i++)
    {
        readBuffer[DRK_REG_INDPOS0 + i] = pIndexedCfg->positions[i];
    }
}
/**END OF FILE*****************************************************************/
