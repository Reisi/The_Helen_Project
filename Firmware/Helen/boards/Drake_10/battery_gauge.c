/**
  ******************************************************************************
  * @file    battery gauge.c
  * @author  Thomas Reisnecker
  * @brief   tbd.
  ******************************************************************************
  */

/* logger configuration ----------------------------------------------_-------*/
#define BG_CONFIG_LOG_ENABLED 1 /// TODO: just temporary until project specific sdk_config
#define BG_CONFIG_LOG_LEVEL   3

#define NRF_LOG_MODULE_NAME bg
#if BG_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL BG_CONFIG_LOG_LEVEL
#else //BASE_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL 0
#endif //BASE_CONFIG_LOG_ENABLED
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

/* Includes ------------------------------------------------------------------*/
#include "battery_gauge.h"
#include "stdlib.h"

/* Private typedef -----------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/

/* Private defines -----------------------------------------------------------*/
#define LUT_SIZE        256
#define LOAD_TIMEOUT    180
#define MIN_SOC_DIFF    64
#define MIN_VOLTAGE     (3 << 13)

/* Private read only variables -----------------------------------------------*/

/* External variables --------------------------------------------------------*/
q3_13_t const bg_ocvLco[256] __attribute__((section(".cell_data"))) =
{
    22807,26925,26496,27252,27815,27547,28042,28474,28259,28632,28943,28782,
    29074,29333,29199,29424,29580,29499,29639,29715,29689,29740,29792,29844,
    29817,29869,29884,29884,29907,29924,29924,29944,29963,29963,29972,29998,
    29985,30002,30031,30005,30042,30069,30042,30094,30146,30198,30171,30223,
    30274,30248,30299,30351,30324,30336,30365,30325,30403,30465,30439,30458,
    30492,30439,30499,30528,30474,30547,30571,30555,30582,30599,30632,30632,
    30600,30657,30590,30671,30687,30673,30699,30680,30707,30702,30749,30749,
    30736,30743,30789,30768,30789,30733,30773,30787,30828,30828,30807,30847,
    30800,30862,30867,30867,30842,30883,30816,30907,30865,30905,30882,30946,
    30888,30929,30948,30949,30976,30935,31013,30946,31004,31035,30995,31054,
    31017,31058,31047,31074,31101,31094,31143,31143,31153,31179,31125,31165,
    31182,31185,31212,31226,31209,31249,31252,31251,31291,31287,31293,31333,
    31313,31330,31357,31356,31391,31418,31404,31457,31486,31465,31469,31529,
    31475,31525,31567,31527,31546,31597,31530,31588,31631,31564,31670,31651,
    31691,31655,31746,31811,31784,31836,31925,31871,31952,32039,31986,32068,
    32154,32100,32184,32236,32209,32261,32313,32286,32322,32350,32324,32375,
    32427,32479,32452,32480,32517,32490,32541,32593,32566,32613,32635,32629,
    32656,32708,32681,32733,32784,32758,32771,32804,32737,32847,32899,32950,
    32924,32975,33027,33000,33052,33078,33071,33074,33102,33075,33146,33218,
    33192,33243,33295,33268,33320,33372,33345,33397,33449,33501,33474,33490,
    33536,33496,33552,33595,33568,33670,33770,33743,33795,33832,33818,33836,
    33937,33843,33967,34075
};

q3_13_t const bg_ocvNmc[256] __attribute__((section(".cell_data"))) =
{
    22807,23078,23413,23747,24045,23860,24147,24411,24650,24889,24734,24957,
    25148,25339,25215,25395,25558,25721,25901,25777,25954,26120,26311,26463,
    26370,26513,26682,26839,26747,26890,27033,27177,27283,27222,27331,27452,
    27548,27682,27590,27716,27812,27907,27845,27930,27977,28025,28073,28042,
    28090,28138,28154,28154,28163,28211,28258,28306,28275,28323,28371,28419,
    28466,28435,28462,28523,28616,28577,28631,28689,28770,28818,28787,28834,
    28882,28930,28978,28947,28995,29042,29090,29059,29107,29155,29202,29199,
    29214,29214,29276,29323,29371,29340,29373,29396,29444,29413,29461,29491,
    29517,29565,29534,29582,29609,29638,29686,29655,29703,29750,29766,29766,
    29776,29824,29871,29919,29888,29936,29984,30031,30079,30048,30096,30144,
    30192,30161,30208,30256,30304,30352,30321,30369,30416,30464,30512,30481,
    30529,30577,30624,30593,30641,30689,30737,30784,30754,30801,30849,30897,
    30945,30914,30962,31009,31057,31026,31071,31116,31169,31200,31184,31183,
    31196,31285,31338,31307,31355,31403,31450,31420,31465,31473,31498,31569,
    31523,31588,31636,31684,31693,31693,31709,31757,31805,31774,31821,31869,
    31917,31965,31934,31982,32029,32107,32164,32133,32181,32229,32277,32246,
    32304,32381,32428,32512,32450,32532,32580,32658,32715,32684,32732,32804,
    32867,32836,32883,32931,32979,33027,32996,33044,33091,33139,33148,33148,
    33164,33187,33221,33190,33227,33227,33254,33266,33266,33266,33288,33305,
    33305,33305,33318,33341,33345,33345,33345,33364,33370,33419,33372,33430,
    33454,33493,33522,33506,33547,33606,33620,33620,33653,33762,33850,33912,
    33881,33957,34156,34404
};

/* Private variables ---------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
static bool isLowCurrent(q15_16_t current, q15_16_t cap)
{
    if ((int64_t)current * 3600 * 20 > cap)
        return false;
    else
        return true;
}

static q8_t getSOCfromLUT(q3_13_t voltage, q3_13_t const* pOCV)
{
    for (uint16_t i = 0; i < LUT_SIZE; i++)
    {
        if (voltage <= pOCV[i])
            return i;
    }

    return 255;
}

static q8_t getSOCfromCC(q3_13_t lastOCV, q3_13_t const* pOCV, q15_16_t coulumbCnt, q15_16_t cap)
{
    int64_t socChange;
    int16_t soc = getSOCfromLUT(lastOCV, pOCV);

    socChange = coulumbCnt;
    socChange <<= 8;
    socChange /= cap;

    soc += socChange;

    if (soc < 0)
        return 0;
    else if (soc > 255)
        return 255;
    else
        return soc;
}

static q15_16_t calculateCap(q3_13_t voltageStart, q3_13_t voltageEnd, q3_13_t const* pOCV, q15_16_t coulombCnt)
{
    int32_t socDiff;
    int64_t cap;

    socDiff = (int32_t)getSOCfromLUT(voltageEnd, pOCV) - (int32_t)getSOCfromLUT(voltageStart, pOCV);

    if (voltageStart == 0 || voltageEnd == 0 || coulombCnt == 0 || abs(socDiff) < MIN_SOC_DIFF)
        return 0;

    cap = coulombCnt;
    cap <<= 8;
    cap /= socDiff;

    return cap;
}

/* Public functions ----------------------------------------------------------*/
ret_code_t bg_Init(bg_inst_t* pInst, bg_init_t const* pInit)
{
    if (pInst == NULL || pInit == NULL || pInit->pOCV == NULL)
        return NRF_ERROR_NULL;

    if (pInst->capNominal == 0)
        return NRF_ERROR_INVALID_PARAM;

    memset(pInst, 0, sizeof(bg_inst_t));

    pInst->pOCV       = pInit->pOCV;
    pInst->capNominal = pInit->capNominal;
    pInst->capActual  = pInit->capActual ? pInit->capActual : pInit->capNominal;
    pInst->handler    = pInit->handler;

    return NRF_SUCCESS;
}

q8_t bg_Feed(bg_inst_t* pInst, q15_16_t current, q3_13_t voltage)
{
    if (pInst == NULL)
        return 0;

    static bool discharged;
    q8_t soc;

    if (current < 0 && voltage < MIN_VOLTAGE)   // set discharged flag if discharging and voltage drops below min voltage
        discharged = true;

    if (isLowCurrent(current, pInst->capActual))
    {
        if (pInst->loadTimeout)
        {
            //static q3_13_t lastVoltage;

            // still relaxing from last load, calculate SOC using coulomb counter
            soc = getSOCfromCC(pInst->lastOCV, pInst->pOCV, pInst->colCnt, pInst->capActual);

            pInst->loadTimeout--;

            //NRF_LOG_INFO("battery relaxing, voltage (mV): %d, diff %d", (voltage * 1000l) >> 13, lastVoltage - voltage);

            //lastVoltage = voltage;
        }
        else
        {
            if (pInst->colCnt)
            {
                // timeout just ended, actual capacity can be recalculated
                q15_16_t newCap = calculateCap(pInst->lastOCV, voltage, pInst->pOCV, pInst->colCnt);
                if (newCap) // data only valid if battery charge changed more than 25%
                {
                    pInst->capActual -= pInst->capActual >> 2;  // use low pass filter for updating the actual capacity
                    pInst->capActual += newCap >> 2;

                    if (pInst->handler)
                    {
                        bg_evt_t evt =
                        {
                            .type = BG_EVT_TYPE_CAP_RECALC,
                            .capActual = pInst->capActual,
                        };
                        pInst->handler(&evt);
                    }
                }

                // load off, timeout elapsed, use data to calculate actual capacity

                pInst->colCnt = 0;
            }

            // SOC can be calculated using battery voltage
            soc = getSOCfromLUT(voltage, pInst->pOCV);

            pInst->lastOCV = voltage;   /// TODO: timeout
        }
    }
    else
    {
        // feeding coulomb counter
        pInst->colCnt += current;

        // calculate SOC using coulomb counter
        soc = getSOCfromCC(pInst->lastOCV, pInst->pOCV, pInst->colCnt, pInst->capActual);

        pInst->loadTimeout = LOAD_TIMEOUT;

        if (current > 0)    // reset discharged flag if charging
            discharged = false;
    }

    return discharged ? 0 : soc;
}

q3_13_t const* getOCVTable()
{
    return bg_ocvNmc;
}

/**END OF FILE*****************************************************************/
