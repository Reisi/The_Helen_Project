/**
  ******************************************************************************
  * @file    remote.h
  * @author  Thomas Reisnecker
  * @brief   remote Control driver
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef REMOTE_H_INCLUDED
#define REMOTE_H_INCLUDED

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include "ble_gap.h"
#include "ble_hids_c.h"
#include "nrf_section.h"

/* Exported typed ----------------------------------------------------------- */
/** @brief function to check if a device is a compatiple remote control
 *
 * @param[in] p_data the advertising data of the device in question
 * @param[in] len    the length of the advertising data
 * @return true if the device advertises as a compatible remote, false if not
 */
typedef bool (*rem_is_device)(uint8_t const* p_data, uint16_t len);

/** @brief function to get the handles for the remote control
 *
 * instead of using the database discovery module you can use this function to
 * relay the handles to the HID Service client module
 * (@ref ble_hids_c_handles_assign)
 *
 * @return the device handles
 */
typedef ble_hids_c_db_t const* (*rem_get_handles)(void);

/** @brief function to enable the notifications for the remote
 *
 * @param[in] p_ble_hids_c pointer to the hids client structure
 * @param[in] enable true to enable, false to disable reprts
 * @return the return value of @ref ble_hids_c_report_notif_enable
 */
typedef uint32_t (*rem_notif_enable)(ble_hids_c_t * p_ble_hids_c, bool enable);

typedef struct
{
    rem_is_device            is_device;
    rem_get_handles          get_handles;
    rem_notif_enable         notif_enable;
    ble_hids_c_evt_handler_t evt_handler;
} rem_driver_t;

NRF_SECTION_DEF(remote_driver, rem_driver_t);

#define REMOTE_REGISTER(remote) NRF_SECTION_ITEM_REGISTER(remote_driver, remote)
#define REMOTE_CNT()            NRF_SECTION_ITEM_COUNT(remote_driver, rem_driver_t)
#define REMOTE_GET(cnt)         NRF_SECTION_ITEM_GET(remote_driver, rem_driver_t, cnt)

#endif // REMOTE_H_INCLUDED

/**END OF FILE*****************************************************************/
