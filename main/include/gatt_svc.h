#pragma once
#include <stdint.h>
#include "host/ble_gap.h"
#include "host/ble_gatt.h"

#ifdef __cplusplus
extern "C" { 
#endif

void     gatt_svc_register(void);       /* called once after stack sync   */
extern   uint16_t g_raw_val_handle;     /* written by gatt_svc_register() */

#ifdef __cplusplus
}
#endif
