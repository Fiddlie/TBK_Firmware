#ifndef BLE_COMMON_H
#define BLE_COMMON_H

/* NimBLE Port */
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"

#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_att.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"

/* Services */
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

/* NimBLE */
#include "nimble/ble.h"
#include "os/os_mbuf.h"

#endif 