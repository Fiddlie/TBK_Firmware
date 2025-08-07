#ifndef DFU_SERVICE_H
#define DFU_SERVICE_H

#include <stdint.h>
#include "host/ble_hs.h"

/* DFU Service UUIDs */
#define DFU_SERVICE_UUID    0xFE59  // Standard DFU Service
#define DFU_CONTROL_UUID    0x8EC90001  // Control Point
#define DFU_PACKET_UUID     0x8EC90002  // Data Packet

/* DFU Commands */
#define DFU_CMD_START_DFU   0x01
#define DFU_CMD_INIT_PARAMS 0x02
#define DFU_CMD_RECEIVE_FW  0x03
#define DFU_CMD_VALIDATE    0x04
#define DFU_CMD_ACTIVATE    0x05
#define DFU_CMD_RESET       0x06
#define DFU_CMD_ABORT       0x07

/* DFU Response Codes */
#define DFU_RSP_SUCCESS     0x01
#define DFU_RSP_ERROR       0x02
#define DFU_RSP_INVALID     0x03
#define DFU_RSP_NOT_READY   0x04

/* Function declarations */
void dfu_service_init(void);
int dfu_service_register(void);
bool dfu_is_active(void);

#endif /* DFU_SERVICE_H */