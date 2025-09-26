#include "dfu_service.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "ble_common.h"

static const char *TAG = "DFU";

/* DFU State */
typedef enum {
    DFU_STATE_IDLE,
    DFU_STATE_PREPARING,
    DFU_STATE_DOWNLOADING,
    DFU_STATE_VALIDATING,
    DFU_STATE_ACTIVATING
} dfu_state_t;

/* DFU Context */
static struct {
    dfu_state_t state;
    esp_ota_handle_t ota_handle;
    const esp_partition_t *update_partition;
    uint32_t bytes_received;
    uint32_t total_size;
    uint16_t conn_handle;
} dfu_ctx = {
    .state = DFU_STATE_IDLE,
    .ota_handle = 0,
    .update_partition = NULL,
    .bytes_received = 0,
    .total_size = 0,
    .conn_handle = BLE_HS_CONN_HANDLE_NONE
};

static uint16_t dfu_control_handle = 0;
static uint16_t dfu_packet_handle = 0;

static void dfu_send_response(uint16_t conn_handle, uint8_t cmd, uint8_t status)
{
    if (dfu_control_handle == 0) {
        ESP_LOGW(TAG, "Control handle not set");
        return;
    }
    
    uint8_t response[3] = {0x60, cmd, status}; /* 0x60 = Response opcode */
    struct os_mbuf *om = ble_hs_mbuf_from_flat(response, sizeof(response));
    
    if (om) {
        ble_gatts_notify_custom(conn_handle, dfu_control_handle, om);
    }
}

static int dfu_control_cb(uint16_t conn_handle, uint16_t attr_handle,
                         struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) {
        return BLE_ATT_ERR_UNLIKELY;
    }

    /* Store the handle if not already set */
    if (dfu_control_handle == 0) {
        dfu_control_handle = attr_handle;
        ESP_LOGI(TAG, "DFU control handle set: 0x%04x", dfu_control_handle);
    }

    uint8_t cmd = ctxt->om->om_data[0];
    ESP_LOGI(TAG, "DFU command: 0x%02X", cmd);

    switch (cmd) {
        case DFU_CMD_START_DFU: {
            if (dfu_ctx.state != DFU_STATE_IDLE) {
                dfu_send_response(conn_handle, cmd, DFU_RSP_NOT_READY);
                return 0;
            }

            /* Get next OTA partition */
            dfu_ctx.update_partition = esp_ota_get_next_update_partition(NULL);
            if (!dfu_ctx.update_partition) {
                ESP_LOGE(TAG, "No OTA partition available");
                dfu_send_response(conn_handle, cmd, DFU_RSP_ERROR);
                return 0;
            }

            /* Begin OTA */
            esp_err_t err = esp_ota_begin(dfu_ctx.update_partition, 
                                         OTA_SIZE_UNKNOWN, 
                                         &dfu_ctx.ota_handle);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
                dfu_send_response(conn_handle, cmd, DFU_RSP_ERROR);
                return 0;
            }

            dfu_ctx.state = DFU_STATE_DOWNLOADING;
            dfu_ctx.conn_handle = conn_handle;
            dfu_ctx.bytes_received = 0;
            
            ESP_LOGI(TAG, "DFU started, partition: %s", dfu_ctx.update_partition->label);
            dfu_send_response(conn_handle, cmd, DFU_RSP_SUCCESS);
            break;
        }

        case DFU_CMD_VALIDATE: {
            if (dfu_ctx.state != DFU_STATE_DOWNLOADING) {
                dfu_send_response(conn_handle, cmd, DFU_RSP_NOT_READY);
                return 0;
            }

            /* End OTA and validate */
            esp_err_t err = esp_ota_end(dfu_ctx.ota_handle);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Validation failed: %s", esp_err_to_name(err));
                dfu_send_response(conn_handle, cmd, DFU_RSP_ERROR);
                dfu_ctx.state = DFU_STATE_IDLE;
                return 0;
            }

            dfu_ctx.state = DFU_STATE_VALIDATING;
            ESP_LOGI(TAG, "Firmware validated successfully");
            dfu_send_response(conn_handle, cmd, DFU_RSP_SUCCESS);
            break;
        }

        case DFU_CMD_ACTIVATE: {
            if (dfu_ctx.state != DFU_STATE_VALIDATING) {
                dfu_send_response(conn_handle, cmd, DFU_RSP_NOT_READY);
                return 0;
            }

            /* Set boot partition */
            esp_err_t err = esp_ota_set_boot_partition(dfu_ctx.update_partition);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to set boot partition: %s", esp_err_to_name(err));
                dfu_send_response(conn_handle, cmd, DFU_RSP_ERROR);
                return 0;
            }

            dfu_ctx.state = DFU_STATE_ACTIVATING;
            ESP_LOGI(TAG, "New firmware activated, rebooting in 2 seconds...");
            dfu_send_response(conn_handle, cmd, DFU_RSP_SUCCESS);
            
            /* Delay before restart to ensure response is sent */
            vTaskDelay(pdMS_TO_TICKS(2000));
            esp_restart();
            break;
        }

        case DFU_CMD_ABORT: {
            if (dfu_ctx.state == DFU_STATE_DOWNLOADING && dfu_ctx.ota_handle) {
                esp_ota_abort(dfu_ctx.ota_handle);
            }
            dfu_ctx.state = DFU_STATE_IDLE;
            ESP_LOGI(TAG, "DFU aborted");
            dfu_send_response(conn_handle, cmd, DFU_RSP_SUCCESS);
            break;
        }

        default:
            dfu_send_response(conn_handle, cmd, DFU_RSP_INVALID);
            break;
    }

    return 0;
}

/* Handle firmware data packets */
static int dfu_packet_cb(uint16_t conn_handle, uint16_t attr_handle,
                        struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) {
        return BLE_ATT_ERR_UNLIKELY;
    }

    /* Store the handle if not already set */
    if (dfu_packet_handle == 0) {
        dfu_packet_handle = attr_handle;
        ESP_LOGI(TAG, "DFU packet handle set: 0x%04x", dfu_packet_handle);
    }

    if (dfu_ctx.state != DFU_STATE_DOWNLOADING) {
        ESP_LOGW(TAG, "Received data packet in wrong state: %d", dfu_ctx.state);
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    /* Write data to partition */
    uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
    uint8_t *data = malloc(len);
    if (!data) {
        return BLE_ATT_ERR_INSUFFICIENT_RES;
    }

    os_mbuf_copydata(ctxt->om, 0, len, data);
    
    esp_err_t err = esp_ota_write(dfu_ctx.ota_handle, data, len);
    free(data);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OTA write failed: %s", esp_err_to_name(err));
        return BLE_ATT_ERR_UNLIKELY;
    }

    dfu_ctx.bytes_received += len;
    
    /* Log progress every 10KB */
    if (dfu_ctx.bytes_received % 10240 == 0) {
        ESP_LOGI(TAG, "DFU progress: %lu bytes", dfu_ctx.bytes_received);
    }

    return 0;
}

/* DFU Service Definition - Simplified for integration */
static const struct ble_gatt_svc_def dfu_service_defs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(DFU_SERVICE_UUID),
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                /* DFU Control Point */
                .uuid = BLE_UUID16_DECLARE(0xFE60),
                .access_cb = dfu_control_cb,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY,
            },
            {
                /* DFU Data Packet */
                .uuid = BLE_UUID16_DECLARE(0xFE61),
                .access_cb = dfu_packet_cb,
                .flags = BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            { 0 } 
        },
    },
    { 0 } 
};

void dfu_service_init(void)
{
    dfu_ctx.state = DFU_STATE_IDLE;
    ESP_LOGI(TAG, "DFU service initialized");
}

/* Register DFU service - Just add to the service list, don't start GATT */
int dfu_service_register(void)
{
    int rc = ble_gatts_count_cfg(dfu_service_defs);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to count DFU service: %d", rc);
        return rc;
    }

    rc = ble_gatts_add_svcs(dfu_service_defs);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to add DFU service: %d", rc);
        return rc;
    }

    ESP_LOGI(TAG, "DFU service added to GATT");
    return 0;
}

bool dfu_is_active(void)
{
    return dfu_ctx.state != DFU_STATE_IDLE;
}