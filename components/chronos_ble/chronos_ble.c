
#include "chronos_ble.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "esp_err.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"

#include "host/ble_hs.h"
#include "host/ble_att.h"

#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

/* -------------------------------------------------------------------------- */
/* Configuration                                                              */
/* -------------------------------------------------------------------------- */

#define CHRONOS_BLE_TX_QUEUE_LENGTH    16
#define CHRONOS_BLE_TX_TASK_STACK      4096
#define CHRONOS_BLE_TX_TASK_PRIORITY   5
#define CHRONOS_BLE_NOTIFY_TIMEOUT_MS  1000

static const char *TAG = "chronos_ble";

/* -------------------------------------------------------------------------- */
/* UUIDs                                                                      */
/* NimBLE stores 128-bit UUID bytes in reverse order relative to the          */
/* conventional UUID string representation.                                  */
/* -------------------------------------------------------------------------- */

static const ble_uuid128_t s_service_uuid =
    BLE_UUID128_INIT(
        0x9e, 0xca, 0xdc, 0x24,
        0x0e, 0xe5, 0xa9, 0xe0,
        0x93, 0xf3, 0xa3, 0xb5,
        0x01, 0x00, 0x40, 0x6e
    );

static const ble_uuid128_t s_rx_uuid =
    BLE_UUID128_INIT(
        0x9e, 0xca, 0xdc, 0x24,
        0x0e, 0xe5, 0xa9, 0xe0,
        0x93, 0xf3, 0xa3, 0xb5,
        0x02, 0x00, 0x40, 0x6e
    );

static const ble_uuid128_t s_tx_uuid =
    BLE_UUID128_INIT(
        0x9e, 0xca, 0xdc, 0x24,
        0x0e, 0xe5, 0xa9, 0xe0,
        0x93, 0xf3, 0xa3, 0xb5,
        0x03, 0x00, 0x40, 0x6e
    );

/* -------------------------------------------------------------------------- */
/* Private state                                                              */
/* -------------------------------------------------------------------------- */

typedef struct {
    uint16_t length;
    uint8_t data[CHRONOS_CORE_MAX_PACKET_SIZE];
} chronos_ble_tx_item_t;

static chronos_ble_config_t s_config;

static QueueHandle_t s_tx_queue;
static SemaphoreHandle_t s_notify_sem;

static TaskHandle_t s_tx_task_handle;

static uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t s_tx_val_handle;

static uint8_t s_own_addr_type;

static bool s_connected;
static bool s_tx_subscribed;

/* -------------------------------------------------------------------------- */
/* Forward declarations                                                       */
/* -------------------------------------------------------------------------- */

static int gap_event_cb(struct ble_gap_event *event, void *arg);
static int rx_access_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
static int tx_access_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);

static void start_advertising(void);
static void tx_task(void *arg);
static void ble_host_task(void *arg);

static void core_tx_callback(uint8_t *data, size_t length);
static void core_connection_callback(bool connected);

/* -------------------------------------------------------------------------- */
/* GATT definition                                                            */
/* -------------------------------------------------------------------------- */

static const struct ble_gatt_chr_def s_characteristics[] = {
    {
        .uuid = &s_rx_uuid.u,
        .access_cb = rx_access_cb,
        .flags = BLE_GATT_CHR_F_WRITE |
                 BLE_GATT_CHR_F_WRITE_NO_RSP,
    },
    {
        .uuid = &s_tx_uuid.u,
        .access_cb = tx_access_cb,
        .val_handle = &s_tx_val_handle,
        .flags = BLE_GATT_CHR_F_NOTIFY,
    },
    {0}
};

static const struct ble_gatt_svc_def s_services[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &s_service_uuid.u,
        .characteristics = s_characteristics,
    },
    {0}
};

/* -------------------------------------------------------------------------- */
/* Chronos core callbacks                                                     */
/* -------------------------------------------------------------------------- */

/**
 * @brief Called by the Chronos core when it wants to transmit a packet.
 *
 * Copy the packet into the queue because the core may reuse its buffer after
 * this callback returns.
 */
static void core_tx_callback(uint8_t *data, size_t length){
    if (data == NULL || length == 0 ||
        length > CHRONOS_CORE_MAX_PACKET_SIZE) {
        ESP_LOGW(TAG, "Ignoring invalid TX packet");
        return;
    }

    if (!s_connected || !s_tx_subscribed || s_tx_queue == NULL) {
        ESP_LOGW(TAG, "TX skipped: no subscribed BLE client");
        return;
    }

    chronos_ble_tx_item_t item = {
        .length = (uint16_t)length,
    };

    memcpy(item.data, data, length);

    if (xQueueSend(s_tx_queue, &item, 0) != pdTRUE) {
        ESP_LOGW(TAG, "TX queue full; packet dropped");
    }
}

static int tx_access_cb(
    uint16_t conn_handle,
    uint16_t attr_handle,
    struct ble_gatt_access_ctxt *ctxt,
    void *arg)
{
    (void)conn_handle;
    (void)attr_handle;
    (void)ctxt;
    (void)arg;

    return BLE_ATT_ERR_UNLIKELY;
}

/**
 * @brief Forward connection changes from the core to the application.
 */
static void core_connection_callback(bool connected){
    if (s_config.on_connection != NULL) {
        s_config.on_connection(connected);
    }
}

/* -------------------------------------------------------------------------- */
/* GATT RX                                                                     */
/* -------------------------------------------------------------------------- */

/**
 * @brief Receive a write to the Chronos RX characteristic.
 *
 * Pass the received BLE data to the Chronos core unchanged. The core handles
 * Chronos packet parsing and BLE chunk assembly.
 */
static int rx_access_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg){
    (void)conn_handle;
    (void)attr_handle;
    (void)arg;

    if (ctxt == NULL || ctxt->om == NULL) {
        return BLE_ATT_ERR_UNLIKELY;
    }

    const uint16_t packet_length = OS_MBUF_PKTLEN(ctxt->om);

    if (packet_length == 0 ||
        packet_length > CHRONOS_CORE_MAX_PACKET_SIZE) {
        ESP_LOGW(TAG, "Invalid RX length: %u", packet_length);
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    uint8_t packet[CHRONOS_CORE_MAX_PACKET_SIZE];
    uint16_t copied_length = 0;

    int rc = ble_hs_mbuf_to_flat(ctxt->om, packet, sizeof(packet), &copied_length);

    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to copy RX packet: %d", rc);
        return BLE_ATT_ERR_UNLIKELY;
    }

    chronos_status_t status = chronos_core_receive_ble_packet(packet, copied_length);

    if (status != CHRONOS_STATUS_OK && status != CHRONOS_STATUS_PENDING && status != CHRONOS_STATUS_PACKET_READY) {
        ESP_LOGD(TAG, "Chronos RX status: %d", (int)status);
    }

    /* A valid BLE write is not necessarily a complete Chronos packet. */
    return 0;
}

/* -------------------------------------------------------------------------- */
/* BLE advertising                                                            */
/* -------------------------------------------------------------------------- */

static void start_advertising(void){
    struct ble_hs_adv_fields adv_fields = {0};

    adv_fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    adv_fields.uuids128 = (ble_uuid128_t *)&s_service_uuid;
    adv_fields.num_uuids128 = 1;
    adv_fields.uuids128_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&adv_fields);

    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to set advertising fields: %d", rc);
        return;
    }

    struct ble_hs_adv_fields rsp_fields = {0};

    rsp_fields.name = (uint8_t *)s_config.device_name;
    rsp_fields.name_len = strlen(s_config.device_name);
    rsp_fields.name_is_complete = 1;

    rc = ble_gap_adv_rsp_set_fields(&rsp_fields);

    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to set scan response: %d", rc);
        return;
    }

    struct ble_gap_adv_params adv_params = {0};

    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(s_own_addr_type, NULL, BLE_HS_FOREVER, &adv_params, gap_event_cb, NULL);

    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to start advertising: %d", rc);
        return;
    }

    ESP_LOGI(TAG, "Advertising as '%s'", s_config.device_name);
}

/* -------------------------------------------------------------------------- */
/* BLE GAP events                                                             */
/* -------------------------------------------------------------------------- */

static int gap_event_cb(struct ble_gap_event *event, void *arg){
    (void)arg;

    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            s_conn_handle = event->connect.conn_handle;
            s_connected = true;
            s_tx_subscribed = false;

            chronos_core_set_connection_status(true, false);

            ESP_LOGI(TAG, "BLE client connected");
        } else {
            ESP_LOGW(TAG, "Connection failed: %d",
                     event->connect.status);
            start_advertising();
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "BLE client disconnected; reason=%d",
                 event->disconnect.reason);

        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        s_connected = false;
        s_tx_subscribed = false;

        chronos_core_set_connection_status(false, false);

        start_advertising();
        return 0;

    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.attr_handle == s_tx_val_handle) {
            s_tx_subscribed = event->subscribe.cur_notify != 0;

            chronos_core_set_connection_status(
                s_connected,
                s_tx_subscribed
            );

            ESP_LOGI(TAG, "TX notifications %s",
                     s_tx_subscribed ? "enabled" : "disabled");

            if (s_connected && s_tx_subscribed) {
                chronos_status_t status =
                    chronos_core_send_sync_request();

                if (status != CHRONOS_STATUS_OK) {
                    ESP_LOGW(TAG, "Sync request status: %d",
                             (int)status);
                }
            }
        }
        return 0;

    case BLE_GAP_EVENT_NOTIFY_TX:
        if (s_notify_sem != NULL) {
            xSemaphoreGive(s_notify_sem);
        }

        if (event->notify_tx.status != 0) {
            ESP_LOGW(TAG, "Notification TX status: %d",
                     event->notify_tx.status);
        }
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        start_advertising();
        return 0;

    default:
        return 0;
    }
}

/* -------------------------------------------------------------------------- */
/* BLE TX task                                                                */
/* -------------------------------------------------------------------------- */

/**
 * @brief Serialize outgoing notifications.
 *
 * Each notification must fit within the negotiated ATT MTU minus 3 bytes.
 * Chronos protocol chunking must be enabled when a larger logical packet is
 * transmitted over a connection using the default MTU.
 */
static void tx_task(void *arg){
    (void)arg;

    chronos_ble_tx_item_t item;

    while (true) {
        if (xQueueReceive(s_tx_queue, &item, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        if (!s_connected || !s_tx_subscribed ||
            s_conn_handle == BLE_HS_CONN_HANDLE_NONE) {
            ESP_LOGD(TAG, "Skipping queued TX: client unavailable");
            continue;
        }

        const uint16_t mtu = ble_att_mtu(s_conn_handle);

        if (mtu < 3 || item.length > (size_t)(mtu - 3)) {
            ESP_LOGW(TAG,
                     "TX packet too large: len=%u, MTU=%u, max=%u",
                     item.length,
                     mtu,
                     (mtu >= 3) ? (mtu - 3) : 0);
            continue;
        }

        /* Clear any previous completion signal before sending. */
        (void)xSemaphoreTake(s_notify_sem, 0);

        struct os_mbuf *om = ble_hs_mbuf_from_flat(
            item.data,
            item.length
        );

        if (om == NULL) {
            ESP_LOGE(TAG, "Failed to allocate notification buffer");
            continue;
        }

        int rc = ble_gatts_notify_custom(
            s_conn_handle,
            s_tx_val_handle,
            om
        );

        if (rc != 0) {
            os_mbuf_free_chain(om);
            ESP_LOGW(TAG, "Notification failed: %d", rc);
            continue;
        }

        /*
         * Wait for NimBLE's notification completion event before sending
         * the next queued packet. The timeout prevents a stuck TX task.
         */
        if (xSemaphoreTake(
                s_notify_sem,
                pdMS_TO_TICKS(CHRONOS_BLE_NOTIFY_TIMEOUT_MS)
            ) != pdTRUE) {
            ESP_LOGW(TAG, "Notification completion timed out");
        }
    }
}

/* -------------------------------------------------------------------------- */
/* NimBLE host                                                                 */
/* -------------------------------------------------------------------------- */

static void on_ble_reset(int reason)
{
    ESP_LOGW(TAG, "NimBLE host reset; reason=%d", reason);
}

static void on_ble_sync(void)
{
    int rc = ble_hs_id_infer_auto(0, &s_own_addr_type);

    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to infer BLE address type: %d", rc);
        return;
    }

    start_advertising();
}

static void ble_host_task(void *arg)
{
    (void)arg;

    nimble_port_run();
    nimble_port_freertos_deinit();
}

/* -------------------------------------------------------------------------- */
/* Public API                                                                  */
/* -------------------------------------------------------------------------- */

esp_err_t chronos_ble_init(const chronos_ble_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (config->device_name == NULL ||
        config->device_name[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_tx_queue != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    s_config = *config;

    s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
    s_connected = false;
    s_tx_subscribed = false;

    /*
     * Initialize the hardware-independent Chronos protocol core and
     * register the BLE transport callback.
     */
    chronos_core_init();

    chronos_core_set_tx_cb(core_tx_callback);
    chronos_core_set_connection_cb(core_connection_callback);
    chronos_core_set_notification_cb(s_config.on_notification);
    chronos_core_set_ringer_cb(s_config.on_ringer);
    chronos_core_set_config_cb(s_config.on_config);
    chronos_core_set_health_request_cb(s_config.on_health_request);
    chronos_core_set_time_cb(s_config.get_time);

    s_tx_queue = xQueueCreate(
        CHRONOS_BLE_TX_QUEUE_LENGTH,
        sizeof(chronos_ble_tx_item_t)
    );

    if (s_tx_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create TX queue");
        return ESP_ERR_NO_MEM;
    }

    s_notify_sem = xSemaphoreCreateBinary();

    if (s_notify_sem == NULL) {
        ESP_LOGE(TAG, "Failed to create notification semaphore");
        vQueueDelete(s_tx_queue);
        s_tx_queue = NULL;
        return ESP_ERR_NO_MEM;
    }

    BaseType_t task_created = xTaskCreate(
        tx_task,
        "chronos_ble_tx",
        CHRONOS_BLE_TX_TASK_STACK,
        NULL,
        CHRONOS_BLE_TX_TASK_PRIORITY,
        &s_tx_task_handle
    );

    if (task_created != pdPASS) {
        ESP_LOGE(TAG, "Failed to create TX task");

        vSemaphoreDelete(s_notify_sem);
        vQueueDelete(s_tx_queue);

        s_notify_sem = NULL;
        s_tx_queue = NULL;

        return ESP_ERR_NO_MEM;
    }

    /*
     * NVS must already be initialized by the application before this call.
     */
    esp_err_t err = nimble_port_init();

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NimBLE initialization failed: %s",
                 esp_err_to_name(err));
        return err;
    }

    ble_hs_cfg.reset_cb = on_ble_reset;
    ble_hs_cfg.sync_cb = on_ble_sync;

    /* Request a larger MTU; the peer must negotiate it as well. */
    ble_att_set_preferred_mtu(517);

    ble_svc_gap_init();
    ble_svc_gatt_init();

    int rc = ble_svc_gap_device_name_set(s_config.device_name);

    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to set BLE device name: %d", rc);
        return ESP_FAIL;
    }

    rc = ble_gatts_count_cfg(s_services);

    if (rc != 0) {
        ESP_LOGE(TAG,
             "GATT service counting failed: rc=%d (0x%x)",
             rc, rc);
        return ESP_FAIL;
    }

    rc = ble_gatts_add_svcs(s_services);

    if (rc != 0) {
        ESP_LOGE(TAG, "GATT service registration failed: %d", rc);
        return ESP_FAIL;
    }

    nimble_port_freertos_init(ble_host_task);

    ESP_LOGI(TAG, "Chronos BLE initialization started");

    return ESP_OK;
}
