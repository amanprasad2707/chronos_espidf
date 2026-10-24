#ifndef CHRONOS_BLE_H
#define CHRONOS_BLE_H


#ifdef __cplusplus
extern "C" {
#endif

/* ------------ Includes ------------ */
#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "chronos_core.h"


typedef struct {
    const char *device_name;

    /* BLE connection and disconnection events */
    chronos_connection_callback_t on_connection;

    /* Chronos application events */
    chronos_notification_callback_t on_notification;
    chronos_ringer_callback_t on_ringer;
    chronos_config_callback_t on_config;
    chronos_health_request_callback_t on_health_request;
    chronos_time_callback_t get_time;
} chronos_ble_config_t;


/**
 * @brief Initialize the Chronos core and NimBLE transport.
 * 
 * NVS must be initialized by the application before calling this function.
 *
 * @param config BLE device name and application callbacks.
 * @return ESP_OK on successful initialization.
 */
esp_err_t chronos_ble_init(const chronos_ble_config_t *config);



#ifdef __cplusplus
}
#endif

#endif  // CHRONOS_BLE_H