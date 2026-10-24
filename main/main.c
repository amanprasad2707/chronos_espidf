#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <inttypes.h>
#include <time.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "chronos_core.h"
#include "chronos_ble.h"


#define CHRONOS_DEVICE_NAME "ESP_Clock"

static const char *TAG = "CHRONOS_APP";



/**
 * @brief Called when Chronos delivers a notification.
 */

static void app_on_notification(chronos_notification_t *notification){
    if (notification == NULL) {
        ESP_LOGW(TAG, "Received NULL notification");
        return;
    }

    ESP_LOGI(TAG, "========== NOTIFICATION RECEIVED ==========");
    ESP_LOGI(TAG, "App       : %s", notification->app);
    ESP_LOGI(TAG, "Icon ID   : %d", notification->icon);
    ESP_LOGI(TAG, "Timestamp : %" PRIu32, notification->timestamp);
    ESP_LOGI(TAG, "Title     : %s", notification->title);
    ESP_LOGI(TAG, "Message   : %s", notification->message);
    ESP_LOGI(TAG, "============================================");
}


/**
 * @brief Called when the Chronos ringer state changes.
 */
static void app_on_ringer(const char *caller, bool active){
    ESP_LOGI(TAG, "========== RING ==========");
    ESP_LOGI(TAG, "Caller : %s", caller != NULL ? caller : "(unknown)");
    ESP_LOGI(TAG, "Active : %s", active ? "YES" : "NO");
    ESP_LOGI(TAG, "============================================");
}

/**
 * @brief Called when Chronos receives a configuration update.
 *
 * The two integer parameters are printed as raw values. Their meanings
 * depend on the definitions in chronos_core.h.
 */
static void app_on_config(chronos_config_t config, uint32_t value1, uint32_t value2){
    ESP_LOGI(TAG, "============= CONFIG =====================");
    ESP_LOGI(TAG, "Config value : %" PRIu32, (uint32_t)config);
    ESP_LOGI(TAG, "Parameter 1  : %" PRIu32, value1);
    ESP_LOGI(TAG, "Parameter 2  : %" PRIu32, value2);
    ESP_LOGI(TAG, "============================================");
}

/**
 * @brief Called when Chronos requests health-related information.
 *
 * The request is logged here. Implement the actual health-data response
 * according to the Chronos protocol's expected semantics.
 */
static void app_on_health_request(chronos_health_request_t request, bool enabled){
    ESP_LOGI(TAG, "Request : %" PRIu32, (uint32_t)request);
    ESP_LOGI(TAG, "Enabled : %s", enabled ? "YES" : "NO");
}

static void app_on_connection(bool connected){
    ESP_LOGI("CHRONOS_APP", "BLE status: %s", connected ? "CONNECTED" : "DISCONNECTED");
}



void app_main(void){

    /* Initialize NVS before initializing Chronos BLE. */
    esp_err_t err = nvs_flash_init();

    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {

        ESP_LOGW(TAG, "NVS requires reinitialization");

        err = nvs_flash_erase();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "NVS erase failed: %s", esp_err_to_name(err));
            return;
        }

        err = nvs_flash_init();
    }

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS initialization failed: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "NVS initialized successfully");

    /* Register all application-level Chronos callbacks. */
    const chronos_ble_config_t ble_config = {
        .device_name       = CHRONOS_DEVICE_NAME,
        .on_connection     = app_on_connection,
        .on_notification   = app_on_notification,
        .on_ringer         = app_on_ringer,
        .on_config         = app_on_config,
        .on_health_request = app_on_health_request,
    };

    /* Initialize Chronos core and the NimBLE transport. */
    err = chronos_ble_init(&ble_config);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Chronos BLE initialization failed: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "Chronos BLE initialization requested successfully");
    ESP_LOGI(TAG, "Device name : %s", CHRONOS_DEVICE_NAME);
    ESP_LOGI(TAG, "Waiting for BLE client and Chronos events...");

}
