/**
 ******************************************************************************
 *  file           : ntp.h
 *  brief          : Getting the time from a NTP Server
 ******************************************************************************
 *  Copyright (C) 2021 Florian Brandner
 */

/* Includes ------------------------------------------------------------------*/
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_sntp.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define SYNC_INTERVAL (15 * 60 * 1000) //< Sync Interval in ms (15 Minutes)
#define NTP_SERVER "192.168.178.1"

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static const char *TAG = "NTP";

/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/

void ntp_cb(struct timeval *tv) {
    time_t now;
    char strftime_buf[64];
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    ESP_LOGI(TAG, "NTP-Time received: %s", strftime_buf);
}

/**
 * @brief Get the time from a NTP Server and starts the NTP Client in the background
 * 
 * @return esp_err_t 
 */
esp_err_t NTP_Init(void) {
    ESP_LOGI(TAG, "Current time is: ");

    // Setup NTP-Client
    ESP_LOGI(TAG, "Initializing SNTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_set_sync_mode(SNTP_SYNC_MODE_IMMED); // Sync mode: Update immediately
    sntp_set_time_sync_notification_cb(ntp_cb);
    sntp_setservername(0, NTP_SERVER);
    sntp_set_sync_interval(SYNC_INTERVAL); // Set sync Interval in ms
    sntp_init();

    return (ESP_OK);
}
