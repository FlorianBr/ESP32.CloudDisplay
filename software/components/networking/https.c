/**
 ******************************************************************************
 *  file           : https.h
 *  brief          : The HTTPS-Server
 ******************************************************************************
 *  Copyright (C) 2021 Florian Brandner
 */

/* Includes ------------------------------------------------------------------*/

/* Private includes ----------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include <esp_https_server.h>
#include "esp_event.h"
#include "esp_log.h"

/* Private typedef -----------------------------------------------------------*/
#include "../display/include/lcd.h"

/* Private define ------------------------------------------------------------*/
#define MAX_POST_SIZE   512

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static const char *TAG = "HTTPS";

static httpd_handle_t server = NULL; //< The Webserver

/* HTTP GET handler */
static esp_err_t root_get_handler(httpd_req_t *req) {
    time_t now;
    char strftime_buf[64];
    struct tm timeinfo;
    time(&now);
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send_chunk(req, "<!DOCTYPE html><html lang=\"en\"><meta charset=\"UTF-8\"><title>IoT ESP32 Display</title><body>", -1);
    httpd_resp_send_chunk(req, "<h1>IoT ESP32 Display</h1>Firmware: " __DATE__ " " __TIME__ "<br>", -1);
    setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1);
    tzset();
    localtime_r(&now, &timeinfo);
    strftime(strftime_buf, sizeof(strftime_buf), "Time: %c  ", &timeinfo);
    httpd_resp_send_chunk(req, strftime_buf, -1);
    httpd_resp_send_chunk(req, "<form method=\"post\" enctype=\"multipart/form-data\"><p><label>Choose image:</label><br/><input type=\"file\" name=\"image\"/></p><p><input type=\"submit\"/></p></form>", -1);
    httpd_resp_send_chunk(req, "</body></html>", -1);
    httpd_resp_send_chunk(req, "", 0); // Finalise
    return ESP_OK;
}

/* HTTP GET handler */
static esp_err_t root_post_handler(httpd_req_t *req) {
    char *  pContent = NULL;
    size_t  rxsize = 0;
    int ret = 0;
    
    pContent = malloc(MAX_POST_SIZE);
    if (NULL==pContent) {
        ESP_LOGE(TAG, "%s(): Unable to allocate memory!", __FUNCTION__);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    do {
        /* Truncate if content length larger than the buffer */
        size_t recv_size = MIN(req->content_len, MAX_POST_SIZE);

        // Reduce if necessarz for last chunk
        recv_size = MIN((req->content_len-rxsize), MAX_POST_SIZE);
        
        ret = httpd_req_recv(req, pContent, recv_size);
        ESP_LOGI(TAG, "%s(): POST working, chunk of %d, offset %d, size %d", __FUNCTION__, ret, rxsize, req->content_len);

        if (ret > 0) {
            LCD_SetRawData((uint8_t*)pContent, rxsize, ret);
            rxsize += ret;
        } else if (ret < 0) {  /* 0 return value indicates connection closed */
            ESP_LOGW(TAG, "%s(): POST Receive error %d", __FUNCTION__, ret);

            /* Check if timeout occurred */
            if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
                /* In case of timeout one can choose to retry calling
                * httpd_req_recv(), but to keep it simple, here we
                * respond with an HTTP 408 (Request Timeout) error */
                httpd_resp_send_408(req);
            }
            /* In case of error, returning ESP_FAIL will
            * ensure that the underlying socket is closed */
            free(pContent);
            return ESP_FAIL;
        } else {
            // ret == 0 means connection closed
        }

    } while (ret != 0);

    /* Send a simple response */
    const char resp[] = "Transfer complete";
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);

    ESP_LOGI(TAG, "%s(): POST complete, got %d of %d", __FUNCTION__, rxsize, req->content_len);

    free(pContent);

    return ESP_OK;
}

static const httpd_uri_t root = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = root_get_handler};

static const httpd_uri_t image_post = {
    .uri = "/",
    .method = HTTP_POST,
    .handler = root_post_handler};


/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/

/**
 * @brief Start the Webserver
 */
static httpd_handle_t start_webserver(void) {
    httpd_handle_t server = NULL;

    ESP_LOGI(TAG, "Starting HTTP-Server");

    httpd_ssl_config_t conf = HTTPD_SSL_CONFIG_DEFAULT();

    extern const unsigned char cacert_pem_start[] asm("_binary_https_cacert_pem_start");
    extern const unsigned char cacert_pem_end[] asm("_binary_https_cacert_pem_end");
    conf.cacert_pem = cacert_pem_start;
    conf.cacert_len = cacert_pem_end - cacert_pem_start;

    extern const unsigned char prvtkey_pem_start[] asm("_binary_https_prvtkey_pem_start");
    extern const unsigned char prvtkey_pem_end[] asm("_binary_https_prvtkey_pem_end");
    conf.prvtkey_pem = prvtkey_pem_start;
    conf.prvtkey_len = prvtkey_pem_end - prvtkey_pem_start;

    esp_err_t ret = httpd_ssl_start(&server, &conf);
    if (ESP_OK != ret) {
        ESP_LOGI(TAG, "Error starting HTTP-Server!");
        return NULL;
    }

    // Set URI handlers
    ESP_LOGI(TAG, "Registering URI handlers");
    httpd_register_uri_handler(server, &root);
    httpd_register_uri_handler(server, &image_post);   // fw_update (Post)
    return server;
}

/**
 * @brief Stop the Webserver
 */
static void stop_webserver(httpd_handle_t server) {
    httpd_ssl_stop(server);
}

/**
 * @brief Handler for HTTP Disconnect
 */
static void http_disconnect_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    httpd_handle_t *server = (httpd_handle_t *)arg;
    if (*server) {
        stop_webserver(*server);
        *server = NULL;
    }
}

/**
 * @brief Handler for HTTP Connect
 */
static void http_connect_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    httpd_handle_t *server = (httpd_handle_t *)arg;
    if (*server == NULL) {
        *server = start_webserver();
    }
}

/**
 * Init the HTTPS-Server
 * 
 */
void HTTPS_Init(void) {
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &http_connect_handler, &server));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &http_disconnect_handler, &server));
}
