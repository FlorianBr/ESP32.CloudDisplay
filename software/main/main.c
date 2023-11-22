/**
 ******************************************************************************
 *  file           : main.c
 *  brief          : Start-Up and main loop
 ******************************************************************************
 *  Copyright (C) 2021 Florian Brandner
 */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_spi_flash.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_spiffs.h"

#include "wifi.h"
#include "https.h"
#include "mqtt.h"
#include "ntp.h"
#include "lcd.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define OSSTATS_ARRAY_SIZE_OFFS 5  // OS-Statistics: Increase this if print_real_time_stats returns ESP_ERR_INVALID_SIZE
#define OSSTATS_TIME 60000         // OS-Statistics: Delay-Time in seconds


/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static const char *TAG = "MAIN";

/* Private function prototypes -----------------------------------------------*/
void TaskOSStats(void* pvParameters);

/* Private user code ---------------------------------------------------------*/

/**
 * @brief Task to print FreeRTOS runtime statistics
 */
void TaskOSStats(void* pvParameters) {
  while (1) {
    TaskStatus_t *start_array = NULL, *end_array = NULL;
    UBaseType_t start_array_size, end_array_size;
    uint32_t start_run_time, end_run_time;

    // Allocate array to store current task states
    start_array_size = uxTaskGetNumberOfTasks() + OSSTATS_ARRAY_SIZE_OFFS;
    start_array = malloc(sizeof(TaskStatus_t) * start_array_size);
    if (start_array == NULL) {
      ESP_LOGE(TAG, "OSStats: Out of memory!\r\n");
      goto exit;
    }

    // Get current task states
    start_array_size = uxTaskGetSystemState(start_array, start_array_size, &start_run_time);
    if (start_array_size == 0) {
      ESP_LOGE(TAG, "OSStats: Invalid size!\r\n");
      goto exit;
    }

    vTaskDelay(OSSTATS_TIME / portTICK_PERIOD_MS);

    // Allocate array to store tasks states post delay
    end_array_size = uxTaskGetNumberOfTasks() + OSSTATS_ARRAY_SIZE_OFFS;
    end_array = malloc(sizeof(TaskStatus_t) * end_array_size);
    if (end_array == NULL) {
      ESP_LOGE(TAG, "OSStats: Out of memory!\r\n");
      goto exit;
    }

    // Get post delay task states
    end_array_size = uxTaskGetSystemState(end_array, end_array_size, &end_run_time);
    if (end_array_size == 0) {
      ESP_LOGE(TAG, "OSStats: Invalid size!\r\n");
      goto exit;
    }

    // Calculate total_elapsed_time in units of run time stats clock period.
    uint32_t total_elapsed_time = (end_run_time - start_run_time);
    if (total_elapsed_time == 0) {
      ESP_LOGE(TAG, "OSStats: Invalid state!\r\n");
      goto exit;
    }

    printf("+-------------------------------------------------------------------+\n");
    printf("| Task              |   Run Time  | Percentage | State |      Stack |\n");
    printf("+-------------------------------------------------------------------+\n");
    // Match each task in start_array to those in the end_array
    for (int i = 0; i < start_array_size; i++) {
      int k = -1;
      for (int j = 0; j < end_array_size; j++) {
        if (start_array[i].xHandle == end_array[j].xHandle) {
          k = j;
          // Mark that task have been matched by overwriting their handles
          start_array[i].xHandle = NULL;
          end_array[j].xHandle = NULL;
          break;
        }
      }
      // Check if matching task found
      if (k >= 0) {
        char OutputLine[80];
        char charbuffer[20];

        uint32_t task_elapsed_time = end_array[k].ulRunTimeCounter - start_array[i].ulRunTimeCounter;
        uint32_t percentage_time = (task_elapsed_time * 100UL) / (total_elapsed_time * portNUM_PROCESSORS);

        memset(&OutputLine[0], ' ', sizeof(OutputLine));

        // Task Name
        OutputLine[0] = '|';
        memcpy(&OutputLine[2], start_array[i].pcTaskName, strlen(start_array[i].pcTaskName));

        // Time (absolute)
        snprintf(&charbuffer[0], sizeof(charbuffer), "| %10d", task_elapsed_time);
        memcpy(&OutputLine[20], &charbuffer[0], strlen(charbuffer));

        // Time (percentage)
        snprintf(&charbuffer[0], sizeof(charbuffer), "| %5d", percentage_time);
        memcpy(&OutputLine[34], &charbuffer[0], strlen(charbuffer));

        // Task State
        OutputLine[47] = '|';
        switch (start_array[i].eCurrentState) {
          case eRunning:
            memcpy(&OutputLine[49], "Run", 3);
            break;
          case eReady:
            memcpy(&OutputLine[49], "Rdy", 3);
            break;
          case eBlocked:
            memcpy(&OutputLine[49], "Blk", 3);
            break;
          case eSuspended:
            memcpy(&OutputLine[49], "Sus", 3);
            break;
          case eDeleted:
            memcpy(&OutputLine[49], "Del", 3);
            break;
          default:
            memcpy(&OutputLine[49], "Ukn", 3);
            break;
        }  // switch

        // Stack Usage
        snprintf(&charbuffer[0], sizeof(charbuffer), "| %10d", start_array[i].usStackHighWaterMark);
        memcpy(&OutputLine[55], &charbuffer[0], strlen(charbuffer));

        OutputLine[68] = '|';
        OutputLine[69] = '\n';
        OutputLine[70] = '\0';

        printf(&OutputLine[0]);
      }
    }  // for

    // Print unmatched tasks
    for (int i = 0; i < start_array_size; i++) {
      if (start_array[i].xHandle != NULL) {
        printf("| %s Deleted\n", start_array[i].pcTaskName);
      }
    }
    for (int i = 0; i < end_array_size; i++) {
      if (end_array[i].xHandle != NULL) {
        printf("| %s Created\n", end_array[i].pcTaskName);
      }
    }
#if 1
    printf("+-------------------------------------------------------------------+\n");

    printf("| HEAP           Free    MinFree     MaxBlk\n");
    printf("| All      %10d %10d %10d\n", heap_caps_get_free_size(MALLOC_CAP_8BIT),
                heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT), heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));

    printf("| Internal %10d %10d %10d\n", heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL),
                heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
    printf("| SPI      %10d %10d %10d\n", heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
                heap_caps_get_minimum_free_size(MALLOC_CAP_SPIRAM),
                heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM));
    printf("+-------------------------------------------------------------------+\n\n");
#endif
  exit:  // Common return path
    free(start_array);
    free(end_array);
  }  // while (1)
}  // TaskOSStats()


/**
 * The Main
 */
void app_main(void)
{
    /****************** Initial Bootup Info */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    ESP_LOGI(TAG, "+------------------------------------------+");
    // Core info
    ESP_LOGI(TAG, "| This is %s chip with %d CPU cores, WiFi%s%s, ",
           CONFIG_IDF_TARGET,
           chip_info.cores,
           (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");
    ESP_LOGI(TAG, "| Silicon revision %d, ", chip_info.revision);
    ESP_LOGI(TAG, "| %d MB %s flash", spi_flash_get_chip_size() / (1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
    ESP_LOGI(TAG, "| Free heap: %d", esp_get_free_heap_size());

    ESP_LOGI(TAG, "| Reset reason: %d", esp_reset_reason());

    // Memory info
    ESP_LOGI(TAG, "+------------------------------------------+");
    ESP_LOGI(TAG, "| HEAP           Free    MinFree     MaxBlk");
    ESP_LOGI(TAG, "| All      %10d %10d %10d", heap_caps_get_free_size(MALLOC_CAP_8BIT),
                heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT), heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));

    ESP_LOGI(TAG, "| Internal %10d %10d %10d", heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL),
                heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
    ESP_LOGI(TAG, "| SPI      %10d %10d %10d", heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
                heap_caps_get_minimum_free_size(MALLOC_CAP_SPIRAM),
                heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM));
    ESP_LOGI(TAG, "+------------------------------------------+");

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    {
        nvs_stats_t nvs_stats;
        nvs_get_stats("nvs", &nvs_stats);

        ESP_LOGI(TAG, "| NVS: Used = %d, Free = %d, All = %d", nvs_stats.used_entries, nvs_stats.free_entries, nvs_stats.total_entries);
        ESP_LOGI(TAG, "| NVS initialised");
        ESP_LOGI(TAG, "+------------------------------------------+");

        nvs_iterator_t it = nvs_entry_find("nvs", "nvs", NVS_TYPE_ANY);
        ESP_LOGI(TAG, "| NVS Keys:");
        while (it != NULL) {
                nvs_entry_info_t info;
                nvs_entry_info(it, &info);
                it = nvs_entry_next(it);
                ESP_LOGI(TAG, "| Key '%s', type '%d'", info.key, info.type);
        }
        ESP_LOGI(TAG, "+------------------------------------------+");
    }



    // Init SPIFFS
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true
    };
    ret = esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "| SPIFFS: Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "| SPIFFS: Failed to find SPIFFS partition");
        } else {
            ESP_LOGE(TAG, "| SPIFFS: Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return;
    }

    size_t total = 0, used = 0;
    ret = esp_spiffs_info(conf.partition_label, &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "| SPIFFS: Failed to get partition information (%s)", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "| SPIFFS: Size = %d, used - %d", total/1024, used/1024);
    }
    ESP_LOGI(TAG, "+------------------------------------------+");


    // Init LCD
    ESP_LOGI(TAG, "--- Setting up LCD ---");
    ESP_ERROR_CHECK(LCD_Init());
    ESP_ERROR_CHECK(LCD_Start());

    // Initialize networking stuff
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Init Webserver
    ESP_LOGI(TAG, "--- Setting up Webserver ---");
    HTTPS_Init();

    // Initialize WiFi
    ESP_LOGI(TAG, "--- Setting up WiFi-Client ---");
    WIFI_Init();

    while (WIFI_IsConnected() != true) {
        vTaskDelay(10);
    }

    // Init MQTT
    ESP_LOGI(TAG, "--- Setting up MQTT ---");
    ESP_ERROR_CHECK(MQTT_Init());

    // Getting Time
    ESP_LOGI(TAG, "--- Setting up NTP ---");
    ESP_ERROR_CHECK(NTP_Init());


    xTaskCreate(TaskOSStats, "FreeRTOS Stats", 4096, NULL, tskIDLE_PRIORITY, NULL);


    // Main Worker
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = (125 / portTICK_PERIOD_MS); // every 125ms
    xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        static time_t old;
        time_t now;
        char strftime_buf[64];
        struct tm timeinfo;
        time(&now);

        if (now != old) {
            setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1);
            tzset();
            localtime_r(&now, &timeinfo);
            strftime(strftime_buf, sizeof(strftime_buf), "%c  ", &timeinfo);
            lcd_rgb_color_t Fore = {0xFF, 0xFF, 0xFF};
            lcd_rgb_color_t Back = {0, 0, 0};
            LCD_WriteString(5, 5, strftime_buf, Fore, Back);

            MQTT_SendTime(strftime_buf);
            old = now;
        }
    }

} // app_main
