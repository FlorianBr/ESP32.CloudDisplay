/**
 ******************************************************************************
 *  file           : mqtt.h
 *  brief          : Connecting to MQTT
 ******************************************************************************
 *  Copyright (C) 2021 Florian Brandner
 */

#ifndef MQTT_H_
#define MQTT_H_

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/

/* Private includes ----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
esp_err_t MQTT_Init(void);
esp_err_t MQTT_SendTime(char *cBuffer);

#ifdef __cplusplus
}
#endif

#endif // MQTT_H_
