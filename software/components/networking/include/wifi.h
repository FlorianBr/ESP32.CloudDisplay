/**
 ******************************************************************************
 *  file           : wifi.h
 *  brief          : WiFi-Stuff
 ******************************************************************************
 *  Copyright (C) 2021 Florian Brandner
 */

#ifndef WIFI_H_
#define WIFI_H_

#ifdef __cplusplus
extern "C"
{

#endif

/* Includes ------------------------------------------------------------------*/
#include "wifi_secrets.h"

/* Private includes ----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void WIFI_Init(void);

bool WIFI_IsConnected();

#ifdef __cplusplus
}
#endif

#endif // WIFI_H_
