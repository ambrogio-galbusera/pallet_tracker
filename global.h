/*
 * global.h
 *
 *  Created on: 26 Sep 2021
 *      Author: AmbrogioGalbusera
 */

#ifndef GLOBAL_H_
#define GLOBAL_H_

#include <FreeRTOS.h>
#include <Task.h>

/* structure to share data among different tasks
 *
 */
#define JSON_PAYLOAD_MAX_SIZE	8192
#define SSID_MAX_SIZE			32

typedef struct {

	TaskHandle_t tcp_client_task_handle;
	TaskHandle_t hibernate_task_handle;
	int rssi;
	char ssid[SSID_MAX_SIZE];
	char jsonPayload[JSON_PAYLOAD_MAX_SIZE];
} GLOBAL_DATA;

extern GLOBAL_DATA global_data;

#endif /* GLOBAL_H_ */
