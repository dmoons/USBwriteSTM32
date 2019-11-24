/*
 * PeriodicPrintTask.c
 *
 *  Created on: Oct 2, 2019
 *      Author: markos
 */
#include "PeriodicPrintTask.h"
#include "FreeRTOS.h"
#include "task.h"
#include "helperFuctions.h"
#include <stdio.h>
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"

void vPeriodicPrintTask(void *prvParameters)
{
	const int len = 17;
	uint8_t buffer[17] = "Periodic Task\r\n";
	while(1)
	{
		//printf("Periodic Print Task");
		//CDC_Transmit_FS(buffer, len);
		toggleAllLEDs();
		vTaskDelay(pdMS_TO_TICKS(3000));
	}
}
