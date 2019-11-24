/*
 * ADCtoUSB.c
 *
 *  Created on: Nov 1, 2019
 *      Author: markos
 */
#include "main.h"
#include "buttonPressTask.h"
#include "globalVariables.h"
#include "helperFuctions.h"
#include <stdio.h>
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "semphr.h"

#define BUFFER_LENGTH 4


extern TIM_HandleTypeDef htim7;

uint8_t transmitBuffer1[TRANSMIT_BUFFER_LENGTH];
uint8_t transmitBuffer2[TRANSMIT_BUFFER_LENGTH];

uint8_t* currentBuffer;
uint16_t currentPosition;


void TransmitTask(void *params)
{
	uint8_t curBuf = 1;
	currentPosition = 0;
	currentBuffer = &transmitBuffer1[0];
	while(1)
	{
		if(currentPosition >= TRANSMIT_BUFFER_LENGTH-2)
		{
			uint8_t *prev_buffer = currentBuffer;
			//uint16_t prev_position = currentPosition;
			portDISABLE_INTERRUPTS();
			if(curBuf == 1)
			{
				curBuf = 2;
				currentBuffer = &transmitBuffer2[0];
				currentPosition = 0;
			}
			else
			{
				curBuf = 1;
				currentBuffer = &transmitBuffer1[0];
				currentPosition = 0;
			}
			portENABLE_INTERRUPTS();
			// Have to make sure transmission happens before the buffers are swapped
			*(prev_buffer + TRANSMIT_BUFFER_LENGTH - 2) = '\r';
			*(prev_buffer + TRANSMIT_BUFFER_LENGTH - 1) = '\n';
			HAL_GPIO_WritePin(USBTransmit_GPIO_Port, USBTransmit_Pin, GPIO_PIN_SET);
			CDC_Transmit_FS(prev_buffer, TRANSMIT_BUFFER_LENGTH);
			HAL_GPIO_WritePin(USBTransmit_GPIO_Port, USBTransmit_Pin, GPIO_PIN_RESET);
		}
	}
}

