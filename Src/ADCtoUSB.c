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



extern TIM_HandleTypeDef htim7;
SemaphoreHandle_t transmitBufferSemaphore;

uint8_t transmitBuffer[TRANSMIT_BUFFER_LENGTH + 6];
//uint8_t transmitBuffer2[TRANSMIT_BUFFER_LENGTH];

uint8_t* currentBuffer;
uint16_t currentPosition;

extern state currentState;

void TransmitTask(void *params)
{
	//uint8_t curBuf = 1;
	currentPosition = 0;
	currentBuffer = &transmitBuffer[0];
	uint8_t stopMsg[16] = "Stop\r\n";//more unlikely to occur in binary (TODO check if it can be represented in 00 xx xx xx xx xx xx 00)

	transmitBufferSemaphore = xSemaphoreCreateBinary();

	while(1)
	{
		xSemaphoreTake(transmitBufferSemaphore, portMAX_DELAY); //Wait for buffer to fill
		currentState = stopped; //stop recording
		HAL_TIM_Base_Stop_IT(&htim7); //stop timer
		//flash LEDS to indicate end of read
		HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
		// Have to make sure transmission happens before the buffers are swapped
		transmitBuffer[TRANSMIT_BUFFER_LENGTH-6] = 'S';
		transmitBuffer[TRANSMIT_BUFFER_LENGTH-5] = 't';
		transmitBuffer[TRANSMIT_BUFFER_LENGTH-4] = 'o';
		transmitBuffer[TRANSMIT_BUFFER_LENGTH-3] = 'p';
		transmitBuffer[TRANSMIT_BUFFER_LENGTH-2] = '\r';
		transmitBuffer[TRANSMIT_BUFFER_LENGTH-1] = '\n';
		currentPosition = 0;
		HAL_GPIO_WritePin(USBTransmit_GPIO_Port, USBTransmit_Pin, GPIO_PIN_SET);
		CDC_Transmit_FS(&transmitBuffer[0], TRANSMIT_BUFFER_LENGTH);
		HAL_GPIO_WritePin(USBTransmit_GPIO_Port, USBTransmit_Pin, GPIO_PIN_RESET);
		//CDC_Transmit_FS(stopMsg, 16);
	}
}

