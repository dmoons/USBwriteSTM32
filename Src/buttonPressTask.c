/*
 * buttonPressTask.c
 *
 *  Created on: Oct 3, 2019
 *      Author: markos
 */


#include "buttonPressTask.h"
#include "globalVariables.h"
#include "helperFuctions.h"
#include <stdio.h>
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "semphr.h"

#define MSG_LENGTH 16

extern TIM_HandleTypeDef htim7;

state currentState = stopped;


void buttonPressTask(void *params)
{

	uint8_t startMsg[MSG_LENGTH] = "Start \r\n";
	uint8_t stopMsg[MSG_LENGTH] = "Stop  \r\n";

	while(1)
	{

		xSemaphoreTake(buttonBinarySemph, portMAX_DELAY);
		if(currentState != transmitting)
		{
			CDC_Transmit_FS(startMsg, MSG_LENGTH);
			HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
			currentState = transmitting;
			HAL_TIM_Base_Start_IT(&htim7);
		}

		/*

		CDC_Transmit_FS(stopMsg, MSG_LENGTH);
		CDC_Transmit_FS(stopMsg, MSG_LENGTH);
		HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
		currentState = stopped;
		HAL_TIM_Base_Stop_IT(&htim7);


		*/
	}
}
