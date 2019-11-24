/*
 * helperFunctions.c
 *
 *  Created on: Oct 2, 2019
 *      Author: markos
 */


#include "helperFuctions.h"
#include "main.h"

void toggleAllLEDs(void)
{
	HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
	HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
}
