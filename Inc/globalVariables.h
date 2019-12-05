/*
 * globalVariables.h
 *
 *  Created on: Oct 3, 2019
 *      Author: markos
 */

#ifndef GLOBALVARIABLES_H_
#define GLOBALVARIABLES_H_

#include "FreeRTOS.h"
#include "semphr.h"
#include "queue.h"

#define TRANSMIT_BUFFER_LENGTH 1024

extern SemaphoreHandle_t buttonBinarySemph;
extern QueueHandle_t HADCQueue;

typedef enum state {
	transmitting,
	stopped
}state;


#endif /* GLOBALVARIABLES_H_ */
