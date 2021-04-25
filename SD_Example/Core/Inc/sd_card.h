/*
 * sd.h
 *
 *  Created on: Apr 25, 2021
 *      Author: andrew
 */

#ifndef INC_SD_CARD_H_
#define INC_SD_CARD_H_

#include "fatfs.h"
#include "main.h"

typedef struct {
	FRESULT res;
	FILINFO finfo;
} SD_CARD;

//functions
int init_sd(SD_CARD *sd);

#endif /* INC_SD_CARD_H_ */
