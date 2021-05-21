/*
 * sd.h
 *
 *  Created on: May 18, 2021
 *      Author: andrew
 */

#ifndef INC_SD_H_
#define INC_SD_H_

#include "fatfs.h"
#include "main.h"

typedef struct {
	FRESULT res;
	FILINFO finfo;
	uint8_t read_buffer[1024];
	uint8_t write_buffer[1024];
} SD_CARD;

//functions
int8_t init_sd(SD_CARD *sd);
int32_t read_from_file(SD_CARD *sd, uint8_t file_path[50], uint16_t read_amount);
int32_t write_to_file(SD_CARD *sd, uint8_t file_path[50], uint16_t write_amount);
void unmount(SD_CARD *sd);



#endif /* INC_SD_H_ */
