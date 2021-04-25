/*
 * sd_card.c
 *
 *  Created on: Apr 25, 2021
 *      Author: andrew
 */

#include "sd_card.h"

/*
 * initialize and mount sd card
 * return -1 for failure and 0 for success
 */

int8_t
init_sd(SD_CARD *sd) {
  	 //check that SD Card is slotted correctly
  	 if(!BSP_SD_IsDetected()) {
  		 return -1;
  	 }

  	 //instantly mount SD card on startup
  	 sd->res = f_mount(&SDFatFS, (TCHAR const*)SDPath, 1);
  	 if(sd->res != FR_OK) {
  		 return -1;
  	 }

  	 //initialize R/W buffers
  	 if(memset(sd->read_buffer, 0, sizeof(sd->read_buffer)) == NULL ||
  			 memset(sd->write_buffer, 0, sizeof(sd->write_buffer)) == NULL) {
  		 return -1;
  	 }

  	 //return success
  	 return 0;
}

/*
 * open a file and read a certain amount of bytes from it (max = buffer size)
 * return -1 for failure and bytes read for success
 */

int32_t
read_file(SD_CARD *sd, uint8_t file_path[50], uint16_t read_amount) {

	UINT bytes_read;

	//open file for reading
	sd->res = f_open(&SDFile, (char*)file_path, FA_READ);
	if(sd->res != FR_OK) {
		return -1;
	}

	//prevent out of range access
	if(f_size(&SDFile) < read_amount) {
		read_amount = f_size(&SDFile);
	}
	//read bytes from file and store them in read buffer
	sd->res = f_read(&SDFile, &sd->read_buffer, read_amount, &bytes_read);

	return bytes_read;
}


