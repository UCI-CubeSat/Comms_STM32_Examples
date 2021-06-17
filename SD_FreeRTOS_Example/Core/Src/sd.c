/*
 * sd.c
 *
 *  Created on: May 18, 2021
 *      Author: andrew
 */

#include "sd.h"

/*
 * initialize and mount sd card
 * return -1 for failure and 0 for success
 */

int8_t
init_sd(SD_CARD *sd //SD Card instance
		)
{
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
read_from_file(SD_CARD *sd,			//SD card instance
		  uint8_t file_path[50],//path to the file
		  uint16_t read_amount  //amount of bytes to read
		  )
{

	UINT bytes_read;

	//set default state, will be overwritten if file exists
	sd->res = FR_NO_FILE;

	//check if file exists
	sd->res = f_stat((TCHAR *)file_path, &sd->finfo);
	//file can't be read if it doesn't exist
	if(sd->res != FR_OK) {
		return -1;
	}

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
	f_close(&SDFile);

	return bytes_read;
}

/*
 * open a file and write a certain amount of bytes to it (max = buffer size)
 * return -1 for failure and bytes written for success
 */

int32_t
write_to_file(SD_CARD *sd,		 //SD card instance
		   uint8_t file_path[50],//path to the file
		   uint16_t write_amount //amount of bytes to be written
		   )
{
	UINT bytes_written;
	uint8_t w_str[write_amount+1];
	//copy buffer to local array (may need to improve this later)
	memcpy(w_str, &sd->write_buffer, write_amount);

	//check if file exists
	sd->res = f_stat((TCHAR *)file_path, &sd->finfo);

	//adhere to different cases
	switch(sd->res) {

	case FR_OK:
		//file exists, append to it
		sd->res = f_open(&SDFile, (char*)file_path, FA_OPEN_APPEND | FA_WRITE);
		break;
	case FR_NO_FILE:
		//file does not exist, create it
		sd->res = f_open(&SDFile, (char*)file_path, FA_CREATE_NEW | FA_WRITE);
		break;
	default:
		//default case should overwrite to prevent system failure
		sd->res = f_open(&SDFile, (char*)file_path, FA_CREATE_NEW | FA_WRITE);
		break;
	}

	//write to file
	sd->res = f_write(&SDFile, w_str, (UINT)write_amount, (UINT*)&bytes_written);
	f_close(&SDFile);

	return bytes_written;
}

/*
 * Unmount SD card
 */
void
unmount(SD_CARD *sd //SD Card instance
		)
{
	sd->res = f_mount(&SDFatFS, (TCHAR const*)NULL, 0);
}


