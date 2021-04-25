/*
 * sd_card.c
 *
 *  Created on: Apr 25, 2021
 *      Author: andrew
 */

#include "sd_card.h"

int
init_sd(SD_CARD *SD) {
  	 //check that SD Card is slotted correctly
  	 if(!BSP_SD_IsDetected()) {
  		 return -1;
  	 }

  	 //instantly mount SD card on startup
  	 SD->res = f_mount(&SDFatFS, (TCHAR const*)SDPath, 1);
  	 if(SD->res != FR_OK) {
  		 return -1;
  	 }

  	 return 0;
}
