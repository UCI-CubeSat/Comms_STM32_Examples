/*
 * ultrasonic.c
 *
 *  Created on: Jan 22, 2021
 *      Author: andrew
 */

#include "ultrasonic.h"

char measure_distance(UART_HandleTypeDef *huart4) {
	//query sensor
	char send_buffer[] = {header1, header2, ultra_address,
					data_length_read, read_distance, checksum};

	char recv_buffer[8];

	HAL_UART_Transmit(huart4, (uint8_t*)send_buffer, sizeof(send_buffer), HAL_MAX_DELAY);
	HAL_UART_Receive(huart4, (uint8_t*)recv_buffer, sizeof(recv_buffer), HAL_MAX_DELAY);

	//returns the acquired value
	return recv_buffer[6];
}


