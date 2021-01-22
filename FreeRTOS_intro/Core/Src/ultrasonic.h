/*
 * ultrasonic.h
 *
 *  Created on: Jan 22, 2021
 *      Author: andrew
 */

#ifndef SRC_ULTRASONIC_H_
#define SRC_ULTRASONIC_H_

#include "stm32l4xx_hal.h"

//hexadecimal definitions from the datasheet
#define ultra_address 0x11

//frame consists of:
//header, address, data length, command, data, checksum

#define header1 0x55
#define header2 0xAA
#define data_length_read 0x00
#define read_distance 0x02
#define read_temperature 0x03
#define set_adddress 0x55
#define set_baud_rate 0x08
#define checksum 0x12

char measure_distance(UART_HandleTypeDef *huart4);

void setBaud(UART_HandleTypeDef *huart4);
//warning this would override the existing address that is in the define
void setAddress(UART_HandleTypeDef *huart4);


#endif /* SRC_ULTRASONIC_H_ */
