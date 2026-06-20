/*
 * comm_parser.h
 *
 *  Created on: Jun 10, 2026
 *      Author: Osama Mohammed
 */
#ifndef COMM_PARSER_H
#define COMM_PARSER_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* Packet buffers */
extern uint8_t          packet_buffer[8];
extern uint8_t          packet_index;
extern volatile uint8_t new_packet_ready;
extern volatile char    valid_cmd;

/* Handshake flag — set after STM sends ACK, cleared when Pi sends READY */
extern volatile uint8_t waiting_for_pi_ready;

/* API */
void USB_Print(const char *msg);
void Send_ACK(uint8_t cmd, uint8_t status);
void Send_Mode_Confirm(uint8_t mode_byte);
void Process_USB_Data(uint8_t* Buf, uint32_t Len);

#endif /* COMM_PARSER_H */
