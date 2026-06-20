/*
 * comm_parser.c
 *
 *  Created on: Jun 10, 2026
 *      Author: Osama Mohammed
 */
#include "comm_parser.h"
#include "usbd_cdc_if.h"
#include <string.h>
#include <stdio.h>

/* --- USB PACKET PARSER VARIABLES --- */
uint8_t  packet_buffer[8];
uint8_t  packet_index = 0;
volatile uint8_t new_packet_ready = 0;
volatile char    valid_cmd = 0;

void USB_Print(const char *msg)
{
    CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
    HAL_Delay(10);
}

/* ACK packet sent back to Raspberry Pi after every map step command.
 * Format: [0xFF, 0xBB, echo_cmd, status, 0x00, 0x00, 0x00, checksum]
 * IMPORTANT: HAL_Delay(20) after transmit to ensure CDC flushes this packet
 * fully before any subsequent USB_Print telemetry is sent. Without this delay
 * the CDC driver bundles the ACK and the next text line into one USB transfer,
 * causing the Pi serial reader to misparse the ACK bytes. */
void Send_ACK(uint8_t cmd, uint8_t status)
{
    uint8_t ack[8];
    ack[0] = 0xFF;
    ack[1] = 0xBB;
    ack[2] = cmd;
    ack[3] = status;
    ack[4] = 0x00;
    ack[5] = 0x00;
    ack[6] = 0x00;
    ack[7] = (cmd + status) % 256;
    CDC_Transmit_FS(ack, 8);
    HAL_Delay(20);   // <-- THE FIX: flush ACK before any text telemetry follows
}

/* Mode confirmation packet sent back to Raspberry Pi after 'M' or 'P' command.
 * Format: [0xFF, 0xCC, mode_byte, 0x00, 0x00, 0x00, 0x00, checksum]
 * mode_byte: 0x00 = Manual, 0x01 = Autonomous */
void Send_Mode_Confirm(uint8_t mode_byte)
{
    uint8_t pkt[8];
    pkt[0] = 0xFF;
    pkt[1] = 0xCC;
    pkt[2] = mode_byte;
    pkt[3] = 0x00;
    pkt[4] = 0x00;
    pkt[5] = 0x00;
    pkt[6] = 0x00;
    pkt[7] = mode_byte % 256;
    CDC_Transmit_FS(pkt, 8);
    HAL_Delay(20);   // same flush guard
}

void Process_USB_Data(uint8_t* Buf, uint32_t Len)
{
    static uint32_t last_rx_time = 0;

    // 1. TIMEOUT RESET
    if (HAL_GetTick() - last_rx_time > 50) {
        packet_index = 0;
    }
    last_rx_time = HAL_GetTick();

    for (uint32_t i = 0; i < Len; i++)
    {
        uint8_t rx_byte = Buf[i];

        // 2. ASCII OVERRIDE
        if (packet_index == 0 &&
           ((rx_byte >= 'A' && rx_byte <= 'Z') ||
            (rx_byte >= 'a' && rx_byte <= 'z') ||
            (rx_byte >= '0' && rx_byte <= '9')))
        {
            if (rx_byte >= 'a' && rx_byte <= 'z') rx_byte -= 32;
            valid_cmd = rx_byte;
            new_packet_ready = 1;
            packet_index = 0;
            continue;
        }

        // 3. BINARY PACKET PARSER
        if (packet_index == 0) {
            if (rx_byte == 0xFF) packet_buffer[packet_index++] = rx_byte;
        }
        else if (packet_index == 1) {
            if (rx_byte == 0xAA) packet_buffer[packet_index++] = rx_byte;
            else packet_index = 0;
        }
        else if (packet_index < 8) {
            packet_buffer[packet_index++] = rx_byte;
            if (packet_index == 8) {
                uint8_t cmd      = packet_buffer[2];
                uint8_t b1       = packet_buffer[3];
                uint8_t b2       = packet_buffer[4];
                uint8_t b3       = packet_buffer[5];
                uint8_t b4       = packet_buffer[6];
                uint8_t checksum = packet_buffer[7];
                uint8_t calc_checksum = (cmd + b1 + b2 + b3 + b4) % 256;
                if (checksum == calc_checksum) {
                    valid_cmd = cmd;
                    new_packet_ready = 1;
                }
                packet_index = 0;
            }
        }
    }
}
