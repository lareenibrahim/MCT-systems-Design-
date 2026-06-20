/*
 * chassis_control.h
 *
 *  Created on: Jun 10, 2026
 *      Author: Osama Mohammed
 */

#ifndef CHASSIS_CONTROL_H
#define CHASSIS_CONTROL_H

#include "main.h"
#include <stdint.h>

extern int robot_mode;
extern int auto_is_running;

void Chassis_Init(I2C_HandleTypeDef *hi2c);
void Chassis_UpdateTick(void);
void Chassis_ProcessCommands(void);

#endif /* CHASSIS_CONTROL_H */
