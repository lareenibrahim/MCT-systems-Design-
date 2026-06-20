/*
 * telemetry.h
 *
 *  Created on: Jun 10, 2026
 *      Author: Osama Mohammed
 */

#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "main.h"
#include <stdint.h>

extern uint16_t battery_voltage;
extern int32_t  encoders[4];
extern float    distances[4];
extern float    rpms[4];

void Telemetry_UpdateTick(void);

#endif /* TELEMETRY_H */
