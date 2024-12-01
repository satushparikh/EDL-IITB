/*
 * motor_enocders.h
 *
 *  Created on: Feb 16, 2024
 *      Author: Sarvadnya
 */

#ifndef SRC_MOTOR_ENOCDERS_H_
#define SRC_MOTOR_ENOCDERS_H_

#include "stdint.h"
#include "main.h"

typedef struct{
	int16_t velocity;
	int32_t position;
	uint32_t last_counter_value;
}encoder_instance;

void update_encoder(TIM_HandleTypeDef *htim, encoder_instance *encoder_value);
void reset_encoder(encoder_instance *encoder_value);

#endif /* SRC_MOTOR_ENOCDERS_H_ */
