/*
 * cytron.cpp
 *
 *  Created on: Nov 13, 2024
 *      Author: aryan
 */

#include "CYTRON.h"
Cytron::Cytron(UART_HandleTypeDef* huart, uint8_t address, uint8_t channel) {
	this->huart = huart;
	this->address = address;
	this->channel = channel;
}

uint8_t map(uint8_t value, uint8_t start1, uint8_t stop1, uint8_t start2, uint8_t stop2) {
	return (uint8_t)(start2 + (stop2 - start2) * (float)((float)(value - start1) / (float)(stop1 - start1)));
}

void Cytron::send_dummy_bit(void) {
	HAL_UART_Transmit(huart, &dummy_bit, 1, HAL_MAX_DELAY);
	HAL_Delay(500);
}

void Cytron::clockwise(uint8_t pwm) {
	pwm = map(pwm, 0, 255, 127, 255);
	this->send_data(pwm);
}

void Cytron::anti_clockwise(uint8_t pwm) {
	pwm = map(pwm, 0, 255, 127, 0);
	this->send_data(pwm);
}

void Cytron::brake(void) {
	this->send_data(127);
}

void Cytron::send_data(uint8_t speed) {
	this->packet[1] = (this->channel << 3) | this->address;
	this->packet[2] = speed;
	this->packet[3] = this->packet[0] + this->packet[1] + this->packet[2];

	HAL_UART_Transmit(huart, packet, 4, HAL_MAX_DELAY);
}


void Cytron::setSpeed(int speed) {

    if(speed < 0 ) {
    	this->clockwise(-speed);
    }
    else if(speed > 0) {
    	this->anti_clockwise(speed);
    }
    else {
    	this->brake();
    }
}
