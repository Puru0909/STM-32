/*
 * GY_25.cpp
 *
 *  Created on: Nov 22, 2024
 *      Author: Krish Saija
 */

#include "main.h"
#include "GY_25.h"

GY_25::GY_25(uint8_t* sending_data, uint8_t data_size) {
    for (int i = 0; i < data_size; i++) {
        this->sending_data[i] = sending_data[i];
    }
    this->data_size = data_size;
}

void GY_25::GY_25Init(UART_HandleTypeDef* huart, uint8_t startbit, uint8_t stopbit) {
    this->startbit = startbit;
    this->stopbit = stopbit;
    this->current_angle = 0;
    this->last_angle = 0;
    this->rotation_count = 0;

    // Transmit initial setup command to the GY-25
    HAL_UART_Transmit(huart, sending_data, data_size, HAL_MAX_DELAY);
}

void GY_25::update_angle(UART_HandleTypeDef* huart) {
    // Send query command to the GY-25
    HAL_UART_Transmit(huart, sending_data, data_size, HAL_MAX_DELAY);

    // Receive response from GY-25
    HAL_UART_Receive(huart, received_data, 8, HAL_MAX_DELAY);

    // Reorder data based on startbit and stopbit
    for (uint8_t i = 0; i < 8; i++) {
        if (received_data[i] == startbit && received_data[(i + 7) % 8] == stopbit) {
            for (uint8_t j = 0; j < 8; j++) {
                ordered_data[j] = received_data[(i + j) % 8];
            }
            break;
        }
    }

    // Calculate the current angle
    this->current_angle = ((ordered_data[1] << 8) | ordered_data[2]);

    // Adjust rotation count based on angle crossing
    if (last_angle > 345 && current_angle < 15) {
        rotation_count += 1;
    } else if (last_angle < 15 && current_angle > 345) {
        rotation_count -= 1;
    }

    this->last_angle = current_angle;
}

float GY_25::get_angle() {
    return 360 * this->rotation_count + this->current_angle;
}
