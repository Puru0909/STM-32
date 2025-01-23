/*
 * GY_25.h
 *
 *  Created on: Nov 22, 2024
 *      Author: Krish Saija
 */

#ifndef INC_GY_25_H_
#define INC_GY_25_H_

#include "main.h"

class GY_25 {
protected:
    uint8_t sending_data[10];
    uint8_t received_data[8];
    uint8_t startbit;
    uint8_t stopbit;
    uint8_t ordered_data[8];
    float current_angle;
    float last_angle;
    int16_t rotation_count;
    uint8_t data_size;

public:
    GY_25(uint8_t* sending_data, uint8_t data_size);
    void GY_25Init(UART_HandleTypeDef* huart, uint8_t startbit, uint8_t stopbit);
    void update_angle(UART_HandleTypeDef* huart);
    float get_angle();
};

#endif /* INC_GY_25_H_ */
