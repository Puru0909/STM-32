/*
 * libmotor.h
 *
 *  Created on: Oct 3, 2024
 *      Author: PURUSHOTTAM
 */
//
#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include "main.h"
#include <stdint.h>

class MotorControl {
public:
    GPIO_TypeDef* GPIOx1;
    GPIO_TypeDef* GPIOx2;
    uint16_t GPIO_PIN_a;
    uint16_t GPIO_PIN_b;
    TIM_HandleTypeDef* htimx;
    uint32_t TIM_CHANNEL_y;

    MotorControl(GPIO_TypeDef* GPIOx1, GPIO_TypeDef* GPIOx2, uint16_t GPIO_PIN_a, uint16_t GPIO_PIN_b);


    void init(TIM_HandleTypeDef *htimx, uint32_t TIM_CHANNEL_y);
    void clockwise(uint32_t pwm);
    void anticlock(uint32_t pwm);
    void motorstop();
};

#endif // MOTORCONTROL_H
 /* MOTORCONTROL_H_ */






//




/* LIBMOTOR_H_ */
