/*
 * library.c
 *
 *  Created on: Oct 4, 2024
 *      Author: PURUSHOTTAM
 */



//#include<main.h>
//#include <MotorControl.h>
#include "motorcontrol.h"


MotorControl::MotorControl(GPIO_TypeDef* GPIOx1, GPIO_TypeDef* GPIOx2, uint16_t GPIO_PIN_a, uint16_t GPIO_PIN_b) {
    this->GPIOx1 = GPIOx1;
    this->GPIOx2 = GPIOx2;
    this->GPIO_PIN_a = GPIO_PIN_a;
    this->GPIO_PIN_b = GPIO_PIN_b;
    this->htimx = nullptr;
    this->TIM_CHANNEL_y = 0;
}
void MotorControl::init(TIM_HandleTypeDef *htimx, uint32_t TIM_CHANNEL_y) {
    this->htimx = htimx;
    this->TIM_CHANNEL_y = TIM_CHANNEL_y;
    HAL_TIM_PWM_Start(this->htimx, this->TIM_CHANNEL_y);
}
void MotorControl::clockwise(uint32_t pwm) {
    HAL_GPIO_WritePin(GPIOx1, GPIO_PIN_a, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOx2, GPIO_PIN_b, GPIO_PIN_SET);

    __HAL_TIM_SET_COMPARE(this->htimx, this->TIM_CHANNEL_y, pwm);
}

void MotorControl::anticlock(uint32_t pwm) {
    HAL_GPIO_WritePin(GPIOx1, GPIO_PIN_a, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOx2, GPIO_PIN_b, GPIO_PIN_RESET);

    __HAL_TIM_SET_COMPARE(this->htimx, this->TIM_CHANNEL_y, pwm);
}

void MotorControl::motorstop() {
    HAL_GPIO_WritePin(GPIOx1, GPIO_PIN_a, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOx2, GPIO_PIN_b, GPIO_PIN_RESET);

    __HAL_TIM_SET_COMPARE(this->htimx, this->TIM_CHANNEL_y, 0);
}

/*#include "main.h"
#include "motorcontrol.h"
void MotorControl::clockwise(int pwm) {
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_SET);
	TIM1->CCR1 = pwm;

}
void MotorControl::anticlock(int pwm) {
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_SET);
	TIM1->CCR1 = pwm;
}
void MotorControl::motorstop(){
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_RESET);*/

//}
