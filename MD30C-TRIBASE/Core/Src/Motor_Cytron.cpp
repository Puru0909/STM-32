
//Can be used for Hercules and single channel cytron and urat packectized
#include "main.h"
#include "Motor_Cytron.h"
#include "stdio.h"

//Motor::Motor() {
//
//}

Motor::Motor(GPIO_TypeDef* PORT_1, uint16_t Pin1, TIM_HandleTypeDef *htim, uint32_t Channel) {
	this->PORT_1 = PORT_1;
	this->Pin1 = Pin1;
	this->htim = htim;
	this->Channel = Channel;
}

void Motor::init(){
	HAL_TIM_PWM_Start(htim, Channel);
}
void Motor::clockwise(uint8_t pwm) {
	       HAL_GPIO_WritePin(PORT_1, Pin1, GPIO_PIN_SET);
			__HAL_TIM_SET_COMPARE(this->htim, this->Channel , pwm);
}

void Motor::anti_clockwise(uint8_t pwm) {
		HAL_GPIO_WritePin(PORT_1, Pin1, GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(this->htim, this->Channel , pwm);

}


void Motor::brake() {
		HAL_GPIO_WritePin(PORT_1, Pin1, GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(htim, Channel , 0);

}
void Motor::setSpeed(int speed) {

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

