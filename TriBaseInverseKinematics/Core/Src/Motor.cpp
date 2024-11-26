#include "Motor.h"
#include "stdlib.h"


MOTOR::MOTOR(GPIO_TypeDef * PORTA, uint16_t Pin1, GPIO_TypeDef * PORTB, uint16_t Pin2){
	this->PORTA = PORTA;
	this->Pin1 = Pin1;
	this->PORTB = PORTB;
	this->Pin2 = Pin2;
	this->base_anti_clockwise_pwm = 255;
	this->base_clockwise_pwm = 255;
}

void MOTOR::motor_init(TIM_HandleTypeDef * timer, uint32_t CHANNEL){
	this->timer = timer;
	this->CHANNEL = CHANNEL;
	HAL_TIM_PWM_Start(timer, CHANNEL);
}

void MOTOR::clockwise(int pwm){
	HAL_GPIO_WritePin(PORTA, Pin1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(PORTB, Pin2, GPIO_PIN_RESET);
	__HAL_TIM_SET_COMPARE(this->timer, this->CHANNEL , pwm * base_clockwise_pwm / 255);
}
void MOTOR::anti_clockwise(int pwm){
	HAL_GPIO_WritePin(PORTA, Pin1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(PORTB, Pin2, GPIO_PIN_SET);
	__HAL_TIM_SET_COMPARE(timer, CHANNEL , pwm * base_anti_clockwise_pwm / 255);
}
void MOTOR::brake(){
	HAL_GPIO_WritePin(PORTA, Pin1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(PORTB, Pin2, GPIO_PIN_SET);
	__HAL_TIM_SET_COMPARE(timer, CHANNEL , 255);
}


int percentage_to_pwm(float percentage, uint8_t resolution = 255){
	return (int) percentage * resolution / 100;
}

void MOTOR::set_speed(float percentage){
	// 100 is clockwise, -100 is anti_clockwise and 0 is break
	if (percentage > 0){
		this->anti_clockwise(percentage_to_pwm(abs(percentage)));
	}
	else if (percentage < 0){
		this->clockwise(percentage_to_pwm(abs(percentage)));
	} else {
		this->brake();
	}
}
