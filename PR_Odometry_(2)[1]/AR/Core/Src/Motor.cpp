

#include "Motor.h"


MOTOR::MOTOR(GPIO_TypeDef * PORTA, uint16_t Pin1, GPIO_TypeDef * PORTB, uint16_t Pin2){
	this->PORTA = PORTA;
	this->Pin1 = Pin1;
	this->PORTB = PORTB;
	this->Pin2 = Pin2;
}

void MOTOR::motor_init(TIM_HandleTypeDef * timer, uint32_t CHANNEL){
	this->timer = timer;
	this->CHANNEL = CHANNEL;
	HAL_TIM_PWM_Start(timer, CHANNEL);
}

void MOTOR::clockwise(int pwm){
	HAL_GPIO_WritePin(PORTA, Pin1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(PORTB, Pin2, GPIO_PIN_RESET);
	__HAL_TIM_SET_COMPARE(this->timer, this->CHANNEL , pwm);
}
void MOTOR::anti_clockwise(int pwm){
	HAL_GPIO_WritePin(PORTA, Pin1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(PORTB, Pin2, GPIO_PIN_SET);
	__HAL_TIM_SET_COMPARE(timer, CHANNEL , pwm);
}
void MOTOR::brake(){
	HAL_GPIO_WritePin(PORTA, Pin1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(PORTB, Pin2, GPIO_PIN_RESET);
	__HAL_TIM_SET_COMPARE(timer, CHANNEL , 255);

}

void MOTOR::jam(){
	HAL_GPIO_WritePin(PORTA, Pin1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(PORTB, Pin2, GPIO_PIN_SET);
	__HAL_TIM_SET_COMPARE(timer, CHANNEL , 0);
}

void MOTOR::set_speed(int speed){
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



