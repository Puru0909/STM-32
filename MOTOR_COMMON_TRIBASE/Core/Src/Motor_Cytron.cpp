
//Can be used for Hercules and single channel cytron and urat packectized
#include "main.h"
#include "Motor_Cytron.h"
#include "stdio.h"

//Motor::Motor() {
//
//}

Motor::Motor(GPIO_TypeDef* PORT_1, uint16_t Pin1, TIM_HandleTypeDef *htim, uint32_t Channel) {
	//this->cytron1 = true;
	//this->cytron2 = false;
	this->PORT_1 = PORT_1;
	this->Pin1 = Pin1;
	this->htim = htim;
	this->Channel = Channel;
}

void Motor::init(){
	HAL_TIM_PWM_Start(htim, Channel);
}

//Motor::Motor(GPIO_TypeDef* GPIOx_1, uint16_t GPIO_Pin_1, GPIO_TypeDef* GPIOx_2, uint16_t GPIO_Pin_2, TIM_HandleTypeDef *htim, uint32_t Channel, TIM_TypeDef *tim) {
//	this->cytron1 = false;
//	this->cytron2 = false;
//	this->GPIOx_1 = GPIOx_1;
//	this->GPIO_Pin_1 = GPIO_Pin_1;
//	this->GPIOx_2 = GPIOx_2;
//	this->GPIO_Pin_2 = GPIO_Pin_2;
//	this->htim = htim;
//	this->Channel = Channel;
//	this->tim = tim;
//	HAL_TIM_PWM_Start(htim, this->Channel);
//	HAL_GPIO_WritePin(this->GPIOx_1, this->GPIO_Pin_1, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(this->GPIOx_2, this->GPIO_Pin_2, GPIO_PIN_RESET);
//}
//Motor::Motor(UART_HandleTypeDef* huart, uint8_t address, uint8_t channel) {
//	//1. The UART on which you are planning to use the Cytron
//	//2. The address of the motor which is configured via DIP switch
//	//3. Channel
//	this->cytron1 = false;
//	this->cytron2 = true;
//	this->huart = huart;
//	this->address = address;
//	this->channel = channel;
//}
//
//uint8_t map(uint8_t value, uint8_t start1, uint8_t stop1, uint8_t start2, uint8_t stop2) {
//	return start2 + (stop2 - start2) * ((value - start1) / (stop1 - start1));
//}
////The send_dummy_bit is to initialize all the cytrons connected.
////The send_dummy_bit is to initialize all the cytrons connected.
//void Motor::send_dummy_bit(void) {
//	HAL_UART_Transmit(huart, &dummy_bit, 1, HAL_MAX_DELAY);
//	HAL_Delay(500);
//}
//void Motor::set_speed(float speed){
//	motor_id = channel << 3 | address;
//  uint8_t pwm = (speed + 100) * 1.27;
//  uint8_t packet[4] = {0x55, motor_id, pwm, 0};
//  packet[3] = packet[0] + packet[1] + packet[2];
//  HAL_UART_Transmit(huart, packet, 4, HAL_MAX_DELAY);
// // channel.write(packet, 4);
//  this->channel=channel;
//  this->address=address;
//  this->huart=huart;
//}
//void Motor::send_data(uint8_t speed) {
//	this->packet[1] = (this->channel << 3) | this->address;
//	this->packet[2] = speed;
//	this->packet[3] = this->packet[0] + this->packet[1] + this->packet[2];
//
//	HAL_UART_Transmit(huart, packet, 4, HAL_MAX_DELAY);
//}
void Motor::clockwise(uint8_t pwm) {
//	if(cytron1) {
	       HAL_GPIO_WritePin(PORT_1, Pin1, GPIO_PIN_SET);
			__HAL_TIM_SET_COMPARE(this->htim, this->Channel , pwm);
}
//	} else if(cytron2){
//		pwm = map(pwm, 0, 255, 127, 255);
//		this->send_data(pwm);
//	}else {
//		HAL_GPIO_WritePin(this->GPIOx_1, this->GPIO_Pin_1, GPIO_PIN_SET);
//		HAL_GPIO_WritePin(this->GPIOx_2, this->GPIO_Pin_2, GPIO_PIN_RESET);
//	}
//	switch(this->Channel) {
//	case TIM_CHANNEL_1:
//		this->tim->CCR1 = pwm;
//		break;
//	case TIM_CHANNEL_2:
//		this->tim->CCR2 = pwm;
//		break;
//	case TIM_CHANNEL_3:
//		this->tim->CCR3 = pwm;
//		break;
//	case TIM_CHANNEL_4:
//		this->tim->CCR4 = pwm;
//		break;
//	default:
//		printf("\nErr.\n");
//	}
//}

void Motor::anti_clockwise(uint8_t pwm) {
//	if(cytron1) {
		HAL_GPIO_WritePin(PORT_1, Pin1, GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(this->htim, this->Channel , pwm);

}
//	} else if(cytron2){
//		pwm = map(pwm, 0, 255, 127, 0);
//		this->send_data(pwm);
//	} else {
//		HAL_GPIO_WritePin(this->GPIOx_1, this->GPIO_Pin_1, GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(this->GPIOx_2, this->GPIO_Pin_2, GPIO_PIN_SET);
//	}
//	switch(this->Channel) {
//	case TIM_CHANNEL_1:
//		this->tim->CCR1 = pwm;
//		break;
//	case TIM_CHANNEL_2:
//		this->tim->CCR2 = pwm;
//		break;
//	case TIM_CHANNEL_3:
//		this->tim->CCR3 = pwm;
//		break;
//	case TIM_CHANNEL_4:
//		this->tim->CCR4 = pwm;
//		break;
//	default:
//		printf("\nErr.\n");
//	}
//}

void Motor::brake() {
//	if(cytron1) {
		HAL_GPIO_WritePin(PORT_1, Pin1, GPIO_PIN_RESET);

		__HAL_TIM_SET_COMPARE(htim, Channel , 0);


}
//	} else if(cytron2){
//	  this->send_data(127);
//	} else {
//		HAL_GPIO_WritePin(this->GPIOx_1, this->GPIO_Pin_1, GPIO_PIN_SET);
//		HAL_GPIO_WritePin(this->GPIOx_2, this->GPIO_Pin_2, GPIO_PIN_SET);
//	}
//	switch(this->Channel) {
//	case TIM_CHANNEL_1:
//		this->tim->CCR1 = 0;
//		break;
//	case TIM_CHANNEL_2:
//		this->tim->CCR2 = 0;
//		break;
//	case TIM_CHANNEL_3:
//		this->tim->CCR3 = 0;
//		break;
//	case TIM_CHANNEL_4:
//		this->tim->CCR4 = 0;
//		break;
//	default:
//		printf("\nErr.\n");
//	}
//}


