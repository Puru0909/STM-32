/*
 * Motor_Cytron.h
 *
 *  Created on: Nov 29, 2024
 *      Author: Dell
 */

#ifndef INC_MOTOR_CYTRON_H_
#define INC_MOTOR_CYTRON_H_
#define DUMMY_BIT 128
#define HEADER 85

#include "main.h"
class Motor {
public:
//	Motor();
	Motor(GPIO_TypeDef* PORT_1, uint16_t Pin1, TIM_HandleTypeDef *htim, uint32_t Channel);
//	Motor(GPIO_TypeDef* GPIOx_1, uint16_t GPIO_Pin_1, GPIO_TypeDef* GPIOx_2, uint16_t GPIO_Pin_2, TIM_HandleTypeDef *htim, uint32_t Channel, TIM_TypeDef *tim);
//	Motor(UART_HandleTypeDef* huart, uint8_t address, uint8_t channel);
//	void send_dummy_bit(void);
	void init();
	void clockwise(uint8_t pwm);
	void anti_clockwise(uint8_t pwm);
	void brake();
	//void set_speed(float speed);
private:
//	UART_HandleTypeDef* huart;
//	uint8_t address;
//	uint8_t channel;
//	uint8_t packet[4] = {HEADER, 0, 0, 0};
//	uint8_t dummy_bit = DUMMY_BIT;
//	uint8_t motor_id;
//	void send_data(uint8_t speed);
	GPIO_TypeDef* PORT_1;
	//GPIO_TypeDef* GPIOx_2;
	uint16_t Pin1; //, GPIO_Pin_2;
	TIM_HandleTypeDef *htim;
	uint32_t Channel;
//	TIM_TypeDef *tim;
//	bool cytron1;
//	bool cytron2;

};




#endif /* INC_MOTOR_COMMON_H_ */

