#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "main.h"

class MOTOR{
protected:
	GPIO_TypeDef *PORTA, *PORTB;
	uint16_t Pin1, Pin2;
	TIM_HandleTypeDef * timer;
	uint32_t CHANNEL;

public:
	MOTOR(GPIO_TypeDef * PORTA, uint16_t Pin1, GPIO_TypeDef * PORTB, uint16_t Pin2);
	void motor_init(TIM_HandleTypeDef *timer, uint32_t CHANNEL);
	void clockwise(int pwm);
	void anti_clockwise(int pwm);
	void brake();
	void set_speed(float percentage);
	int base_anti_clockwise_pwm;
	int base_clockwise_pwm;
};



#endif /* INC_MOTOR_H_ */
