#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "main.h"

class MOTOR{
protected:
	UART_HandleTypeDef& huart;
	uint8_t motor_id;

	const uint8_t dummy_byte = 128;

	uint8_t packet[4] = {85, 0, 0, 0};

public:
	MOTOR(UART_HandleTypeDef& huart, uint8_t motor_id);
	void motor_init();
	void brake();
	void set_speed(float percentage);
};



#endif /* INC_MOTOR_H_ */
