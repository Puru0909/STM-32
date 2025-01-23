

#ifndef INC_CYTRON_H_
#define INC_CYTRON_H_
#include <math.h>

#define DUMMY_BIT 128
#define HEADER 85

#include "main.h"

class Cytron {
public:
	Cytron(UART_HandleTypeDef* huart, uint8_t address, uint8_t channel);

	void send_dummy_bit(void);
	void clockwise(uint8_t pwm);
	void anti_clockwise(uint8_t pwm);
	void brake();
	void setSpeed(int speed);

private:
	UART_HandleTypeDef* huart;
	uint8_t address;
	uint8_t channel;
	uint8_t packet[4] = {HEADER, 0, 0, 0};
	uint8_t dummy_bit = DUMMY_BIT;
	void send_data(uint8_t speed);
};








#endif /* INC_CYTRON_H_ */

