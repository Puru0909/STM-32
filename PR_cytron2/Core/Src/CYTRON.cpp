

#include "CYTRON.h"
Cytron::Cytron(UART_HandleTypeDef* huart, uint8_t address, uint8_t channel) {
	//1. The UART on which you are planning to use the Cytron
	//2. The address of the motor which is configured via DIP switch
	//3. Channel
	this->huart = huart;
	this->address = address;
	this->channel = channel;
}

uint8_t map(uint8_t value, uint8_t start1, uint8_t stop1, uint8_t start2, uint8_t stop2) {
	return start2 + (stop2 - start2) * ((value - start1) / (stop1 - start1));
}
//The send_dummy_bit is to initialize all the cytrons connected.
void Cytron::send_dummy_bit(void) {
	HAL_UART_Transmit(huart, &dummy_bit, 1, HAL_MAX_DELAY);
	HAL_Delay(500);
}
//Below given are the clockwise and anti_clockwise functions.
//Since running in serial packetized mode, we can only pass 0 - 255 to a
//single motor, so if we want to run the motor in forward direction, we pass 255
//else 0, and to stop we pass 127. So here we are simply calculating the slope,
//and scaling the passed pwm.
void Cytron::clockwise(uint8_t pwm) {
	pwm = map(pwm, 0, 255, 127, 255);
	this->send_data(pwm);
}

void Cytron::anti_clockwise(uint8_t pwm) {
	pwm = map(pwm, 0, 255, 127, 0);
	this->send_data(pwm);
}

void Cytron::brake(void) {
	this->send_data(127);
}

/*We already have an array, where the first block stores the header, i.e. 85
 * so, what we need to do is calculate the last 3 blocks and send the packet to the
 * cytron.
 * Example Channel is 1 and address is 101
 * So if we perform three times right shift we get 1000 (8) and finally, we perform
 * OR operation, we get 1101 (13), which is the required value.
 *
 * The packet 2 is the PWM, and finally the packet 3 is the checksum, which is the
 * addition of first three packets.
 */
void Cytron::send_data(uint8_t speed) {
	this->packet[1] = (this->channel << 3) | this->address;
	this->packet[2] = speed;
	this->packet[3] = this->packet[0] + this->packet[1] + this->packet[2];

	HAL_UART_Transmit(huart, packet, 4, HAL_MAX_DELAY);
}
int percentage_to_pwm(float percentage, uint8_t resolution = 255){
	return (int) percentage * resolution / 100;
}

void Cytron::set_speed(float percentage){
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
