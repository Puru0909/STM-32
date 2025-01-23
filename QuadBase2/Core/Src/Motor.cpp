#include "Motor.h"
#include "stdlib.h"


MOTOR::MOTOR(UART_HandleTypeDef& huart, uint8_t motor_id) : huart(huart), motor_id(motor_id){

}

void MOTOR::motor_init(){
	packet[1] = motor_id;
	HAL_UART_Transmit(&huart, &dummy_byte, 1, HAL_MAX_DELAY);
}

void MOTOR::brake(){
	packet[2] = 127;
	packet[3] = packet[0] + packet[1] + packet[2];
	HAL_UART_Transmit(&huart, packet, 4, HAL_MAX_DELAY);
}

void MOTOR::set_speed(float percentage){
	// 100 is clockwise, -100 is anti_clockwise and 0 is break
	if (percentage > 100) {
		percentage = 100;
	} else if (percentage < -100) {
		percentage = -100;
	}
	packet[2] = (uint8_t)((percentage + 100)*1.275);
	packet[3] = packet[0] + packet[1] + packet[2];
	HAL_UART_Transmit(&huart, packet, 4, HAL_MAX_DELAY);
}
