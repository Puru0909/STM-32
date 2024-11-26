
#include"main.h"
#include"GY_25.h"

GY_25::GY_25(uint8_t* sending_data, uint8_t data_size){
	for (int i = 0; i < data_size; i++){
		this->sending_data[i]=sending_data[i];
	}
	this->data_size=data_size;
}
void GY_25::GY_25Init(UART_HandleTypeDef *huart, uint8_t startbit, uint8_t stopbit){
	this->startbit=startbit;
	this->stopbit=stopbit;
	this->current_angle = 0;
	this->last_angle = 0;
	this->rotaion_count = 0;
	uint8_t auto_mode_cmd[2] = {0xA5, 0x52};
	HAL_UART_Transmit(huart , auto_mode_cmd, 8, HAL_MAX_DELAY);
	HAL_UART_Receive_DMA(huart, sending_data, 8);
}
void GY_25::update_angle(UART_HandleTypeDef *huart){

	//HAL_UART_Transmit_DMA(huart, received_data, 8);

	for(uint8_t i=0;i<8;i++){
		if(received_data[i]==startbit && received_data[(i+7)%8]==stopbit){
			for(uint8_t j = 0;j<8;j++){
				ordered_data[j]=received_data[(i+j)%8];
			}
		}
	}
	this->current_angle = ( (ordered_data[1] << 8) | ordered_data[2] );
	if (last_angle > 345 && current_angle < 15){
		rotaion_count += 1;
	} else if (last_angle < 15 && current_angle > 345) {
		rotaion_count -= 1;
	}
	this->last_angle = current_angle;
}
float GY_25::get_angle(){
	return 360 * this->rotaion_count + this->current_angle;
}
