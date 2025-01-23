#ifndef ENCODER_H
#define ENCODER_H



#include "main.h"





class Encoder {
public:
	Encoder(TIM_HandleTypeDef *htim);
	Encoder(TIM_HandleTypeDef *htim, uint32_t prescaler_);
	void write(long value);
	long read();
private:
	uint8_t direction;
	long counter = 0;
	uint32_t value = 0;
	TIM_HandleTypeDef *htim;
	uint32_t prescaler_;
	int counter_s = 0;
	bool init = true;
};

#endif
