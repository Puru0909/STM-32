#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#include "main.h"

class encoder {
public:
    encoder(TIM_HandleTypeDef *htim, int16_t cpr, float radius);
    void encoder_init();
    void update();
    int16_t read_revolutions();
    float read_distance();

    TIM_HandleTypeDef *htim;
    int16_t rev;
    int16_t current_cnt;
    float distance;

private:
    int32_t prev_cnt;
    int32_t direction;
    int16_t cpr;
    float r;
};

#endif
