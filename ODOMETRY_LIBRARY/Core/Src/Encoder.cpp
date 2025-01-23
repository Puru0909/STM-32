//#include "encoder.h"
//#include "main.h"
//#include "math.h"
//
//encoder::encoder(TIM_HandleTypeDef *htim) : htim(htim) {
//    // Constructor initializes the encoder with the provided timer handle
//}
//
//void encoder::encoder_init() {
//    HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);  // Start encoder with both channels
//    this->prev_cnt = 0;                           // Initialize previous count to 0
//    this->direction = 0;                          // Initialize direction to 0
//}
//
//void encoder::update( ) {
//    this->current_cnt = htim->Instance->CNT;
//    if(current_cnt>2400){
//     rev++;
//     current_cnt = 0;
//    }
//    else if(current_cnt <- 2400) {
//    	rev--;
//    	current_cnt = 0;
//    }
//
//    // Calculate revolutions, distance in cm, and distance in meters based on current count
//   // 2400 counts per revolution (adjust if necessary)
////    CM = floor(current_cnt * 18.85 / 2400);         // Convert counts to cm (adjust if necessary)
////    meter = floor(CM / 100);
//    // Convert cm to meters
//}
//
//int16_t encoder::read_revolutions() {
//    rev = floor(current_cnt / 2400);
//    int32_t current =rev + current_cnt;
//    return current;                        // Return the number of revolutions
//}
//
//int16_t encoder::read_distance() {
////    meter = floor((current_cnt * 18.85) / (2400 * 100);   // Convert counts to cm
//	 distance=((2*3.14*r)*rev ) + ((current_cnt*2*3.14*r)/2400)                 // Convert cm to meters
//     return distance;                             // Return the distance in meters
//}

#include "encoder.h"
#include "main.h"

encoder::encoder(TIM_HandleTypeDef *htim, int16_t cpr, float radius)
    : htim(htim), cpr(cpr), r(radius) {}

void encoder::encoder_init() {
    HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);
    this->prev_cnt = 0;
    this->direction = 0;
}

void encoder::update() {
    this->current_cnt = htim->Instance->CNT;

    if (current_cnt > cpr) {
        rev++;
        current_cnt = 0;
    } else if (current_cnt < -cpr) {
        rev--;
        current_cnt = 0;
    }
}

int16_t encoder::read_revolutions() {
	return rev + (current_cnt / cpr);
}

float encoder::read_distance() {


    distance = (2 * 3.14 * r * rev) + ((current_cnt * 2 * 3.14 * r) / cpr);
    return distance;
}






























