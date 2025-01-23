#include "Encoder_v1.0.h"

// return the value in different unitss
float convert_unit(float distance, uint8_t current_unit, uint8_t desired_unit){
	// converting current unit to meters
	float distance_in_meters;
	switch (current_unit) {
		case ENCODER_UNITS.MM:
			distance_in_meters = distance / 1000;
			break;
		case ENCODER_UNITS.CM:
			distance_in_meters = distance / 100;
			break;
		case ENCODER_UNITS.M:
			distance_in_meters = distance;
			break;
		case ENCODER_UNITS.KM:
			distance_in_meters = distance * 1000;
			break;
		case ENCODER_UNITS.IN:
			distance_in_meters = distance / 39.3701;
			break;
		case ENCODER_UNITS.FT:
			distance_in_meters = distance / 3.28084;
			break;
		case ENCODER_UNITS.YD:
			distance_in_meters = distance / 1.09361;
			break;
	}
	// converting to desired unit
    switch (desired_unit) {
        case ENCODER_UNITS.MM:
            return distance_in_meters * 1000;
        	break;
        case ENCODER_UNITS.CM:
            return distance_in_meters * 100;
			break;
        case ENCODER_UNITS.M:
			return distance_in_meters;
			break;
        case ENCODER_UNITS.KM:
            return distance_in_meters / 1000;
			break;
        case ENCODER_UNITS.IN:
            return distance_in_meters * 39.3701;
			break;
        case ENCODER_UNITS.FT:
            return distance_in_meters * 3.28084;
			break;
        case ENCODER_UNITS.YD:
            return distance_in_meters * 1.09361;
			break;
    }
    exit(1);
}

Encoder::Encoder(TIM_HandleTypeDef* timer_handle, int pulses_per_revolution, float diameter, uint8_t diameter_unit) {
	this->timer_handle = timer_handle;
	this->counts_per_revolution = pulses_per_revolution*4;
	this->diameter_in_meter = convert_unit(diameter, diameter_unit, ENCODER_UNITS.METER);
	this->meters_per_revolution = M_PI * this->diameter_in_meter;
	this->meters_per_count = meters_per_revolution / counts_per_revolution;
}

void Encoder::init() {
	HAL_TIM_Encoder_Start(timer_handle, TIM_CHANNEL_ALL);
}

void Encoder::reset() {
	this->offset_count = this->offset_count + this->getTotalCount();
}

void Encoder::update() {
	this->last_count = current_count;
	this->current_count = this->timer_handle->Instance->CNT;

	float upper_threshold = 0.8 * this->counts_per_revolution;
	float lower_threshold = 0.2 * this->counts_per_revolution;

	if (last_count < lower_threshold && current_count > upper_threshold) {
		this->revolution_count -= 1;
	}
	if (current_count < lower_threshold && last_count > upper_threshold) {
		this->revolution_count += 1;
	}
}

float Encoder::getDistance(uint8_t unit) const {
	return convert_unit((this->getTotalCount() - offset_count) * meters_per_count, ENCODER_UNITS.METER, unit);
}

int Encoder::getRevolutionCount() const {
    return this->revolution_count;
}

int Encoder::getTotalCount() const {
	return (this->current_count + (this->revolution_count * this->counts_per_revolution) - this->offset_count);
}
