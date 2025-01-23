#include "GY_25.h"

GY_25::GY_25(UART_HandleTypeDef& huart) : huart(huart) {
  this->counts_per_revolution = 65536;
  this->count_per_reset = 65536;
}
GY_25::GY_25(UART_HandleTypeDef& huart, uint32_t counts_per_revolution) : huart(huart) {
  this->counts_per_revolution = counts_per_revolution;
  this->count_per_reset = 65536;
}
GY_25::GY_25(UART_HandleTypeDef& huart, uint32_t counts_per_revolution, uint32_t count_per_reset) : huart(huart) {
  this->counts_per_revolution = counts_per_revolution;
  this->count_per_reset = count_per_reset;
}

void GY_25::correction_mode() {
    HAL_UART_Transmit(&huart, correction_heading, 2, 2000);
}
void GY_25::update_counts() {
//    channel.write(query_mode, sizeof(query_mode));
//    channel.readBytes(received_data, 8);
	HAL_UART_Transmit(&huart, query_mode, 2, HAL_MAX_DELAY);
	HAL_UART_Receive(&huart, received_data, 8, 200);


    for (uint8_t i = 0; i < 8; i++) {
        if (received_data[i] == startbit && received_data[(i + 7) % 8] == stopbit) {
            for (uint8_t j = 0; j < 8; j++) {
                ordered_data[j] = received_data[(i + j) % 8];
            }
            break;
        }
    }

    int last_x_angle_count = x_angle_count;
    x_angle_count = ( (ordered_data[1] << 8) | ordered_data[2] );
    int last_y_angle_count = y_angle_count;
    y_angle_count = ( (ordered_data[3] << 8) | ordered_data[4] );
    int last_z_angle_count = z_angle_count;
    z_angle_count = ( (ordered_data[5] << 8) | ordered_data[6] );

    int upper_threshold = 0.8 * this->count_per_reset;
    int lower_threshold = 0.2 * this->count_per_reset;

    if (last_x_angle_count > upper_threshold && x_angle_count < lower_threshold){
      x_angle_reset_count += 1;
    }
    else if (x_angle_count > upper_threshold && last_x_angle_count < lower_threshold){
      x_angle_reset_count -= 1;
    }

    if (last_y_angle_count > upper_threshold && y_angle_count < lower_threshold) y_angle_reset_count += 1;
    else if (y_angle_count > upper_threshold && last_y_angle_count < lower_threshold) y_angle_reset_count -= 1;

    if (last_z_angle_count > upper_threshold && z_angle_count < lower_threshold) z_angle_reset_count += 1;
    else if (z_angle_count > upper_threshold && last_z_angle_count < lower_threshold) z_angle_reset_count -= 1;
}

void GY_25::update_angles() {
  int total_x_count = this->get_x_revolution_counts();
  int total_y_count = this->get_y_revolution_counts();
  int total_z_count = this->get_z_revolution_counts();

  this->x_angle = (float) total_x_count * 360 / counts_per_revolution;
  this->y_angle = (float) total_y_count * 360 / counts_per_revolution;
  this->z_angle = (float) total_z_count * 360 / counts_per_revolution;
}

int GY_25::get_x_revolution_counts() {
    return (this->counts_per_revolution * x_angle_reset_count) + x_angle_count;
}

int GY_25::get_y_revolution_counts() {
    return (this->counts_per_revolution * y_angle_reset_count) + y_angle_count;
}

int GY_25::get_z_revolution_counts() {
    return (this->counts_per_revolution * z_angle_reset_count) + z_angle_count;
}
