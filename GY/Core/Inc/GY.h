#ifndef INC_GY_H_
#define INC_GY_H_

#include "stdint.h"
#include "stm32f1xx_hal.h"

class GY {
public:
    uint8_t sending_data[2];
    uint8_t startbit = 0xAA;
    uint8_t stopbit = 0x55;
    uint8_t received_data[8];
    uint8_t ordered_data[8];

    uint32_t count_per_reset;
    uint32_t counts_per_revolution;

    int x_angle_count = 0;
    int x_angle_reset_count = 0;
    int y_angle_count = 0;
    int y_angle_reset_count = 0;
    int z_angle_count = 0;
    int z_angle_reset_count = 0;

    float x_angle = 0;
    float y_angle = 0;
    float z_angle = 0;

    float x_omega = 0;  // Angular velocity (degrees per second)
    float y_omega = 0;
    float z_omega = 0;

    UART_HandleTypeDef *huart;
    uint8_t query_mode[2] = {0xA5, 0x51};

    uint32_t last_time = 0;  // Last update time for calculating angular velocity

    GY(UART_HandleTypeDef *huart);
    GY(UART_HandleTypeDef *huart, uint16_t counts_per_revolution);
    GY(UART_HandleTypeDef *huart, uint16_t counts_per_revolution, uint16_t count_per_reset);

    void update_angles();
    void update_counts();
    void update_angular_velocity(uint32_t current_time);  // New method to calculate angular velocity
    int get_x_revolution_counts();
    int get_y_revolution_counts();
    int get_z_revolution_counts();
};

#endif /* INC_GY_H_ */

