#include "BNO085.h"

Sensors::IMU::IMU(UART_HandleTypeDef *huart) {
  this->IMU_UART = huart;
}

bool Sensors::IMU::update() {
    uint8_t header[2];

    // Check for header (0xAA, 0xAA)
    if (HAL_UART_Receive(IMU_UART, header, 2, 100) != HAL_OK) {
        return false;
    }
    if (header[0] != 0xAA || header[1] != 0xAA) {
        return false;
    }

    // Read the remaining 17 bytes (data + checksum)
    if (HAL_UART_Receive(IMU_UART, buffer, 17, 100) != HAL_OK) {
        return false;
    }

    // Verify checksum
    uint8_t sum = 0;
    for (uint8_t i = 0; i < 16; i++) {
        sum += buffer[i];
    }
    if (sum != buffer[16]) {
        return false;
    }

    // Process IMU data (convert endianess)
    int16_t buffer_16[6];
    for (uint8_t i = 0; i < 6; i++) {
        buffer_16[i] = (buffer[1 + (i * 2)]);
        buffer_16[i] += (buffer[1 + (i * 2) + 1] << 8);
    }

    // Update yaw, pitch, and roll
    yaw = (float)buffer_16[0] * DEGREE_SCALE;
    pitch = (float)buffer_16[1] * DEGREE_SCALE;
    roll = (float)buffer_16[2] * DEGREE_SCALE;

    return true;
}
