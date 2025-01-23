/*
 * BNO085.h
 *
 *  Created on: Jan 9, 2025
 *      Author: Dell
 */

#ifndef INC_BNO085_H_
#define INC_BNO085_H_

#define MILLI_G_TO_MS2 0.0098067 ///< Scalar to convert milli-gs to m/s^2
#define DEGREE_SCALE 0.01        ///< To convert the degree values

#include "stm32f4xx_hal.h"  // Include the STM32 HAL header (adjust according to your STM32 series)

namespace Sensors {
  class IMU {
    private:
      UART_HandleTypeDef *IMU_UART;  // Pointer to UART handle for STM32
      float yaw,                     ///< Yaw in Degrees
            pitch,                   ///< Pitch in Degrees
            roll;                    ///< Roll in Degrees

    public:
      uint8_t buffer[19];  // Buffer for storing IMU data

      // Constructor that accepts the UART handle
      IMU(UART_HandleTypeDef *IMU_UART);

      // Update function that reads new data from IMU
      bool update();

      // Get yaw value
      float getYaw() {
        // You can also implement UART debug print here
      return yaw;
      }
  };
};

#endif  // IMU_H
