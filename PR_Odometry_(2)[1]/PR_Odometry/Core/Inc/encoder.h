/*
 * encoder.h
 *
 *  Created on: Dec 8, 2024
 *      Author: Hetvi
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#include "main.h"
#include <cmath>

/**
 * @brief Defines constant unit codes for distance measurements.
 */
struct EncoderUnits {
    static constexpr uint8_t MM = 1;         /**< Millimeter */
    static constexpr uint8_t MILLIMETER = 1; /**< Alias for Millimeter */
    static constexpr uint8_t CM = 2;         /**< Centimeter */
    static constexpr uint8_t CENTIMETER = 2; /**< Alias for Centimeter */
    static constexpr uint8_t M = 3;          /**< Meter */
    static constexpr uint8_t METER = 3;      /**< Alias for Meter */
    static constexpr uint8_t KM = 4;         /**< Kilometer */
    static constexpr uint8_t KILOMETER = 4;  /**< Alias for Kilometer */
    static constexpr uint8_t IN = 10;        /**< Inch */
    static constexpr uint8_t INCH = 10;      /**< Alias for Inch */
    static constexpr uint8_t FT = 11;        /**< Foot */
    static constexpr uint8_t FOOT = 11;      /**< Alias for Foot */
    static constexpr uint8_t YD = 12;        /**< Yard */
    static constexpr uint8_t YARD = 12;      /**< Alias for Yard */
};

/**
 * Extern declaration of the EncoderUnits instance.
 */
extern const EncoderUnits ENCODER_UNITS;

/**
 * @brief Class to manage an incremental encoder.
 */
class Encoder {
public:
    /**
     * @brief Constructor.
     * @param timer_handle Handle to the timer peripheral.
     * @param diameter Diameter of the wheel or mechanism (in meters).
     * @param pulses_per_revolution Pulses per revolution of the encoder.
     */
    Encoder(TIM_HandleTypeDef* timer_handle, int pulses_per_revolution, float diameter, uint8_t diameter_unit);

    /**
     * @brief Initializes the encoder and associated timer.
     */
    void init();

    /**
     * @brief Resets the encoder count and revolution count.
     */
    void reset();

    /**
     * @brief Updates internal encoder variables. Should be called periodically.
     */
    void update();

    /**
     * @brief Calculates the total traveled distance since the last reset.
     * @param unit Desired unit for the result (default: meters).
     * @return Total distance in the specified unit.
     */
    float getDistance(uint8_t unit = ENCODER_UNITS.METER) const;

    /**
     * @brief Retrieves the total number of revolutions since the last reset.
     * @return Number of revolutions.
     */
    int getRevolutionCount() const;

    /**
     * @brief Retrieves the total encoder count since the last reset.
     * @return Total encoder count.
     */
    int getTotalCount() const;

    int offset_count = 0;          /**< Offset for distance calculations. */
    int last_count = 0;            /**< Previous encoder count. */
    int current_count = 0;         /**< Current encoder count. */
    int revolution_count = 0;      /**< Total number of completed revolutions. */
    int counts_per_revolution;     /**< Pulses per revolution of the encoder. */

    float meters_per_revolution;
    float meters_per_count;

    TIM_HandleTypeDef* timer_handle; /**< Timer handle to interface with the timer peripheral. */
    float diameter_in_meter;                /**< Diameter of the wheel or mechanism (in meters). */
};


#endif /* INC_ENCODER_H_ */
