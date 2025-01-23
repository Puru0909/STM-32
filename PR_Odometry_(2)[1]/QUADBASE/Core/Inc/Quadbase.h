/*
 * Quadbase.h
 *
 *  Created on: Dec 10, 2024
 *      Author: Dell
 */

#ifndef INC_QUADBASE_H_
#define INC_QUADBASE_H_

#include "main.h"
#include "CYTRON.h"

class Quadbase {
protected:
    Cytron* motor_1;
    Cytron* motor_2;
    Cytron* motor_3;
    Cytron* motor_4;

public:
    // Constructor
    Quadbase( Cytron* motor_1,  Cytron* motor_2,  Cytron* motor_3, Cytron* motor_4);

    // Inverse kinematics function
    void IK(float omega_bz, float b_x, float b_y);

    // Brake function
    void brake();
};








#endif /* INC_QUADBASE_H_ */
