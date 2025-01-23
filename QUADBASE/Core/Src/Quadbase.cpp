/*
 * Quadbase.cpp
 *
 *  Created on: Dec 10, 2024
 *      Author: Dell
 */




/*
 * Tribase.cpp
 *
 *  Created on: Nov 11, 2024
 *      Author: PURUSHOTTAM
 */
/*
 * Tribase.c
 *
 *  Created on: Oct 26, 2024
 *      Author: PURUSHOTTAM
 */

#include "Quadbase.h"
#include "CYTRON.h"




Quadbase::Quadbase(Cytron* motor_1,  Cytron* motor_2,  Cytron* motor_3, Cytron* motor_4)
{
	this->motor_1 = motor_1;
	this->motor_2 = motor_2;
	this->motor_3 = motor_3;
}
Cytron motor_1();
Cytron motor_2();
Cytron motor_3();
Cytron motor_4();






void Quadbase::IK(float omega_bz, float v_bx, float v_by) {


    float r = 0.063;
    float d = 0.24;

//    float speedA = (1 / r) * ((-d * omega_bz) + (v_bx ));
//    float speedB = (1 / r) * ((-d * omega_bz) + ((-0.5) * v_bx ) + ((-0.866) * v_by ));
//    float speedC =  (1 / r) * ((-d * omega_bz) + ((-0.5) * v_bx) + (0.866 * v_by ));


    //for 45 degree quad base
   // v_w = (1 / r) * [ -d * ω_bz + cos(β) * v_bx + sin(β) * v_by ]

    float speedA = (1 / r) * ((-d * omega_bz) + (0.707 * v_bx) + (0.707 * v_by));
    float speedB = (1 / r) * ((-d * omega_bz) + (0.707 * v_bx) + (0.707 * v_by));
    float speedC = (1 / r) * ((-d * omega_bz) + (-0.707 * v_bx) + (-0.707 * v_by));
    float speedD = (1 / r) * ((-d * omega_bz) + (0.707 * v_bx) + (-0.707 * v_by));








   motor_1->setSpeed(speedA);  // Set speed instead of assigning directly
   motor_2->setSpeed(speedB);
   motor_3->setSpeed(speedC);
   motor_4->setSpeed(speedD);

}







void Quadbase::brake() {
  motor_1->brake();
  motor_2->brake();
  motor_3->brake();
  motor_4->brake();

}




