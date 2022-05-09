#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <math.h>

#include <sensors/proximity.h>
#include <motors.h>
#include <main.h>
#include "ctrl_direction.h"
#include "msgbus/messagebus.h"

//vitesse constante
#define SPEED 600

//treshold gyro axe Z
#define GZ	0.4

#define DIST_THRESHOLD	150

void ir_check(float gz){
	if(gz>GZ){
      right_motor_set_speed(SPEED);
      left_motor_set_speed(SPEED);
      if(get_calibrated_prox(0) > DIST_THRESHOLD ||
		 get_calibrated_prox(7) > DIST_THRESHOLD){
		right_motor_set_speed(2*SPEED);
		left_motor_set_speed(-2*SPEED);
		 }
	}else if(gz<-GZ){
		right_motor_set_speed(-SPEED);
		left_motor_set_speed(-SPEED);
		if(get_calibrated_prox(3) > DIST_THRESHOLD ||
		   get_calibrated_prox(4) > DIST_THRESHOLD){
			right_motor_set_speed(-2*SPEED);
			left_motor_set_speed(2*SPEED);
        }
	}else{
		right_motor_set_speed(0);
		left_motor_set_speed(0);
	}
}

/*
static THD_WORKING_AREA(waDistanceDetec, 256);
static THD_FUNCTION(DistanceDetec, arg) {

	chRegSetThreadName(__FUNCTION__);
	(void)arg;


	while(1){
			if(get_calibrated_prox(0) > DIST_THRESHOLD ||
			   get_calibrated_prox(7) > DIST_THRESHOLD ||
			   get_calibrated_prox(3) > DIST_THRESHOLD ||
			   get_calibrated_prox(4) > DIST_THRESHOLD){
						right_motor_set_speed(2*SPEED);
						left_motor_set_speed(-2*SPEED);
			}
		chThdSleepMilliseconds(50);
	}
}

void distance_start(void) {
	chThdCreateStatic(waDistanceDetec, sizeof(waDistanceDetec), NORMALPRIO, DistanceDetec, NULL);
}
*/

