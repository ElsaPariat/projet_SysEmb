#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <math.h>

#include <sensors/proximity.h>
#include <motors.h>
#include <main.h>
#include "ir_sensors.h"
#include "msgbus/messagebus.h"

#define DIST_THRESHOLD	100

static THD_WORKING_AREA(waDistanceDetec, 256);
static THD_FUNCTION(DistanceDetec, arg) {

chRegSetThreadName(__FUNCTION__);
(void)arg;

while(1) {
	if(get_calibrated_prox(0) > DIST_THRESHOLD ||
	   get_calibrated_prox(1) > DIST_THRESHOLD ||
	   get_calibrated_prox(2) > DIST_THRESHOLD ||
	   get_calibrated_prox(3) > DIST_THRESHOLD ||
	   get_calibrated_prox(4) > DIST_THRESHOLD ||
	   get_calibrated_prox(5) > DIST_THRESHOLD ||
	   get_calibrated_prox(6) > DIST_THRESHOLD ||
	   get_calibrated_prox(7) > DIST_THRESHOLD || ){
		right_motor_set_speed(0);
		left_motor_set_speed(0);
		}
	chThdSleepMilliseconds(50);
	}
}

void distance_start(void) {
	chThdCreateStatic(waDistanceDetec, sizeof(waDistanceDetec), NORMALPRIO, DistanceDetec, NULL);
}
