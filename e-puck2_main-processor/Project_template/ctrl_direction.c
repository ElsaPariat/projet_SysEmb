#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <math.h>

#include <sensors/proximity.h>
#include <motors.h>
#include <imu.h>
#include <main.h>
#include "ctrl_direction.h"
#include "msgbus/messagebus.h"
#include "audio/play_melody.h"
#include <usbcfg.h>
#include <i2c_bus.h>

//deux états possibles
#define PANIK 1
#define KALM 0


#define PI                  3.1415926536f
//TO ADJUST IF NECESSARY. NOT ALL THE E-PUCK2 HAVE EXACTLY THE SAME WHEEL DISTANCE
#define WHEEL_DISTANCE      5.4f    //cm
#define PERIMETER_EPUCK     (PI * WHEEL_DISTANCE)
//vitesse constante
#define SPEED 600
//threshold for detection of Z axis rotation
#define GZ	0.4
//threshold for panik mode
#define GXY	1
//threshold for gravity check (to not use the leds when the robot is too horizontal)
#define GXYZ  0.6

#define DIST_THRESHOLD	150

void ctrl_direction(float gz){
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


void panik_check(float gx, float gy){
	static int etat = KALM;
	static int old_etat = KALM;


    if(fabs(gx) > GXY || fabs(gy) > GXY){
    	etat = PANIK;
    	chprintf((BaseSequentialStream *)&SD3, "*paniiik*	\n");
    	right_motor_set_speed(0);
    	left_motor_set_speed(0);
    	playNote(NOTE_A5, 50);
    }
    else{
    	etat = KALM;
    	chprintf((BaseSequentialStream *)&SD3, "*kalm*	\n");
    }

    if (etat!=old_etat){
  		palTogglePad(GPIOD, GPIOD_LED_FRONT);
  		palTogglePad(GPIOB, GPIOB_LED_BODY);
  		old_etat=etat;
    }
}

void show_gravity(imu_msg_t *imu_values){

    //we create variables for the led in order to turn them off at each loop and to
    //select which one to turn on
    uint8_t led1 = 0, led3 = 0, led5 = 0, led7 = 0;
    //create a pointer to the array for shorter name
    float *accel = imu_values->acceleration;
    //variable to measure the time some functions take
    //volatile to not be optimized out by the compiler if not used
    volatile uint16_t time = 0;

    if(fabs(accel[X_AXIS]) > GXYZ || fabs(accel[Y_AXIS]) > GXYZ){

     chSysLock();
     //GPTD11.tim->CNT = 0;

    //we find which led of each axis should be turned on
    if(accel[X_AXIS] > GXYZ)
        led7 = 1;
    else if(accel[X_AXIS] < -GXYZ)
        led3 = 1;
    if(accel[Y_AXIS] > GXYZ)
        led5 = 1;
    else if(accel[Y_AXIS] < -GXYZ)
        led1 = 1;

    //if two leds are turned on, turn off the one with the smaller
    //accelerometer value
    if(led1 && led3){
        if(accel[Y_AXIS] < accel[X_AXIS])
            led3 = 0;
        else
            led1 = 0;
    }else if(led3 && led5){
        if(accel[X_AXIS] < -accel[Y_AXIS])
            led5 = 0;
        else
            led3 = 0;
    }else if(led5 && led7){
        if(accel[Y_AXIS] > accel[X_AXIS])
            led7 = 0;
        else
            led5 = 0;
    }else if(led7 && led1){
        if(accel[X_AXIS] > -accel[Y_AXIS])
            led1 = 0;
        else
            led7 = 0;
    }
    //time = GPTD11.tim->CNT;
    chSysUnlock();
    }
    //to see the duration on the console
    //chprintf((BaseSequentialStream *)&SD3, "time = %dus\n",time);
    //we invert the values because a led is turned on if the signal is low
    palWritePad(GPIOD, GPIOD_LED1, led1 ? 0 : 1);
    palWritePad(GPIOD, GPIOD_LED3, led3 ? 0 : 1);
    palWritePad(GPIOD, GPIOD_LED5, led5 ? 0 : 1);
    palWritePad(GPIOD, GPIOD_LED7, led7 ? 0 : 1);

}
