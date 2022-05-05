#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <main.h>
#include <chprintf.h>
#include <i2c_bus.h>
#include <imu.h>
#include <usbcfg.h>
#include <motors.h>
#include "sensors/proximity.h"

//deux états possibles
#define PANIK 1
#define KALM 0
//vitesse constante
#define speed 600

#define PI                  3.1415926536f
//TO ADJUST IF NECESSARY. NOT ALL THE E-PUCK2 HAVE EXACTLY THE SAME WHEEL DISTANCE
#define WHEEL_DISTANCE      5.4f    //cm
#define PERIMETER_EPUCK     (PI * WHEEL_DISTANCE)
#define NB_SAMPLES_OFFSET     200

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

static void serial_start(void)
{
    static SerialConfig ser_cfg = {
        115200,
        0,
        0,
        0,
    };

    sdStart(&SD3, &ser_cfg); // UART3. Connected to the second com port of the programmer
}

static void timer11_start(void){
    //General Purpose Timer configuration
    //timer 11 is a 16 bit timer so we can measure time
    //to about 65ms with a 1Mhz counter
    static const GPTConfig gpt11cfg = {
        1000000,        /* 1MHz timer clock in order to measure uS.*/
        NULL,           /* Timer callback.*/
        0,
        0
    };

    gptStart(&GPTD11, &gpt11cfg);
    //let the timer count to max value
    gptStartContinuous(&GPTD11, 0xFFFF);
}

void show_gravity(imu_msg_t *imu_values){

    //we create variables for the led in order to turn them off at each loop and to
    //select which one to turn on
    uint8_t led1 = 0, led3 = 0, led5 = 0, led7 = 0;
    //threshold value to not use the leds when the robot is too horizontal
    float threshold = 0.6;
    //create a pointer to the array for shorter name
    float *accel = imu_values->acceleration;
    //variable to measure the time some functions take
    //volatile to not be optimized out by the compiler if not used
    volatile uint16_t time = 0;

    if(fabs(accel[X_AXIS]) > threshold || fabs(accel[Y_AXIS]) > threshold){

     chSysLock();
     GPTD11.tim->CNT = 0;

    //we find which led of each axis should be turned on
    if(accel[X_AXIS] > threshold)
        led7 = 1;
    else if(accel[X_AXIS] < -threshold)
        led3 = 1;
    if(accel[Y_AXIS] > threshold)
        led5 = 1;
    else if(accel[Y_AXIS] < -threshold)
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
    time = GPTD11.tim->CNT;
    chSysUnlock();
    }
    //to see the duration on the console
    chprintf((BaseSequentialStream *)&SD3, "time = %dus\n",time);
    //we invert the values because a led is turned on if the signal is low
    palWritePad(GPIOD, GPIOD_LED1, led1 ? 0 : 1);
    palWritePad(GPIOD, GPIOD_LED3, led3 ? 0 : 1);
    palWritePad(GPIOD, GPIOD_LED5, led5 ? 0 : 1);
    palWritePad(GPIOD, GPIOD_LED7, led7 ? 0 : 1);

}

void panik_check(float gx, float gy){
	float threshold = 1;
	static int etat = KALM;
	static int old_etat = KALM;

    if(fabs(gx) > threshold || fabs(gy) > threshold){
    	etat = PANIK;
    	chprintf((BaseSequentialStream *)&SD3, "*paniiik*	\n");
    	right_motor_set_speed(0);
    	left_motor_set_speed(0);
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

void speed_switch(float gz){
		if(gz>0.4){
              	right_motor_set_speed(speed);
              	left_motor_set_speed(speed);
	    }
		else if(gz<-0.4){
			right_motor_set_speed(-speed);
			left_motor_set_speed(-speed);
	    }
		else{
			right_motor_set_speed(0);
			left_motor_set_speed(0);
	    }
}

int main(void)
{

    halInit();
    chSysInit();
    mpu_init();
    serial_start();
    timer11_start();
    i2c_start();
    imu_start();
    motors_init();
    proximity_start();

        /** Inits the Inter Process Communication bus. */
        messagebus_init(&bus, &bus_lock, &bus_condvar);

        messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");
        imu_msg_t imu_values;

        //wait 2 sec to be sure the e-puck is in a stable position
        chThdSleepMilliseconds(2000);
        imu_compute_offset(imu_topic, NB_SAMPLES_OFFSET);

        calibrate_ir();

        while(1){
            //wait for new measures to be published
            messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));

            //prints values in readable units
            chprintf((BaseSequentialStream *)&SD3, "%Ax=%.2f Ay=%.2f Az=%.2f Gx=%.2f Gy=%.2f Gz=%.2f (%x)\r\n\n",
                    imu_values.acceleration[X_AXIS], imu_values.acceleration[Y_AXIS], imu_values.acceleration[Z_AXIS],
                    imu_values.gyro_rate[X_AXIS], imu_values.gyro_rate[Y_AXIS], imu_values.gyro_rate[Z_AXIS],
                    imu_values.status);

            show_gravity(&imu_values);
            chThdSleepMilliseconds(100);
            speed_switch(imu_values.gyro_rate[Z_AXIS]);
            panik_check(imu_values.gyro_rate[X_AXIS], imu_values.gyro_rate[Y_AXIS]);
            chprintf((BaseSequentialStream *)&SD3, "A=%d B=%d C=%d D=%d E=%d F=%d G=%d H=%d\r\n\n",
                      get_calibrated_prox(0),get_calibrated_prox(1),get_calibrated_prox(2),get_calibrated_prox(3),
                      get_calibrated_prox(4),get_calibrated_prox(5),get_calibrated_prox(6),get_calibrated_prox(7));
        }

    }

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
