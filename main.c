#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
//#include <usbcfg.h>
#include <main.h>
#include <motors.h>
#include <chprintf.h>
#include <messagebus.h>
#include <i2c_bus.h>
#include <imu.h>

#define NB_SAMPLES_OFFSET     200

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

void panic_handler(const char *reason)
{
    (void)reason;

    palClearPad(GPIOD, GPIOD_LED1);
    palClearPad(GPIOD, GPIOD_LED3);
    palClearPad(GPIOD, GPIOD_LED5);
    palClearPad(GPIOD, GPIOD_LED7);
    palClearPad(GPIOD, GPIOD_LED_FRONT);
    palClearPad(GPIOB, GPIOB_LED_BODY);

    while (true) {

    }
}

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

static THD_WORKING_AREA(waThdBodyLed, 128);
static THD_FUNCTION(ThdBodyLed, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    while(1){
        palTogglePad(GPIOB, GPIOB_LED_BODY);

        /*
        *   1st case :  pause the thread during 500ms
        */
        chThdSleepMilliseconds(500);

        /*
        *   2nd case :  make the thread work during the 500ms
        */

        // //about 500ms at 168MHz
        // for(uint32_t i = 0 ; i < 21000000 ; i++){
        //     __asm__ volatile ("nop");
        // }

        /*
        *   3rd case :  make the thread work during the 500ms
        *               and block the preemption
        */

        // chSysLock();
        // for(uint32_t i = 0 ; i < 21000000 ; i++){
        //     __asm__ volatile ("nop");
        // }
        // chSysUnlock();
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


	//inits the motors
	motors_init();

	 /** Inits the Inter Process Communication bus. */
	 messagebus_init(&bus, &bus_lock, &bus_condvar);

	 chThdCreateStatic(waThdBodyLed, sizeof(waThdBodyLed), NORMALPRIO, ThdBodyLed, NULL);

	 //to change the priority of the thread invoking the function. The main function in this case
	 //chThdSetPriority(NORMALPRIO+2);

	 messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");
	 imu_msg_t imu_values;

	 //wait 2 sec to be sure the e-puck is in a stable position
	 chThdSleepMilliseconds(2000);
	 imu_compute_offset(imu_topic, NB_SAMPLES_OFFSET);


    /* Infinite loop. */
    while (1) {
        //wait for new measures to be published
        messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));
        //prints raw values
        chprintf((BaseSequentialStream *)&SD3, "%Ax=%-7d Ay=%-7d Az=%-7d Gx=%-7d Gy=%-7d Gz=%-7d\r\n",
                imu_values.acc_raw[X_AXIS], imu_values.acc_raw[Y_AXIS], imu_values.acc_raw[Z_AXIS],
                imu_values.gyro_raw[X_AXIS], imu_values.gyro_raw[Y_AXIS], imu_values.gyro_raw[Z_AXIS]);

        //prints raw values with offset correction
        chprintf((BaseSequentialStream *)&SD3, "%Ax=%-7d Ay=%-7d Az=%-7d Gx=%-7d Gy=%-7d Gz=%-7d\r\n",
                imu_values.acc_raw[X_AXIS]-imu_values.acc_offset[X_AXIS],
                imu_values.acc_raw[Y_AXIS]-imu_values.acc_offset[Y_AXIS],
                imu_values.acc_raw[Z_AXIS]-imu_values.acc_offset[Z_AXIS],
                imu_values.gyro_raw[X_AXIS]-imu_values.gyro_offset[X_AXIS],
                imu_values.gyro_raw[Y_AXIS]-imu_values.gyro_offset[Y_AXIS],
                imu_values.gyro_raw[Z_AXIS]-imu_values.gyro_offset[Z_AXIS]);

        //prints values in readable units
        chprintf((BaseSequentialStream *)&SD3, "%Ax=%.2f Ay=%.2f Az=%.2f Gx=%.2f Gy=%.2f Gz=%.2f (%x)\r\n\n",
                imu_values.acceleration[X_AXIS], imu_values.acceleration[Y_AXIS], imu_values.acceleration[Z_AXIS],
                imu_values.gyro_rate[X_AXIS], imu_values.gyro_rate[Y_AXIS], imu_values.gyro_rate[Z_AXIS],
                imu_values.status);

    	//waits 1 second
        chThdSleepMilliseconds(1000);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
