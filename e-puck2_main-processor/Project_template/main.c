/*
 * Fichier modifié du TP3
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
//#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <main.h>
#include <chprintf.h>
#include <ctrl_direction.h>
#include <i2c_bus.h>
#include <imu.h>
#include <usbcfg.h>
#include <motors.h>
#include "sensors/proximity.h"
#include "audio/play_melody.h"
#include "audio/audio_thread.h"
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
/*
static void timer11_start(void){
    //General Purpose Timer configuration
    //timer 11 is a 16 bit timer so we can measure time
    //to about 65ms with a 1Mhz counter
    static const GPTConfig gpt11cfg = {
        1000000,        // 1MHz timer clock in order to measure uS.
        NULL,           // Timer callback.
        0,
        0
    };

    gptStart(&GPTD11, &gpt11cfg);
    //let the timer count to max value
    gptStartContinuous(&GPTD11, 0xFFFF);
}
*/


int main(void)
{
    halInit();
    chSysInit();
    mpu_init();
    serial_start();
    //timer11_start();
    i2c_start();
    imu_start();

    // initialise les moteurs
    motors_init();

    // enclenche les capteurs IR
    proximity_start();

    // enclenche le microphone
    dac_start();

	// Inits the Inter Process Communication bus.
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


		chprintf((BaseSequentialStream *)&SD3, "A=%d B=%d C=%d D=%d E=%d F=%d G=%d H=%d\r\n\n",
				  get_calibrated_prox(0),get_calibrated_prox(1),get_calibrated_prox(2),get_calibrated_prox(3),
				  get_calibrated_prox(4),get_calibrated_prox(5),get_calibrated_prox(6),get_calibrated_prox(7));


		show_gravity(&imu_values);
		ctrl_direction(imu_values.gyro_rate[Z_AXIS]);
		panik_check(imu_values.gyro_rate[X_AXIS], imu_values.gyro_rate[Y_AXIS]);
		chThdSleepMilliseconds(100);
	}
}



#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}

