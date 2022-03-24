#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include <pi_regulator.h>
#include <process_image.h>

static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    int16_t speed = 0;
    int16_t rotspd =0;

    const float objective = 10.;
    float err = 0;
    float angerr = 0;

    int16_t Kp = 400;
    int16_t Ki = 3;
    //int16_t Kd = 0.;

    int16_t Rp = 400;

    static float int_fact = 0.;

    while(1){
        time = chVTGetSystemTime();
        
        //chBSemWait(&dist_ready_sem);

        err = get_distance_cm() - objective;
        angerr = get_ang_norm();

        int_fact += err;

        if(int_fact > INTLIM) int_fact = 0;
        if(int_fact < INTLIM) int_fact = 0;

        speed = (Kp * err) + (Ki * int_fact);
        rotspd = Rp * angerr;

        //chprintf((BaseSequentialStream *) &SD3, "speed: %i rotspd: %i \r\n",speed,rotspd);
        chprintf((BaseSequentialStream *) &SD3, "err: %.2f integ : %.2f speed: %i \r\n",err,int_fact,speed);
        //applies the speed from the PI regulator
		right_motor_set_speed(speed -rotspd);
		left_motor_set_speed(speed +rotspd);

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}
