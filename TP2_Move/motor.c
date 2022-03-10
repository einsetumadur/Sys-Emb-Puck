#include <stdlib.h>
#include <stdint.h>
#include <stm32f4xx.h>
#include <gpio.h>
#include <motor.h>

#define TIMER_CLOCK         84000000
#define TIMER_FREQ          100000 // 100 kHz
#define TIMER_PRESCALER     840
#define MOTOR_SPEED_LIMIT   13 // [cm/s]
#define NSTEP_ONE_TURN      1000 // number of step for 1 turn of the motor
#define NSTEP_ONE_EL_TURN   4  //number of steps to do 1 electrical turn
#define NB_OF_PHASES        4  //number of phases of the motors
#define WHEEL_PERIMETER     13 // [cm]
#define STEPS_PER_CM 		((float)NSTEP_ONE_TURN / (float)WHEEL_PERIMETER)

//timers to use for the motors
#define MOTOR_RIGHT_TIMER       TIM6
#define MOTOR_RIGHT_TIMER_EN    RCC_APB1ENR_TIM6EN
#define MOTOR_RIGHT_IRQHandler  TIM6_DAC_IRQHandler
#define MOTOR_RIGHT_IRQ         TIM6_DAC_IRQn

#define MOTOR_LEFT_TIMER        TIM7
#define MOTOR_LEFT_TIMER_EN     RCC_APB1ENR_TIM7EN
#define MOTOR_LEFT_IRQ          TIM7_IRQn
#define MOTOR_LEFT_IRQHandler   TIM7_IRQHandler

/*
 *
 *   TO COMPLETE
 *   Complete the right GPIO port and pin to be able to control the motors
 */
#define MOTOR_RIGHT_A	GPIOE, 13
#define MOTOR_RIGHT_B   GPIOE, 12
#define MOTOR_RIGHT_C	GPIOE, 14
#define MOTOR_RIGHT_D	GPIOE, 15

#define MOTOR_LEFT_A	GPIOE, 9
#define MOTOR_LEFT_B	GPIOE, 8
#define MOTOR_LEFT_C	GPIOE, 11
#define MOTOR_LEFT_D	GPIOE, 10


#define NUM_MAX_MOVEMENTS 16


/*
 *
 *   TO COMPLETE
 *   step_halt is an array containing 4 elements describing the state when the motors are off.
 *   step_table is an array of 4 lines of 4 elements. Each line describes a step.
 */
static const uint8_t step_halt[NB_OF_PHASES] = {0, 0, 0, 0};
static const uint8_t step_table[NSTEP_ONE_EL_TURN][NB_OF_PHASES] = {
		{0, 1, 1, 0},
		{0, 1, 0, 1},
		{1, 0, 0, 1},
		{1, 0, 1, 0},
};

/*
 *
 *   Hint :
 *   You can declare here static variables which can be used to store the steps counter of the motors
 *   for example. They will be available only for the code of this file.
 */

static uint32_t right_motor_steps = 0;
static uint32_t left_motor_steps = 0;
static bool right_motor_reverse = false;
static bool left_motor_reverse = false;

static Movement_parameters static_movement_sequence[NUM_MAX_MOVEMENTS] = {};
static uint32_t num_movements = 0;
static uint32_t current_movement = 0;

/*
 *
 *   TO COMPLETE
 *
 *   Performs the init of the timers and of the gpios used to control the motors
 */
void motor_init(void)
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;

	gpio_config_output_pushpull(MOTOR_RIGHT_A);
	gpio_config_output_pushpull(MOTOR_RIGHT_B);
	gpio_config_output_pushpull(MOTOR_RIGHT_C);
	gpio_config_output_pushpull(MOTOR_RIGHT_D);

	gpio_config_output_pushpull(MOTOR_LEFT_A);
	gpio_config_output_pushpull(MOTOR_LEFT_B);
	gpio_config_output_pushpull(MOTOR_LEFT_C);
	gpio_config_output_pushpull(MOTOR_LEFT_D);

	// Enable MOTOR_RIGHT_TIMER clock
	RCC->APB1ENR |= MOTOR_RIGHT_TIMER_EN;
	// Enable MOTOR_RIGHT_TIMER interrupt vector
	NVIC_EnableIRQ(MOTOR_RIGHT_IRQ);
	// Configure MOTOR_RIGHT_TIMER
	MOTOR_RIGHT_TIMER->PSC = TIMER_PRESCALER - 1; 	// Note: final timer clock  = timer clock / (prescaler + 1)
	MOTOR_RIGHT_TIMER->ARR = 300;
	MOTOR_RIGHT_TIMER->DIER |= TIM_DIER_UIE;		// Enable update interrupt
	MOTOR_RIGHT_TIMER->CR1 |= TIM_CR1_CEN; 			// Enable timer

	// Enable MOTOR_LEFT_TIMER clock
	RCC->APB1ENR |= MOTOR_LEFT_TIMER_EN;
	// Enable MOTOR_RIGHT_TIMER interrupt vector
	NVIC_EnableIRQ(MOTOR_LEFT_IRQ);
	// Configure MOTOR_RIGHT_TIMER
	MOTOR_LEFT_TIMER->PSC = TIMER_PRESCALER - 1; 	// Note: final timer clock  = timer clock / (prescaler + 1)
	MOTOR_LEFT_TIMER->ARR = 300;
	MOTOR_LEFT_TIMER->DIER |= TIM_DIER_UIE;			// Enable update interrupt
	MOTOR_LEFT_TIMER->CR1 |= TIM_CR1_CEN; 			// Enable timer

	motor_stop();
}

/*
 *
 *   TO COMPLETE
 *
 *   Updates the state of the gpios of the right motor given an array of 4 elements
 *   describing the state. For example step_table[0] which gives the first step.
 */
static void right_motor_update(const uint8_t *out)
{
	if (out[0]) gpio_set(MOTOR_RIGHT_A);
	else gpio_clear(MOTOR_RIGHT_A);

	if (out[1]) gpio_set(MOTOR_RIGHT_B);
	else gpio_clear(MOTOR_RIGHT_B);

	if (out[2]) gpio_set(MOTOR_RIGHT_C);
	else gpio_clear(MOTOR_RIGHT_C);

	if (out[3]) gpio_set(MOTOR_RIGHT_D);
	else gpio_clear(MOTOR_RIGHT_D);
}

/*
 *
 *   TO COMPLETE
 *
 *   Updates the state of the gpios of the left motor given an array of 4 elements
 *   describing the state. For exeample step_table[0] which gives the first step.
 */
static void left_motor_update(const uint8_t *out)
{
	if (out[0]) gpio_set(MOTOR_LEFT_A);
	else gpio_clear(MOTOR_LEFT_A);

	if (out[1]) gpio_set(MOTOR_LEFT_B);
	else gpio_clear(MOTOR_LEFT_B);

	if (out[2]) gpio_set(MOTOR_LEFT_C);
	else gpio_clear(MOTOR_LEFT_C);

	if (out[3]) gpio_set(MOTOR_LEFT_D);
	else gpio_clear(MOTOR_LEFT_D);
}

/*
 *
 *   TO COMPLETE
 *
 *   Stops the motors (all the gpio must be clear to 0) and set 0 to the ARR register of the timers to prevent
 *   the interrupts of the timers (because it never reaches 0 after an increment)
 */
void motor_stop(void)
{
	right_motor_update(step_halt);
	left_motor_update(step_halt);

	MOTOR_RIGHT_TIMER->ARR = 0;
	MOTOR_LEFT_TIMER->ARR = 0;
}

/*
 *
 *   TO COMPLETE
 *
 *   Sets the position to reach for each motor.
 *   The parameters are in cm for the positions and in cm/s for the speeds.
 */
void motor_set_position(float position_r, float position_l, float speed_r, float speed_l)
{
	right_motor_steps = (uint32_t)(position_r * STEPS_PER_CM);
	left_motor_steps = (uint32_t)(position_l * STEPS_PER_CM);

	motor_set_speed(speed_r, speed_l);
}

void motor_set_sequence(const Movement_parameters *movement_parameters, uint32_t size)
{
	for (uint32_t i = 0; i < size; ++i)
	{
		static_movement_sequence[i] = movement_parameters[i];
	}
	num_movements = size;

	// Start sequence
	const Movement_parameters params = static_movement_sequence[0];
	motor_set_position(params.pos_r, params.pos_l, params.vel_r, params.vel_l);

	current_movement = 0;
}

/*
 *
 *   TO COMPLETE
 *
 *   Sets the speed of the motors.
 *   The parameters are in cm/s for the speed.
 *   To set the speed, you need to change the ARR value of the timers.
 *   Remember : the timers generate an interrupt when they reach the value of ARR.
 *   Don't forget to convert properly the units in order to have the correct ARR value
 *   depending on the TIMER_FREQ and the speed chosen.
 */
void motor_set_speed(float speed_r, float speed_l)
{
	float speed_r_abs = speed_r >= 0.0f ? speed_r : -speed_r;
	float speed_l_abs = speed_l >= 0.0f ? speed_l : -speed_l;

	right_motor_reverse = speed_r < 0;
	left_motor_reverse = speed_l < 0;

	// Ensure the speed is limited to MOTOR_SPEED_LIMIT
	if (speed_r_abs > MOTOR_SPEED_LIMIT)
		speed_r_abs = MOTOR_SPEED_LIMIT;
	if (speed_l_abs > MOTOR_SPEED_LIMIT)
		speed_l_abs = MOTOR_SPEED_LIMIT;

	const float steps_per_second_r = speed_r_abs * STEPS_PER_CM;
	const float steps_per_second_l = speed_l_abs * STEPS_PER_CM;
	const uint32_t timer_ticks_per_step_r = (uint32_t)(TIMER_FREQ / steps_per_second_r);
	const uint32_t timer_ticks_per_step_l = (uint32_t)(TIMER_FREQ / steps_per_second_l);

	MOTOR_RIGHT_TIMER->ARR = timer_ticks_per_step_r - 1; // Note: timer reload takes 1 cycle, thus -1
	MOTOR_LEFT_TIMER->ARR = timer_ticks_per_step_l - 1;
}

/*
 *
 *   TO COMPLETE
 *
 *   Interrupt of the timer of the right motor.
 *   Performs a step of the motor and stops it if it reaches the position given in motor_set_position().
 */
void MOTOR_RIGHT_IRQHandler(void)
{
	/*
	 *
	 *   BEWARE !!
	 *   Based on STM32F40x and STM32F41x Errata sheet - 2.1.13 Delay after an RCC peripheral clock enabling
	 *
	 *   As there can be a delay between the instruction of clearing of the IF (Interrupt Flag) of corresponding register (named here CR) and
	 *   the effective peripheral IF clearing bit there is a risk to enter again in the interrupt if the clearing is done at the end of ISR.
	 *
	 *   As tested, only the workaround 3 is working well, then read back of CR must be done before leaving the ISR
	 *
	 */

	// Clear interrupt flag
	MOTOR_RIGHT_TIMER->SR &= ~TIM_SR_UIF;
	MOTOR_RIGHT_TIMER->SR;	// Read back in order to ensure the effective IF clearing

	static uint32_t current_step_r = 0;

	if (right_motor_steps > 0)
	{
		if (!right_motor_reverse)
			current_step_r = (current_step_r + 1) % NSTEP_ONE_EL_TURN;
		else
			current_step_r = (current_step_r - 1) % NSTEP_ONE_EL_TURN;

		right_motor_update(step_table[current_step_r]);
		--right_motor_steps;
	}
	else
	{
		right_motor_update(step_halt);
		if (left_motor_steps == 0)
		{
			++current_movement;
			if (current_movement < num_movements)
			{
				const Movement_parameters params = static_movement_sequence[current_movement];
				motor_set_position(params.pos_r, params.pos_l, params.vel_r, params.vel_l);
			}
			else
			{
				motor_stop();
			}
		}
	}
}

/*
 *
 *   TO COMPLETE
 *
 *   Interrupt of the timer of the left motor.
 *   Performs a step of the motor and stops it if it reaches the position given in motor_set_position().
 */
void MOTOR_LEFT_IRQHandler(void)
{
	/*
	 *
	 *   BEWARE !!
	 *   Based on STM32F40x and STM32F41x Errata sheet - 2.1.13 Delay after an RCC peripheral clock enabling
	 *
	 *   As there can be a delay between the instruction of clearing of the IF (Interrupt Flag) of corresponding register (named here CR) and
	 *   the effective peripheral IF clearing bit there is a risk to enter again in the interrupt if the clearing is done at the end of ISR.
	 *
	 *   As tested, only the workaround 3 is working well, then read back of CR must be done before leaving the ISR
	 *
	 */

	// Clear interrupt flag
	MOTOR_LEFT_TIMER->SR &= ~TIM_SR_UIF;
	MOTOR_LEFT_TIMER->SR;	// Read back in order to ensure the effective IF clearing

	static uint32_t current_step_l = 0;

	if (left_motor_steps > 0)
	{
		if (!left_motor_reverse)
			current_step_l = (current_step_l + 1) % NSTEP_ONE_EL_TURN;
		else
			current_step_l = (current_step_l - 1) % NSTEP_ONE_EL_TURN;

		left_motor_update(step_table[current_step_l]);
		--left_motor_steps;
	}
	else
	{
		left_motor_update(step_halt);
		if (right_motor_steps == 0)
		{
			++current_movement;
			if (current_movement < num_movements)
			{
				const Movement_parameters params = static_movement_sequence[current_movement];
				motor_set_position(params.pos_r, params.pos_l, params.vel_r, params.vel_l);
			}
			else
			{
				motor_stop();
				gpio_clear(GPIOD, 14);
			}
		}
	}
}

