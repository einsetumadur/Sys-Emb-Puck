#include <stm32f4xx.h>
#include <system_clock_config.h>
#include <gpio.h>
#include <main.h>
#include <timer.h>
#include <motor.h>
#include <selector.h>

#define PI                  3.1415926536f
//TO ADJUST IF NECESSARY. NOT ALL THE E-PUCK2 HAVE EXACTLY THE SAME WHEEL DISTANCE
#define WHEEL_DISTANCE      5.35f    //cm
#define PERIMETER_EPUCK     (PI * WHEEL_DISTANCE)

// Init function required by __libc_init_array
void _init(void) {}

// Simple delay function
void delay(unsigned int n)
{
	while (n--) {
		__asm__ volatile ("nop");
	}
}


int main(void)
{
	SystemClock_Config();

	// Enable GPIOD peripheral clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;


	// Configure the pin of FRONT_LED to Alternate Function 2 (TIM4_CH3)
	//gpio_config_output_af_pushpull(FRONT_LED, 2);

	gpio_config_output_pushpull(FRONT_LED);

	//timer4_start();

	motor_init();

	/*const Movement_parameters movement_sequence[] =
		{
		{10, 10, -5, 5},
		{PERIMETER_EPUCK/4, PERIMETER_EPUCK/4, 2, 2},
		{10, 10, -5, 5},
		{PERIMETER_EPUCK/4, PERIMETER_EPUCK/4, 2, 2},
		{10, 10, -5, 5},
		{PERIMETER_EPUCK/4, PERIMETER_EPUCK/4, 2, 2},
		{10, 10, -5, 5},
		{PERIMETER_EPUCK/4, PERIMETER_EPUCK/4, 2, 2}
		};*/

		const Movement_parameters movement_sequence[] = {{31.4f, 65.0f, -3.14f, 6.5f}};

	motor_set_sequence(movement_sequence, sizeof(movement_sequence) / sizeof(Movement_parameters));

	gpio_set(FRONT_LED);

	while(1) {}
}

