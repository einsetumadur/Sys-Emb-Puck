#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <camera/po8030.h>

#include <process_image.h>

static float distance_cm = 0;
static float pos_ang = 0;

typedef struct {
	uint8_t av;
	uint8_t threshold;
	int16_t pos;
	uint16_t length;
} Line_info;

static uint8_t average(uint8_t image[IMAGE_BUFFER_SIZE]) {
	uint16_t sum = 0;
	for (uint16_t i = 0; i < IMAGE_BUFFER_SIZE; ++i) {
		sum += image[i];
	}
	return (uint8_t) (sum / IMAGE_BUFFER_SIZE);
}

static Line_info detect_line(uint8_t image[IMAGE_BUFFER_SIZE]) {

	const uint8_t av = average(image);
	const uint8_t threshold = av; //(uint8_t) (((uint16_t) av) * 90) / 100;

	uint16_t pos_x0 = 0;
	uint16_t final_x0 = 0;
	uint16_t length = 0;
	uint16_t max_length = 0;
	bool in_stripe = false;

	for (uint16_t i = 0; i < IMAGE_BUFFER_SIZE; ++i) {
		if (image[i] < threshold) { // We are in the stripe
			if (in_stripe) {
				++length;
			} else {		// We enter the dark stripe
				pos_x0 = i;
				in_stripe = true;
				length = 1;
			}
		} else {
			if (in_stripe) {		// We exit the dark stripe
				in_stripe = false;
				if (length > max_length) {
					max_length = length;
					length = 0;
					final_x0 = pos_x0;
				}
			}
		}
	}

	const Line_info result = { .av = av, .threshold = threshold, .pos = (final_x0
			+ max_length / 2)-320, .length = max_length };
	return result;
}

static float get_distance(uint16_t length_pixels) {
	const float tan_angle = 0.4142135f; // if total angle = 45°, tan_angle = 1 else
	const float length_strip_cm = 2.0f;
	const float scale_constant = (float) (IMAGE_BUFFER_SIZE) * length_strip_cm
			/ (2 * tan_angle);
	return scale_constant / (float) (length_pixels);
}

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

	chRegSetThreadName(__FUNCTION__);
	(void) arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 10 + 11 (minimum 2 lines because reasons)
	po8030_advanced_config(FORMAT_RGB565, 0, 10, IMAGE_BUFFER_SIZE, 2,
			SUBSAMPLING_X1, SUBSAMPLING_X1);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

	while (1) {
		//starts a capture
		dcmi_capture_start();
		//waits for the capture to be done
		wait_image_ready();
		//signals an image has been captured
		chBSemSignal(&image_ready_sem);
	}
}

static BSEMAPHORE_DECL(dist_ready_sem, TRUE);

static THD_WORKING_AREA(waProcessImage, 1024);
static THD_FUNCTION(ProcessImage, arg) {

	chRegSetThreadName(__FUNCTION__);
	(void) arg;

	uint8_t *img_buff_ptr = NULL;
	uint8_t image[IMAGE_BUFFER_SIZE] = { 0 };

	uint8_t current_loop = 0;

	//systime_t last_time = chVTGetSystemTime();

	while (1) {

		/*systime_t current_time = chVTGetSystemTime();
		 chprintf((BaseSequentialStream *) &SD3, "%i ms\r\n", ST2MS(current_time - last_time));
		 last_time = current_time;*/

		// Waits until an image has been captured
		chBSemWait(&image_ready_sem);

		// Gets the pointer to the array filled with the last image in RGB565
		img_buff_ptr = dcmi_get_last_image_ptr();

		for (uint16_t i = 0; i < IMAGE_BUFFER_SIZE; ++i) {
			// Extract red component
			image[i] = (uint8_t) ((img_buff_ptr[2 * i] & (uint8_t) 0b11111000)
					>> 3);
		}

		if (current_loop == 0) {
			//SendUint8ToComputer(image, IMAGE_BUFFER_SIZE);
		}

		// Send only one image out of 4
		//current_loop = (current_loop + 1) % 4;

		const Line_info line_info = detect_line(image);

		distance_cm = get_distance(line_info.length);
		pos_ang = (line_info.pos / 150.) - 0.3;

		//chprintf((BaseSequentialStream *) &SD3,"pos: %i ang: %.2f distance: %.2f\r\n",line_info.pos,pos_ang, distance_cm);

		chBSemSignal(&dist_ready_sem);
	}
}

float get_distance_cm(void) {
	return distance_cm;
}

float get_ang_norm(void) {
	return pos_ang;
}
void process_image_start(void) {
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO,
			ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO,
			CaptureImage, NULL);
}
