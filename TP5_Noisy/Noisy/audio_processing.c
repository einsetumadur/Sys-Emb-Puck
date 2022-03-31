#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <motors.h>
#include <audio/microphone.h>
#include <audio_processing.h>
#include <communications.h>
#include <fft.h>
#include <arm_math.h>

//semaphore
static BSEMAPHORE_DECL(sendToComputer_sem, TRUE);

//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
static float micLeft_cmplx_input[2 * FFT_SIZE];
static float micRight_cmplx_input[2 * FFT_SIZE];
static float micFront_cmplx_input[2 * FFT_SIZE];
static float micBack_cmplx_input[2 * FFT_SIZE];
//Arrays containing the computed magnitude of the complex numbers
static float micLeft_output[FFT_SIZE];
static float micRight_output[FFT_SIZE];
static float micFront_output[FFT_SIZE];
static float micBack_output[FFT_SIZE];

/*
 *	Callback called when the demodulation of the four microphones is done.
 *	We get 160 samples per mic every 10ms (16kHz)
 *
 *	params :
 *	int16_t *data			Buffer containing 4 times 160 samples. the samples are sorted by micro
 *							so we have [micRight1, micLeft1, micBack1, micFront1, micRight2, etc...]
 *	uint16_t num_samples	Tells how many data we get in total (should always be 640)
 */
void processAudioData(int16_t *data, uint16_t num_samples) {

	/*
	 *
	 *	We get 160 samples per mic every 10ms
	 *	So we fill the samples buffers to reach
	 *	1024 samples, then we compute the FFTs.
	 *
	 */

	float* const buffer_left_cmplx_input = get_audio_buffer_ptr(
			LEFT_CMPLX_INPUT);
	float* const buffer_right_cmplx_input = get_audio_buffer_ptr(
			RIGHT_CMPLX_INPUT);
	float* const buffer_front_cmplx_input = get_audio_buffer_ptr(
			FRONT_CMPLX_INPUT);
	float* const buffer_back_cmplx_input = get_audio_buffer_ptr(
			BACK_CMPLX_INPUT);
	float* const buffer_left_output = get_audio_buffer_ptr(LEFT_OUTPUT);
	float* const buffer_right_output = get_audio_buffer_ptr(RIGHT_OUTPUT);
	float* const buffer_front_output = get_audio_buffer_ptr(FRONT_OUTPUT);
	float* const buffer_back_output = get_audio_buffer_ptr(BACK_OUTPUT);

	static uint16_t index = 0;

	for (uint16_t i = 0; i < num_samples && index < FFT_SIZE * 2; ++i) {
		buffer_left_cmplx_input[index] = data[i * 4 + 0];
		buffer_left_cmplx_input[index + 1] = 0;

		buffer_right_cmplx_input[index] = data[i * 4 + 1];
		buffer_right_cmplx_input[index + 1] = 0;

		buffer_front_cmplx_input[index] = data[i * 4 + 2];
		buffer_front_cmplx_input[index + 1] = 0;

		buffer_back_cmplx_input[index] = data[i * 4 + 3];
		buffer_back_cmplx_input[index + 1] = 0;

		index += 2;
	}

	static uint8_t num_FFTs = 0;

	if (index == FFT_SIZE * 2) {
		// Compute the FFT

		doFFT_optimized(FFT_SIZE, buffer_left_cmplx_input);
		arm_cmplx_mag_f32(buffer_left_cmplx_input, buffer_left_output,
		FFT_SIZE);

		doFFT_optimized(FFT_SIZE, buffer_right_cmplx_input);
		arm_cmplx_mag_f32(buffer_right_cmplx_input, buffer_right_output,
		FFT_SIZE);

		//doFFT_optimized(FFT_SIZE, buffer_front_cmplx_input);
		arm_cmplx_mag_f32(buffer_front_cmplx_input, buffer_front_output,
		FFT_SIZE);

		doFFT_optimized(FFT_SIZE, buffer_back_cmplx_input);
		arm_cmplx_mag_f32(buffer_back_cmplx_input, buffer_back_output,
		FFT_SIZE);

		index = 0;

		++num_FFTs;
	}

	if (num_FFTs == 10) {
		chBSemSignal(&sendToComputer_sem);
		num_FFTs = 0;
	}
}

void wait_send_to_computer(void) {
	chBSemWait(&sendToComputer_sem);
}

float* get_audio_buffer_ptr(BUFFER_NAME_t name) {
	if (name == LEFT_CMPLX_INPUT) {
		return micLeft_cmplx_input;
	} else if (name == RIGHT_CMPLX_INPUT) {
		return micRight_cmplx_input;
	} else if (name == FRONT_CMPLX_INPUT) {
		return micFront_cmplx_input;
	} else if (name == BACK_CMPLX_INPUT) {
		return micBack_cmplx_input;
	} else if (name == LEFT_OUTPUT) {
		return micLeft_output;
	} else if (name == RIGHT_OUTPUT) {
		return micRight_output;
	} else if (name == FRONT_OUTPUT) {
		return micFront_output;
	} else if (name == BACK_OUTPUT) {
		return micBack_output;
	} else {
		return NULL;
	}
}
