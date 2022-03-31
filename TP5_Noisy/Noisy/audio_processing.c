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

	for (uint16_t i = 0; i < num_samples; i += 4) {
		buffer_right_cmplx_input[index] = data[i];
		buffer_right_cmplx_input[index + 1] = 0;

		buffer_left_cmplx_input[index] = data[i + 1];
		buffer_left_cmplx_input[index + 1] = 0;

		buffer_back_cmplx_input[index] = data[i + 2];
		buffer_back_cmplx_input[index + 1] = 0;

		buffer_front_cmplx_input[index] = data[i + 3];
		buffer_front_cmplx_input[index + 1] = 0;

		index += 2;
		if (index == FFT_SIZE * 2) {
			break;
		}
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

		doFFT_optimized(FFT_SIZE, buffer_front_cmplx_input);
		arm_cmplx_mag_f32(buffer_front_cmplx_input, buffer_front_output,
		FFT_SIZE);

		doFFT_optimized(FFT_SIZE, buffer_back_cmplx_input);
		arm_cmplx_mag_f32(buffer_back_cmplx_input, buffer_back_output,
		FFT_SIZE);

		index = 0;

		++num_FFTs;
	}

// #define SEND_DATA_TO_PC

#ifdef SEND_DATA_TO_PC
	if (num_FFTs == 10) {
		chBSemSignal(&sendToComputer_sem);

		const uint16_t peak_pos = get_peak_pos(buffer_front_output, FFT_SIZE);
		const float peak_frequency = get_frequency(peak_pos);
		chprintf((BaseSequentialStream *) &SDU1, "peak at %f Hz\r\n",
				peak_frequency);

		command_motors(peak_frequency);

		num_FFTs = 0;
	}
#else
	if (num_FFTs > 0) {
		const uint16_t peak_pos = get_peak_pos(buffer_front_output, FFT_SIZE);
		const float peak_frequency = get_frequency(peak_pos);

		command_motors(peak_frequency);

		num_FFTs = 0;
	}
#endif
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

uint16_t get_peak_pos(const float *buffer, uint16_t size) {
	float max_value = 0.0f;
	uint16_t index = size / 2;
	for (uint16_t i = size / 2; i < size; ++i) {
		if (buffer[i] > max_value) {
			max_value = buffer[i];
			index = i;
		}
	}
	return index;
}

float get_frequency(uint16_t peak_pos) {
	// 2 measurements to determine the slope and offset
	const float x0 = 513.0f;
	const float y0 = 7800.0f;
	const float x1 = 1004.0f;
	const float y1 = 300.0f;

	// y = slope * (x - x0) + y0
	const float slope = (y1 - y0) / (x1 - x0);
	return slope * (peak_pos - x0) + y0;
}

void command_motors(float frequency) {
	if (frequency > 1000.0f) {
		uint16_t speed = (uint16_t) (frequency / 4.0f);
		if (speed > 2200)
			speed = 2200;
		right_motor_set_speed(speed);
		left_motor_set_speed(speed);
	} else {
		right_motor_set_speed(0);
		left_motor_set_speed(0);
	}
}

