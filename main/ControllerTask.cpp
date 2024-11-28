/**
 * @file ControllerTask.cpp
 * @author ACMAX (aavaloscorrales@gmail.com), Zyanya
 * @brief 
 * @version 0.1
 * @date 2024-10-18
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#ifndef CONTROLLER_TASK_CPP
#define CONTROLLER_TASK_CPP

#include <stdio.h>
#include <algorithm>
#include <numeric>
#include <chrono>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "globalVar.h"
#include "taskClass.hpp"

#include "Fuzzyficator.hpp"
#include "TakagiTsugenoController.hpp"
#include "MamdaniController.hpp"
#include "pid.hpp"
#include "pid.cpp"
#include "Filters.hpp"
#include "pwm.h"

using namespace std::chrono;

inline float bezierCurve(float t, float P0, float P1, float P2, float P3) {
	const float oneMinusT = 1-t;
	const float oneMinusTSquared = oneMinusT*oneMinusT;
	const float oneMinusTCubed   = oneMinusTSquared*oneMinusT;
	const float tSquared  = t*t;
	const float tCubed    = tSquared*t;
	return
		  P0*oneMinusTCubed +
		3*P1*oneMinusTSquared*t +
		3*P2*oneMinusT*tSquared +
		  P3*tCubed;
}

class ControllerTask : public Task {
private:
	const QueueHandle_t refer_speed_q;
	const QueueHandle_t motor_speed_q;
	const ledc_channel_t pwm_channel;

	const QueueHandle_t _tel_ref_speed_q;
	const QueueHandle_t _tel_motor_speed_q;
	const QueueHandle_t _tel_error_q;
	const QueueHandle_t _tel_error_der_q;
	const QueueHandle_t _tel_control_signal_q;
	const QueueHandle_t _tel_exec_time_q;

	static const TickType_t QUEUE_TIMEOUT = 10;

	// taskFunction constants:
	static constexpr float OUT_MIN = 0.17f;
	static constexpr float OUT_MAX = 0.95f;
	static constexpr float U_MIN = 0.0f;
	static constexpr float U_MAX = 1.0f;

	static constexpr float CRR_CERO     = 0.00;
	static constexpr float CRR_PEQUE    = 0.25;
	static constexpr float CRR_MEDIA    = 0.50;
	static constexpr float CRR_MODERADA = 0.75;
	static constexpr float CRR_GRANDE   = 1.00;

	static constexpr float K_I = 0.02f;
	static constexpr float REF_RC   = 0.85f;
	static constexpr float INPUT_RC = 0.85f;
	static constexpr float U_RC     = 0.80f;
	static constexpr float U_ALPHA  = SAMPLE_TIME_s / (SAMPLE_TIME_s + U_RC);

public:
	ControllerTask(
		const char *name,
		uint32_t    stack_size,
		QueueHandle_t _refer_speed_q,
		QueueHandle_t _motor_speed_q,
		ledc_channel_t _pwm_channel,
		QueueHandle_t tel_ref_speed_q,
		QueueHandle_t tel_motor_speed_q,
		QueueHandle_t tel_error_q,
		QueueHandle_t tel_error_der_q,
		QueueHandle_t tel_control_signal_q,
		QueueHandle_t tel_exec_time_q
	)
	: Task(name, stack_size, 2),
		refer_speed_q(_refer_speed_q), motor_speed_q(_motor_speed_q),
		pwm_channel(_pwm_channel),
		_tel_ref_speed_q(tel_ref_speed_q),
		_tel_motor_speed_q(tel_motor_speed_q),
		_tel_error_q(tel_error_q),
		_tel_error_der_q(tel_error_der_q),
		_tel_control_signal_q(tel_control_signal_q),
		_tel_exec_time_q(tel_exec_time_q)
	{}

	void taskFunction() override {
		LowPass refFilter(REF_RC, SAMPLE_TIME_s);
		LowPass motorFilter(INPUT_RC, SAMPLE_TIME_s);
		LowPass uFilter(U_RC, SAMPLE_TIME_s);

		const Fuzzyficator errorFuzz {
			Tria_memf(-210.0, -200.0, -100.0, LEFTMOST),
			Tria_memf(-200.0, -100.0,    0.0),
			Tria_memf(-100.0,    0.0,  100.0),
			Tria_memf(   0.0,  100.0,  200.0),
			Tria_memf( 100.0,  200.0,  210.0,  RIGHTMOST)
		};
		const Fuzzyficator errorDerivativeFuzz {
			Tria_memf(-1030.0, -1020.0, -510.0, LEFTMOST),
			Tria_memf(-1020.0, -510.0,    0.0),
			Tria_memf(-510.0,    0.0,  510.0),
			Tria_memf(   0.0,  510.0,  1020.0),
			Tria_memf( 510.0,  1020.0, 1030.0, RIGHTMOST)
		};
		const vMatrix_t<float> FAM = {
			{     CRR_CERO,     CRR_CERO,     CRR_CERO,    CRR_PEQUE,    CRR_PEQUE},
			{     CRR_CERO,     CRR_CERO,    CRR_PEQUE,    CRR_PEQUE,    CRR_MEDIA},
			{     CRR_CERO,    CRR_PEQUE,    CRR_PEQUE,    CRR_MEDIA, CRR_MODERADA},
			{    CRR_PEQUE,    CRR_PEQUE,    CRR_MEDIA, CRR_MODERADA,   CRR_GRANDE},
			{    CRR_PEQUE,    CRR_MEDIA, CRR_MODERADA,   CRR_GRANDE,   CRR_GRANDE}
		};
		MamdaniController mamdani(SAMPLE_TIME_s, errorFuzz, errorDerivativeFuzz, FAM);
		Derivator derror(SAMPLE_TIME_s);
		Integrator ierror(SAMPLE_TIME_s);

		float motor_speed = 0.0f;
		float refer_speed = 0.0f;

		TickType_t x_last_time_awake = xTaskGetTickCount();
		high_resolution_clock::time_point task_st;
		high_resolution_clock::time_point task_en;

		// loop variables
		float err;
		float error_integral = 0.0f;
		float error_derivative;
		float u;
		float exec_time_us;

		while (true) {
			(void)xQueueReceive(motor_speed_q, &motor_speed, QUEUE_TIMEOUT);
			(void)xQueueReceive(refer_speed_q, &refer_speed, QUEUE_TIMEOUT);

			task_st = std::chrono::high_resolution_clock::now();
			motor_speed = motorFilter(motor_speed);
			refer_speed = refFilter(refer_speed);

			err = refer_speed - motor_speed;

			if (motor_speed < rpm2rad_s(50)) {
				u = OUT_MIN;
			}
			else {
				error_integral = (OUT_MIN < u && u < OUT_MAX) ? ierror(err) : error_integral;
				u = mamdani(rad_s2rpm(err)) + K_I*error_integral;
				u = uFilter(u);
			}
			error_derivative = derror(err);

			uint8_t pwm_out = (uint8_t)(std::clamp((u-U_MIN)/(U_MAX-U_MIN), OUT_MIN, OUT_MAX)*PWM_MAX);
			pwm_out &= PWM_MAX;

			pwm_set_duty(pwm_channel, pwm_out);

			task_en = std::chrono::high_resolution_clock::now();
			microseconds task_duration_us = duration_cast<microseconds>(task_en-task_st);
			exec_time_us = task_duration_us.count();

			(void)xQueueOverwrite(_tel_ref_speed_q,   &refer_speed);
			(void)xQueueOverwrite(_tel_motor_speed_q, &motor_speed);
			(void)xQueueOverwrite(_tel_error_q,       &error_integral);
			(void)xQueueOverwrite(_tel_error_der_q,   &error_derivative);
			(void)xQueueOverwrite(_tel_control_signal_q, &u);
			(void)xQueueOverwrite(_tel_exec_time_q, &exec_time_us);

			vTaskDelayUntil(&x_last_time_awake,SAMPLE_TIME_ms / portTICK_PERIOD_MS);
		}
	}
};

#endif