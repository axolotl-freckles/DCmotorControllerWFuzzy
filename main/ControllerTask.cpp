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

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "globalVar.h"
#include "taskClass.hpp"

#include "Fuzzyficator.hpp"
#include "TakagiTsugenoController.hpp"
#include "MamdaniController.hpp"
#include "pid.hpp"
#include "pid.cpp"
#include "k_values.h"
#include "pwm.h"

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

	static const TickType_t QUEUE_TIMEOUT = 10;

	// taskFunction constants:
	static constexpr int   N_PREV_SPEEDS = 6;
	static constexpr float TRANSITION_DURATION = 2.0f;
	static constexpr float TRANSITION_STEP     = SAMPLE_TIME_s / TRANSITION_DURATION;
	static constexpr float MAX_REFER_CHANGE  = 50.0f;
	static constexpr float BEZIER_SMOOTHNESS =  0.1f;

	static constexpr float CORR_ZERO     = 0.1;
	static constexpr float CORR_POCA     = 0.2;
	static constexpr float CORR_MODERADA = 0.6;
	static constexpr float CORR_GRANDE   = 0.95;

public:
	ControllerTask(
		const char *name,
		uint32_t    stack_size,
		QueueHandle_t _refer_speed_q,
		QueueHandle_t _motor_speed_q,
		ledc_channel_t _pwm_channel
	)
	: Task(name, stack_size, 2),
		refer_speed_q(_refer_speed_q), motor_speed_q(_motor_speed_q),
		pwm_channel(_pwm_channel)
	{}

	void taskFunction() {
		// PIDController pid(
		// 	SAMPLE_TIME_s,
		// 	0.02f, 0.005f, 0.0003f
		// );
		// pid.addAntiWindup(0.0, 1.0);
		const int N_FUZZY = 5;
		std::vector<PIDController> pids = {
			PIDController(SAMPLE_TIME_s, 10.6534, 8.7664, 0.0),
			PIDController(SAMPLE_TIME_s, 10.3998, 8.9971, 0.0),
			PIDController(SAMPLE_TIME_s, 10.1461, 9.2278, 0.0),
			PIDController(SAMPLE_TIME_s,  9.8924, 9.4585, 0.0),
			PIDController(SAMPLE_TIME_s,  9.6388, 9.6892, 0.0)
		};
		for (PIDController &pid : pids)
			pid.addAntiWindup(0.0, 30.0);

		TkTsController takagi (
			{
				Tria_memf(-10.0, 0.0, 100.0, -1),
				Tria_memf(0.0, 100.0, 200.0),
				Tria_memf(100.0, 200.0, 300.0),
				Tria_memf(200.0, 300.0, 400.0),
				Tria_memf(300.0, 400.0, 410.0, 1)
			},
			pids
		);
		float mu[N_FUZZY] = {0};

		const Fuzzyficator errorFuzz {
			Tria_memf(-10.0,   0.0, 100.0, -1),
			Tria_memf(  0.0, 100.0, 200.0),
			Tria_memf(100.0, 200.0, 300.0),
			Tria_memf(200.0, 300.0, 400.0),
			Tria_memf(300.0, 400.0, 410.0, 1)
		};
		const Fuzzyficator errorDerivativeFuzz {
			Tria_memf(-10.0,   0.0,  50.0, -1),
			Tria_memf(  0.0,  50.0, 100.0),
			Tria_memf( 50.0, 100.0, 150.0),
			Tria_memf(100.0, 150.0, 200.0),
			Tria_memf(150.0, 200.0, 210.0, 1)
		};
		const vMatrix_t<float> FAM = {
			{  CORR_GRANDE, CORR_MODERADA, CORR_POCA, CORR_POCA, CORR_ZERO},
			{CORR_MODERADA,     CORR_POCA, CORR_POCA, CORR_ZERO, CORR_ZERO},
			{CORR_MODERADA,     CORR_POCA, CORR_ZERO, CORR_ZERO, CORR_ZERO},
			{    CORR_POCA,     CORR_ZERO, CORR_ZERO, CORR_ZERO, CORR_ZERO},
			{    CORR_ZERO,     CORR_ZERO, CORR_ZERO, CORR_ZERO, CORR_ZERO}
		};
		MamdaniController mamdani(SAMPLE_TIME_s, errorFuzz, errorDerivativeFuzz, FAM);

		float prev_motor_speeds[N_PREV_SPEEDS] = {0};
		int   motor_idx = 0;

		float motor_speed = 0.0f;
		float refer_speed = 0.0f;

		float bezier_t = 0.0f;
		float bezier_speed_ref = 0.0f;
		float prev_refer_speed = 0.0f;

		while (true) {
			(void)xQueueReceive(motor_speed_q, &motor_speed, QUEUE_TIMEOUT);
			(void)xQueueReceive(refer_speed_q, &refer_speed, QUEUE_TIMEOUT);

			prev_motor_speeds[motor_idx] = motor_speed;
			motor_idx = (motor_idx+1)%N_PREV_SPEEDS;
			motor_speed = 
				std::accumulate(prev_motor_speeds, prev_motor_speeds+N_PREV_SPEEDS, 0.0f)
				/N_PREV_SPEEDS;

			if (std::abs(refer_speed - prev_refer_speed) > rpm2rad_s(MAX_REFER_CHANGE)) {
				float P0 = prev_refer_speed;
				float P1 = prev_refer_speed + BEZIER_SMOOTHNESS;
				float P2 = refer_speed      - BEZIER_SMOOTHNESS;
				float P3 = refer_speed;

				bezier_speed_ref = bezierCurve(bezier_t, P0, P1, P2, P3);
				bezier_t += TRANSITION_STEP;
				if (bezier_t > TRANSITION_DURATION)
					prev_refer_speed = bezier_speed_ref;
			}
			else {
				bezier_t = 0.0f;
				bezier_speed_ref = refer_speed;
			}

			float err = bezier_speed_ref - motor_speed;
			// float u   = pid(err);
			// float u = takagi(rad_s2rpm(motor_speed), err);
			float u = mamdani(rad_s2rpm(err));
			// takagi.fuzzyficator()(rad_s2rpm(motor_speed), mu);

			constexpr float U_MIN = 0.0f, U_MAX = 1.0f;
			// u = std::clamp(u, U_MIN, U_MAX);
			constexpr float OUT_MIN = 0.17f, OUT_MAX = 0.95f;
			uint8_t pwm_out = (uint8_t)(std::clamp((u-U_MIN)/(U_MAX-U_MIN), OUT_MIN, OUT_MAX)*PWM_MAX);
			pwm_out &= PWM_MAX;

			pwm_set_duty(pwm_channel, pwm_out);

			char buffer[29+N_FUZZY*4+23+1] = {0};
			int  offset = 0;
			offset  = sprintf(buffer       ,"\rR:%6.2f ", rad_s2rpm(refer_speed));
			offset += sprintf(buffer+offset,  "M:%6.2f ", rad_s2rpm(motor_speed));
			offset += sprintf(buffer+offset,  "e:%6.2f [", err);
			for (int i=0; i<N_FUZZY; i++)
				offset += sprintf(buffer+offset, "%3.1f ", mu[i]);
			offset += sprintf(buffer+offset,  "] => u:%9.2e o: %3d", u, pwm_out);
			(void)printf("%s", buffer);

			vTaskDelay(80 / portTICK_PERIOD_MS);
		}
	}
};

#endif