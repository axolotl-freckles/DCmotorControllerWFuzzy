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
#include "pid.hpp"
#include "pid.cpp"
#include "k_values.h"
#include "pwm.h"

float bezierCurve(float t, float P0, float P1, float P2, float P3) {
	return
		  P0*(1-t)*(1-t)*(1-t) +
		3*P1*(1-t)*(1-t)*t +
		3*P2*(1-t)*t*t +
		  P3*t*t*t;
}

class ControllerTask : public Task {
private:
	const QueueHandle_t refer_speed_q;
	const QueueHandle_t motor_speed_q;

public:
	ControllerTask(
		const char *name,
		uint32_t    stack_size,
		QueueHandle_t _refer_speed_q,
		QueueHandle_t _motor_speed_q
	)
	: Task(name, stack_size, 2),
		refer_speed_q(_refer_speed_q), motor_speed_q(_motor_speed_q)
	{}

	void taskFunction() {
		PIDController pid(
			SAMPLE_TIME_s,
			0.02f, 0.005f, 0.0003f
		);
		pid.addAntiWindup(0.0, 1.0);
		const int N_FUZZY = 5;

		TkTsController takagi (
			{
				Tria_memf(-10.0, 0.0, 100.0, -1),
				Tria_memf(0.0, 100.0, 200.0),
				Tria_memf(100.0, 200.0, 300.0),
				Tria_memf(200.0, 300.0, 400.0),
				Tria_memf(300.0, 400.0, 410.0, 1)
			},
			std::vector<PIDController>(N_FUZZY, pid)
		);
		float mu[N_FUZZY] = {0};

		const int N_PREV_SPEEDS = 6;
		float prev_motor_speeds[N_PREV_SPEEDS] = {0};
		int   motor_idx = 0;

		float motor_speed = 0.0f;
		float refer_speed = 0.0f;

		float t = 0.0f;
		float bezier_speed_ref = 0.0f;
		const float transition_duration = 2.0f;
		const float transition_step     = SAMPLE_TIME_s / transition_duration;
		const float MAX_REFER_CHANGE  = 50.0f;
		const float BEZIER_SMOOTHNESS =  0.1f;

		while (true) {
			(void)xQueueReceive(motor_speed_q, &motor_speed, 10);
			(void)xQueueReceive(refer_speed_q, &refer_speed, 10);

			prev_motor_speeds[motor_idx] = motor_speed;
			motor_idx = (motor_idx+1)%N_PREV_SPEEDS;
			motor_speed = 
				std::accumulate(prev_motor_speeds, prev_motor_speeds+N_PREV_SPEEDS, 0.0f)
				/N_PREV_SPEEDS;

			if (std::abs(refer_speed - bezier_speed_ref) > rpm2rad_s(MAX_REFER_CHANGE)) {
				float P0 = bezier_speed_ref;
				float P1 = bezier_speed_ref + BEZIER_SMOOTHNESS;
				float P2 = refer_speed      - BEZIER_SMOOTHNESS;
				float P3 = refer_speed;

				bezier_speed_ref = bezierCurve(t, P0, P1, P2, P3);
				t += transition_step;
			}
			else {
				t = 0.0f;
				bezier_speed_ref = refer_speed;
			}

			float err = bezier_speed_ref - motor_speed;
			// float u   = pid(err);
			float u = takagi(rad_s2rpm(motor_speed), err);
			takagi.fuzzyficator()(rad_s2rpm(motor_speed), mu);

			const float U_MIN = 0.17f, U_MAX = 0.95f;
			uint8_t pwm_out = (uint8_t)(std::clamp(u, U_MIN, U_MAX)*PWM_MAX);
			pwm_out &= PWM_MAX;

			pwm_set_duty(LEDC_CHANNEL_0, pwm_out);

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