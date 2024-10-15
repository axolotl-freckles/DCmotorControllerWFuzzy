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

class ControllerTask : public Task {
	private:
		TkTsController<float*> controller;
		Fuzzyficator        fuzzyficator;
		const QueueHandle_t refer_speed_q;
		const QueueHandle_t motor_speed_q;

	public:
		ControllerTask(
			const char *name,
			uint32_t    stack_size,
			QueueHandle_t _refer_speed_q,
			QueueHandle_t _motor_speed_q,
			std::initializer_list<Mem_func> _membership_functions,
			const std::vector<
				std::function<float(float*, float)>
			> &_control_laws
		)
		: Task(name, stack_size, 2),
		  controller(_membership_functions, _control_laws),
		  fuzzyficator(_membership_functions),
		  refer_speed_q(_refer_speed_q), motor_speed_q(_motor_speed_q)
		{}

		void taskFunction() {
			PIDController pid(
				SAMPLE_TIME_s,
				0.1f, 0.0f, 0.0f
			);
			while (true) {
				// const int N_FUZZY = fuzzyficator.size();
				const int N_PREV_SPEEDS = 10;
				static float prev_motor_speeds[N_PREV_SPEEDS] = {0};
				static int   motor_idx = 0;

				static float motor_speed = 0.0f;

				(void)xQueueReceive(motor_speed_q, &motor_speed, 10);
				static float refer_speed = 0.0f;
				(void)xQueueReceive(refer_speed_q, &refer_speed, 10);

				prev_motor_speeds[motor_idx] = motor_speed;
				motor_idx = (motor_idx+1)%N_PREV_SPEEDS;
				motor_speed = std::accumulate(prev_motor_speeds, prev_motor_speeds+N_PREV_SPEEDS, 0.0f)/N_PREV_SPEEDS;
				// P I D

				// float u = controller(motor_speed, plant, refer_speed);
				float err = refer_speed - motor_speed;
				float u = pid(err);
				float real_u = u;

				const float U_MIN = 0.1f, U_MAX = 0.6f;// refer_speed;
				u = std::min(U_MAX, u);
				u = std::max(U_MIN, u);

				// const float OUT_MIN = 0.0f, OUT_MAX = 30.0f;

				// uint8_t pwm_out = (uint8_t)(u-OUT_MIN)*PWM_MAX/(OUT_MAX-OUT_MIN);
				uint8_t pwm_out = (uint8_t)(u*PWM_MAX);
				pwm_out = std::min((uint8_t)(PWM_MAX-2), pwm_out);
				pwm_out = std::max((uint8_t)10, pwm_out);
				// pwm_out &= PWM_MAX;

				pwm_set_duty(LEDC_CHANNEL_0, pwm_out);

				char buffer[29+21+1] = {0};
				int  offset = 0;
				offset  = sprintf(buffer,"\rR:%6.2f M:%6.2f e:%9.2e", rad_s2rpm(refer_speed), rad_s2rpm(motor_speed), err);
				offset += sprintf(buffer+offset, " => u:%9.2e o: %2d", real_u, pwm_out);
				(void)printf("%s", buffer);

				vTaskDelay(50 / portTICK_PERIOD_MS);
			}
		}
};

#endif