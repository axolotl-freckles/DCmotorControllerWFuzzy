#ifndef CONTROLLER_TASK_CPP
#define CONTROLLER_TASK_CPP

#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "taskClass.hpp"

#include "Fuzzyficator.hpp"
#include "TakagiTsugenoController.hpp"

class ControllerTask : public Task {
	private:
		TkTsController<float> controller;
		Fuzzyficator        fuzzyficator;
		const QueueHandle_t refer_speed;
		const QueueHandle_t motor_speed;

	public:
		ControllerTask(
			const char *name,
			uint32_t    stack_size,
			QueueHandle_t _refer_speed,
			QueueHandle_t _motor_speed,
			std::initializer_list<Mem_func> _membership_functions,
			std::initializer_list<
				std::function<float(float, float)>
			> _control_laws
		)
		: Task(name, stack_size, 2),
		  controller(_membership_functions, _control_laws),
		  fuzzyficator(_membership_functions),
		  refer_speed(_refer_speed), motor_speed(_motor_speed)
		{}

		void taskFunction() {
			while (true) {
				const int N_FUZZY = fuzzyficator.size();
				char buffer[10+7*N_FUZZY+10+1] = {0};
				int  offset = 0;
				float mu[N_FUZZY] = {0};

				float motor_speed_f = 0.0f;
				(void)xQueueReceive(motor_speed, &motor_speed_f, 10);
				float refer_speed_f = 0.0f;
				(void)xQueueReceive(refer_speed, &refer_speed_f, 10);

				float u = controller(motor_speed_f, motor_speed_f, refer_speed_f);
				fuzzyficator(refer_speed_f, mu);

				offset = sprintf(buffer,"\r%6.2f: [", refer_speed_f);
				for (int i=0; i<N_FUZZY; i++)
					offset += sprintf(buffer+offset, " %6.3f", mu[i]);
				offset += sprintf(buffer+offset, "] u:%6.2f", u);
				(void)printf("%s", buffer);

				vTaskDelay(50 / portTICK_PERIOD_MS);
			}
		}
};

#endif