#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_adc/adc_oneshot.h"
#include "driver/ledc.h"
#include "esp_timer.h"

#include "pwm.h"

#include "Fuzzyficator.hpp"
#include "Fuzzyficator.cpp"
#include "TakagiTsugenoController.hpp"
#include "TakagiTsugenoController.cpp"
#include "DCmotor_ControlLaw.cpp"
#include "ControllerTask.cpp"

extern "C" {

esp_err_t set_adc(
	adc_oneshot_unit_handle_t *adc_handle_out,
	adc_unit_t     adc_unit,
	adc_bitwidth_t adc_bitwidth,
	adc_channel_t  adc_channel
);

QueueHandle_t adc_refer_speed = xQueueCreate(8, sizeof(float));

void status_update(void* argp) {
	const float MIN = -20.0f, MAX = 20.0f;
	adc_oneshot_unit_handle_t adc_handle = (adc_oneshot_unit_handle_t)argp;
	int adc_read = 0;
	(void)adc_oneshot_read(adc_handle, ADC_CHANNEL_0, &adc_read);

	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	float motor_speed = (float)adc_read*(MAX-MIN)/(float)(0b111111111) + MIN;
	xQueueSendFromISR(adc_refer_speed, &motor_speed, &xHigherPriorityTaskWoken);
}

void app_main(void)
{
	adc_oneshot_unit_handle_t adc0_handle;
	if (set_adc(&adc0_handle, ADC_UNIT_1, ADC_BITWIDTH_9, ADC_CHANNEL_0))
		return;

	if (innit_pwm(13, LEDC_CHANNEL_0, LEDC_TIMER_1, 1250000, LEDC_TIMER_6_BIT, 0x0F, 0).esp_err)
		return;
	
	esp_timer_create_args_t timer_config = {
		.callback = status_update,
		.arg      = (void*)adc0_handle,
		.dispatch_method = ESP_TIMER_TASK,
		.name = "Plant status update",
		.skip_unhandled_events = false
	};
	esp_timer_handle_t timer_handle;
	esp_timer_create(&timer_config, &timer_handle);
	esp_timer_start_periodic(timer_handle, 100000);

	Fuzzyficator fuzzyficator = {
		Tria_memf(-20.0f, -15.0f, -7.5f, -1),
		Tria_memf(-15.0f, - 7.5f,  0.0f),
		Tria_memf(- 7.5f,   0.0f,  7.5f),
		Tria_memf(  0.0f,   7.5f, 15.0f),
		Tria_memf(  7.5f,  15.0f, 20.0f, 1)
	};

	float mu[5] = {0};
	
	TkTsController<float> controller(
		fuzzyficator,
		CONTROL_LAWS()
	);

	(void)printf("\n\n");

	while (true) {
		char buffer[10+7*5+10+1] = {0};
		int  offset = 0;

		float motor_speed = 0.0;
		(void)xQueueReceive(adc_refer_speed, &motor_speed, 10);
		float u = controller(motor_speed, motor_speed, 0.0f);
		fuzzyficator(motor_speed, mu);

		offset = sprintf(buffer,"\r%6.2f: [", motor_speed);
		for (int i=0; i<5; i++)
			offset += sprintf(buffer+offset, " %6.3f", mu[i]);
		offset += sprintf(buffer+offset, "] u:%6.2f", u);
		(void)printf("%s", buffer);

		vTaskDelay(50 / portTICK_PERIOD_MS);
	}
}

esp_err_t set_adc(
	adc_oneshot_unit_handle_t *adc_handle_out,
	adc_unit_t     adc_unit,
	adc_bitwidth_t adc_bitwidth,
	adc_channel_t  adc_channel
) {
	adc_oneshot_unit_init_cfg_t unit_config = {
		.unit_id  = adc_unit,
		.clk_src  = ADC_RTC_CLK_SRC_DEFAULT,
		.ulp_mode = ADC_ULP_MODE_DISABLE
	};

	esp_err_t err = adc_oneshot_new_unit(&unit_config, adc_handle_out);
	if (err != ESP_OK) {
		(void)printf("ERROR[0x%X]: configuracion de unidad fallida\n", err);
		return err;
	}

	adc_oneshot_chan_cfg_t channel_config = {
		.atten    = ADC_ATTEN_DB_2_5,
		.bitwidth = adc_bitwidth
	};
	err = adc_oneshot_config_channel(*adc_handle_out, adc_channel, &channel_config);
	if (err != ESP_OK) {
		(void)printf("ERROR[0x%X]: configuracion de canal fallida\n", err);
		return err;
	}

	return ESP_OK;
}
}
