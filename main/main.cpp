#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_adc/adc_oneshot.h"
#include "driver/ledc.h"
#include "esp_timer.h"
#include "esp_intr_alloc.h"
#include "soc/soc.h"

#include "globalVar.h"
#include "pwm.h"
#include "Fuzzyficator.hpp"
#include "Fuzzyficator.cpp"
#include "TakagiTsugenoController.hpp"
#include "TakagiTsugenoController.cpp"
#include "DCmotor_ControlLaw.cpp"
#include "DataProcessTask.cpp"
#include "ACControllerTask.cpp"
#include "Telemetry.cpp"

extern "C" {

esp_err_t set_adc(
	adc_oneshot_unit_handle_t *adc_handle_out,
	adc_unit_t     adc_unit,
	adc_bitwidth_t adc_bitwidth,
	adc_channel_t  adc_channel
);

QueueHandle_t motor_count_q = xQueueCreate(1, sizeof(int32_t));
QueueHandle_t raw_data_q    = xQueueCreate(1, sizeof(Raw_data));
QueueHandle_t data_out_q    = xQueueCreate(1, sizeof(Data_out));

QueueHandle_t tel_ref_speed_q   = xQueueCreate(1, sizeof(float));
QueueHandle_t tel_motor_speed_q = xQueueCreate(1, sizeof(float));
QueueHandle_t tel_error_q       = xQueueCreate(1, sizeof(float));
QueueHandle_t tel_error_der_q   = xQueueCreate(1, sizeof(float));
QueueHandle_t tel_control_signal_q = xQueueCreate(1, sizeof(float));
QueueHandle_t tel_exec_time_q      = xQueueCreate(1, sizeof(float));
QueueHandle_t channels[N_TELEMETRY_CHANNELS] = {
	tel_ref_speed_q,
	tel_motor_speed_q,
	tel_error_q,
	tel_error_der_q,
	tel_control_signal_q,
	tel_exec_time_q
};

typedef struct {
	adc_oneshot_unit_handle_t adc_handle;
} timer_args;

void IRAM_ATTR send_status(void* argp) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	timer_args *args = (timer_args*)argp;
	adc_oneshot_unit_handle_t adc_handle = args->adc_handle;

	Raw_data raw_data;

	int adc_read = 0;
	(void)adc_oneshot_read(adc_handle, ADC_CHANNEL_0, &adc_read);
	raw_data.adc_read = adc_read;

	int32_t motor_count = 0;
	xQueueReceiveFromISR(motor_count_q, &motor_count, &xHigherPriorityTaskWoken);
	raw_data.motor_count = motor_count;

	xQueueOverwriteFromISR(raw_data_q, &raw_data, &xHigherPriorityTaskWoken);

	if (xHigherPriorityTaskWoken) {
		portYIELD_FROM_ISR();
	}
}

void count_encoder(void* args) {
	BaseType_t higherTaskWoken = pdFALSE;
	static int32_t motor_count = 0;
	motor_count ++;
	xQueueOverwriteFromISR(motor_count_q, &motor_count, &higherTaskWoken);
	if (higherTaskWoken) portYIELD_FROM_ISR();
}

void app_main(void)
{
	adc_oneshot_unit_handle_t adc0_handle;
	if (set_adc(&adc0_handle, ADC_UNIT_1, static_cast<adc_bitwidth_t>(ADC_BITWIDTH), ADC_CHANNEL_0))
		return;

	if (
		innit_pwm (
			PWM_OUT_GPIO, PWM_CHANNEL, LEDC_TIMER_1,
			20000, (ledc_timer_bit_t)PWM_RESOLUTION,
			0x0F, 0
		).esp_err
	)
	{
		return;
	}

	printf("Configurando Interrupcion GPIO\n");
	if (gpio_install_isr_service(0))
		return;
	// ESP_ERROR_CHECK(gpio_install_isr_service(0));
	
	if (gpio_isr_handler_add((gpio_num_t)ENCODER_GPIO, count_encoder, NULL))
		return;

	gpio_config_t motor_input_config = {
		.pin_bit_mask = (1<<ENCODER_GPIO),
		.mode         = GPIO_MODE_INPUT,
		.pull_up_en   = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_ENABLE,
		.intr_type    = GPIO_INTR_POSEDGE
	};
	printf("Configurando GPIO\n");
	if (gpio_config(&motor_input_config) != ESP_OK)
		return;
	printf("Habilitando interrupcion\n");
	if (gpio_intr_enable((gpio_num_t)ENCODER_GPIO) != ESP_OK)
		return;
	
	timer_args tmr_args = {
		.adc_handle = adc0_handle
	};
	esp_timer_create_args_t timer_config = {
		.callback = send_status,
		.arg      = (void*)&tmr_args,
		.dispatch_method = ESP_TIMER_TASK,
		.name = "Plant status update",
		.skip_unhandled_events = false
	};
	esp_timer_handle_t timer_handle;
	esp_timer_create(&timer_config, &timer_handle);
	esp_timer_start_periodic(timer_handle, SAMPLE_TIME_us);

	DataProcessTask dataProcessTask(
		"Data process Task", 800, 2,
		raw_data_q, data_out_q, SAMPLE_TIME_ms
	);
	dataProcessTask.start();
	ACControllerTask controllerTask(
		"AC Controller Task", 1000, 2,
		data_out_q, channels, SAMPLE_TIME_ms
	);
	UART uartComm(
		UART_NUM_2, TELEMETRY_TX_PIN, UART_BAUD_RATE, UART_PARITY, UART_STOP_BITS
	);
	Telemetry<N_TELEMETRY_CHANNELS> telemetryTask(
		"Telemetry Task", 2048,
		&uartComm,
		channels
	);

	(void)printf("\n\n");

	controllerTask.start();
	telemetryTask.start();
	while (true) {
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
}
// END OF MAIN ####################################################

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
