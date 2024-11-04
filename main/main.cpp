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
#include "esp_dsp.h"
#include "Fuzzyficator.hpp"
#include "Fuzzyficator.cpp"
#include "TakagiTsugenoController.hpp"
#include "TakagiTsugenoController.cpp"
#include "DCmotor_ControlLaw.cpp"
#include "ControllerTask.cpp"
#define SAMPLE_TIME_s 0.001  
#define RC 0.1        
#define FFT_SIZE 256
static float adc_buffer[FFT_SIZE];
static int adc_index = 0;
static bool fft_ready = false;
float alpha = SAMPLE_TIME_s / (SAMPLE_TIME_s + RC);
static float time_domain_data[FFT_SIZE];
#include "Telemetry.cpp"

extern "C" {

esp_err_t set_adc(
	adc_oneshot_unit_handle_t *adc_handle_out,
	adc_unit_t     adc_unit,
	adc_bitwidth_t adc_bitwidth,
	adc_channel_t  adc_channel
);

QueueHandle_t refer_speed_q = xQueueCreate(1, sizeof(float));
QueueHandle_t motor_speed_q = xQueueCreate(1, sizeof(float));
QueueHandle_t motor_count_q = xQueueCreate(1, sizeof(int));

QueueHandle_t tel_ref_speed_q   = xQueueCreate(1, sizeof(float));
QueueHandle_t tel_motor_speed_q = xQueueCreate(1, sizeof(float));
QueueHandle_t tel_error_q       = xQueueCreate(1, sizeof(float));
QueueHandle_t tel_error_der_q   = xQueueCreate(1, sizeof(float));
QueueHandle_t tel_control_signal_q = xQueueCreate(1, sizeof(float));

typedef struct {
	adc_oneshot_unit_handle_t adc_handle;
	// volatile uint32_t *motor_counter;
} timer_args;
void init_fft() {
    dsps_fft2r_init_fc32(NULL, CONFIG_DSP_MAX_FFT_SIZE);
}

void IRAM_ATTR send_status(void* argp) {
	timer_args *args = (timer_args*)argp;
	adc_oneshot_unit_handle_t adc_handle = args->adc_handle;
	const float REF_MIN = 50.f*(2*M_PI/60), REF_MAX = 400.0f*(2*M_PI/60);
	// const int N_ENCODER_SLITS = 20;

	int adc_read = 0;
	(void)adc_oneshot_read(adc_handle, ADC_CHANNEL_0, &adc_read);

	//lowpass filter
	static float prev_adc_value = 0.0;
    float adc_value = (float)adc_read * (REF_MAX - REF_MIN) / (float)(0b111111111) + REF_MIN;
    adc_value = alpha * adc_value + (1 - alpha) * prev_adc_value;
    prev_adc_value = adc_value;

	adc_buffer[adc_index++] = adc_value;
    if (adc_index >= FFT_SIZE) {
        adc_index = 0;
        fft_ready = true;  // Signal that FFT processing can start
    }

	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	float adc_value = (float)adc_read*(REF_MAX-REF_MIN)/(float)(0b111111111) + REF_MIN;

	float refer_speed = adc_value;
	float motor_speed = 0.0;

	// 20 ranuras encoder, 20 intr por revolución
	static int32_t motor_count = 0;
	int32_t prev_count = motor_count;
	xQueueReceiveFromISR(motor_count_q, &motor_count, &xHigherPriorityTaskWoken);
	motor_speed = (float)(motor_count-prev_count)*2.0f*M_PI / (ENCODER_SLITS*SAMPLE_TIME_s);

	xQueueOverwriteFromISR(refer_speed_q, &refer_speed, &xHigherPriorityTaskWoken);
	xQueueOverwriteFromISR(motor_speed_q, &motor_speed, &xHigherPriorityTaskWoken);

	if (xHigherPriorityTaskWoken) {
		portYIELD_FROM_ISR();
	}
}

void count_encoder(void* args) {
	BaseType_t higherTaskWoken = pdFALSE;
	static int32_t motor_count = 0;
	motor_count ++;
	xQueueOverwriteFromISR(motor_count_q, &motor_count, &higherTaskWoken);
}
void FFtToIFFT(void *param) {
    float fft_data[FFT_SIZE * 2];  // Buffer for complex FFT input (real and imaginary)
    
    while (true) {
        if (fft_ready) {
            fft_ready = false;

            //Buffer
            for (int i = 0; i < FFT_SIZE; i++) {
                fft_data[i * 2] = adc_buffer[i]; 
                fft_data[i * 2 + 1] = 0.0;        
            }

            //FFT
            dsps_fft2r_fc32(fft_data, FFT_SIZE);
            dsps_bit_rev_fc32(fft_data, FFT_SIZE);
            dsps_cplx2reC_fc32(fft_data, FFT_SIZE);

            //IFFT
            dsps_ifft2r_fc32(fft_data, FFT_SIZE);
            dsps_bit_rev_fc32(fft_data, FFT_SIZE);
            dsps_cplx2reC_fc32(fft_data, FFT_SIZE);

           //Normalization 
            for (int i = 0; i < FFT_SIZE; i++) {
                time_domain_data[i] = fft_data[i * 2] / FFT_SIZE;  // Only real part, normalized
            }

            //
			//xQueueSend(reconstructed_data_q, time_domain_data, portMAX_DELAY);
        }

        vTaskDelay(pdMS_TO_TICKS(10));  //Cambiar delay
    }
}
void app_main(void)
{
	adc_oneshot_unit_handle_t adc0_handle;
	if (set_adc(&adc0_handle, ADC_UNIT_1, ADC_BITWIDTH_9, ADC_CHANNEL_0))
		return;

	if (
		innit_pwm (
			14, LEDC_CHANNEL_0, LEDC_TIMER_1,
			20000, (ledc_timer_bit_t)PWM_RESOLUTION,
			0x0F, 0
		).esp_err
	)
	{
		return;
	}

	// init_fft();
    // reconstructed_data_q = xQueueCreate(10, sizeof(time_domain_data));
    
    // Start FFT processing task
    xTaskCreatePinnedToCore(FFTToIFFT, "FFT_Task", 4096, NULL, 5, NULL, 0);

	printf("Configurando Interrupcion GPIO\n");
	if (gpio_install_isr_service(0))
		return;
	
	if (gpio_isr_handler_add((gpio_num_t)26, count_encoder, NULL))
		return;

	gpio_config_t motor_input_config = {
		.pin_bit_mask = (1<<26),
		.mode         = GPIO_MODE_INPUT,
		.pull_up_en   = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_ENABLE,
		.intr_type    = GPIO_INTR_POSEDGE
	};
	printf("Configurando GPIO\n");
	if (gpio_config(&motor_input_config) != ESP_OK)
		return;
	printf("Habilitando interrupcion\n");
	if (gpio_intr_enable((gpio_num_t)26) != ESP_OK)
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

	ControllerTask controllerTask(
		"Controller Task", 2500,
		refer_speed_q,
		motor_speed_q,
		LEDC_CHANNEL_0,
		tel_ref_speed_q,
		tel_motor_speed_q,
		tel_error_q,
		tel_error_der_q,
		tel_control_signal_q
	);
	Telemetry telemetryTask(
		"Telemetry Task", 2048,
		UART_NUM_2, TELEMETRY_TX_PIN,
		tel_ref_speed_q,
		tel_error_q,
		tel_error_der_q,
		tel_control_signal_q
	);

	(void)printf("\n\n");

	controllerTask.start();
	telemetryTask.start();
	while (true) {
		vTaskDelay(100 / portTICK_PERIOD_MS);
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
