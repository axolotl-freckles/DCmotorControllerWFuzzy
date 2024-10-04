#pragma once

#include "driver/ledc.h"

extern "C" {

/**
 * @brief PWM configuration status(error) codes
 * 
 */
typedef enum {
	PWM_FAIL          = -1,
	PWM_OK            =  0,
	PWM_TIMER_ERR     =  1,
	PWM_CHANNEL_ERR   =  2,
	PWM_INTERRUPT_ERR =  3
} pwm_err_t;

/**
 * @brief Combined status codes for PWM configuration
 * 
 */
typedef struct {
	pwm_err_t pwm_err;
	esp_err_t esp_err;
} pwm_config_err_t;


/**
 * @brief Sets the duty cycle of a PWM channel, sets the hpoint to 0
 * 
 * @param channel PWM channel
 * @param duty    PWM duty cycle value
 * @return esp_err_t 
 */
inline esp_err_t pwm_set_duty(ledc_channel_t channel, uint32_t duty) {
	return ledc_set_duty_and_update(LEDC_HIGH_SPEED_MODE, channel, duty, 0);
}

/**
 * @brief Initializes a PWM channel
 * 
 * @param gpio_num      GPIO pin for output signal
 * @param channel       PWM channel to identify
 * @param timer_num     Timer to habilitate the PWM signal
 * @param freq_hz       PWM frequency, from the start to start of the duty cycle
 * @param resolution    Duty cycle increment resolution
 * @param duty          Starting duty cycle
 * @param output_invert Inverts the PWM signal
 * @return pwm_config_err_t 
 * 
 * @note The duty cycle resolution and PWM frequency are inversly proportional,
 * incrementing the PWM frequency may require a smaller resolution.
 */
inline pwm_config_err_t innit_pwm
(
	int              gpio_num,
	ledc_channel_t   channel,
	ledc_timer_t     timer_num,
	uint32_t         freq_hz,
	ledc_timer_bit_t resolution,
	uint32_t         duty,
	unsigned int     output_invert
)
{
	ledc_timer_config_t timer_config = {
		.speed_mode      = LEDC_HIGH_SPEED_MODE,
		.duty_resolution = resolution,
		.timer_num       = timer_num,
		.freq_hz         = freq_hz,
		.clk_cfg         = LEDC_AUTO_CLK,
		.deconfigure     = false
	};

	esp_err_t err = ledc_timer_config(&timer_config);
	if (err != ESP_OK) {
		return {
			.pwm_err = PWM_TIMER_ERR,
			.esp_err = err
		};
	}

	ledc_channel_config_t channel_config = {
		.gpio_num   = gpio_num,
		.speed_mode = LEDC_HIGH_SPEED_MODE,
		.channel    = channel,
		.intr_type  = LEDC_INTR_DISABLE,
		.timer_sel  = timer_num,
		.duty       = duty,
		.hpoint     = 0,
		.flags = {.output_invert = output_invert}
	};

	err = ledc_channel_config(&channel_config);
	if (err != ESP_OK) {
		return {
			.pwm_err = PWM_CHANNEL_ERR,
			.esp_err = err
		};
	}
	err = ledc_fade_func_install(0);
	if (err != ESP_OK) {
		return {
			.pwm_err = PWM_INTERRUPT_ERR,
			.esp_err = err
		};
	}

	return {
		.pwm_err = PWM_OK,
		.esp_err = ESP_OK
	};
}
}
