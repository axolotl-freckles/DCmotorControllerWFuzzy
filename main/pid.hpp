#pragma once

class Integrator {
	private:
		const float SAMPLE_TIME_s;
		float       integral_acum;

	public:
		Integrator(const float _SAMPLE_TIME_s, const float starting_value = 0.0f);
		Integrator(const Integrator &_other);

		float getSampleTime() const;
		float getIntegralAcumulator() const;

		/**
		 * @brief Calculates the integral of the given value
		 * 
		 * @param value Next value to integrate
		 * @return float Integral of the history of values
		 */
		float operator() (float value);
};

class Derivator {
	private:
		const float SAMPLE_TIME_s;
		float       prev_val;

	public:
		Derivator(const float _SAMPLE_TIME_S, const float starting_value = 0.0f);
		Derivator(const Derivator &_other);

		float getSampleTime() const;
		float getPreviousValue() const;
		
		/**
		 * @brief Calculates the derivative of the given value
		 * 
		 * @param value Next value to calculate the derivative
		 * @return float Derivative of the value
		 */
		float operator() (float value);
};

class PIDController {
	private:
		const float SAMPLE_TIME_s;
		float K_gains[3];
		Integrator intgr;
		Derivator  deriv;

		float saturator_max;
		float saturator_min;
		float integral;

	public:
		static const int KP = 0;
		static const int KI = 1;
		static const int KD = 2;

		PIDController (
			const float _SAMPLE_TIME_s,
			const float K_p,
			const float K_i,
			const float K_d,
			const float integrator_starting_value = 0.0f,
			const float derivator_starting_value  = 0.0f
		);
		PIDController (
			const float _SAMPLE_TIME_s,
			const float _K_gains[],
			const float integrator_starting_value = 0.0f,
			const float derivator_starting_value  = 0.0f
		);
		PIDController (
			const PIDController &_other
		);

		float getSampleTime() const;
		const Derivator&  getDerivator() const;
		const Integrator& getIntegrator() const;

		/**
		 * @brief Get the K gains of the controller
		 * 
		 * @param out primitive array of size 3 where the gains are to be copied
		 */
		void getKs(float *out) const;

		float getSaturatorMin() const;
		float getSaturatorMax() const;

		/**
		 * @brief Adds anti-windup functionality to the PID controller
		 * 
		 * @param _saturator_min 
		 * @param _saturator_max 
		 */
		void addAntiWindup(float _saturator_min, float _saturator_max);

		/**
		 * @brief Apply the control law to the value
		 * 
		 * @param value 
		 * @return float "u" control signal
		 */
		float operator() (float value);
		/**
		 * @brief Get the K gain
		 * 
		 * @param idx K gain number
		 * @return float K gain value
		 */
		float operator[] (int idx) const;
};
