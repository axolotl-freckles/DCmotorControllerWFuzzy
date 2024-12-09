/**
 * @file pid.hpp
 * @author ACMAX (aavaloscorrales@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2024-10-17
 * 
 * @copyright Copyright (c) 2024
 * 
 */
/*
Classes:
  |-> Integrator
  |-> Derivator
  |-> PIDController
*/
#pragma once

class Integrator {
public:
	Integrator();
	Integrator(const float SAMPLE_TIME_s, const float starting_value = 0.0f);
	explicit Integrator(const Integrator &_other);

	inline float sampleTime() const         { return _SAMPLE_TIME_s;}
	inline float integralAcumulator() const { return _integral_acum;}
	inline void setIntegralAcumulator(float integral_acum) {
		_integral_acum = integral_acum;
	}

	/**
	 * @brief Calculates the integral of the given value
	 * 
	 * @param value Next value to integrate
	 * @return float Integral of the history of values
	 */
	float operator() (float value);
private:
	const float _SAMPLE_TIME_s;
	float       _integral_acum;
};

class Derivator {
public:
	Derivator();
	Derivator(const float SAMPLE_TIME_S, const float starting_value = 0.0f);
	explicit Derivator(const Derivator &_other);

	inline float sampleTime() const    { return _SAMPLE_TIME_s; }
	inline float previousValue() const { return _prev_val; }
	
	/**
	 * @brief Calculates the derivative of the given value
	 * 
	 * @param value Next value to calculate the derivative
	 * @return float Derivative of the value
	 */
	float operator() (float value);
private:
	const float _SAMPLE_TIME_s;
	float       _prev_val;
};

class PIDController {
public:
	static constexpr int KP = 0;
	static constexpr int KI = 1;
	static constexpr int KD = 2;

	PIDController ();
	PIDController (
		const float SAMPLE_TIME_s,
		const float K_p,
		const float K_i,
		const float K_d,
		const float integrator_starting_value = 0.0f,
		const float derivator_starting_value  = 0.0f
	);
	PIDController (
		const float SAMPLE_TIME_s,
		const float K_gains[],
		const float integrator_starting_value = 0.0f,
		const float derivator_starting_value  = 0.0f
	);
	explicit PIDController (
		const PIDController &_other
	);

	inline float sampleTime() const { return _SAMPLE_TIME_s; }
	inline const Derivator&  derivator()  const { return _deriv; }
	inline const Integrator& integrator() const { return _intgr; }

	/**
	 * @brief Get the K gains of the controller
	 * 
	 * @param out primitive array of size 3 where the gains are to be copied
	 */
	void getKs(float *out) const;

	inline float saturatorMin() const { return _saturator_min; }
	inline float saturatorMax() const { return _saturator_max; }

	/**
	 * @brief Adds anti-windup functionality to the PID controller
	 * to prevent the integrator value to explode
	 * 
	 * @param _saturator_min 
	 * @param _saturator_max 
	 */
	void addAntiWindup(float saturator_min, float saturator_max);

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
private:
	const float _SAMPLE_TIME_s;
	float _K_gains[3];
	Integrator _intgr;
	Derivator  _deriv;

	float _saturator_max;
	float _saturator_min;
	float _integral;
};
