#ifndef PID_CPP
#define PID_CPP

#include "pid.hpp"

#include <math.h>

Integrator::Integrator (const float _SAMPLE_TIME_s, const float starting_value)
: SAMPLE_TIME_s(_SAMPLE_TIME_s), integral_acum(starting_value)
{}
Integrator::Integrator (const Integrator &_other)
: SAMPLE_TIME_s(_other.getSampleTime()), integral_acum(_other.getIntegralAcumulator())
{}

float Integrator::getSampleTime()         const { return SAMPLE_TIME_s; }
float Integrator::getIntegralAcumulator() const { return integral_acum; }
float Integrator::operator() (float value) {
	integral_acum += value*SAMPLE_TIME_s;
	return integral_acum;
}


Derivator::Derivator (const float _SAMPLE_TIME_s, const float starting_value)
: SAMPLE_TIME_s(_SAMPLE_TIME_s), prev_val(starting_value)
{}
Derivator::Derivator (const Derivator &_other)
: SAMPLE_TIME_s(_other.getSampleTime()), prev_val(_other.getPreviousValue())
{}

float Derivator::getSampleTime()    const { return SAMPLE_TIME_s; }
float Derivator::getPreviousValue() const { return prev_val; }
float Derivator::operator() (float value) {
	float out = (value - prev_val)/SAMPLE_TIME_s;
	prev_val = value;
	return out;
}


PIDController::PIDController (
	const float _SAMPLE_TIME_s,
	const float K_p,
	const float K_i,
	const float K_d,
	const float integrator_starting_value,
	const float derivator_starting_value
)
:
	SAMPLE_TIME_s(_SAMPLE_TIME_s),
	intgr(_SAMPLE_TIME_s, integrator_starting_value),
	deriv(_SAMPLE_TIME_s, derivator_starting_value),
	saturator_max(infinityf()), saturator_min(-infinityf()),
	integral(integrator_starting_value)
{
	K_gains[KP] = K_p;
	K_gains[KI] = K_i;
	K_gains[KD] = K_d;
}
PIDController::PIDController (
	const float _SAMPLE_TIME_s,
	const float _K_gains[],
	const float integrator_starting_value,
	const float derivator_starting_value
)
:
	SAMPLE_TIME_s(_SAMPLE_TIME_s),
	intgr(_SAMPLE_TIME_s, integrator_starting_value),
	deriv(_SAMPLE_TIME_s, derivator_starting_value),
	saturator_max(infinityf()), saturator_min(-infinityf()),
	integral(integrator_starting_value)
{
	K_gains[KP] = _K_gains[KP];
	K_gains[KI] = _K_gains[KI];
	K_gains[KD] = _K_gains[KD];
}
PIDController::PIDController (
	const PIDController &_other
)
:
	SAMPLE_TIME_s(_other.getSampleTime()),
	intgr(_other.getIntegrator()),
	deriv(_other.getDerivator()),
	saturator_max(_other.getSaturatorMax()),
	saturator_min(_other.getSaturatorMin()),
	integral(_other.getIntegrator().getIntegralAcumulator())
{
	_other.getKs(K_gains);
}

float PIDController::getSampleTime() const { return SAMPLE_TIME_s; }
const Derivator&  PIDController::getDerivator()  const { return deriv; }
const Integrator& PIDController::getIntegrator() const { return intgr; }

void PIDController::getKs(float *out) const {
	out[KP] = K_gains[KP];
	out[KI] = K_gains[KI];
	out[KD] = K_gains[KD];
}
float PIDController::getSaturatorMin() const { return saturator_min; }
float PIDController::getSaturatorMax() const { return saturator_max; }

void PIDController::addAntiWindup(float _saturator_min, float _saturator_max) {
	saturator_max = _saturator_max;
	saturator_min = _saturator_min;
}

float PIDController::operator() (float value) {
	float derivative = deriv(value);

	if (value > saturator_min && value < saturator_max)
		integral = intgr(value);

	return value*K_gains[KP] + integral*K_gains[KI] + derivative*K_gains[KD];
}

float PIDController::operator[] (int idx) const {
	return K_gains[idx%3];
}

#endif