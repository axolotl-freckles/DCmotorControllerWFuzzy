#ifndef PID_CPP
#define PID_CPP

#include "pid.hpp"

#include <math.h>

Integrator::Integrator (const float SAMPLE_TIME_s, const float starting_value)
: _SAMPLE_TIME_s(SAMPLE_TIME_s), _integral_acum(starting_value)
{}
Integrator::Integrator (const Integrator &_other)
: _SAMPLE_TIME_s(_other.sampleTime()), _integral_acum(_other.integralAcumulator())
{}

float Integrator::operator() (float value) {
	_integral_acum += value*_SAMPLE_TIME_s;
	return _integral_acum;
}


Derivator::Derivator (const float SAMPLE_TIME_s, const float starting_value)
: _SAMPLE_TIME_s(SAMPLE_TIME_s), _prev_val(starting_value)
{}
Derivator::Derivator (const Derivator &_other)
: _SAMPLE_TIME_s(_other.sampleTime()), _prev_val(_other.previousValue())
{}

float Derivator::operator() (float value) {
	float out = (value - _prev_val)/_SAMPLE_TIME_s;
	_prev_val = value;
	return out;
}


PIDController::PIDController (
	const float SAMPLE_TIME_s,
	const float K_p,
	const float K_i,
	const float K_d,
	const float integrator_starting_value,
	const float derivator_starting_value
)
:
	_SAMPLE_TIME_s(SAMPLE_TIME_s),
	_intgr(SAMPLE_TIME_s, integrator_starting_value),
	_deriv(SAMPLE_TIME_s, derivator_starting_value),
	_saturator_max(infinityf()), _saturator_min(-infinityf()),
	_integral(integrator_starting_value)
{
	_K_gains[KP] = K_p;
	_K_gains[KI] = K_i;
	_K_gains[KD] = K_d;
}
PIDController::PIDController (
	const float SAMPLE_TIME_s,
	const float K_gains[],
	const float integrator_starting_value,
	const float derivator_starting_value
)
:
	_SAMPLE_TIME_s(SAMPLE_TIME_s),
	_intgr(SAMPLE_TIME_s, integrator_starting_value),
	_deriv(SAMPLE_TIME_s, derivator_starting_value),
	_saturator_max(infinityf()), _saturator_min(-infinityf()),
	_integral(integrator_starting_value)
{
	_K_gains[KP] = K_gains[KP];
	_K_gains[KI] = K_gains[KI];
	_K_gains[KD] = K_gains[KD];
}
PIDController::PIDController (
	const PIDController &_other
)
:
	_SAMPLE_TIME_s(_other.sampleTime()),
	_intgr(_other.integrator()),
	_deriv(_other.derivator()),
	_saturator_max(_other.saturatorMax()),
	_saturator_min(_other.saturatorMin()),
	_integral(_other.integrator().integralAcumulator())
{
	_other.getKs(_K_gains);
}

void PIDController::getKs(float *out) const {
	out[KP] = _K_gains[KP];
	out[KI] = _K_gains[KI];
	out[KD] = _K_gains[KD];
}

void PIDController::addAntiWindup(float saturator_min, float saturator_max) {
	_saturator_max = saturator_max;
	_saturator_min = saturator_min;
}

float PIDController::operator() (float value) {
	float derivative = _deriv(value);

	if (value > _saturator_min && value < _saturator_max)
		_integral = _intgr(value);

	return value*_K_gains[KP] + _integral*_K_gains[KI] + derivative*_K_gains[KD];
}

float PIDController::operator[] (int idx) const {
	return _K_gains[idx%3];
}

#endif