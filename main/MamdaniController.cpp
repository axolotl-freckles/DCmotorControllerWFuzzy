/**
 * @file MamdaniController.cpp
 * @author ACMAX (aavaloscorrales@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2024-10-29
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef MAMDANI_CONTROLLER_CPP
#define MAMDANI_CONTROLLER_CPP

#include "MamdaniController.hpp"

float MamdaniController::operator() (float error) {
	float error_derivative = _error_derivator(error);

	(void)_error_fuzzyficator(error, error_mu);
	(void)_error_derivative_fuzzyficator(error_derivative, error_derivative_mu);

	float acum_rules = 0.0f;
	float acum_infer = 0.0f;

	for (int i=0; i<_error_fuzzyficator.size(); i++) {
		for (int j=0; j<_error_derivative_fuzzyficator.size(); j++) {
			float inference = error_mu[i]*error_derivative_mu[j];

			acum_rules += inference*_FAM[i][j];
			acum_infer += inference;
		}
	}

	return acum_rules / acum_infer;
}

MamdaniController::MamdaniController(const MamdaniController &_other)
:
	_SAMPLE_TIME_s(_other.SAMPLE_TIME_s()),
	_error_fuzzyficator(_other.errorFuzzyficator()),
	_error_derivative_fuzzyficator(_other.errorDerivativeFuzzyficator()),
	_error_derivator(_other.errorDerivator()),
	_FAM(_other.FAM()),
	error_mu(_error_fuzzyficator.size()),
	error_derivative_mu(_error_derivative_fuzzyficator.size())
{}
MamdaniController::MamdaniController(
	const float SAMPLE_TIME_s,
	const Fuzzyficator &error_fuzzyficator,
	const Fuzzyficator &error_derivative_fuzzyficator,
	const vMatrix_t<float> &FAM
)
:
	_SAMPLE_TIME_s(SAMPLE_TIME_s),
	_error_fuzzyficator(error_fuzzyficator),
	_error_derivative_fuzzyficator(error_derivative_fuzzyficator),
	_error_derivator(SAMPLE_TIME_s),
	_FAM(FAM),
	error_mu(_error_fuzzyficator.size()),
	error_derivative_mu(_error_derivative_fuzzyficator.size())
{}
MamdaniController::MamdaniController(
	const float SAMPLE_TIME_s,
	const Fuzzyficator &error_fuzzyficator,
	const Fuzzyficator &error_derivative_fuzzyficator,
	const cMatrix_t<float> FAM
)
:
	_SAMPLE_TIME_s(SAMPLE_TIME_s),
	_error_fuzzyficator(error_fuzzyficator),
	_error_derivative_fuzzyficator(error_derivative_fuzzyficator),
	_error_derivator(SAMPLE_TIME_s),
	_FAM(_error_fuzzyficator.size(), std::vector<float>(_error_derivative_fuzzyficator.size())),
	error_mu(_error_fuzzyficator.size()),
	error_derivative_mu(_error_derivative_fuzzyficator.size())
{
	for (size_t i=0; i<_error_fuzzyficator.size(); i++) {
		for (size_t j=0; j<_error_derivative_fuzzyficator.size(); j++) {
			_FAM[i][j] = FAM[i][j];
		}
	}
}
MamdaniController::MamdaniController(
	const float SAMPLE_TIME_s,
	std::initializer_list<Mem_func> _error_membership_functions,
	std::initializer_list<Mem_func> _error_derivative_membership_functions,
	const vMatrix_t<float> &FAM
)
:
	_SAMPLE_TIME_s(SAMPLE_TIME_s),
	_error_fuzzyficator(_error_membership_functions),
	_error_derivative_fuzzyficator(_error_derivative_membership_functions),
	_error_derivator(SAMPLE_TIME_s),
	_FAM(FAM),
	error_mu(_error_fuzzyficator.size()),
	error_derivative_mu(_error_derivative_fuzzyficator.size())
{}
MamdaniController::MamdaniController(
	const float SAMPLE_TIME_s,
	std::initializer_list<Mem_func> _error_memebership_functions,
	std::initializer_list<Mem_func> _error_derivative_membership_functions,
	const cMatrix_t<float> FAM
)
:
	_SAMPLE_TIME_s(SAMPLE_TIME_s),
	_error_fuzzyficator(_error_memebership_functions),
	_error_derivative_fuzzyficator(_error_derivative_membership_functions),
	_error_derivator(SAMPLE_TIME_s),
	_FAM(_error_fuzzyficator.size(), std::vector<float>(_error_derivative_fuzzyficator.size())),
	error_mu(_error_fuzzyficator.size()),
	error_derivative_mu(_error_derivative_fuzzyficator.size())
{
	for (size_t i=0; i<_error_fuzzyficator.size(); i++) {
		for (size_t j=0; j<_error_derivative_fuzzyficator.size(); j++) {
			_FAM[i][j] = FAM[i][j];
		}
	}
}

#endif