/**
 * @file MamdaniController.hpp
 * @author ACMAX (aavaloscorrales@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2024-10-29
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#pragma once

#include <vector>

#include "pid.hpp"
#include "Fuzzyficator.hpp"

/**
 * @brief C++'s Array of Arrays that represent the FAM, the first index corresponds
 * to the error, the second to the derivative of the error
 * 
 * @tparam T 
 */
template <typename T>
using vMatrix_t = std::vector<std::vector<T>>;

/**
 * @brief Primitive (or C style) Array of Arrays that represents the FAM, the
 * first index corresponds to the error, the second to the derivative of the error
 * 
 * @tparam T 
 */
template <typename T>
using cMatrix_t = T**;

class MamdaniController {
public:
	MamdaniController();
	explicit MamdaniController(const MamdaniController &_other);
	MamdaniController(
		const float SAMPLE_TIME_s,
		const Fuzzyficator &error_fuzzyficator,
		const Fuzzyficator &error_derivative_fuzzyficator,
		const vMatrix_t<float> &FAM
	);
	MamdaniController(
		const float SAMPLE_TIME_s,
		const Fuzzyficator &error_fuzzyficator,
		const Fuzzyficator &error_derivative_fuzzyficator,
		const cMatrix_t<float> FAM
	);
	MamdaniController(
		const float SAMPLE_TIME_s,
		std::initializer_list<Mem_func> _error_membership_functions,
		std::initializer_list<Mem_func> _error_derivative_membership_functions,
		const vMatrix_t<float> &FAM
	);
	MamdaniController(
		const float SAMPLE_TIME_s,
		std::initializer_list<Mem_func> _error_memebership_functions,
		std::initializer_list<Mem_func> _error_derivative_membership_functions,
		const cMatrix_t<float> FAM
	);

	// Accessors
	inline       float               SAMPLE_TIME_s()               const {return _SAMPLE_TIME_s;}
	inline const Fuzzyficator&       errorFuzzyficator()           const {return _error_fuzzyficator;}
	inline const Fuzzyficator&       errorDerivativeFuzzyficator() const {return _error_derivative_fuzzyficator;}
	inline const Derivator&          errorDerivator()              const {return _error_derivator;}
	inline const vMatrix_t<float>&   FAM()                         const {return _FAM;}
	inline const std::vector<float>& operator[] (int idx)          const {return _FAM[idx];}

	/**
	 * @brief Operates over the error and returns a control signal
	 * 
	 * @param error 
	 * @return u, control signal
	 */
	float operator() (float error);

private:
	const float      _SAMPLE_TIME_s;
	Fuzzyficator     _error_fuzzyficator;
	Fuzzyficator     _error_derivative_fuzzyficator;
	Derivator        _error_derivator;
	vMatrix_t<float> _FAM;
	std::vector<float> error_mu;
	std::vector<float> error_derivative_mu;
};
