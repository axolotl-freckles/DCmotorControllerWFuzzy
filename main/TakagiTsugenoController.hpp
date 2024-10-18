/**
 * @file TakagiTsugenoController.hpp
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
  |-> TkTsController
*/
#pragma once

#include <vector>
#include <functional>

#include "Fuzzyficator.hpp"
#include "pid.hpp"
#include "pid.cpp"

class TkTsController {
public:
	TkTsController();
	explicit TkTsController(const TkTsController &_tkts_controller);
	TkTsController(
		std::initializer_list<Mem_func> _fuzzyficator,
		std::initializer_list<PIDController> _PID_controllers
	);
	TkTsController(
		const Fuzzyficator &_fuzzyficator,
		std::initializer_list<PIDController> _PID_controllers
	);
	TkTsController(
		const Fuzzyficator &_fuzzyficator,
		const std::vector<PIDController> &_PID_controllers
	);
	TkTsController(
		std::initializer_list<Mem_func> _fuzzyficator,
		const std::vector<PIDController> &_control_laws
	);

	inline const std::vector<PIDController,
		std::allocator<PIDController>
	>::const_iterator begin() const { return control_laws.begin(); }
	inline const std::vector<PIDController,
		std::allocator<PIDController>
	>::const_iterator end()  const { return control_laws.end(); }

	inline const Fuzzyficator& fuzzyficator() const { return fuzzyficator_; }

	/**
	 * @brief Calculates the control signal u using Takagi-Tsugeno
	 * fuzzy logic
	 * 
	 * @param fuzzyficable_val value that determines the fuzzyfication
	 * @param input value to pass to the PID controllers
	 * @return u
	 */
	float operator() (float fuzzyficable_val, float input);
private:
	Fuzzyficator fuzzyficator_;
	std::vector<PIDController> control_laws;
};
