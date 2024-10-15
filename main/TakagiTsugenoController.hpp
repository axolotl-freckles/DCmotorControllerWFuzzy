#pragma once

#include <vector>
#include <functional>

#include "Fuzzyficator.hpp"
#include "pid.hpp"
#include "pid.cpp"

class TkTsController {
	private:
		Fuzzyficator fuzzyficator;
		std::vector<PIDController> control_laws;

	public:
		TkTsController(const TkTsController &_tkts_controller);
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

		const std::vector<
			PIDController,
			std::allocator<PIDController>
		>::const_iterator begin() const;
		const std::vector<
			PIDController,
			std::allocator<PIDController>
		>::const_iterator end() const;

		const Fuzzyficator& getFuzzyficator() const;

		/**
		 * @brief Calculates the control signal u using Takagi-Tsugeno
		 * fuzzy logic
		 * 
		 * @param fuzzyficable_val value that determines the fuzzyfication
		 * @param input value to pass to the PID controllers
		 * @return u
		 */
		float operator() (float fuzzyficable_val, float input);
};
