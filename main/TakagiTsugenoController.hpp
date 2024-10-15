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

		float operator() (float fuzzyficable_val, float input);
};
