#pragma once

#include <vector>
#include <functional>

#include "Fuzzyficator.hpp"

template <typename ControlLawInput>
using ControlLaw = std::function<float(ControlLawInput, float)>;

template <typename ControlLawInput>
class TkTsController {
	private:
		Fuzzyficator fuzzyficator;
		std::vector<
			std::function<float(ControlLawInput, float)>
		> control_laws;

	public:
		TkTsController(const TkTsController &_tkts_controller);
		TkTsController(
			std::initializer_list<Mem_func> _fuzzyficator,
			std::initializer_list<
				std::function<float(ControlLawInput, float)>
			> _control_laws
		);
		TkTsController(
			const Fuzzyficator &_fuzzyficator,
			std::initializer_list<
				std::function<float(ControlLawInput, float)>
			> _control_laws
		);
		TkTsController(
			const Fuzzyficator &_fuzzyficator,
			const std::vector<
				std::function<float(ControlLawInput, float)>
			> &_control_laws
		);
		TkTsController(
			std::initializer_list<Mem_func> _fuzzyficator,
			const std::vector<
				std::function<float(ControlLawInput, float)>
			> &_control_laws
		);

		const std::vector<
			std::function<float(ControlLawInput, float)>,
			std::allocator<std::function<float(ControlLawInput, float)>>
		>::const_iterator begin() const;
		const std::vector<
			std::function<float(ControlLawInput, float)>,
			std::allocator<std::function<float(ControlLawInput, float)>>
		>::const_iterator end() const;

		float operator() (float value, ControlLawInput sys_values, float reference);
};
