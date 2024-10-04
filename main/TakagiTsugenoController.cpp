#ifndef TAKAGI_TSUGENO_CONTROLLER_CPP
#define TAKAGI_TSUGENO_CONTROLLER_CPP

#include <numeric>
#include <cassert>

#include "TakagiTsugenoController.hpp"

template <typename ControlLawInput>
TkTsController<ControlLawInput>::TkTsController(const TkTsController &_tkts_controller)
: control_laws(_tkts_controller.begin(), _tkts_controller.end())
{}
template <typename ControlLawInput>
TkTsController<ControlLawInput>::TkTsController (
	std::initializer_list<Mem_func> _fuzzyficator,
	std::initializer_list<std::function<float(ControlLawInput, float)>> _control_laws
)
: fuzzyficator(_fuzzyficator), control_laws(_control_laws)
{
	assert(fuzzyficator.size() == control_laws.size());
}
template <typename ControlLawInput>
TkTsController<ControlLawInput>::TkTsController (
	const Fuzzyficator &_fuzzyficator,
	std::initializer_list<std::function<float(ControlLawInput, float)>> _control_laws
)
: fuzzyficator(_fuzzyficator), control_laws(_control_laws)
{
	assert(fuzzyficator.size() == control_laws.size());
}
template <typename ControlLawInput>
TkTsController<ControlLawInput>::TkTsController(
	const Fuzzyficator &_fuzzyficator,
	const std::vector<std::function<float(ControlLawInput, float)>> &_control_laws
)
: fuzzyficator(_fuzzyficator), control_laws(_control_laws)
{}
template <typename ControlLawInput>
TkTsController<ControlLawInput>::TkTsController (
	std::initializer_list<Mem_func> _fuzzyficator,
	const std::vector<
		std::function<float(ControlLawInput, float)>
	> &_control_laws
)
: fuzzyficator(_fuzzyficator), control_laws(_control_laws)
{}

template <typename ControlLawInput>
const std::vector<
	std::function<float(ControlLawInput, float)>,
	std::allocator<std::function<float(ControlLawInput, float)>>
>::const_iterator
TkTsController<ControlLawInput>::begin() const {
	return control_laws.begin();
}
template <typename ControlLawInput>
const std::vector<
	std::function<float(ControlLawInput, float)>,
	std::allocator<std::function<float(ControlLawInput, float)>>
>::const_iterator
TkTsController<ControlLawInput>::end() const {
	return control_laws.end();
}

template <typename ControlLawInput>
float TkTsController<ControlLawInput>::operator() (float value, ControlLawInput sys_values, float reference)
{
	std::vector<float> mu(fuzzyficator.size());
	std::vector<float> control_out(fuzzyficator.size());
	(void)fuzzyficator(value, mu);
	for (size_t i=0; i<fuzzyficator.size(); i++) {
		control_out[i] = control_laws[i](sys_values, reference);
	}

	float acum = std::inner_product(mu.begin(), mu.end(), control_out.begin(), 0.0f);
	return acum / std::accumulate(mu.begin(), mu.end(), 0.0f);
}

#endif