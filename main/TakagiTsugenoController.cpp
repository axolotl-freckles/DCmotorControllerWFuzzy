#ifndef TAKAGI_TSUGENO_CONTROLLER_CPP
#define TAKAGI_TSUGENO_CONTROLLER_CPP

#include <numeric>
#include <cassert>

#include "TakagiTsugenoController.hpp"

TkTsController::TkTsController (const TkTsController &_tkts_controller)
:
	fuzzyficator(_tkts_controller.getFuzzyficator()),
	control_laws(_tkts_controller.begin(), _tkts_controller.end())
{
	assert(fuzzyficator.size() == control_laws.size());
}
TkTsController::TkTsController (
	std::initializer_list<Mem_func> _fuzzyficator,
	std::initializer_list<PIDController> _PID_controllers
)
: fuzzyficator(_fuzzyficator), control_laws(_PID_controllers)
{
	assert(fuzzyficator.size() == control_laws.size());
}
TkTsController::TkTsController (
	const Fuzzyficator &_fuzzyficator,
	std::initializer_list<PIDController> _PID_controllers
)
: fuzzyficator(_fuzzyficator), control_laws(_PID_controllers)
{
	assert(fuzzyficator.size() == control_laws.size());
}
TkTsController::TkTsController (
	const Fuzzyficator &_fuzzyficator,
	const std::vector<PIDController> &_PID_controllers
)
: fuzzyficator(_fuzzyficator), control_laws(_PID_controllers)
{
	assert(fuzzyficator.size() == control_laws.size());
}
TkTsController::TkTsController (
	std::initializer_list<Mem_func> _fuzzyficator,
	const std::vector<PIDController> &_control_laws
)
: fuzzyficator(_fuzzyficator), control_laws(_control_laws)
{
	assert(fuzzyficator.size() == control_laws.size());
}

const std::vector<
	PIDController,
	std::allocator<PIDController>
>::const_iterator
TkTsController::begin() const {
	return control_laws.begin();
}
const std::vector<
	PIDController,
	std::allocator<PIDController>
>::const_iterator
TkTsController::end() const {
	return control_laws.end();
}

const Fuzzyficator& TkTsController::getFuzzyficator() const {
	return fuzzyficator;
}

float TkTsController::operator() (float fuzzyficable_val, float input)
{
	std::vector<float> mu(fuzzyficator.size());
	std::vector<float> control_out(fuzzyficator.size());
	(void)fuzzyficator(fuzzyficable_val, mu);

	for (size_t i=0; i<fuzzyficator.size(); i++) {
		control_out[i] = control_laws[i](input);
	}

	float acum = std::inner_product(mu.begin(), mu.end(), control_out.begin(), 0.0f);
	return acum / std::accumulate(mu.begin(), mu.end(), 0.0f);
}

#endif