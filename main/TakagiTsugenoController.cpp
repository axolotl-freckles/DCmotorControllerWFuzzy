#ifndef TAKAGI_TSUGENO_CONTROLLER_CPP
#define TAKAGI_TSUGENO_CONTROLLER_CPP

#include "TakagiTsugenoController.hpp"

#include <numeric>
#include <cassert>

TkTsController::TkTsController (const TkTsController &_tkts_controller)
:
	fuzzyficator_(_tkts_controller.fuzzyficator()),
	control_laws(_tkts_controller.begin(), _tkts_controller.end())
{
	assert(fuzzyficator_.size() == control_laws.size());
}
TkTsController::TkTsController (
	std::initializer_list<Mem_func> _fuzzyficator,
	std::initializer_list<PIDController> _PID_controllers
)
: fuzzyficator_(_fuzzyficator), control_laws(_PID_controllers)
{
	assert(fuzzyficator_.size() == control_laws.size());
}
TkTsController::TkTsController (
	const Fuzzyficator &_fuzzyficator,
	std::initializer_list<PIDController> _PID_controllers
)
: fuzzyficator_(_fuzzyficator), control_laws(_PID_controllers)
{
	assert(fuzzyficator_.size() == control_laws.size());
}
TkTsController::TkTsController (
	const Fuzzyficator &_fuzzyficator,
	const std::vector<PIDController> &_PID_controllers
)
: fuzzyficator_(_fuzzyficator), control_laws(_PID_controllers)
{
	assert(fuzzyficator_.size() == control_laws.size());
}
TkTsController::TkTsController (
	std::initializer_list<Mem_func> _fuzzyficator,
	const std::vector<PIDController> &_control_laws
)
: fuzzyficator_(_fuzzyficator), control_laws(_control_laws)
{
	assert(fuzzyficator_.size() == control_laws.size());
}

float TkTsController::operator() (float fuzzyficable_val, float input)
{
	std::vector<float> mu(fuzzyficator_.size());
	std::vector<float> control_out(fuzzyficator_.size());
	(void)fuzzyficator_(fuzzyficable_val, mu);

	for (size_t i=0; i<fuzzyficator_.size(); i++) {
		control_out[i] = control_laws[i](input);
	}

	float acum = std::inner_product(mu.begin(), mu.end(), control_out.begin(), 0.0f);
	return acum / std::accumulate(mu.begin(), mu.end(), 0.0f);
}

#endif