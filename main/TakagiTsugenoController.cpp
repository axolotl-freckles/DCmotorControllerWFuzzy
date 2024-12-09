#ifndef TAKAGI_TSUGENO_CONTROLLER_CPP
#define TAKAGI_TSUGENO_CONTROLLER_CPP

#include "TakagiTsugenoController.hpp"

#include <numeric>
#include <cassert>

TkTsController::TkTsController (const TkTsController &_tkts_controller)
:
	_fuzzyficator(_tkts_controller.fuzzyficator()),
	_control_laws(_tkts_controller.begin(), _tkts_controller.end())
{
	assert(_fuzzyficator.size() == _control_laws.size());
}
TkTsController::TkTsController (
	std::initializer_list<Mem_func> fuzzyficator,
	std::initializer_list<PIDController> PID_controllers
)
: _fuzzyficator(fuzzyficator), _control_laws(PID_controllers)
{
	assert(_fuzzyficator.size() == _control_laws.size());
}
TkTsController::TkTsController (
	const Fuzzyficator &fuzzyficator,
	std::initializer_list<PIDController> PID_controllers
)
: _fuzzyficator(fuzzyficator), _control_laws(PID_controllers)
{
	assert(_fuzzyficator.size() == _control_laws.size());
}
TkTsController::TkTsController (
	const Fuzzyficator &fuzzyficator,
	const std::vector<PIDController> &PID_controllers
)
: _fuzzyficator(fuzzyficator), _control_laws(PID_controllers)
{
	assert(_fuzzyficator.size() == _control_laws.size());
}
TkTsController::TkTsController (
	std::initializer_list<Mem_func> fuzzyficator,
	const std::vector<PIDController> &control_laws
)
: _fuzzyficator(fuzzyficator), _control_laws(control_laws)
{
	assert(_fuzzyficator.size() == _control_laws.size());
}

float TkTsController::operator() (float fuzzyficable_val, float input)
{
	std::vector<float> mu(_fuzzyficator.size());
	std::vector<float> control_out(_fuzzyficator.size());
	(void)_fuzzyficator(fuzzyficable_val, mu);

	for (size_t i=0; i<_fuzzyficator.size(); i++) {
		control_out[i] = _control_laws[i](input);
	}

	float acum = std::inner_product(mu.begin(), mu.end(), control_out.begin(), 0.0f);
	return acum / std::accumulate(mu.begin(), mu.end(), 0.0f);
}

#endif