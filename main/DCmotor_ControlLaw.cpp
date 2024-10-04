#ifndef DCMOTOR_CONTROLLAW_CPP
#define DCMOTOR_CONTROLLAW_CPP

#include <vector>
#include <functional>

#include "k_values.h"

inline const std::vector<std::function<float(float, float)>> CONTROL_LAWS() {
	std::vector<std::function<float(float, float)>> control_laws(N_CONTROL_LAWS);

	for (int i=0; i<N_CONTROL_LAWS; i++) {
		control_laws[i] = [i](float val, float ref) -> float {
			return ref - K_VALUES[i][0]*val;
		};
	}

	return control_laws;
}

#endif