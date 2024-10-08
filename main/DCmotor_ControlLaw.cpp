#ifndef DCMOTOR_CONTROLLAW_CPP
#define DCMOTOR_CONTROLLAW_CPP

#include <vector>
#include <functional>

#include "k_values.h"

inline const std::vector<std::function<float(float*, float)>> CONTROL_LAWS() {
	std::vector<std::function<float(float*, float)>> control_laws(N_CONTROL_LAWS);

	for (int i=0; i<N_CONTROL_LAWS; i++) {
		control_laws[i] = [i](float* val, float ref) -> float {
			float retval = 0.0;
			for (int j=0; j<3; j++) {
				retval += K_VALUES[i][j]*val[j];
			}
			return retval;
		};
	}

	return control_laws;
}

#endif