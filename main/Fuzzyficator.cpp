#ifndef FUZZYFICATOR_CPP
#define FUZZYFICATOR_CPP

#include "Fuzzyficator.hpp"

#include <math.h>

Fuzzyficator::Fuzzyficator()
: mem_funcs({Fuzzyficator::ANY_memf})
{}
Fuzzyficator::Fuzzyficator(std::initializer_list<Mem_func> _membership_functions)
: mem_funcs(_membership_functions)
{}
Fuzzyficator::Fuzzyficator(const Fuzzyficator &_fuzzyficator)
: mem_funcs(_fuzzyficator.begin(), _fuzzyficator.end())
{}
Fuzzyficator::~Fuzzyficator() {
	mem_funcs.~vector();
}

const std::vector<Mem_func, std::allocator<Mem_func>>::const_iterator
Fuzzyficator::begin() const {
	return mem_funcs.begin();
}
const std::vector<Mem_func, std::allocator<Mem_func>>::const_iterator
Fuzzyficator::end() const {
	return mem_funcs.end();
}

int Fuzzyficator::operator() (const float value, std::vector<float> &out) const {
	if (out.size() != mem_funcs.size()) return -1;

	for (size_t i=0; i<mem_funcs.size(); i++) {
		out[i] = mem_funcs[i](value);
	}
	return 0;
}
void Fuzzyficator::operator() (const float value, float *const out) const {
	for (size_t i=0; i<mem_funcs.size(); i++) {
		out[i] = mem_funcs[i](value);
	}
}

Tria_memf::Tria_memf(const float a, const float m, const float b, const int edge)
: a(a), m(m), b(b), edge(edge)
{}
float Tria_memf::operator() (const float value) {
	if (edge < 0 && value < m) return 1.0;
	if (edge > 0 && value > m) return 1.0;
	return tria_memf(value, a, m, b);
}

Trap_memf::Trap_memf(const float a, const float b, const float c, const float d, const int edge)
: a(a), b(b), c(c), d(d), edge(edge)
{}
float Trap_memf::operator()(const float value) {
	if (edge < 0 && value < b) return 1.0;
	if (edge > 0 && value > c) return 1.0;
	return trap_memf(value, a, b, c, d);
}

Bell_memf::Bell_memf(const float std_dev, const float mid, const int edge)
: std_dev(std_dev), mid(mid), edge(edge)
{}
float Bell_memf::operator()(const float value) {
	if (edge < 0 && value < mid) return 1.0;
	if (edge > 0 && value > mid) return 1.0;
	return bell_memf(value, std_dev, mid);
}

float tria_memf(const float value, const float a, const float m, const float b) {
	if (value < a) return 0.0f;
	if (value > b) return 0.0f;
	if (value < m)
		return (value - a)/(m - a);
	return (b - value)/(b - m);
}
float trap_memf(const float value, const float a, const float b, const float c, const float d) {
	if (value < a) return 0.0f;
	if (value > d) return 0.0f;
	if (value < b)
		return (value - a)/(b - a);
	if (value > c)
		return (d - value)/(d - c);
	return 1.0f;
}
float bell_memf(const float value, const float std_dev, const float mid) {
	float exponent = (value - mid)/std_dev;
	exponent = exponent*exponent;
	return exp(-exponent/2) / (std_dev*sqrt(2*M_PI));
}

#endif