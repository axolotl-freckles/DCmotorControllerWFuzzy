/**
 * @file Filters.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-11-05
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#pragma once

class Filter {
public:
	/**
	 * @brief Passes the value through a filter and returns the filtered value
	 * 
	 * @param value Next sample on the signal to filter
	 * @return Sample of the filtered signal
	 */
	virtual float operator() (float value) = 0;
};

/**
 * @brief Sliding average filter, filters the signal by
 * passing it through an average of the last 'n' samples
 * 
 * @tparam av_window_size: the amount of samples to average.
 */
template <int av_window_size = 2>
class SlidingAverage : public Filter {
public:
	explicit SlidingAverage(const SlidingAverage<av_window_size> &other);
	SlidingAverage();
	SlidingAverage(float starting_average);

	inline const float* samples()  const { return _samples; }
	inline float current_average() const { return _curr_av; }

	inline virtual float operator() (float value) override {
		_samples[idx] = value;
		_curr_av += (_samples[idx] - _samples[(idx+av_window_size)%av_window_size])/av_window_size;
		idx = (idx+1)%av_window_size;
		return _curr_av;
	}

private:
	float _samples[av_window_size];
	float _curr_av;
	int idx;
};

/**
 * @brief LowPass capacitive filter
 * 
 */
class LowPass : public Filter {
public:
	explicit LowPass(const LowPass &other);
	LowPass(float rc, float sampleTime_s);

	inline float alpha()   const { return _alpha; }
	inline float prevVal() const { return _prev_val; }

	inline virtual float operator() (float value) override {
		float new_val = _alpha*value + (1-_alpha)*_prev_val;
		_prev_val = new_val;
		return new_val;
	}

private:
	const float _alpha;
	float _prev_val;
};

template<int av_window_size> SlidingAverage<av_window_size>::SlidingAverage(const SlidingAverage<av_window_size> &other)
:
	Filter(),
	_curr_av(other._curr_av), idx(0)
{
	for (int i=0; i<av_window_size; i++) _samples[i] = other.samples()[i];
}
template<int av_window_size> SlidingAverage<av_window_size>::SlidingAverage()
:
	Filter(),
	_samples{0}, _curr_av(0), idx(0)
{}
template<int av_window_size> SlidingAverage<av_window_size>::SlidingAverage(float starting_average)
:
	Filter(),
	_samples{0}, _curr_av(starting_average), idx(0)
{}

LowPass::LowPass(const LowPass &other)
:
	Filter(),
	_alpha(other.alpha()), _prev_val(other.prevVal())
{}
LowPass::LowPass(float rc, float sampleTime_s)
:
	Filter(),
	_alpha(sampleTime_s/(sampleTime_s+rc)), _prev_val(0.0)
{}