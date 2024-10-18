/**
 * @file Fuzzyficator.hpp
 * @author ACMAX (aavaloscorrales@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2024-10-17
 * 
 * @copyright Copyright (c) 2024
 * 
 */
/*
Functions:
  |-> tria_memf
Functors:
  |-> Tria_memf
  |-> Trap_memf
  |-> Bell_memf
Classes:
  |-> Fuzzyficator
*/
#pragma once

#include <vector>
#include <functional>

using Mem_func = std::function<float(const float)>;

class Fuzzyficator {
public:
/**
 * @brief Construct a new Fuzzyficator object
 * 
 * @param _membership_functions A list of the membership functions that
 * correspond with each linguistic value and will operate on a given input
 * value.
 */
	Fuzzyficator();
	Fuzzyficator(std::initializer_list<Mem_func> _membership_functions);
	explicit Fuzzyficator(const Fuzzyficator &_fuzzyficator);
	~Fuzzyficator();

	/**
	 * @brief Get the number of linguistic terms of the fuzzyficator.
	 * 
	 * @return size_t 
	 */
	inline size_t size() const { return mem_funcs.size(); }

	const std::vector<Mem_func, std::allocator<Mem_func>>::const_iterator begin() const;
	const std::vector<Mem_func, std::allocator<Mem_func>>::const_iterator end() const;

	/**
	 * @brief Performs the fuzzyfication of the input value
	 * 
	 * @param[in] value Value to fuzzyfy
	 * @param[out] out Output vector to store the values corresponding to each
	 * linguistic value
	 * @return int 0 if succesfull, any other if there was an error
	 */
	int operator()(const float value, std::vector<float> &out) const;
	/**
	 * @brief Performs the fuzzyfication of the input value
	 * 
	 * @param[in] value Value to fuzzyfy
	 * @param[out] out Output array of enough size to store the results of the
	 * membership functions, it is assumed that it is a valid pointer
	 */
	void operator()(const float value, float *const out) const;
private:
	std::vector<Mem_func> mem_funcs;
};

/**
 * @brief Triangle membership function
 * 
 */
struct Tria_memf {
	const float a;
	const float m;
	const float b;
	const int edge;

	/**
	 * @brief Constructs a new Triangle membership function
	 * 
	 * @param a Leftmost start of the triangle slope, must be less that 'm'
	 * @param m Middle of the triangle
	 * @param b Rightmost end of the triangle slope, must be greater than 'm'
	 * @param edge Specifies if the function is at the edge of the linguistic
	 * terms. A negative value indicates that anything to the left of 'm' will
	 * yield 1.0, a positive value indicates that anything to the right of 'm'
	 * will yield 1.0. Zero makes the function behave normally.
	 */
	Tria_memf(const float a, const float m, const float b, const int edge=0);

	/**
	 * @brief Calculates membership function
	 * 
	 * @param value 
	 * @return float 
	 */
	float operator()(const float value);
};
/**
 * @brief Trapezoidal membership function
 * 
 */
struct Trap_memf {
	const float a;
	const float b;
	const float c;
	const float d;
	const int edge;

	/**
	 * @brief Constructs a new Trapezoidal membership function
	 * 
	 * @param a Leftmost start of the triangle slope, must be less that 'b'.
	 * @param b Leftmost end of the triangle slope, must be greater that 'a' and
	 * lesser than 'c'.
	 * @param c Rightmost start of the triangle slope, must be greater than 'b'
	 * and lesser that 'd'.
	 * @param d Rightmost end of the triangle slope, must be greater than 'c'.
	 * @param edge Specifies if the function is at the edge of the linguistic
	 * terms. A negative value indicates that anything to the left of 'm' will
	 * yield 1.0, a positive value indicates that anything to the right of 'm'
	 * will yield 1.0. Zero makes the function behave normally.
	 */
	Trap_memf(const float a, const float b, const float c, const float d, const int edge=0);
	/**
	 * @brief Calculates membership function
	 * 
	 * @param value 
	 * @return float 
	 */
	float operator()(const float value);
};
struct Bell_memf {
	const float std_dev;
	const float mid;
	const int edge;

	Bell_memf(const float std_dev, const float mid, const int edge=0);
	float operator()(const float value);
};

/**
 * @brief Triangle membership function
 * 
 * @param value Value to calculate the membership
 * @param a 
 * @param m 
 * @param c 
 * @return mu [0-1]
 */
float tria_memf(const float value, const float a, const float m, const float c);

/**
 * @brief Trapezoidal membership function
 * 
 * @param value Value to calculate the membership
 * @param a 
 * @param b 
 * @param c 
 * @param d 
 * @return mu [0-1]
 */
float trap_memf(const float value, const float a, const float b, const float c, const float d);

/**
 * @brief Bell curve membership function
 * 
 * @param value Value to calculate the membership
 * @param std_dev 
 * @param mid 
 * @return mu [0-1]
 */
float bell_memf(const float value, const float std_dev, const float mid);
