#ifndef UTILS_UTILS_H
#define UTILS_UTILS_H

#include <cassert>
#include "units.h"
#include <chrono>
#include <thread>


namespace utils {

	const double pi = 3.1415926535897932384626433832795;

	/**
	 * limits a value to a certain range
	 *
	 * @param _value value to check if it's in range, otherwise it's being clipped
	 * @param _minValue
	 * @param _maxValue
	 *
	 * Note: _minValue needs to be smaller or equal to _maxValue
	 */
	template<typename T1, typename T2, typename T3>
	T1 limited(T1 const& _value, T2 const& _minValue, T3 const& _maxValue) {
		assert(_minValue <= _maxValue);
		if (_value < _minValue) {
			return _minValue;
		} else if (_value > _maxValue) {
			return _maxValue;
		}
		return _value;
	}

	Radian normalize(Radian _angle);

	Second getCurrentTime();

	void delay(Second s);
	bool kbhit();
	void nonblock(bool _nonblocking);


}

#define UNUSED(stuff) (void)(stuff);

#endif
