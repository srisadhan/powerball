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

// code for color display in the terminal
namespace Color {
    enum Code {
        FG_RED      = 31,
        FG_GREEN    = 32,
        FG_BLUE     = 34,
        FG_DEFAULT  = 39,
        BG_RED      = 41,
        BG_GREEN    = 42,
        BG_BLUE     = 44,
        BG_DEFAULT  = 49
    };
    class Modifier {
        Code code;
    public:
        Modifier(Code pCode) : code(pCode) {}
        friend std::ostream&
        operator<<(std::ostream& os, const Modifier& mod) {
            return os << "\033[" << mod.code << "m";
        }
    };
}

#if defined(__linux__)
  //the following are UBUNTU/LINUX ONLY terminal color codes.
    #define DEFAULT     "\033[0m"
    #define RED         "\033[31m"              /* Red */
    #define GREEN       "\033[32m"              /* Green */
#endif

#define UNUSED(stuff) (void)(stuff);

#endif
