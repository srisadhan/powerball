#include <utils.h>

#include <cstdio>
#include <sys/select.h>
#include <fcntl.h>
#include <termios.h>

namespace utils {

	/**
	 * Normalization of an angle into [-pi to +pi[
	 * (excluding +pi)
	 *
	 *
	 * @param _angle value to normalize
	 */
	Radian normalize(Radian _angle) {
		if (_angle < pi * radians && _angle >= -pi * radians)
			return _angle;

		/* from -2pi to 2*pi */
		double normalizedAngle = fmod(_angle.value(), 2.*pi);

		/* from -pi to pi */
		if (normalizedAngle >= pi) {
			normalizedAngle = normalizedAngle - 2.*pi;
		} else if (normalizedAngle < -pi)
			normalizedAngle = normalizedAngle + 2.*pi;

		return normalizedAngle * radians;
	}

	Second getCurrentTime() {
		auto now = std::chrono::steady_clock::now().time_since_epoch();
		double timeSinceEpoch = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
		return timeSinceEpoch / 1000000000 * seconds;
	}

	void delay(Second s) {
		std::chrono::milliseconds duration(int(s.value() * 1000.));
		std::this_thread::sleep_for(duration);
	}

	bool kbhit()
	{
		struct timeval tv;
		fd_set fds;
		tv.tv_sec = 0;
		tv.tv_usec = 0;
		FD_ZERO(&fds);
		FD_SET(STDIN_FILENO, &fds); //STDIN_FILENO is 0
		select(STDIN_FILENO+1, &fds, NULL, NULL, &tv);
		return bool(FD_ISSET(STDIN_FILENO, &fds));
	}

	void nonblock(bool _nonblocking)
	{
		struct termios ttystate;

		//get the terminal state
		tcgetattr(STDIN_FILENO, &ttystate);

		if (_nonblocking)
		{
			//turn off canonical mode
			ttystate.c_lflag &= ~ICANON;
			//minimum of number input read.
			ttystate.c_cc[VMIN] = 1;
		}
		else if (_nonblocking)
		{
			//turn on canonical mode
			ttystate.c_lflag |= ICANON;
		}
		//set the terminal attributes.
		tcsetattr(STDIN_FILENO, TCSANOW, &ttystate);

	}

}
