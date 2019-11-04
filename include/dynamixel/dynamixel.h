/*
 * dynamixelMotor.h
 *
 *  Created on: 17.02.2015
 *      Author: lutz
 */

#ifndef DYNAMIXEL_H_
#define DYNAMIXEL_H_

#include <stdint.h>
#include <inttypes.h>
#include <vector>

namespace dynamixel
{
	typedef uint8_t motorID;

	const uint8_t broadcastID = 0xfe;

	typedef uint8_t length;
	typedef std::vector<uint8_t> parameter;
	typedef uint8_t checksum;

	enum class Instruction : uint8_t
	{
		PING       = 0x01,
		READ       = 0x02,
		WRITE      = 0x03,
		REG_WRITE  = 0x04,
		ACTION     = 0x05,
		RESET      = 0x06,
		SYNC_WRITE = 0x83,
	};

	enum class Register : uint8_t
	{
		MODEL_NUMBER        = 0x00,
		VERSION_FIRMWARE    = 0x02,
		ID                  = 0x03,
		BAUD_RATE           = 0x04,
		RETURN_DELAY_TIME   = 0x05,
		CW_ANGLE_LIMIT      = 0x06,
		CCW_ANGLE_LIMIT     = 0x08,
		TEMPERATURE_LIMIT   = 0x0b,
		VOLTAGE_LIMIT_LOW   = 0x0c,
		VOLTAGE_LIMIT_HIGH  = 0x0d,
		MAX_TORQUE          = 0x0e,
		STATUS_RETURN_LEVEL = 0x10,
		ALARM_LED           = 0x11,
		ALARM_SHUTDOWN      = 0x12,
		MULTI_TURN_OFFSET   = 0x14,
		RESOLUTION_DIVIDER  = 0x16,
		TORQUE_ENABLE       = 0x18,
		LED                 = 0x19,
		GAIN                = 0x13,
		D_GAIN              = 0x1a,
		I_GAIN              = 0x1b,
		P_GAIN              = 0x1c,
		GOAL_POSITION       = 0x1e,
		MOVING_SPEED        = 0x20,
		TORQUE_LIMIT        = 0x22,
		PRESENT_POSITION    = 0x24,
		PRESENT_SPEED       = 0x26,
		PRESENT_LOAD        = 0x28,
		PRESENT_VOLTAGE     = 0x2a,
		PRESENT_TEMPERATURE = 0x2b,
		REGISTERED          = 0x2c,
		MOVING              = 0x2e,
		LOCK                = 0x2f,
		PUNCH               = 0x30,
		GOAL_ACCELERATION   = 0x49,
	};

	inline uint32_t baudIndexToBaudrate(uint8_t baudIdx)
	{
		if (baudIdx < 250) {
			return (2000000 / (baudIdx + 1));
		} else {
			switch (baudIdx) {
				case 250:
					return 2250000;
					break;
				case 251:
					return 2500000;
					break;
				case 252:
					return 3000000;
					break;
				default:
					break;
			}
		}
		return 57600; // return factory default
	}
}



#endif /* DYNAMIXEL_H_ */
