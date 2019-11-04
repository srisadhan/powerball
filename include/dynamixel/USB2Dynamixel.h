/*
 * USB2Dynamixel.h
 *
 *  Created on: 17.02.2015
 *      Author: lutz
 */

#ifndef SRC_TRANSPORT_USB2DYNAMIXEL_H_
#define SRC_TRANSPORT_USB2DYNAMIXEL_H_

#include <string>
#include <map>

#include "dynamixel.h"
#include <utils.h>
#include <mutex>

class USB2Dynamixel_pimpl;

class USB2Dynamixel {
public:

	typedef std::function<void(dynamixel::motorID motor, bool success, uint8_t error, const uint8_t* receiveBuffer, uint8_t rxPayloadLen)> callback;


	USB2Dynamixel(int baudrate, std::vector<std::string> deviceNames, uint maxJobCount);
	USB2Dynamixel(std::vector<std::string> deviceNames, uint maxJobCount);

	virtual ~USB2Dynamixel();

	void setBaudrate(uint newBaudrate);

	bool ping(dynamixel::motorID motor, Second timeout, callback cb, std::mutex* mutex = nullptr);

	bool read(dynamixel::motorID motor, dynamixel::Register baseRegister, uint8_t length, Second timeout, callback cb, std::mutex* mutex = nullptr);

	bool write(dynamixel::motorID motor, dynamixel::Register baseRegister, dynamixel::parameter const& writeBuffer, std::mutex* mutex = nullptr);

	// write reg is not implemented since it's not necessary see sync_write
	// action is not implemented (see above)

	bool reset(dynamixel::motorID motor, std::mutex* mutex = nullptr);

	bool sync_write(std::map<dynamixel::motorID, dynamixel::parameter> const& motorParams, dynamixel::Register baseRegister, std::mutex* mutex = nullptr);

private:
	USB2Dynamixel_pimpl *m_pimpl;

	uint8_t calculateChecksum(dynamixel::parameter const& packet) const;
};

#endif /* SRC_TRANSPORT_USB2DYNAMIXEL_H_ */
