/*
 * USB2Dynamixel.cpp
 *
 *  Created on: 17.02.2015
 *      Author: lutz
 */

#include <USB2Dynamixel.h>

#include <stdio.h>         // Standard input/output definitions
#include <string.h>        // String function definitions
#include <unistd.h>        // UNIX standard function definitions
#include <fcntl.h>         // File control definitions
#include <errno.h>         // Error number definitions
#include <termios.h>
#include <sys/ioctl.h>
#include <inttypes.h>
#include <linux/types.h>
#include <linux/serial.h>
#include <iostream>

#include <queue>
#include <thread>
#include <condition_variable>


// copied from termbits.h, as it conflicts with other include files
struct termios2 {
	tcflag_t c_iflag;		/* input mode flags */
	tcflag_t c_oflag;		/* output mode flags */
	tcflag_t c_cflag;		/* control mode flags */
	tcflag_t c_lflag;		/* local mode flags */
	cc_t c_line;			/* line discipline */
	cc_t c_cc[19];		/* control characters */
	speed_t c_ispeed;		/* input speed */
	speed_t c_ospeed;		/* output speed */
};

#ifndef BOTHER
#define BOTHER 0010000
#endif


class DynamixelTransaction {
public:

	dynamixel::motorID motorID;
	dynamixel::parameter writeBuffer;
	dynamixel::parameter readBuffer;

	dynamixel::length rxLength{0};
	Second rxTimeout{0 * seconds};

	std::mutex *m_waitMutex{nullptr};

	bool needsResponse{false};
	USB2Dynamixel::callback callback{nullptr};
};

class USB2Dynamixel_pimpl
{
public:
	/// file descriptor
	int fd{-1};
	uint m_maxJobCount;
	std::queue<DynamixelTransaction> m_transactions;

	bool m_running{true};

	std::mutex m_mutex;
	std::condition_variable m_cv;

	std::thread m_thread;

	void run()
	{
		while (m_running)
		{
			DynamixelTransaction currentTransaction;
			{
				std::unique_lock<std::mutex> lock(m_mutex);

				if (m_transactions.size() == 0)
				{
					m_cv.wait(lock);
				}
				if (m_transactions.size() > 0)
				{
					currentTransaction = m_transactions.front();
					m_transactions.pop();
				} else
				{
					continue;
				}
			}
			bool success = false;
			const uint8_t* rxPayload;
			uint8_t error = 0;
			uint8_t rxPayloadLen = 0;

			if (writePacket(currentTransaction.writeBuffer))
			{
				if (currentTransaction.needsResponse) {
					if (readPacket(currentTransaction.readBuffer, currentTransaction.rxLength, currentTransaction.rxTimeout))
					{
						dynamixel::parameter const& i_packet = currentTransaction.readBuffer;
						success = i_packet.size() >= 6;
						success &= 0xff == i_packet[0];
						success &= 0xff == i_packet[1];
						success &= currentTransaction.motorID == i_packet[2];
						success &= i_packet.size() - 4 == i_packet[3];

						uint8_t checkSum = 0;
						for (uint i(2); i < i_packet.size(); ++i) {
							checkSum += i_packet[i];
						}

						success &= 0xff == checkSum;

						if (success) {
							rxPayload = i_packet.data() + 5;
							rxPayloadLen = i_packet.size() - 6;
						} else {
//							std::cout << "received malformed packet" << std::endl;
//							for (auto byte : currentTransaction.readBuffer) {
//								printf("0x%X ", byte);
//							}
//							printf("\n");
						}
					}
				}
			}

			if (currentTransaction.callback)
			{
				currentTransaction.callback(currentTransaction.motorID, success, error, rxPayload, rxPayloadLen);
			}

			if (currentTransaction.m_waitMutex)
			{
				currentTransaction.m_waitMutex->unlock();
			}
		}
	}

	bool addJob(DynamixelTransaction& transaction, std::mutex* mutex = nullptr)
	{
		std::unique_lock<std::mutex> lock(m_mutex);
		if (m_transactions.size() < m_maxJobCount) {
			transaction.m_waitMutex = mutex;
			if (transaction.m_waitMutex)
			{
				transaction.m_waitMutex->lock();
			}

			m_transactions.push(transaction);
			m_cv.notify_one();
			return true;
		}
		return false;
	}

	bool writePacket(dynamixel::parameter const& packet)
	{
		const uint8_t *data = packet.data();
		uint bytesWritten = 0;
		const size_t count = packet.size();
		do {
			ssize_t w = ::write(fd, data + bytesWritten, count - bytesWritten);

			if (w < 0) {
				return false;
			} else {
				bytesWritten += w;
			}
		} while (bytesWritten < count);

		return true;
	}

	bool readPacket(dynamixel::parameter &i_packet, uint8_t incommingLength, Second timeout)
	{
		Second startTime = utils::getCurrentTime();

		i_packet.reserve(incommingLength);
		for (uint i(0); i < incommingLength; ++i) {
			i_packet.push_back(0);
		}
		uint8_t *data = i_packet.data();
		size_t bytesRead = 0;

		do {
			do { // read the header
				ssize_t r = ::read(fd, data + bytesRead, 1);
				if (r > 0 && 0xff == data[bytesRead]) {
					bytesRead += r;
				}
			} while (bytesRead < 2 && (0 * seconds == timeout || utils::getCurrentTime() - startTime < timeout));
			do { // check if the following byte is a "non-header"
				ssize_t r = ::read(fd, data + bytesRead, 1);
				if (r > 0) {
					if (0xff != data[bytesRead]) {
						bytesRead += r;
					} else {
						bytesRead = 1;
						break;
					}
				}
			} while (bytesRead < 3 && (0 * seconds == timeout || utils::getCurrentTime() - startTime < timeout));
		} while (bytesRead < 3 && (0 * seconds == timeout || utils::getCurrentTime() - startTime < timeout));

		do {
			ssize_t r = ::read(fd, data + bytesRead, incommingLength - bytesRead);
			if (r > 0) {
				bytesRead += r;
			}
		} while (bytesRead < incommingLength && (0 * seconds == timeout || utils::getCurrentTime() - startTime < timeout));


		if (bytesRead != incommingLength) {
			uint8_t dummy;
			ssize_t r = 1;
			while (r > 0) {
				r = ::read(fd, &dummy, sizeof(dummy)); // read flush
			}
		}
		return bytesRead == incommingLength;
	}
};






USB2Dynamixel::USB2Dynamixel(int baudrate, std::vector<std::string> deviceNames, uint maxJobCount)
	: m_pimpl(new USB2Dynamixel_pimpl)
{
	m_pimpl->m_maxJobCount = maxJobCount;

	for (auto const& device : deviceNames) {
		m_pimpl->fd = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
		if (m_pimpl->fd != -1) break;
	}

	if (m_pimpl->fd == -1) {
		std::cerr << "Could not open any serial port: ";
		for (auto const& device : deviceNames) {
			std::cerr<<device<<" ";
		}
		std::cerr<<std::endl;
		return;
	}


	// when reading, return immediately
	fcntl(m_pimpl->fd, F_SETFL, FNDELAY);
	struct serial_struct serial;
	ioctl(m_pimpl->fd, TIOCGSERIAL, &serial);

	serial.flags |= ASYNC_LOW_LATENCY;  /* enable low latency  */
	ioctl(m_pimpl->fd, TIOCSSERIAL, &serial);

	struct termios2 options;
	memset(&options, 0, sizeof(options));

	ioctl(m_pimpl->fd, TCGETS2, &options);

	// local line that supports reading
	options.c_cflag |= (CLOCAL | CREAD);

	// set 8N1
	options.c_cflag &= ~PARENB;
	options.c_cflag &= ~CSTOPB;
	options.c_cflag &= ~CSIZE;
	options.c_cflag |= CS8;

	// set baudrate
	options.c_ospeed = baudrate;
	options.c_ispeed = baudrate;
	options.c_cflag  &= ~CBAUD;
	options.c_cflag  |= BOTHER;

	// disable hardware flow control
	options.c_cflag &= ~CRTSCTS;

	// use raw mode (see "man cfmakeraw")
	options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
	options.c_oflag &= ~OPOST;
	options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	options.c_cflag &= ~(CSIZE | PARENB);
	options.c_cflag |= CS8;

	ioctl(m_pimpl->fd, TCSETS2, &options);

	m_pimpl->m_running = true;
	m_pimpl->m_thread = std::thread([&](){m_pimpl->run();});
}

USB2Dynamixel::USB2Dynamixel(std::vector<std::string> deviceNames, uint maxJobCount)
	: m_pimpl(new USB2Dynamixel_pimpl)
{
	m_pimpl->m_maxJobCount = maxJobCount;

	for (auto const& device : deviceNames) {
		m_pimpl->fd = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
		if (m_pimpl->fd != -1) break;
	}

	if (m_pimpl->fd == -1) {
		std::cerr << "Could not open any serial port: ";
		for (auto const& device : deviceNames) {
			std::cerr<<device<<" ";
		}
		std::cerr<<std::endl;
		return;
	}

	// when reading, return immediately
	fcntl(m_pimpl->fd, F_SETFL, FNDELAY);
	struct serial_struct serial;
	ioctl(m_pimpl->fd, TIOCGSERIAL, &serial);

	serial.flags |= ASYNC_LOW_LATENCY;  /* enable low latency  */
	ioctl(m_pimpl->fd, TIOCSSERIAL, &serial);

	struct termios2 options;
	memset(&options, 0, sizeof(options));

	ioctl(m_pimpl->fd, TCGETS2, &options);

	// local line that supports reading
	options.c_cflag |= (CLOCAL | CREAD);

	// set 8N1
	options.c_cflag &= ~PARENB;
	options.c_cflag &= ~CSTOPB;
	options.c_cflag &= ~CSIZE;
	options.c_cflag |= CS8;

	// set baudrate
	options.c_ospeed = 1000000; //57600 default -- modified by sri
	options.c_ispeed = 1000000; //57600 default -- modified by sri

	options.c_cflag  &= ~CBAUD;
	options.c_cflag  |= BOTHER;

	// disable hardware flow control
	options.c_cflag &= ~CRTSCTS;

	// use raw mode (see "man cfmakeraw")
	options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
	options.c_oflag &= ~OPOST;
	options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	options.c_cflag &= ~(CSIZE | PARENB);
	options.c_cflag |= CS8;

	ioctl(m_pimpl->fd, TCSETS2, &options);

	m_pimpl->m_running = true;
	m_pimpl->m_thread = std::thread([&](){m_pimpl->run();});
}

USB2Dynamixel::~USB2Dynamixel() {
	if (m_pimpl->fd != -1) {
		m_pimpl->m_running = false;
		m_pimpl->m_cv.notify_one();
		m_pimpl->m_thread.join();
		::close(m_pimpl->fd);
	}
	delete m_pimpl;
}

void USB2Dynamixel::setBaudrate(uint newBaudrate)
{
	if (m_pimpl->m_running) {
		m_pimpl->m_running = false;
		m_pimpl->m_cv.notify_one();
		m_pimpl->m_thread.join();
	}
	struct termios2 options;
	memset(&options, 0, sizeof(options));
	ioctl(m_pimpl->fd, TCGETS2, &options);

	options.c_cflag &= ~CBAUD;
	options.c_cflag |= BOTHER;

	// set baudrate
	options.c_ospeed = newBaudrate;
	options.c_ispeed = newBaudrate;

	ioctl(m_pimpl->fd, TCSETS2, &options);

	m_pimpl->m_running = true;
	m_pimpl->m_thread = std::thread([&](){m_pimpl->run();});
}

bool USB2Dynamixel::ping(dynamixel::motorID motor, Second timeout, callback cb, std::mutex* mutex)
{
	DynamixelTransaction transaction;
	transaction.callback = cb;
	transaction.needsResponse = true;
	transaction.rxLength = 6;
	transaction.motorID = motor;
	transaction.rxTimeout = timeout;

	transaction.writeBuffer.reserve(6);
	transaction.writeBuffer.push_back(0xff);
	transaction.writeBuffer.push_back(0xff);
	transaction.writeBuffer.push_back(motor);
	transaction.writeBuffer.push_back(2);
	transaction.writeBuffer.push_back(uint8_t(dynamixel::Instruction::PING));
	transaction.writeBuffer.push_back(calculateChecksum(transaction.writeBuffer));

	return m_pimpl->addJob(transaction, mutex);
}

bool USB2Dynamixel::read(dynamixel::motorID motor, dynamixel::Register baseRegister, uint8_t length, Second timeout, callback cb, std::mutex* mutex)
{
	DynamixelTransaction transaction;
	transaction.callback = cb;
	transaction.needsResponse = true;
	transaction.rxLength = 6 + length;
	transaction.motorID = motor;
	transaction.rxTimeout = timeout;

	transaction.writeBuffer.reserve(8);
	transaction.writeBuffer.push_back(0xff);
	transaction.writeBuffer.push_back(0xff);
	transaction.writeBuffer.push_back(motor);
	transaction.writeBuffer.push_back(4);
	transaction.writeBuffer.push_back(uint8_t(dynamixel::Instruction::READ));
	transaction.writeBuffer.push_back(uint8_t(baseRegister));
	transaction.writeBuffer.push_back(length);
	transaction.writeBuffer.push_back(calculateChecksum(transaction.writeBuffer));

	return m_pimpl->addJob(transaction, mutex);
}

bool USB2Dynamixel::write(dynamixel::motorID motor, dynamixel::Register baseRegister, dynamixel::parameter const& writeBuffer, std::mutex* mutex)
{
	if (writeBuffer.size() > 0) {
		DynamixelTransaction transaction;
		transaction.motorID = motor;

		transaction.writeBuffer.reserve(7 + writeBuffer.size());
		transaction.writeBuffer.push_back(0xff);
		transaction.writeBuffer.push_back(0xff);
		transaction.writeBuffer.push_back(motor);
		transaction.writeBuffer.push_back(3 + writeBuffer.size());
		transaction.writeBuffer.push_back(uint8_t(dynamixel::Instruction::WRITE));
		transaction.writeBuffer.push_back(uint8_t(baseRegister));

		for (uint8_t param : writeBuffer) {
			transaction.writeBuffer.push_back(param);
		}

		transaction.writeBuffer.push_back(calculateChecksum(transaction.writeBuffer));

		return m_pimpl->addJob(transaction, mutex);
	}
	return false;
}

bool USB2Dynamixel::reset(dynamixel::motorID motor, std::mutex* mutex)
{
	DynamixelTransaction transaction;
	transaction.motorID = motor;

	transaction.writeBuffer.reserve(6);
	transaction.writeBuffer.push_back(0xff);
	transaction.writeBuffer.push_back(0xff);
	transaction.writeBuffer.push_back(motor);
	transaction.writeBuffer.push_back(2);
	transaction.writeBuffer.push_back(uint8_t(dynamixel::Instruction::RESET));
	transaction.writeBuffer.push_back(calculateChecksum(transaction.writeBuffer));

	return m_pimpl->addJob(transaction, mutex);
}

bool USB2Dynamixel::sync_write(std::map<dynamixel::motorID, dynamixel::parameter> const& motorParams, dynamixel::Register baseRegister, std::mutex* mutex)
{
	if (motorParams.size() > 0)
	{
		const uint8_t len = motorParams.begin()->second.size();
		bool okay = true;
		for (std::pair<dynamixel::motorID, dynamixel::parameter> const& param : motorParams)
		{
			okay &= param.second.size() == len;
		}

		if (len > 0 && okay)
		{
			const uint8_t packetLength = 6 + (len + 1) * motorParams.size();
			DynamixelTransaction transaction;
			transaction.motorID = dynamixel::broadcastID;

			transaction.writeBuffer.reserve(packetLength);
			transaction.writeBuffer.push_back(0xff);
			transaction.writeBuffer.push_back(0xff);
			transaction.writeBuffer.push_back(dynamixel::broadcastID);
			transaction.writeBuffer.push_back(4 + (len + 1) * motorParams.size());
			transaction.writeBuffer.push_back(uint8_t(dynamixel::Instruction::SYNC_WRITE));
			transaction.writeBuffer.push_back(uint8_t(baseRegister));
			transaction.writeBuffer.push_back(len);

			for (std::pair<dynamixel::motorID, dynamixel::parameter> const& param : motorParams)
			{
				transaction.writeBuffer.push_back(param.first);
				for (uint8_t element : param.second) {
					transaction.writeBuffer.push_back(element);
				}
			}

			transaction.writeBuffer.push_back(calculateChecksum(transaction.writeBuffer));

			return m_pimpl->addJob(transaction, mutex);
		}
	}
	return false;
}


uint8_t USB2Dynamixel::calculateChecksum(dynamixel::parameter const& packet) const
{
	uint8_t checkSum = 0;
	for (uint i(2); i < packet.size(); ++i) {
		checkSum += packet[i];
	}
	return 0xff - checkSum;
}
