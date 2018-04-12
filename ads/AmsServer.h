#ifndef _AMSSERVER_H_
#define _AMSSERVER_H_

#include "AmsHeader.h"
#include "Sockets.h"

#include <atomic>
#include <chrono>

using Timepoint = std::chrono::steady_clock::time_point;
//#define WAITING_FOR_RESPONSE ((uint32_t)0xFFFFFFFF)

struct AmsServer {
	AmsServer(IpV4 __destIp);
	~AmsServer();

private:
	//std::atomic<size_t> refCount;
	//std::atomic<uint32_t> invokeId;

	template<class T> void ReceiveFrame(size_t length, uint32_t aoeError) const;
	//void ReceiveJunk(size_t bytesToRead) const;
	void Receive(void* buffer, size_t bytesToRead, timeval* timeout = nullptr) const;
	void Receive(void* buffer, size_t bytesToRead, const Timepoint& deadline) const;
	template<class T> void Receive(T& buffer) const { Receive(&buffer, sizeof(T)); }
	void Write(const AoEHeader& aoeHeader);

public:
	void Recv();
	void TryRecv(TcpSocket* sock);

	TcpSocket socket;
	const IpV4 destIp;
};

#endif /* #ifndef _AMSSERVER_H_ */
