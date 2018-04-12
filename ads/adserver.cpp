
#include "AmsServer.h"
#include "Sockets.h"
//#include <iostream>

TcpSocket ListenSock(NULL, 27015);
TcpSocket ClientSock(NULL, 27015);

void adserver()
//int main()
{
    //static const AmsNetId remoteNetId { 192, 168, 0, 231, 1, 1 };
    //AdsSetLocalAddress({192, 168, 0, 1, 1, 1});
    static const char remoteIpV4[] = "127.0.0.1";

    AmsServer ams{IpV4(remoteIpV4)};
    ListenSock.Listen();
    while (1) {
        ClientSock.Accept(ListenSock.m_Socket);
        //ClientSock.read();
        ams.TryRecv(&ClientSock);
    		//printf("loop.................\n\n")
    	}
}
