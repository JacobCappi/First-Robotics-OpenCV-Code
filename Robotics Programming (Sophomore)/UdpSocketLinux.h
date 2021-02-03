#ifndef UdpSocket_h
#define UdpSocket_h

#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <time.h>
#include <string.h>
#include <stdio.h>

#include <unistd.h>

class UdpSocket
{
public:

    UdpSocket();
    void init( int portNumber, const char *destAddr );
    ~UdpSocket();

    int putDatagram( const char *data, int size );
    int putDatagram( const char *data, int size, unsigned long addr,
                     unsigned short port );

    int putDatagram( const char *data, int size, unsigned short port );
    int getDatagram( char *data, int maxSize );

protected:
    struct sockaddr_in outAddr;
    struct ip_mreq mreq;

    int fd;
    int port;

    bool multicast;
    char mcastAddr[32];
};

#endif

