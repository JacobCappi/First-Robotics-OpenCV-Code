#include <stdio.h>
#include <fcntl.h>
#include "UdpSocketLinux.h"

//static WSAData wsaData;
static bool initialized = 0;

UdpSocket::UdpSocket()
{
    multicast = false;
}

void UdpSocket::init( int portNumber, const char *destAddr )
{
    port      = portNumber;
    strcpy( mcastAddr, destAddr );

/*
    if (initialized == false ) // && WSAStartup(2, &wsaData))
    {
        printf("Startup failed: initialized, winsock error\n" ); // %d", WSAGetLastError());
        exit(1);
    }
*/
    initialized = true;

    if ((fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0)
    {
        printf("socket failed: DGRAM, winsock error" ); //  %d", WSAGetLastError());
        exit(1);
    }

    int sockopt = 1;
    if (setsockopt(fd, SOL_SOCKET, SO_BROADCAST, (char *)&sockopt,
         sizeof(sockopt)) < 0)
    {
        printf("setsockopt failed, winsock error");// %d", WSAGetLastError());
        exit(1);
    }

    sockopt = 1;
    if (setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, (char *)&sockopt,
         sizeof(sockopt)) < 0)
    {
        printf("setsockopt failed, winsock error");// %d", WSAGetLastError());
        exit(1);
    }

    // Get the destination address

    int n1, n2, n3 ,n4;
    //if (sscanf(destAddr, "%d.%d.%d.%d", &n1, &n2, &n3, &n4) != 4)
    if (sscanf("10.29.73.255", "%d.%d.%d.%d", &n1, &n2, &n3, &n4) != 4)
    {
        printf("Failed to convert IP address: %s", destAddr);
        exit(1);
    }
//printf("ns=%d %d %d %d\n", n1, n2, n3, n4 );
    u_long bcastAddr = (n1<<24) | (n2<<16) | (n3<<8) | n4;
    outAddr.sin_family = AF_INET;
    outAddr.sin_port = htons(portNumber);
    outAddr.sin_addr.s_addr = htonl(bcastAddr);

    struct sockaddr_in in_name;
    in_name.sin_family = AF_INET;
    in_name.sin_addr.s_addr = INADDR_ANY;
    in_name.sin_port = htons(portNumber);

    if (bind(fd, (struct sockaddr *)&in_name, sizeof(in_name)) < 0)
    {
        printf("bind failed, winsock error");// %d", WSAGetLastError());
        exit(1);
    }

/* Linux */
    int flags;
    fcntl( fd, F_SETFL, O_NONBLOCK );



// WINDOWS
    //u_long nonBlock = 1;
    //if (ioctlsocket(fd, FIONBIO, &nonBlock) < 0)
    //{
        //printf("ioctlsocket failed, winsock error");// %d", WSAGetLastError());
        //exit(1);
    //}

    if (n1 >= 224 && n1 != 255)
    {
        // This is a multicast address, we must subscribe

        mreq.imr_interface.s_addr = INADDR_ANY;
        mreq.imr_multiaddr.s_addr = htonl(bcastAddr);

        if (setsockopt(fd, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char*)&mreq,
            sizeof(mreq)) < 0)
        {
            printf("setsockopt failed, winsock error");// %d", WSAGetLastError());
            exit(1);
        }

        multicast = true;
    }
}

UdpSocket::~UdpSocket()
{
    if (multicast)
    {
        setsockopt(fd, IPPROTO_IP, IP_DROP_MEMBERSHIP, (char*)&mreq, sizeof(mreq));
    }

    //closesocket(fd);
}

int UdpSocket::putDatagram( const char *data, int size )
{
    int retval = sendto(fd, data, size, 0, (struct sockaddr *)
        &outAddr, sizeof(outAddr));

/*
    if (retval < 0)
    {
        int error = WSAGetLastError();
        if (error == WSAEWOULDBLOCK)
            return 0;
        else
        {
            printf("sendto failed, winsock error");// %d", WSAGetLastError());
            exit(1);
        }
    }
*/

    return retval;
}

int UdpSocket::putDatagram( const char *data, int size, unsigned long addr, unsigned short port )
{
    struct sockaddr_in altOutAddr;
    altOutAddr.sin_addr.s_addr = htonl(addr);
    altOutAddr.sin_port        = htons(port);
    altOutAddr.sin_family      = AF_INET;

    int retval = sendto(fd, data, size, 0, (struct sockaddr *)
        &altOutAddr, sizeof(outAddr));

/*
    if (retval < 0)
    {
        int error = WSAGetLastError();
        if (error == WSAEWOULDBLOCK)
            return 0;
        else
        {
            printf("sendto failed, winsock error %d", WSAGetLastError());
            exit(1);
        }
    }
*/

    return retval;
}

int UdpSocket::putDatagram( const char *data, int size, unsigned short port )
{
    struct sockaddr_in altOutAddr(outAddr);
    altOutAddr.sin_port = htons(port);

    int retval = sendto(fd, data, size, 0, (struct sockaddr *)
        &altOutAddr, sizeof(outAddr));

/*
    if (retval < 0)
    {
        int error = WSAGetLastError();
        if (error == WSAEWOULDBLOCK)
            return 0;
        else
        {
            printf("sendto failed, winsock error %d", WSAGetLastError());
            exit(1);
        }
    }
*/

    return retval;
}

int UdpSocket::getDatagram( char *data, int maxSize )
{
    static unsigned char bundleBuffer[1500];
    static int bbIndex = 0;
    static int bbSize  = 0;

    // Is there some bundled data waiting to be returned to the caller?

    if (bbIndex < bbSize)
    {
        // Get the length of the next PDU in the buffer and sanity check it

        unsigned short pduLen = (bundleBuffer[bbIndex+8] << 8) | (bundleBuffer[bbIndex+9]);
        if ((pduLen < 12) || ((bbIndex + pduLen) >= sizeof(bundleBuffer)))
        {
            // No bundled PDUs left, go to the net for some data

            bbSize  = 0;
            bbIndex = 0;
        }
        else
        {
            // Copy the bundled PDU to the callers buffer

            memcpy(data, &bundleBuffer[bbIndex], pduLen);
            bbIndex += pduLen;
            return pduLen;
        }
    }

    // No bundled PDUs awaiting, go to the UDP socket for more data

    int pktLen = recv(fd, data, maxSize, 0);
/*
    if (pktLen < 0)
    {
        int error = WSAGetLastError();
        if (error == WSAEWOULDBLOCK)
        {
            return 0;
        }
        else
        {
            printf("recv failed, winsock error %d", WSAGetLastError());
            exit(1);
        }
    }
*/

    // We have some data, is the pdu bundled?

    unsigned short pduLen = ((unsigned char)data[8] << 8) | ((unsigned char)data[9]);

    if (pduLen && pduLen+12 < pktLen)
    {
        // This is a bundled PDU, save the rest of it for subsequent calls

        bbIndex = 0;
        bbSize  = pktLen - pduLen;
        memcpy(bundleBuffer, &data[pduLen], bbSize);
        return pduLen;
    }

    return pktLen;
}
