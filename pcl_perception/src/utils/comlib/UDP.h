#ifndef __UDP_H__
#define __UDP_H__

#include <arpa/inet.h>  
#include <netdb.h>
#include <netinet/in.h>
#include <unistd.h>
#include <iostream>
#include "CallbackAble.h"
        

#define MAX_UDP_SIZE  5000

// Don't make this constant too big!
// The maximum Ethernet packet size is 1518: 14 bytes of header, 1500
// bytes of payload, and 4 bytes of CRC.
// This is further reduced by the IP and UDP/TCP headers.
// If you need to send larger packets use subChannels (see UDP_packets.h).


class SendChannelTCP
{
	public:
		SendChannelTCP(char *serverIP, unsigned short serverPort);
  		~SendChannelTCP();
  		
  		void sendMessage(void* msgBuffer, int msgLen);
  		bool clientRequest();
	
	private:
		
		int listenSocket, connectSocket, i;
  		unsigned short int listenPort;
  		socklen_t clientAddressLength;
  		struct sockaddr_in clientAddress, serverAddress;
	
		//buffer that holds message from client
		char line[10];
		
};

class SendChannel
{
 public:

  SendChannel(char *serverIP, unsigned short serverPort);
  ~SendChannel();

  void  sendMessage(void* msgBuffer, int msgLen);

 private:
  int sock;
  struct sockaddr_in serverAddr;
};


class RecvChannel
{
 public:
  RecvChannel(unsigned short port, CallbackAble* cba); 
  ~RecvChannel();

  int getSock() {return sock;}

  void processPacket();

 private:

  int sock;
  struct sockaddr_in serverAddr;
  struct sockaddr_in clientAddr;

  CallbackAble* subscriber;
};

void registerPacketCallback(RecvChannel* channel);
void pollNetworkChannels();

#endif
