#include <stdio.h>      
#include <stdlib.h>  
#include <string.h>     
#include <unistd.h>
#include <fcntl.h>      
#include <signal.h>
#include <errno.h>     
#include <sys/file.h>   
#include <sys/socket.h>
#include <arpa/inet.h>

#include "UDP.h" 
#include "Error.h"




SendChannelTCP::SendChannelTCP(char *serverIP, unsigned short serverPort){
	
	listenPort = serverPort;
	
	//create socket
	listenSocket = socket(AF_INET, SOCK_STREAM, 0);
	if (listenSocket < 0) 
		 fatalError("Could not create a TCP socket.");
		 
	// Bind listen socket to listen port.  First set various fields in
	// the serverAddress structure, then call bind().
	// htonl() and htons() convert long integers and short integers
	// (respectively) from host byte order (on x86 this is Least
	// Significant Byte first) to network byte order (Most Significant
	// Byte first).
 	serverAddress.sin_family = AF_INET;
 	serverAddress.sin_addr.s_addr = htonl(INADDR_ANY);
	serverAddress.sin_port = htons(listenPort);
	
	if (bind(listenSocket,
           (struct sockaddr *) &serverAddress,
           sizeof(serverAddress)) < 0) {
           	
		fatalError("Could not bind the TCP socket.");
  	}
  	
  	// Wait for connections from clients.
  	// This is a non-blocking call; i.e., it registers this program with
  	// the system as expecting connections on this socket, and then
  	// this thread of execution continues on.
  	listen(listenSocket, 5);
  	
  	while (1) {
	    printf("Waiting for TCP! connection on port %i...\n",listenPort);
	
	    // Accept a connection with a client that is requesting one.  The
	    // accept() call is a blocking call; i.e., this thread of
	    // execution stops until a connection comes in.
	    // connectSocket is a new socket that the system provides,
	    // separate from listenSocket.  We *could* accept more
	    // connections on listenSocket, before connectSocket is closed,
	    // but this program doesn't do that.
	    clientAddressLength = sizeof(clientAddress);
	    connectSocket = accept(listenSocket,
	                           (struct sockaddr *) &clientAddress,
	                           &clientAddressLength);
	    printf("%i ", connectSocket);
	    if (connectSocket < 0) {
	      fatalError("cannot accept connection ");
	      
	    }
	    else {
		    // Show the IP address of the client.
		    // inet_ntoa() converts an IP address from binary form to the
		    // standard "numbers and dots" notation.
		    printf("connected!\n");
		
		    // Show the client's port number.
		    // ntohs() converts a short int from network byte order (which is
		    // Most Significant Byte first) to host byte order (which on x86,
		    // for example, is Least Significant Byte first).
		    //cout << ":" << ntohs(clientAddress.sin_port) << "\n";
	    }
	    break;
  	}
  	
}

SendChannelTCP::~SendChannelTCP()
{
  printf("closing socket");
  close(listenSocket);
}    

bool SendChannelTCP::clientRequest(){
	/*memset(line, 0x0, 20);
	if (recv(connectSocket, line, 20, 0) > 0){	
		//printf("%c\n", line[0]);
		if (line[0] == '1'){
			return true;
		}
		else return false;
	}*/
	
	
	return true;
}

void SendChannelTCP::sendMessage(void* msgBuffer, int msgLen){
	
	int i = send(connectSocket, msgBuffer,msgLen, 0);
	if ( i < 0)
        printf("Error: cannot send modified data\n");
	//printf("send output = %i\n", i);
	
	
}

//############################################################
//##                                                        ##
//##                    SENDING  ONLY                       ##
//##                                                        ##
//############################################################

SendChannel::SendChannel(char *serverIP, unsigned short serverPort)
{
  //checkPacketsSizes();  // sanity check

  if ((sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0)
    fatalError("Could not create a UDP socket.");
  
  // Configure the  server address structure
  memset(&(serverAddr), 0, sizeof(serverAddr));       // Clear the structure
  serverAddr.sin_family = AF_INET;                    // Use Internet addr family
  serverAddr.sin_addr.s_addr = inet_addr(serverIP);   // Set Server IP
  serverAddr.sin_port  = htons(serverPort);           // Ser Server port

}

SendChannel::~SendChannel()
{
  close(sock);
}                                                                                                  
                                                                                            
void SendChannel::sendMessage(void* msgBuffer, int msgLen)
{
  int res;
  if ((res=sendto(sock, msgBuffer, msgLen, 0, (struct sockaddr *)
		  &(serverAddr), sizeof(serverAddr))) != msgLen)
    fatalError("sendUDPmessage: number of bytes sent is different [msgLen=%d, sent=%d].", msgLen, res);
}



//############################################################
//##                                                        ##
//##                    RECEIVING ONLY                      ##
//##                                                        ##
//############################################################

int numListenChannels=0;
RecvChannel* listenChannel[5];


void pollNetworkChannels()
  //void SIGIOHandler(int signalType)
{
    fd_set rfds;
    struct timeval tv;
    int i, retval, processed;

    FD_ZERO(&rfds);
    for(i=0; i< numListenChannels; i++)
      FD_SET(listenChannel[i]->getSock(), &rfds);

    // Don't wait 
    tv.tv_sec = 0;
    tv.tv_usec = 0;
    
    retval = select(listenChannel[numListenChannels-1]->getSock() + 1, &rfds, NULL, NULL, &tv);
    processed=0;

    if (retval) 
      {
	for(i=0; i< numListenChannels; i++) {
	  if(FD_ISSET(listenChannel[i]->getSock(), &rfds)) {
	    //printf("Data is available now on sock[%d].\n", i);
	    listenChannel[i]-> processPacket();
	    processed=1;
	  }
	}
      
	if(!processed) 
	  fatalError("Data is available somewhere ELSE ?! (should not happen)\n");
      }
}


void registerPacketCallback(RecvChannel* channel) //void(callbackFunction)(void*) )
{
  // TODO: The channels must be sorted in increasing number of their sockets

  listenChannel[numListenChannels] = channel;
  numListenChannels++;

  if( numListenChannels>=2)
    printf("Warning: registerPacketCallback (must sort the sockID in increasing order [%d %d]!)\n",listenChannel[0]->getSock(), listenChannel[1]->getSock());
  
}

RecvChannel::RecvChannel(unsigned short port, CallbackAble* cba) //void(callbackFunction)(void*) ) 
{
  struct sigaction handler;      

  subscriber=cba;

  // Create an UDP socket 
  if ((sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0)
    fatalError("openRecvChannel(): failed to open socket.");
  
  // Fill in the server address data structure
  memset(&serverAddr, 0, sizeof(serverAddr));  // Fill with 0's
  serverAddr.sin_family = AF_INET;                      // Internet family 
  serverAddr.sin_addr.s_addr = htonl(INADDR_ANY);       // Any incoming interface 
  serverAddr.sin_port = htons(port);                    // Server Port 
  
  // Bind to local address
  if (bind(sock, (struct sockaddr *) &serverAddr, sizeof(serverAddr)) < 0)
    fatalError(" openRecvChannel():  failed to bind socket.");

#if 0  
  // Set signal handler for SIGIO 
  handler.sa_handler =  SIGIOHandler;

  // Create mask that mask all signals 
  if (sigfillset(&handler.sa_mask) < 0)
    fatalError("sigfillset() failed");

  // No flags
  handler.sa_flags = 0;
                                                                                                                
  if (sigaction(SIGIO, &handler, 0) < 0)
    fatalError("sigaction() failed for SIGIO");


       
  // Must own the socket to receive the SIGIO message
  if (fcntl(sock, F_SETOWN, getpid()) < 0)
    fatalError("Unable to set process owner to us");
#endif                                                                                                                  
  // Setup nonblocking I/O and SIGIO delivery 
  if (fcntl(sock, F_SETFL, O_NONBLOCK | FASYNC) < 0)
    fatalError("Unable to put client sock into non-blocking/async mode");

  
  registerPacketCallback(this);
}


void RecvChannel::processPacket()
{
  unsigned int clientLen;            // Address length 
  int  packetSize;                   // Size of datagram 
  char packetBuffer[MAX_UDP_SIZE];   // Datagram buffer 
  
  do  
    {
      clientLen = sizeof(clientAddr);

      packetSize = recvfrom(sock, packetBuffer, MAX_UDP_SIZE, 0,
			    (struct sockaddr *) &clientAddr, &clientLen);
			    
	  printf("size at processPacket() = %d\n",packetSize);
			    
      if (packetSize < 0)
        {
	  // Only acceptable error: recvfrom() would have blocked 
	  if (errno != EWOULDBLOCK)  
	    fatalError("in processPacket():: recvfrom() failed");
        }
      else
        {
	  // Call the appropriate processing function for this packet
          // printf(" Handling client %s\n", inet_ntoa(clientAddr.sin_addr));
	  //	  (*(channel->packetProcessingFunction))((void*)packetBuffer);
	  	//printf("received packet size = %i\n",packetSize);
	  	
	  	subscriber->callback((void*)packetBuffer);
	  	
	}
    }  while (packetSize >= 0);
  
  // No more data available
}

RecvChannel::~RecvChannel()
{
  close(sock);
}
