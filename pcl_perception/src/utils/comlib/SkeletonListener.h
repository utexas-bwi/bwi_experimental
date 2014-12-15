#ifndef _skeleton_listener_
#define _skeleton_listener_

#include "CallbackAble.h"
#include "UDP.h"
#include "UDP_skeleton_packets.h"

class SkeletonListener : public CallbackAble {
	public:
		SkeletonListener(unsigned short listenPort);
		virtual ~SkeletonListener();	
	
		virtual void  callback(void* data);
		
		void listen();
		
		struct KinectPacket* getPacket();
		bool receivedNewPacket();
	
	private:
		RecvChannel* rChannel;
		struct KinectPacket* latest_packet;
		int count;
		bool hasNewPacket;
};

#endif
