#include "SkeletonListener.h"
#include <iostream>


SkeletonListener::SkeletonListener(unsigned short listenPort){
	rChannel = new RecvChannel(listenPort, this);
	count = 0;
	hasNewPacket = false;
}

SkeletonListener::~SkeletonListener(){
	delete rChannel;	
}

void SkeletonListener::listen(){
	
	hasNewPacket = false;
	pollNetworkChannels();	
}

bool SkeletonListener::receivedNewPacket(){
	return hasNewPacket;
}

void SkeletonListener::callback(void *data){
	
	hasNewPacket = true;
	
	latest_packet = (struct KinectPacket*)	data;
	count++;
	printf("heard packet #%i\n",count);
	printf("\t timestamp:\t%f\n\n",latest_packet->timestamp);
	
	/*unsigned char * desmond = (unsigned char *) & latest_packet->timestamp;
    int i;

    for (i = 0; i < sizeof (double); i++) {
         printf ("%02X ", desmond[i]);
    }
    printf ("\n");*/
	
	
	
	
	//printf("\t size of data:%d\n",sizeof(data));
	//printf("\t size of packet:%i\n",sizeof(latest_packet));
	
	//std::cout << latest_packet->timestamp << std::endl;
	
	printf("\t num skels:\t%i\n",latest_packet->skeletonPacket.n_skels);
	for (int i = 0; i < latest_packet->skeletonPacket.n_skels; i++){
		printf("\t\tid %d = %d\n",i,latest_packet->skeletonPacket.ids[i]);
		
		for (int joint = 0; joint < 25; joint++){
			printf("\t\t%f,%f,%f,%i,%i\n",latest_packet->skeletonPacket.bodies[i].joints[joint].x,
			latest_packet->skeletonPacket.bodies[i].joints[joint].y,
			latest_packet->skeletonPacket.bodies[i].joints[joint].z,
			latest_packet->skeletonPacket.bodies[i].joints[joint].u,
			latest_packet->skeletonPacket.bodies[i].joints[joint].v);
		}
	}
	
	printf("\taudio angle:\t%f\n",latest_packet->audioPacket.angle);
	printf("\taudio confidence:\t%f\n",latest_packet->audioPacket.confidence);

}

struct KinectPacket* SkeletonListener::getPacket(){
	return latest_packet;
}
