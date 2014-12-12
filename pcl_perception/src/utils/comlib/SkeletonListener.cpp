#include "SkeletonListener.h"
#include <iostream>


SkeletonListener::SkeletonListener(unsigned short listenPort){
	rChannel = new RecvChannel(listenPort, this);
	count = 0;
}

SkeletonListener::~SkeletonListener(){
	delete rChannel;	
}

void SkeletonListener::listen(){
	
	pollNetworkChannels();	
}

void SkeletonListener::callback(void *data){
	
	
	
	
	
	latest_packet = (struct Skeletons*)	data;
	count++;
	printf("heard packet #%i\n",count);
	printf("\t timestamp:\t%f\n\n",latest_packet->timestamp);
	
	unsigned char * desmond = (unsigned char *) & latest_packet->timestamp;
    int i;

    for (i = 0; i < sizeof (double); i++) {
         printf ("%02X ", desmond[i]);
    }
    printf ("\n");
	
	
	
	
	//printf("\t size of data:%d\n",sizeof(data));
	//printf("\t size of packet:%i\n",sizeof(latest_packet));
	
	//std::cout << latest_packet->timestamp << std::endl;
	
	printf("\t num skels:\t%i\n",latest_packet->n_skels);
	for (int i = 0; i < latest_packet->n_skels; i++){
		printf("\t\tid %d = %d\n",i,latest_packet->ids[i]);
		
		for (int joint = 0; joint < 25; joint++){
			printf("\t\t%f,%f,%f,%i,%i\n",latest_packet->bodies[i].joints[joint].x,
			latest_packet->bodies[i].joints[joint].y,
			latest_packet->bodies[i].joints[joint].z,
			latest_packet->bodies[i].joints[joint].u,
			latest_packet->bodies[i].joints[joint].v);
		}
	}
	
	
}

struct Skeletons* SkeletonListener::getPacket(){
	return latest_packet;
}
