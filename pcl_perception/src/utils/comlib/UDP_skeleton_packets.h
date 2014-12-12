#ifndef _UDP_SKELETON_PACKETS_H__
#define _UDP_SKELETON_PACKETS_H__

#include <stdio.h>
#include <sys/time.h>
#include "Constants.h"

struct Joint5D
{
    float x;
    float y;
    float z;
    int u;
    int v;
};

struct Body
{
    Joint5D joints[25];
};

struct Skeletons
{
	double timestamp;
    Body bodies[6];
    int ids[6];
    int n_skels;
};



#endif
