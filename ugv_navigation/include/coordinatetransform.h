#ifndef __COORDINATETRANSFORM_H
#define __COORDINATETRANSFORM_H

#include <cmath>

#define C_EARTH (double) 6378137.0
#define C_PI (double) 3.141592653589793
#define DEG2RAD(DEG) ((DEG)*((C_PI)/(180.0)))
#define RAD2DEG(RAD) ((RAD)*((180.0)/(C_PI)))

namespace ct{

struct LocalPosition{ // NED frame
    double x; // N
    double y; // E
    double z; // D(-z)
};

struct GlobalPosition{
    double latitude;
    double longitude;
    double altitude;
};

struct Attitude{
    double roll;
    double pitch;
    double yaw;
};


void LocalToGlocal(
        LocalPosition&  localpos,
        GlobalPosition& globalpos0,
        GlobalPosition& globalpos1
        );

void GlobalToLocal(
        GlobalPosition& globalpos0,
        GlobalPosition& globalpos1,
        LocalPosition&  localpos
        );

void YawRotate(
        double          yaw,
        LocalPosition&  localpos0,
        LocalPosition&  localpos1
        );

};


#endif // __COORDINATETRANSFORM_H
