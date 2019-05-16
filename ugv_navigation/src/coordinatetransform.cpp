#include "coordinatetransform.h"


namespace ct{

void LocalToGlocal(
        LocalPosition&  localpos,
        GlobalPosition& globalpos0,
        GlobalPosition& globalpos1
        )
{
    double lat0 = DEG2RAD(globalpos0.latitude);
    double lon0 = DEG2RAD(globalpos0.longitude);
    double alt0 = globalpos0.altitude;

    double dlat = localpos.x / C_EARTH;
    double dlon = localpos.y / (C_EARTH * cos(lat0));
    double dalt = localpos.z;

    double lat1 = lat0 + dlat;
    double lon1 = lon0 + dlon;
    double alt1 = alt0 + dalt;

    globalpos1.latitude = RAD2DEG(lat1);
    globalpos1.longitude = RAD2DEG(lon1);
    globalpos1.altitude = alt1;
}

void GlobalToLocal(
        GlobalPosition& globalpos0,
        GlobalPosition& globalpos1,
        LocalPosition&  localpos
        )
{
    double lat0 = DEG2RAD(globalpos0.latitude);
    double lon0 = DEG2RAD(globalpos0.longitude);
    double alt0 = globalpos0.altitude;

    double lat1 = DEG2RAD(globalpos1.latitude);
    double lon1 = DEG2RAD(globalpos1.longitude);
    double alt1 = globalpos1.altitude;

    double dlat = lat1 - lat0;
    double dlon = lon1 - lon0;
    double dalt = alt1 - alt0;

    localpos.x = dlat * C_EARTH;
    localpos.y = dlon * C_EARTH * cos(lat0);
    localpos.z = dalt;
}

void YawRotate(
        double          yaw,
        LocalPosition&  localpos0,
        LocalPosition&  localpos1
        )
{
    yaw = DEG2RAD(yaw);

    localpos1.x = localpos0.x * cos(yaw) - localpos0.y * sin(yaw);
    localpos1.y = localpos0.x * sin(yaw) + localpos0.y * cos(yaw);
    localpos1.z = localpos0.z;
}

};




