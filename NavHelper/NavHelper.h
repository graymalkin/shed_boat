#ifndef __nav_helper_
#define __nav_helper_

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

#define TWO_PI (M_PI+M_PI)
#define EARTH_RADIUS_METERS 6371000

double degToRad(double deg);
double heading_delta(double a, double b);
double equirectangular(double lat1, double lon1, double lat2, double lon2);
double startBearing(double lat1, double lon1, double lat2, double lon2);

#endif // __nav_helper_ 
