#include "NavHelper.h"

#include <math.h>

double degToRad(double deg) {
	return deg*M_PI/180.0;
}

/* Calculates the distance between two points */
double equirectangular(double lat1, double lon1, double lat2, double lon2) {
	double x = (lon2-lon1)*cos((lat1+lat2)*0.5);
	double y = (lat2-lat1);

	return sqrt(x*x + y*y) * EARTH_RADIUS_METERS;
}

/* Calculates the bearing to travel to a point given a location */
double startHeading(double lat1, double lon1, double lat2, double lon2) {
	double y = sin(lon2-lon1)*cos(lat2);
	double x = cos(lat1)*sin(lat2) - sin(lat1)*cos(lat2)*cos(lon2-lon1);
	double b = atan2(y,x);
	return b<0.0 ? (b+TWO_PI) : b;
}


double heading_delta(double a, double b)
{
	double raw_d = a - b;
	if(raw_d>180.0) return 360.0-raw_d;
	if(raw_d<-180.0) return raw_d+360.0;
	return raw_d;
}
