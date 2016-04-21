#include "NavHelper.h"

#include <stdlib.h>
#include <math.h>

#define MAX_NAV_POINTS 20
nav_list_t nav_list[MAX_NAV_POINTS];
unsigned int current_nav = 0;
unsigned int nav_tail = 0;

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
	if(raw_d > 180.0)
		return 360.0-raw_d;
	if(raw_d < -180.0)
		return raw_d+360.0;
	return raw_d;
}

void add_waypoint(double latitude, double longitude)
{
	nav_list[nav_tail].latitude = latitude;
	nav_list[nav_tail].longitude = longitude;
	nav_tail++;
}

nav_list_t * get_current_nav()
{
	return &nav_list[current_nav];
}

int go_next_nav()
{
	if(current_nav < (MAX_NAV_POINTS-1)) {
		current_nav++;
		return SUCCESS;
	}
	return FAILURE;
}

double distance_to_current_nav(double latitude, double longitude)
{
	return equirectangular(latitude, longitude,	nav_list[current_nav].latitude, nav_list[current_nav].longitude);
}

unsigned int get_nav_num()
{
	return current_nav;
}
