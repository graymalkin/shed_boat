#ifndef __nav_helper_
#define __nav_helper_

#ifndef M_PI
	#define M_PI 3.14159265358979323846
#endif

#define TWO_PI (M_PI+M_PI)
#define EARTH_RADIUS_METERS 6371000

#define SUCCESS 0
#define FAILURE -1

typedef struct nav_list_t {
	double latitude;
	double longitude;
	nav_list_t * next;
} nav_list_t;

double degToRad(double deg);
double heading_delta(double a, double b);
double equirectangular(double lat1, double lon1, double lat2, double lon2);
double startHeading(double lat1, double lon1, double lat2, double lon2);
void add_waypoint(nav_list_t * new_nav);
nav_list_t * get_current_nav();
int go_next_nav();
double distance_to_current_nav(double latitude, double longitude);

#endif // __nav_helper_
