#include "mbed.h"
#include "nmea.h"
#include <math.h>

// Test for correctness when in a room that does produce it's own magnetic field...
// Magnetic Declination for Canterbury = WEST 0deg 16min
#define DECLINATION_ANGLE ((-16.0*M_PI)/(60.0*180.0))
#include "HMC5883L.h"

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

#define TWO_PI (M_PI+M_PI)
#define EARTH_RADIUS_METERS 6371000

Serial usbSerial(USBTX, USBRX);
DigitalOut heartbeat(LED_GREEN);
HMC5883L compass(I2C_SDA, I2C_SCL);

void beat();
double degToRad(double deg);
double heading_delta(double a, double b);
double equirectangular(double lat1, double lon1, double lat2, double lon2);
double startBearing(double lat1, double lon1, double lat2, double lon2);

int main() {
	usbSerial.baud(115200);

    Ticker heartbeat_tkr;
    heartbeat_tkr.attach_us(&beat, 250000);


	usbSerial.printf(	"     _              _   _                 _   \r\n"
						"    | |            | | | |               | |  \r\n"
						" ___| |__   ___  __| | | |__   ___   __ _| |_ \r\n"
						"/ __| '_ \\ / _ \\/ _` | | '_ \\ / _ \\ / _` | __|\r\n"
						"\\__ \\ | | |  __/ (_| | | |_) | (_) | (_| | |_ \r\n"
						"|___/_| |_|\\___|\\__,_| |_.__/ \\___/ \\__,_|\\__|\r\n"
						"                   ______                     \r\n"
						"                  |______|\r\n"
						"\r\n\n");

	NMEA::init();
	compass.init();
	double heading;

	while(NMEA::getSatellites() < 3);

	// Parkwood: 51.298997, 1.056683
	// Chestfield: 51.349215, 1.066184

	while(1)
    {
		wait_ms(500);

		usbSerial.printf("Time:       %02d:%02d:%02d\r\n", NMEA::getHour(), NMEA::getMinute(), NMEA::getSecond());
		usbSerial.printf("Satellites: %d\r\n", NMEA::getSatellites());
		usbSerial.printf("Latitude:   %0.5f\r\n", NMEA::getLatitude());
		usbSerial.printf("Longitude:  %0.5f\r\n", NMEA::getLongitude());
		usbSerial.printf("Altitude:   %0.2fm\r\n", NMEA::getAltitude());
		usbSerial.printf("Speed:      %0.2fkm/h\r\n", NMEA::getSpeed());
		usbSerial.printf("Bearing:    %0.2f degrees\r\n", NMEA::getBearing());
		heading = compass.smoothedHeading();
        usbSerial.printf("Compass Bearing: %03.0f\r\n", heading);
		usbSerial.printf("Heading Delta to South: %03.0f\r\n", heading_delta(heading, 180.0));
		usbSerial.printf("Distance to Parkwood %06.2fm\r\n",
			equirectangular(degToRad(NMEA::getLatitude()), degToRad(NMEA::getLongitude()),
							degToRad(51.298997), degToRad(1.056683))
		);



		usbSerial.printf("Bearing to Parkwood %03.0f\r\n",
			heading_delta(
				startBearing(degToRad(NMEA::getLatitude()), degToRad(NMEA::getLongitude()), degToRad(51.298997), degToRad(1.056683))*(180.0/M_PI),
				compass.smoothedHeading()
			)
		);

    }
}

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
double startBearing(double lat1, double lon1, double lat2, double lon2) {
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

void beat()
{
    heartbeat = !heartbeat;
}
