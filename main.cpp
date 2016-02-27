#include "mbed.h"

#include "gps.h"
#include "nmea.h"

Serial usbSerial(USBTX, USBRX);
DigitalOut heartbeat(LED_GREEN);
// UBX_GPS gps(D0, D1);

void beat();

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

	while(1)
    {
		// gps.poll(MON, ID_HW, &dest);
		// gps.debug_print(usbSerial, &dest);

		while(!NMEA::isDataReady()){
			//usbSerial.printf(".");
		}

		wait_ms(5000);
		usbSerial.printf("Satellites: %d\r\n", NMEA::getSatellites());
		usbSerial.printf("Longitude:  %0.5f\r\n", NMEA::getLongitude());
		usbSerial.printf("Latitude:   %0.5f\r\n", NMEA::getLongitude());

    }
}

void beat()
{
    heartbeat = !heartbeat;
}
