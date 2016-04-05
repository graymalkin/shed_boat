#include "mbed.h"
#include <math.h>
#include <pb_encode.h>
#include "nmea.h"
#include "NavHelper.h"
#include "PID.h"
#include "SimonK_I2C_ESC.h"

#include "boat.pb.h"

#define ESC_ADDRESS_0 0x2B
#define ESC_ADDRESS_1 0x2A
#define RATE 0.1

// PID Limits
#define Rpm_Limit 8913.0
#define Throttle_Limit 32767.0
#define Speed_In_Knots_Limit 3.5

// Test for correctness when in a room that does produce it's own magnetic field...
// Magnetic Declination for Canterbury = WEST 0deg 16min
#define DECLINATION_ANGLE ((-16.0*M_PI)/(60.0*180.0))
#include "HMC5883L.h"

Serial usbSerial(USBTX, USBRX);
I2C i2c(D14, D15);
DigitalOut heartbeat(LED_GREEN);
HMC5883L compass(i2c);

SimonK_I2C_ESC motor_1(i2c, ESC_ADDRESS_0,6);
SimonK_I2C_ESC motor_2(i2c, ESC_ADDRESS_1,6);

// PID declarations
PID speed_over_ground_pid(0.5, 1, 0, RATE);
PID heading_pid(0.5, 1, 0, RATE);
PID motor_1_pid(0.5, 1, 0, RATE);
PID motor_2_pid(0.5, 1, 0, RATE);

double bearing;

// We are using the nanopb version of protocol buffers.
shedBoat_Telemetry telemetry_message = shedBoat_Telemetry_init_zero;

void beat();
void send_telemetry(float heading);
float updateSpeedOverGround();
float updateHeading();
void updateMotors();

int main() {
	usbSerial.baud(115200);

    Ticker heartbeat_tkr;
    heartbeat_tkr.attach_us(&beat, 250000);

    i2c.frequency (400);

	// Initialise PIDs
    speed_over_ground_pid.setInputLimits(0.0,  Speed_In_Knots_Limit); // Assuming speed is in knots -- check this!
    speed_over_ground_pid.setOutputLimits(0.0, 1.0);
    speed_over_ground_pid.setMode(AUTO_MODE);

    heading_pid.setInputLimits(-180,180);
    heading_pid.setOutputLimits(0.0, 1.0);
    heading_pid.setMode(AUTO_MODE);

    motor_1_pid.setInputLimits(0.0,  Rpm_Limit);
    motor_1_pid.setOutputLimits(0.0, Throttle_Limit);
    motor_1_pid.setMode(AUTO_MODE);

    motor_2_pid.setInputLimits(0.0,  Rpm_Limit);
    motor_2_pid.setOutputLimits(0.0, Throttle_Limit);
    motor_2_pid.setMode(AUTO_MODE);



	speed_over_ground_pid.setSetPoint(2.5);
  	heading_pid.setSetPoint(0);

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

	// Wait for a valid GPS fix
	while(!NMEA::isDataReady());

	// Parkwood: 51.298997, 1.056683
	// Chestfield: 51.349215, 1.066184
	// Initialise Motors (Arming Sequence) -- If a value is not sent periodicially, then the motors WILL disarm!
	motor_1.set(0);
	motor_2.set(0);
	wait(1);

	while(1)
    {
		wait_ms(500);
		float bearing_compensation = updateHeading();
		float speed_over_ground_compensation = updateSpeedOverGround();

		// Calculate motor setpoints.
		float max = bearing_compensation>=0.5 ? bearing_compensation : (1-bearing_compensation);
		motor_1_pid.setSetPoint((speed_over_ground_compensation * bearing_compensation * Rpm_Limit)/max);
		motor_2_pid.setSetPoint((speed_over_ground_compensation * (1-bearing_compensation) * Rpm_Limit)/max);
	    updateMotors();
	    send_telemetry(bearing);

		usbSerial.printf("Time:       %02d:%02d:%02d\r\n", NMEA::getHour(), NMEA::getMinute(), NMEA::getSecond());
		usbSerial.printf("Satellites: %d\r\n", NMEA::getSatellites());
		usbSerial.printf("Latitude:   %0.5f\r\n", NMEA::getLatitude());
		usbSerial.printf("Longitude:  %0.5f\r\n", NMEA::getLongitude());
		usbSerial.printf("Altitude:   %0.2fm\r\n", NMEA::getAltitude());
		usbSerial.printf("Speed:      %0.2fkm/h\r\n", NMEA::getSpeed());
		usbSerial.printf("Bearing:    %0.2f degrees\r\n", NMEA::getBearing());
    	usbSerial.printf("Compass Bearing: %03.0f\r\n", bearing);
		usbSerial.printf("Heading Delta to South: %03.0f\r\n", heading_delta(bearing, 180.0));
		usbSerial.printf("Distance to Parkwood %06.2fm\r\n",
		equirectangular(degToRad(NMEA::getLatitude()), degToRad(NMEA::getLongitude()),
						degToRad(51.298997), degToRad(1.056683))
		);



		usbSerial.printf("Heading to Parkwood %03.0f\r\n",
			heading_delta(
				startHeading(degToRad(NMEA::getLatitude()), degToRad(NMEA::getLongitude()), degToRad(51.298997), degToRad(1.056683))*(180.0/M_PI),
				bearing
			)
		);

    }
}

void beat()
{
    heartbeat = !heartbeat;
}

void send_telemetry(float bearing)
{
    telemetry_message.location.latitude = NMEA::getLatitude();
    telemetry_message.location.longitude = NMEA::getLongitude();
    telemetry_message.location.number_of_satellites_visible = NMEA::getSatellites();
	telemetry_message.location.true_heading = bearing;
    telemetry_message.status = shedBoat_Telemetry_Status_UNDEFINED;
    telemetry_message.motor[0].motor_number = 1;
    telemetry_message.motor[0].rpm = motor_1.rpm();
}

float updateSpeedOverGround()
{
    speed_over_ground_pid.setProcessValue(NMEA::getSpeed());
    return speed_over_ground_pid.compute(); // Need to pass this to a function aggregating bearing correction and speed.
}

float updateHeading()
{
	bearing = compass.smoothedBearing();
    heading_pid.setProcessValue(heading_delta(
		startHeading(degToRad(NMEA::getLatitude()), degToRad(NMEA::getLongitude()), degToRad(51.298997), degToRad(1.056683))*(180.0/M_PI),
		bearing
	));
     // Please rename the function call to bearing.
    return heading_pid.compute(); // Need to pass this to a function aggregating bearing correction and speed.
}

void updateMotors()
{
    // Grab new data from the motors
    motor_1.update();
    motor_2.update();
    // If a motor is responsive, then calculate new throttle compensation and pass this to the motor. Also report back.
    if(motor_1.isAlive()){
        motor_1_pid.setProcessValue(motor_1.rpm());
        usbSerial.printf("%d",motor_1.rpm());usbSerial.printf(" Actual Motor1 RPM\t\t");
        float throttle_compensation = motor_1_pid.compute();
        usbSerial.printf("%f",throttle_compensation);usbSerial.printf(" Throttle Compensation Motor1\t\t");
        motor_1.set((short)throttle_compensation);
    }
    if(motor_2.isAlive()){
        motor_2_pid.setProcessValue(motor_2.rpm());
        usbSerial.printf("%d",motor_2.rpm());usbSerial.printf(" Actual Motor2 RPM\t\t");
        float throttle_compensation_2 = motor_2_pid.compute();
        usbSerial.printf("%f",throttle_compensation_2);usbSerial.printf(" Throttle Compensation Motor2\t\t");
        usbSerial.printf("\n\r");
        motor_2.set((short)throttle_compensation_2);
    }
}
