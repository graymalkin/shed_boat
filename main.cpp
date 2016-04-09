#include "mbed.h"
#include <math.h>
#include <pb_encode.h>
#include "nmea.h"
#include "NavHelper.h"
#include "PID.h"
#include "SimonK_I2C_ESC.h"

#include "boat.pb.h"

//#define DEBUG
#define SEND_TELEMETRY

#define LEFT_MOTOR_ESC_ADDRESS 0x2B
#define RIGHT_MOTOR_ESC_ADDRESS 0x2A

// 'Scheduler' Stuff in milliseconds
#define TELEMETRY_UPDATE_RATE 5000
#define TRACK_UPDATE_RATE 1000
#define MOTOR_UPDATE_RATE 500
#define HEARTBEAT_UPDATE_RATE 500

// PID Limits
#define THROTTLE_LIMIT 32767.0
#define SPEED_IN_KNOTS_LIMIT 3.5

// Test for correctness when in a room that does produce it's own magnetic field...
// Magnetic Declination for Canterbury = WEST 0deg 16min
#define DECLINATION_ANGLE ((-16.0*M_PI)/(60.0*180.0))
#include "HMC5883L.h"

#ifdef DEBUG
	Serial host(USBTX, USBRX);
	#define DEBUG_OUTPUT(...) host.printf(__VA_ARGS__)
#else
	#define DEBUG_OUTPUT(...)
#endif

Serial xbee(PTC4, PTC3);

I2C i2c(D14, D15);
DigitalOut heartbeat(LED_GREEN);
HMC5883L compass(i2c);

SimonK_I2C_ESC leftMotor(i2c, LEFT_MOTOR_ESC_ADDRESS,6);
SimonK_I2C_ESC rightMotor(i2c, RIGHT_MOTOR_ESC_ADDRESS,6);
float rightThrottle = 0;
float leftThrottle = 0;

// PID declarations
PID speedOverGroundPid(0.5, 1, 0, TRACK_UPDATE_RATE / 1000.0);
PID headingPid(0.5, 1, 0, TRACK_UPDATE_RATE / 1000.0);

//Scheduler declarations
volatile bool updateTelemetry = true;
volatile bool updateTrackAndSpeed = true;
volatile bool updateMotor = true;

double bearing;
float heading;
float bearingCompensation;
float speedOverGroundCompensation;

//forward declarations
void beat();
void telemetry_update_ISR();
void track_and_speed_update_ISR();
void motor_update_ISR();
void gps_satellite_telemetry();
void send_telemetry();
void update_speed_and_heading();
void update_motors();
void send_xbee_packet(uint8_t* payload, uint8_t payload_len);

int main() {
	//host.baud(115200);
	xbee.baud(9600);

  Ticker heartbeat_tkr;
  heartbeat_tkr.attach_us(&beat, HEARTBEAT_UPDATE_RATE * 1000);

  i2c.frequency (400);

	// Initialise PIDs
  speedOverGroundPid.setInputLimits(0.0, SPEED_IN_KNOTS_LIMIT); // Assuming speed is in knots -- check this!
  speedOverGroundPid.setOutputLimits(0.0, 1.0);
  speedOverGroundPid.setMode(AUTO_MODE);

  headingPid.setInputLimits(-180,180);
  headingPid.setOutputLimits(0.0, 1.0);
  headingPid.setMode(AUTO_MODE);

	speedOverGroundPid.setSetPoint(2.5);
	headingPid.setSetPoint(0);

	DEBUG_OUTPUT(	"     _              _   _                 _   \r\n"
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
	while(NMEA::getSatellites() < 3) {
		wait_ms(5000);
		gps_satellite_telemetry();
	}

	// Parkwood: 51.298997, 1.056683
	// Chestfield: 51.349215, 1.066184
	// Initialise Motors (Arming Sequence) -- If a value is not sent periodicially, then the motors WILL disarm!
	leftMotor.set(0);
	rightMotor.set(0);
	wait(1);

	// Initialise Scheduler
	Ticker telemetryUpdateTkr;
	Ticker trackandSpeedUpdateTkr;
	Ticker motorUpdateTkr;
  telemetryUpdateTkr.attach_us(&telemetry_update_ISR, TELEMETRY_UPDATE_RATE * 1000);
	trackandSpeedUpdateTkr.attach_us(&track_and_speed_update_ISR, TRACK_UPDATE_RATE * 1000);
	motorUpdateTkr.attach_us(&motor_update_ISR, MOTOR_UPDATE_RATE * 1000);

	while(1) {
		if(updateTrackAndSpeed){
			update_speed_and_heading();
			updateTrackAndSpeed = false;
		}
		if(updateMotor){
	  	update_motors();
			updateMotor = false;
		}
		if(updateTelemetry){
	  	send_telemetry();
			updateTelemetry = false;
		}
  }
}

void beat()
{
    heartbeat = !heartbeat;
}

void telemetry_update_ISR()
{
    updateTelemetry = true;
}

void track_and_speed_update_ISR()
{
    updateTrackAndSpeed = true;
}

void motor_update_ISR()
{
    updateMotor = true;
}

void gps_satellite_telemetry() {
	shedBoat_Telemetry telemetryMessage = shedBoat_Telemetry_init_zero;

	telemetryMessage.status = shedBoat_Telemetry_Status_UNDEFINED;
	telemetryMessage.has_location = true;

	telemetryMessage.location.has_number_of_satellites_visible = true;
	telemetryMessage.location.number_of_satellites_visible = NMEA::getSatellites();

#ifdef SEND_TELEMETRY
	uint8_t buffer[100];
  pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
	bool success = pb_encode(&stream, shedBoat_Telemetry_fields, &telemetryMessage);
	if(success) {
		send_xbee_packet(buffer, stream.bytes_written);
	} else {
		error("Failed to encode Proto Buffer");
	}
#endif
}

void send_telemetry()
{
	DEBUG_OUTPUT("Time:       %02d:%02d:%02d\r\n", NMEA::getHour(), NMEA::getMinute(), NMEA::getSecond());
	DEBUG_OUTPUT("Satellites: %d\r\n", NMEA::getSatellites());
	DEBUG_OUTPUT("Latitude:   %0.5f\r\n", NMEA::getLatitude());
	DEBUG_OUTPUT("Longitude:  %0.5f\r\n", NMEA::getLongitude());
	DEBUG_OUTPUT("Altitude:   %0.2fm\r\n", NMEA::getAltitude());
	DEBUG_OUTPUT("Speed:      %0.2fkm/h\r\n", NMEA::getSpeed());
	DEBUG_OUTPUT("GPS Bearing (Track made good):    %0.2f degrees\r\n", NMEA::getBearing());
  DEBUG_OUTPUT("Compass Bearing: %03.0f\r\n", bearing);
	DEBUG_OUTPUT("Heading Delta to South: %03.0f\r\n", heading_delta(180.0, bearing));
	DEBUG_OUTPUT("Distance to Parkwood %06.2fm\r\n",
			equirectangular(degToRad(NMEA::getLatitude()), degToRad(NMEA::getLongitude()),degToRad(51.298997), degToRad(1.056683))
	);

	DEBUG_OUTPUT("Heading to Parkwood %03.0f\r\n", heading_delta( heading, bearing ) );

#ifdef SEND_TELEMETRY
	shedBoat_Telemetry telemetryMessage = shedBoat_Telemetry_init_zero;

	telemetryMessage.status = shedBoat_Telemetry_Status_STATIONARY;

	telemetryMessage.has_location = true;
	telemetryMessage.location.has_latitude = true;
	telemetryMessage.location.latitude = NMEA::getLatitude();
	telemetryMessage.location.has_longitude = true;
	telemetryMessage.location.longitude = NMEA::getLongitude();
	telemetryMessage.location.has_number_of_satellites_visible = true;
	telemetryMessage.location.number_of_satellites_visible = NMEA::getSatellites();
	telemetryMessage.location.has_true_heading = true;
	telemetryMessage.location.true_heading = heading * 1000;
	telemetryMessage.location.has_true_bearing = true;
	telemetryMessage.location.true_bearing = bearing * 1000;
	telemetryMessage.location.has_speed_over_ground = true;
	telemetryMessage.location.speed_over_ground = NMEA::getSpeed() * 1000;
	telemetryMessage.location.has_utc_seconds = true;
	telemetryMessage.location.utc_seconds = NMEA::getSecond();

	telemetryMessage.motor_count = 2;
	telemetryMessage.motor[0].motor_number = 1;
	telemetryMessage.motor[0].has_is_alive = true;
	telemetryMessage.motor[0].is_alive = leftMotor.isAlive();
	telemetryMessage.motor[0].has_rpm = true;
	telemetryMessage.motor[0].rpm = leftMotor.rpm();
	telemetryMessage.motor[0].has_temperature = true;
	telemetryMessage.motor[0].temperature = leftMotor.temperature();
	telemetryMessage.motor[0].has_voltage = true;
	telemetryMessage.motor[0].voltage = leftMotor.voltage();
	telemetryMessage.motor[1].motor_number = 2;
	telemetryMessage.motor[1].has_is_alive = true;
	telemetryMessage.motor[1].is_alive = rightMotor.isAlive();
	telemetryMessage.motor[1].has_rpm = true;
	telemetryMessage.motor[1].rpm = rightMotor.rpm();
	telemetryMessage.motor[1].has_temperature = true;
	telemetryMessage.motor[1].temperature = rightMotor.temperature();
	telemetryMessage.motor[1].has_voltage = true;
	telemetryMessage.motor[1].voltage = rightMotor.voltage();

	telemetryMessage.has_battery = false;

	telemetryMessage.has_debug = true;
	telemetryMessage.debug.has_bearing_compensation = true;
	telemetryMessage.debug.bearing_compensation = bearingCompensation * 1000;
	telemetryMessage.debug.has_speed_over_ground_compensation = true;
	telemetryMessage.debug.speed_over_ground_compensation = speedOverGroundCompensation * 1000;
	telemetryMessage.debug.has_motor_1_throttle_compensation = true;
	telemetryMessage.debug.motor_1_throttle_compensation = leftThrottle * 1000;
	telemetryMessage.debug.has_motor_2_throttle_compensation = true;
	telemetryMessage.debug.motor_2_throttle_compensation = rightThrottle * 1000;

	uint8_t buffer[100];
	pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
	bool success = pb_encode(&stream, shedBoat_Telemetry_fields, &telemetryMessage);
	if(success) {
		send_xbee_packet(buffer, stream.bytes_written);
	} else {
		error("Failed to encode Proto Buffer");
	}
#endif
}

void update_speed_and_heading()
{
		bearing = compass.smoothedBearing();
		heading = startHeading(degToRad(NMEA::getLatitude()), degToRad(NMEA::getLongitude()), degToRad(51.298997), degToRad(1.056683))*(180.0/M_PI);
		headingPid.setProcessValue(heading_delta(heading,bearing));
		float bearingCompensation = headingPid.compute();
		speedOverGroundPid.setProcessValue(NMEA::getSpeed());
		float speedOverGroundCompensation = speedOverGroundPid.compute();
		// Calculate motor setpoints.
		float max = bearingCompensation>=0.5 ? bearingCompensation : (1-bearingCompensation);
		leftThrottle = ((speedOverGroundCompensation * bearingCompensation * THROTTLE_LIMIT)/max);
		rightThrottle = ((speedOverGroundCompensation * (1-bearingCompensation) * THROTTLE_LIMIT)/max);
}

void update_motors()
{
	leftMotor.update();
	rightMotor.update();
    // If a motor is responsive, then send current throttle amount.
  if(leftMotor.isAlive()){
			leftMotor.set(leftThrottle);
  }
  if(rightMotor.isAlive()){
			rightMotor.set(rightThrottle);
  }
}

void send_xbee_packet(uint8_t* payload, uint8_t payload_len) {

	xbee.putc(0x7E); // Starting delimiter
	xbee.putc(0x00); // Length (MSB)
	xbee.putc((uint8_t)(payload_len + 14)); // Length (LSB)

	// Frame content
	xbee.putc(0x10); // Frame type, transmit
	xbee.putc(0x01); // Frame ID

	// Coordinator address (64 bit address)
	xbee.putc(0x00);
	xbee.putc(0x00);
	xbee.putc(0x00);
	xbee.putc(0x00);
	xbee.putc(0x00);
	xbee.putc(0x00);
	xbee.putc(0xFF);
	xbee.putc(0xFF);

	// 16 bit address
	xbee.putc(0xFF); // 16 bit 0xFFFE = unknown or broadcast
	xbee.putc(0xFE);

	// Options
	xbee.putc(0x00);

	// Broadcast Range
	xbee.putc(0x00);

	//checksum is the sum all constant bytes except start delimiter and length
	uint8_t checksum = (uint8_t)(0x10 + 0x01 + 0xFF + 0xFF + 0xFF + 0xFE);

	for(uint8_t i=0; i<payload_len; i++) {
		checksum += payload[i];
		xbee.putc(payload[i]);
	}

	xbee.putc(0xFF-checksum);
}
