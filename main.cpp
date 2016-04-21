#include "mbed.h"
#include <math.h>
#include <pb_encode.h>
#include "nmea.h"
#include "NavHelper.h"
#include "PID.h"
#include "SimonK_I2C_ESC.h"
#include "BufferedSerial.h"
#include "boat.pb.h"

//#define DEBUG
// Enabling BEARING_PID_CALIBRATION or SPEED_PID_CALIBRATION allows the motors to only be effected by one PID; useful for calibration.
//#define BEARING_PID_CALIBRATION
//#define SPEED_PID_CALIBRATION
#define SEND_TELEMETRY

#define LEFT_MOTOR_ESC_ADDRESS 0x2B
#define RIGHT_MOTOR_ESC_ADDRESS 0x2A

// 'Scheduler' Stuff in milliseconds
#define SYSTEM_TELEMETRY_UPDATE_RATE 15000
#define DEBUG_TELEMETRY_UPDATE_RATE 15000
#define GPS_TELEMETRY_UPDATE_RATE 5000

#define TRACK_UPDATE_RATE 1000
#define MOTOR_UPDATE_RATE 500
#define HEARTBEAT_UPDATE_RATE 500

// PID Limits
#define THROTTLE_LIMIT 32767.0
// Max speed ahead
#define SPEED_IN_KNOTS_LIMIT 3.5

// Test for correctness when in a room that does produce it's own magnetic field...
// Magnetic Declination for Canterbury = WEST 0deg 16min
// #define DECLINATION_ANGLE ((-16.0*M_PI)/(60.0*180.0))
#include "HMC5883L.h"

#ifdef DEBUG
	Serial host(USBTX, USBRX);
	#define DEBUG_OUTPUT(...) host.printf(__VA_ARGS__)
#else
	#define DEBUG_OUTPUT(...)
#endif

#define WAYPOINT_RADIUS 12.0

bool autopilot = true;

BufferedSerial  xbee(PTC4, PTC3);

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
volatile bool updateGPSTelemetry = true;
volatile bool updateSystemTelemetry = true;
volatile bool updateDebugTelemetry = true;

volatile bool updateTrackAndSpeed = true;
volatile bool updateMotor = true;

double bearing = 0.0;
float heading = 0.0f;
float bearingCompensation = 0.0f;
float speedOverGroundCompensation = 0.0f;

//forward declarations
void beat();
void gps_telemetry_update_ISR();
void system_telemetry_update_ISR();
void debug_telemetry_update_ISR();
void track_and_speed_update_ISR();
void motor_update_ISR();
void gps_satellite_telemetry();
void send_system_telemetry();
void send_debug_telemetry();
void update_speed_and_heading();
void update_motors();
void send_xbee_packet(uint8_t* payload, uint8_t payload_len);
void setup_waypoints();

int main() {
	//host.baud(115200);
	xbee.baud(115200);

	Ticker heartbeat_tkr;
	heartbeat_tkr.attach_us(&beat, HEARTBEAT_UPDATE_RATE * 1000);

	//i2c.frequency(400);

	// Initialise PIDs
	speedOverGroundPid.setInputLimits(0.0, SPEED_IN_KNOTS_LIMIT); // Assuming speed is in knots -- check this!
	speedOverGroundPid.setOutputLimits(1, THROTTLE_LIMIT);
	speedOverGroundPid.setMode(AUTO_MODE);

	headingPid.setInputLimits(-180,180);
	headingPid.setOutputLimits(0.0, THROTTLE_LIMIT);
	headingPid.setMode(AUTO_MODE);

	speedOverGroundPid.setSetPoint(2.5);
	headingPid.setSetPoint(0);

	DEBUG_OUTPUT("     _              _   _                 _   \r\n"
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
	while(NMEA::getFixQuality() > 3) {
		wait_ms(5000);
		gps_satellite_telemetry();
	}

	// Parkwood: 51.298997, 1.056683
	// Chestfield: 51.349215, 1.066184
	// Initialise Motors (Arming Sequence) -- If a value is not sent periodicially, then the motors WILL disarm!
	leftMotor.set(0);
	rightMotor.set(0);
	wait(1);

	setup_waypoints();
	// Initialise Scheduler
	Ticker gpsTelemetryUpdateTkr;
	Ticker systemTelemetryUpdateTkr;
	Ticker debugTelemetryUpdateTkr;

	Ticker trackandSpeedUpdateTkr;
	Ticker motorUpdateTkr;
	gpsTelemetryUpdateTkr.attach_us(&gps_telemetry_update_ISR, GPS_TELEMETRY_UPDATE_RATE * 1000);
	systemTelemetryUpdateTkr.attach_us(&system_telemetry_update_ISR, SYSTEM_TELEMETRY_UPDATE_RATE * 1000);
	debugTelemetryUpdateTkr.attach_us(&debug_telemetry_update_ISR, DEBUG_TELEMETRY_UPDATE_RATE * 1000);

	trackandSpeedUpdateTkr.attach_us(&track_and_speed_update_ISR, TRACK_UPDATE_RATE * 1000);
	motorUpdateTkr.attach_us(&motor_update_ISR, MOTOR_UPDATE_RATE * 1000);

	while(1) {
		if(updateTrackAndSpeed && autopilot){
			update_speed_and_heading();
			updateTrackAndSpeed = false;
		}
		if(updateMotor){
			update_motors();
			updateMotor = false;
		}
		if(updateGPSTelemetry){
			gps_satellite_telemetry();
			updateGPSTelemetry = false;
		}
		if(updateSystemTelemetry){
			send_system_telemetry();
			updateSystemTelemetry = false;
		}
		if(updateDebugTelemetry){
			send_debug_telemetry();
			updateDebugTelemetry = false;
		}
	}
}

void update_speed_and_heading()
{
	if(distance_to_current_nav(degToRad((double)NMEA::getLatitude()), degToRad((double)NMEA::getLongitude())) < WAYPOINT_RADIUS)
		go_next_nav();

	nav_list_t * current_nav = get_current_nav();

	bearing = compass.getHeadingXYDeg();
	heading = startHeading(degToRad(NMEA::getLatitude()), degToRad(NMEA::getLongitude()), current_nav->latitude, current_nav->longitude)*(180.0/M_PI);
	headingPid.setProcessValue(heading_delta(heading,bearing));
	speedOverGroundPid.setProcessValue(NMEA::getSpeed());
	#ifdef SPEED_PID_CALIBRATION
		bearingCompensation = 0;
	#else
		bearingCompensation = headingPid.compute();
	#endif
	#ifdef BEARING_PID_CALIBRATION
		speedOverGroundCompensation = 0;
	#else
		speedOverGroundCompensation = speedOverGroundPid.compute();
	#endif
	leftThrottle = ((speedOverGroundCompensation - bearingCompensation) < THROTTLE_LIMIT) ? (speedOverGroundCompensation - bearingCompensation) : THROTTLE_LIMIT;
	rightThrottle = ((speedOverGroundCompensation + bearingCompensation) < THROTTLE_LIMIT) ? (speedOverGroundCompensation + bearingCompensation) : THROTTLE_LIMIT;
}

void update_motors()
{
	leftMotor.update();
	rightMotor.update();
	// If a motor is responsive, then send current throttle amount.
	if(leftMotor.isAlive()){
		leftMotor.set(leftThrottle * -1);
	}
	if(rightMotor.isAlive()){
		rightMotor.set(rightThrottle);
	}
}

void send_xbee_packet(uint8_t* payload, uint8_t payload_len) {
	if(xbee.writeable()){
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
}

void gps_satellite_telemetry() {
	DEBUG_OUTPUT("Time:       %02d:%02d:%02d\r\n", NMEA::getHour(), NMEA::getMinute(), NMEA::getSecond());
	DEBUG_OUTPUT("Satellites: %d\r\n", NMEA::getSatellites());
	DEBUG_OUTPUT("Latitude:   %0.5f\r\n", NMEA::getLatitude());
	DEBUG_OUTPUT("Longitude:  %0.5f\r\n", NMEA::getLongitude());
	DEBUG_OUTPUT("Altitude:   %0.2fm\r\n", NMEA::getAltitude());
	DEBUG_OUTPUT("Speed:      %0.2fkm/h\r\n", NMEA::getSpeed());
	DEBUG_OUTPUT("GPS Bearing (Track made good):    %0.2f degrees\r\n", NMEA::getBearing());
	DEBUG_OUTPUT("Compass Bearing: %03.0f\r\n", bearing);
	DEBUG_OUTPUT("Heading Delta to Waypoint: %03.0f\r\n", heading_delta(180.0, bearing));
	DEBUG_OUTPUT("Distance to Next Nav %06.2fm\r\n",
				 distance_to_current_nav(degToRad((double)NMEA::getLatitude()), degToRad((double)NMEA::getLongitude()))
	);
#ifdef SEND_TELEMETRY
	shedBoat_Telemetry telemetryMessage = shedBoat_Telemetry_init_zero;
	telemetryMessage.status = shedBoat_Telemetry_Status_UNDEFINED;
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
	telemetryMessage.location.fix_quality = (_shedBoat_Location_Quality)NMEA::getFixQuality();
	telemetryMessage.location.has_distance_to_waypoint = true;
	telemetryMessage.location.distance_to_waypoint = (int32_t)distance_to_current_nav(degToRad((double)NMEA::getLatitude()), degToRad((double)NMEA::getLongitude()));
	telemetryMessage.location.has_waypoint_number = true;
	telemetryMessage.location.waypoint_number = get_nav_num();

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

void send_system_telemetry()
{

#ifdef SEND_TELEMETRY
	shedBoat_Telemetry telemetryMessage = shedBoat_Telemetry_init_zero;

	telemetryMessage.status = shedBoat_Telemetry_Status_STATIONARY;

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

	uint8_t buffer[100];
	pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
	bool success = pb_encode(&stream, shedBoat_Telemetry_fields, &telemetryMessage);
	if(success) {
		send_xbee_packet(buffer, stream.bytes_written);
	} else {
		error("Failed to encode Proto Buffer");
		DEBUG_OUTPUT("Failed to encode Proto Buffer /r/n");
	}
#endif
}

void send_debug_telemetry()
{

#ifdef SEND_TELEMETRY
	shedBoat_Telemetry telemetryMessage = shedBoat_Telemetry_init_zero;

	telemetryMessage.status = shedBoat_Telemetry_Status_STATIONARY;

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
		DEBUG_OUTPUT("Failed to encode Proto Buffer /r/n");
	}
#endif
}

void setup_waypoints()
{
	add_waypoint(degToRad(51.298349), degToRad(1.069928));
	add_waypoint(degToRad(51.297772), degToRad(1.070301));
	add_waypoint(degToRad(51.297048), degToRad(1.069866));
	add_waypoint(degToRad(51.296581), degToRad(1.068332));
	add_waypoint(degToRad(51.2974), degToRad(1.067618));
	add_waypoint(degToRad(51.29795), degToRad(1.068573));
	add_waypoint(degToRad(51.298356), degToRad(1.069871));
}

void beat()
{
	heartbeat = !heartbeat;
}

void gps_telemetry_update_ISR()
{
	updateGPSTelemetry = true;
}

void system_telemetry_update_ISR(){
	updateSystemTelemetry = true;
}

void debug_telemetry_update_ISR(){
	updateDebugTelemetry = true;
}

void track_and_speed_update_ISR()
{
	updateTrackAndSpeed = true;
}

void motor_update_ISR()
{
	updateMotor = true;
}
