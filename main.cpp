#include "mbed.h"
#include <math.h>
#include <pb_encode.h>
#include "nmea.h"
#include "NavHelper.h"
#include "PID.h"
#include "SimonK_I2C_ESC.h"

#include "boat.pb.h"

// 0 = no xbee debug, 1 = Verbose output (human readable), 2 = Protocol buffer output
#define DEBUG_OUTPUT 2
#define ESC_ADDRESS_0 0x2B
#define ESC_ADDRESS_1 0x2A

// 'Scheduler' Stuff in milliseconds
#define TELEMETRY_UPDATE_RATE 5000
#define TRACK_UPDATE_RATE 1000
#define MOTOR_UPDATE_RATE 500
#define HEARTBEAT_UPDATE_RATE 500

volatile bool updateTelemetry = true;
volatile bool updateTrack = true;
volatile bool updateMotor = true;

// PID Limits
#define Rpm_Limit 8913.0
#define Throttle_Limit 32767.0
#define Speed_In_Knots_Limit 3.5
#define PID_RATE 0.5

// Test for correctness when in a room that does produce it's own magnetic field...
// Magnetic Declination for Canterbury = WEST 0deg 16min
#define DECLINATION_ANGLE ((-16.0*M_PI)/(60.0*180.0))
#include "HMC5883L.h"

//Serial host(USBTX, USBRX);
Serial xbee(PTC4, PTC3);

I2C i2c(D14, D15);
DigitalOut heartbeat(LED_GREEN);
HMC5883L compass(i2c);

SimonK_I2C_ESC motor_1(i2c, ESC_ADDRESS_0,6);
SimonK_I2C_ESC motor_2(i2c, ESC_ADDRESS_1,6);

// PID declarations
PID speed_over_ground_pid(0.5, 1, 0, updateTrack / 1000.0);
PID heading_pid(0.5, 1, 0, updateTrack / 1000.0);
PID motor_1_pid(0.5, 1, 0, MOTOR_UPDATE_RATE / 1000.0);
PID motor_2_pid(0.5, 1, 0, MOTOR_UPDATE_RATE / 1000.0);

double bearing;
float heading;

//forward declarations
void beat();
void telemetry_update_ISR();
void track_update_ISR();
void motor_update_ISR();
void gps_satellite_telemetry();
void send_telemetry();
float updateSpeedOverGround();
float updateHeading();
void updateMotors();
void sendXBeePacket(uint8_t* payload, uint8_t payload_len);

// These need to be got rid of later
float bearing_compensation;
float speed_over_ground_compensation;
float throttle_compensation = 0;
float throttle_compensation_2 = 0;

int main() {
	//host.baud(115200);
	xbee.baud(9600);

  Ticker heartbeat_tkr;
  heartbeat_tkr.attach_us(&beat, HEARTBEAT_UPDATE_RATE * 1000);

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

#if DEBUG_OUTPUT == 1
	host.printf(	"     _              _   _                 _   \r\n"
								"    | |            | | | |               | |  \r\n"
								" ___| |__   ___  __| | | |__   ___   __ _| |_ \r\n"
								"/ __| '_ \\ / _ \\/ _` | | '_ \\ / _ \\ / _` | __|\r\n"
								"\\__ \\ | | |  __/ (_| | | |_) | (_) | (_| | |_ \r\n"
								"|___/_| |_|\\___|\\__,_| |_.__/ \\___/ \\__,_|\\__|\r\n"
								"                   ______                     \r\n"
								"                  |______|\r\n"
								"\r\n\n");
#endif

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
	motor_1.set(0);
	motor_2.set(0);
	wait(1);

	// Initialise Scheduler
	Ticker telemetryUpdateTkr;
	Ticker trackUpdateTkr;
	Ticker motorUpdateTkr;
  telemetryUpdateTkr.attach_us(&telemetry_update_ISR, TELEMETRY_UPDATE_RATE * 1000);
	trackUpdateTkr.attach_us(&track_update_ISR, TRACK_UPDATE_RATE * 1000);
	motorUpdateTkr.attach_us(&motor_update_ISR, MOTOR_UPDATE_RATE * 1000);

	while(1) {
		wait_ms(500);

		// Grab new data from motors
		motor_1.update();
		motor_2.update();

		if(updateTrack){
			bearing_compensation = updateHeading();
			speed_over_ground_compensation = updateSpeedOverGround();
			// Calculate motor setpoints.
			float max = bearing_compensation>=0.5 ? bearing_compensation : (1-bearing_compensation);
			motor_1_pid.setSetPoint((speed_over_ground_compensation * bearing_compensation * Rpm_Limit)/max);
			motor_2_pid.setSetPoint((speed_over_ground_compensation * (1-bearing_compensation) * Rpm_Limit)/max);
			updateTrack = false;
		}
		if(updateMotor){
	  	updateMotors();
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

void track_update_ISR()
{
    updateTrack = true;
}

void motor_update_ISR()
{
    updateMotor = true;
}

void gps_satellite_telemetry() {
	shedBoat_Telemetry telemetry_message = shedBoat_Telemetry_init_zero;

	telemetry_message.status = shedBoat_Telemetry_Status_UNDEFINED;
	telemetry_message.has_location = true;

	telemetry_message.location.has_number_of_satellites_visible = true;
	telemetry_message.location.number_of_satellites_visible = NMEA::getSatellites();

	#if DEBUG_OUTPUT == 2
				uint8_t buffer[100];
		    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
				bool success = pb_encode(&stream, shedBoat_Telemetry_fields, &telemetry_message);
				if(success) {
					sendXBeePacket(buffer, stream.bytes_written);
				} else {
					error("Failed to encode Proto Buffer");
				}
	#endif
}

void send_telemetry()
{
		#if DEBUG_OUTPUT == 1
				host.printf("Time:       %02d:%02d:%02d\r\n", NMEA::getHour(), NMEA::getMinute(), NMEA::getSecond());
				host.printf("Satellites: %d\r\n", NMEA::getSatellites());
				host.printf("Latitude:   %0.5f\r\n", NMEA::getLatitude());
				host.printf("Longitude:  %0.5f\r\n", NMEA::getLongitude());
				host.printf("Altitude:   %0.2fm\r\n", NMEA::getAltitude());
				host.printf("Speed:      %0.2fkm/h\r\n", NMEA::getSpeed());
				host.printf("GPS Bearing (Track made good):    %0.2f degrees\r\n", NMEA::getBearing());
		    host.printf("Compass Bearing: %03.0f\r\n", bearing);
				host.printf("Heading Delta to South: %03.0f\r\n", heading_delta(180.0, bearing));
				host.printf("Distance to Parkwood %06.2fm\r\n",
						equirectangular(degToRad(NMEA::getLatitude()), degToRad(NMEA::getLongitude()),degToRad(51.298997), degToRad(1.056683))
				);

				host.printf("Heading to Parkwood %03.0f\r\n",
					heading_delta( heading, bearing )
				);
		#endif

		#if DEBUG_OUTPUT == 2
					shedBoat_Telemetry telemetry_message = shedBoat_Telemetry_init_zero;

					telemetry_message.status = shedBoat_Telemetry_Status_STATIONARY;

					telemetry_message.has_location = true;
					telemetry_message.location.has_latitude = true;
					telemetry_message.location.latitude = NMEA::getLatitude();
					telemetry_message.location.has_longitude = true;
					telemetry_message.location.longitude = NMEA::getLongitude();
					telemetry_message.location.has_number_of_satellites_visible = true;
					telemetry_message.location.number_of_satellites_visible = NMEA::getSatellites();
					telemetry_message.location.has_true_heading = true;
					telemetry_message.location.true_heading = heading;
					telemetry_message.location.has_true_bearing = true;
					telemetry_message.location.true_bearing = bearing;
					telemetry_message.location.has_speed_over_ground = true;
					telemetry_message.location.speed_over_ground = NMEA::getSpeed();
					telemetry_message.location.has_utc_seconds = true;
					telemetry_message.location.utc_seconds = NMEA::getSecond();

					telemetry_message.motor_count = 2;
					telemetry_message.motor[0].motor_number = 1;
					telemetry_message.motor[0].has_is_alive = true;
					telemetry_message.motor[0].is_alive = motor_1.isAlive();
					telemetry_message.motor[0].has_rpm = true;
					telemetry_message.motor[0].rpm = motor_1.rpm();
					telemetry_message.motor[0].has_temperature = true;
					telemetry_message.motor[0].temperature = motor_1.temperature();
					telemetry_message.motor[0].has_voltage = true;
					telemetry_message.motor[0].voltage = motor_1.voltage();
					telemetry_message.motor[1].motor_number = 2;
					telemetry_message.motor[1].has_is_alive = true;
					telemetry_message.motor[1].is_alive = motor_2.isAlive();
					telemetry_message.motor[1].has_rpm = true;
					telemetry_message.motor[1].rpm = motor_2.rpm();
					telemetry_message.motor[1].has_temperature = true;
					telemetry_message.motor[1].temperature = motor_2.temperature();
					telemetry_message.motor[1].has_voltage = true;
					telemetry_message.motor[1].voltage = motor_2.voltage();

					telemetry_message.has_battery = false;

					telemetry_message.has_debug = true;
					telemetry_message.debug.bearing_compensation = bearing_compensation;
					telemetry_message.debug.speed_over_ground_compensation = speed_over_ground_compensation;
					telemetry_message.debug.motor_1_throttle_compensation = throttle_compensation;
					telemetry_message.debug.motor_2_throttle_compensation = throttle_compensation_2;

					uint8_t buffer[100];
			    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
					bool success = pb_encode(&stream, shedBoat_Telemetry_fields, &telemetry_message);
					if(success) {
						sendXBeePacket(buffer, stream.bytes_written);
					} else {
						error("Failed to encode Proto Buffer");
					}
		#endif
}

float updateSpeedOverGround()
{
    speed_over_ground_pid.setProcessValue(NMEA::getSpeed());
    return speed_over_ground_pid.compute();
}

float updateHeading()
{
	bearing = compass.smoothedBearing();
	heading = startHeading(degToRad(NMEA::getLatitude()), degToRad(NMEA::getLongitude()), degToRad(51.298997), degToRad(1.056683))*(180.0/M_PI);

  heading_pid.setProcessValue(heading_delta(heading,bearing));
  return heading_pid.compute();
}

void updateMotors()
{
    // If a motor is responsive, then calculate new throttle compensation and pass this to the motor. Also report back.
    if(motor_1.isAlive()){
        motor_1_pid.setProcessValue(motor_1.rpm());
				throttle_compensation = motor_1_pid.compute();
				motor_1.set((short)throttle_compensation);
    }
    if(motor_2.isAlive()){
        motor_2_pid.setProcessValue(motor_2.rpm());
				throttle_compensation_2 = motor_2_pid.compute();
				motor_2.set((short)throttle_compensation_2);
    }

#if DEBUG_OUTPUT == 1
			host.printf("%d",motor_1.rpm());host.printf(" Actual Motor1 RPM\t\t");
			host.printf("%f",throttle_compensation);host.printf(" Throttle Compensation Motor1\t\t");
			host.printf("%d",motor_2.rpm());host.printf(" Actual Motor2 RPM\t\t");
			host.printf("%f",throttle_compensation_2);host.printf(" Throttle Compensation Motor2\t\t");
			host.printf("\n\r");
#endif
}

void sendXBeePacket(uint8_t* payload, uint8_t payload_len) {

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
