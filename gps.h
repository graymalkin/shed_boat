#ifndef __gps_h_
#define __gps_h_

#include <inttypes.h>
#include "mbed.h"

#define SYNC_BYTE_A 181
#define SYNC_BYTE_B 98

typedef enum ubx_class_t {
    /* Navigation Results: Position, Speed, Time, Acc, Heading, DOP, SVs used */
    NAV = 0x01,

    /* Receiver Manager Messages: Satellite Status, RTC Status */
    RXM = 0x02,

    /* Information Messages: Printf-Style Messages, with IDs such as Error, Warning, Notice */
    INF = 0x04,

    /* Ack/Nack Messages: as replies to CFG Input Messages */
    ACK = 0x05,

    /* Configuration Input Messages: Set Dynamic Model, Set DOP Mask, Set Baud Rate, etc. */
    CFG = 0x06,

    /* Monitoring Messages: Comunication Status, CPU Load, Stack Usage, Task Status */
    MON = 0x0A,

    /* AssistNow Aiding Messages: Ephemeris, Almanac, other A-GPS data input */
    AID = 0x0B,

    /* Timing Messages: Timepulse Output, Timemark Results */
    TIM = 0x0D,

    /* External Sensor Fusion Messages: External sensor measurements and status information */
    ESF = 0x10
} ubx_class_t;

typedef enum ubx_id_t {
    ID_NAK     = 0x00,
    ID_PRT     = 0x00,
    ID_ERROR   = 0x00,
    ID_ACK     = 0x01,
    ID_MSG     = 0x01,
    ID_WARNING = 0x01,
    ID_INF     = 0x02,
    ID_MEAS    = 0x02,
    ID_NOTICE  = 0x02,
    ID_TEST    = 0x03,
    ID_RST     = 0x04,
    ID_DEBUG   = 0x04,
    ID_DAT     = 0x06,
    ID_TP      = 0x07,
    ID_RATE    = 0x08,
    ID_DATA    = 0x10,
    ID_STATUS  = 0x10,
    ID_EKF     = 0x12,
    ID_ANT     = 0x13,
    ID_SBAS    = 0x16,
    ID_USB     = 0x1B,
    ID_TMODE   = 0x1D,
    ID_ALM     = 0x30,
    ID_TPS     = 0x31,
    ID_ALPSRV  = 0x32,
    ID_AOP     = 0x33,
    ID_RINV    = 0x34,
    ID_ITFM    = 0x39,
    ID_TMODE2  = 0x3D,
    ID_ALP     = 0x50
} ubx_id_t;

typedef struct ubx_packet_t {
    ubx_class_t msg_class;
    ubx_id_t msg_id;
    uint16_t msg_length;

    char * payload;

    uint16_t checksum;
} ubx_packet_t;



class UBX_GPS {
public:
    UBX_GPS(PinName _rx, PinName _tx) : gpsSerial(_tx, _rx) {}
    ~UBX_GPS();

    void poll(ubx_class_t msg_class, ubx_id_t msg_id, ubx_packet_t * dest);

private:
	Serial gpsSerial;

	inline void read(int & len, void* data, Serial &src)
	{
		while(len-->0)
			(*((char*)data++)) = src.getc();
	}

    uint16_t calculate_checksum(ubx_packet_t * packet);
    void debug_print(Serial &out, ubx_packet_t * packet);
    void waitForSync();
};

#endif // __gps_h_
