/*
    File:       nmea.cpp
    Version:    0.1.0
    Date:       Feb. 23, 2013
	License:	GPL v2

	NMEA GPS content parser

    ****************************************************************************
    Copyright (C) 2013 Radu Motisan  <radu.motisan@gmail.com>

	http://www.pocketmagic.net

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
    ****************************************************************************
 */

#include <stdio.h>
#include <stdlib.h>

#include "mbed.h"
#include "nmea.h"
Serial uart1(D1, D0);
//DigitalOut rxtx_led(LED_BLUE);
// extern Serial serial_output;

bool			NMEA::m_bFlagRead;						// flag used by the parser, when a valid sentence has begun
bool			NMEA::m_bFlagDataReady;					// valid GPS fix and data available, user can call reader functions
char			NMEA::tmp_words[50][80];				//	hold parsed words for one given NMEA sentence
char			NMEA::tmp_szChecksum[15];				//	hold the received checksum for one given NMEA sentence

// will be set to true for characters between $ and * only
bool			NMEA::m_bFlagComputedCks;				// used to compute checksum and indicate valid checksum interval (between $ and * in a given sentence)
int				NMEA::m_nChecksum;						// numeric checksum, computed for a given sentence
bool			NMEA::m_bFlagReceivedCks;				// after getting  * we start cuttings the received checksum
int				NMEA::index_received_checksum;			// used to parse received checksum

// word cutting variables
int				NMEA::m_nWordIdx;						// the current word in a sentence
int				NMEA::m_nPrevIdx;						// last character index where we did a cut
int				NMEA::m_nNowIdx;						// current character index

// globals to store parser results
float			NMEA::res_fLongitude;					// GPRMC and GPGGA
float			NMEA::res_fLatitude;					// GPRMC and GPGGA
unsigned char	NMEA::res_nUTCHour;
unsigned char 	NMEA::res_nUTCMin;
unsigned char 	NMEA::res_nUTCSec;
unsigned char	NMEA::res_nUTCDay;
unsigned char 	NMEA::res_nUTCMonth;
unsigned char 	NMEA::res_nUTCYear;	// GPRMC
int				NMEA::res_nSatellitesUsed;			// GPGGA
float			NMEA::res_fQuality;					// GPGGA
float			NMEA::res_fAltitude;					// GPGGA
float			NMEA::res_fSpeed;						// GPRMC
float			NMEA::res_fBearing;					// GPRMC




void NMEA::init()
{
//	rxtx_led = 1;	// Turn the rx tx light off
	m_bFlagRead = false; //are we in a sentence?
	m_bFlagDataReady = false; //is data available?
	res_fQuality = 2000.0;
	uart1.attach(fusedata);
}

/*
 * The serial data is assembled on the fly, without using any redundant buffers.
 * When a sentence is complete (one that starts with $, ending in EOL), all processing is done on
 * this temporary buffer that we've built: checksum computation, extracting sentence "words" (the CSV values),
 * and so on.
 * When a new sentence is fully assembled using the fusedata function, the code calls parsedata.
 * This function in turn, splits the sentences and interprets the data. Here is part of the parser function,
 * handling both the $GPRMC NMEA sentence:
 */
void NMEA::fusedata() {

	char c = uart1.getc();

	if (c == '$') {
//		rxtx_led = 0;
		m_bFlagRead = true;
		// init parser vars
		m_bFlagComputedCks = false;
		m_nChecksum = 0;
		// after getting  * we start cuttings the received m_nChecksum
		m_bFlagReceivedCks = false;
		index_received_checksum = 0;
		// word cutting variables
		m_nWordIdx = 0; m_nPrevIdx = 0; m_nNowIdx = 0;
	}

	if (m_bFlagRead) {
		// check ending
		if (c == '\r' || c== '\n') {
			// catch last ending item too
			tmp_words[m_nWordIdx][m_nNowIdx - m_nPrevIdx] = 0;
			m_nWordIdx++;
			// cut received m_nChecksum
			tmp_szChecksum[index_received_checksum] = 0;
			// sentence complete, read done
			m_bFlagRead = false;
			// parse
			// serial_output.printf("*** Parsing NMEA ***\r\n");
			parsedata();
		} else {
			// computed m_nChecksum logic: count all chars between $ and * exclusively
			if (m_bFlagComputedCks && c == '*') m_bFlagComputedCks = false;
			if (m_bFlagComputedCks) m_nChecksum ^= c;
			if (c == '$') m_bFlagComputedCks = true;
			// received m_nChecksum
			if (m_bFlagReceivedCks)  {
				tmp_szChecksum[index_received_checksum] = c;
				index_received_checksum++;
			}
			if (c == '*') m_bFlagReceivedCks = true;
			// build a word
			tmp_words[m_nWordIdx][m_nNowIdx - m_nPrevIdx] = c;
			if (c == ',') {
				tmp_words[m_nWordIdx][m_nNowIdx - m_nPrevIdx] = 0;
				m_nWordIdx++;
				m_nPrevIdx = m_nNowIdx;
			}
			else m_nNowIdx++;
		}
	}
//	rxtx_led = 1;
	// return m_nWordIdx;
}


/*
 * parse internal tmp_ structures, fused by pushdata, and set the data flag when done
 */
void NMEA::parsedata() {
	int received_cks = 16*digit2dec(tmp_szChecksum[0]) + digit2dec(tmp_szChecksum[1]);
	//uart1.Send("seq: [cc:%X][words:%d][rc:%s:%d]\r\n", m_nChecksum,m_nWordIdx, tmp_szChecksum, received_cks);
	// check checksum, and return if invalid!
	if (m_nChecksum != received_cks) {
		//m_bFlagDataReady = false;
		//serial_output.printf("*** Checksum check failed ***\r\n");
		return;
	}
	/* $GPGGA
	 * $GPGGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx*hh
	 * ex: $GPGGA,230600.501,4543.8895,N,02112.7238,E,1,03,3.3,96.7,M,39.0,M,,0000*6A,
	 *
	 * WORDS:
	 *  1    = UTC of Position
	 *  2    = Latitude
	 *  3    = N or S
	 *  4    = Longitude
	 *  5    = E or W
	 *  6    = GPS quality indicator (0=invalid; 1=GPS fix; 2=Diff. GPS fix)
	 *  7    = Number of satellites in use [not those in view]
	 *  8    = Horizontal dilution of position
	 *  9    = Antenna altitude above/below mean sea level (geoid)
	 *  10   = Meters  (Antenna height unit)
	 *  11   = Geoidal separation (Diff. between WGS-84 earth ellipsoid and mean sea level.
	 *      -geoid is below WGS-84 ellipsoid)
	 *  12   = Meters  (Units of geoidal separation)
	 *  13   = Age in seconds since last update from diff. reference station
	 *  14   = Diff. reference station ID#
	 *  15   = Checksum
	 */
	if (mstrcmp(tmp_words[0], "$GNGGA") == 0) {
		// serial_output.printf("*** Parse Message `$GPGGA'\r\n");
		// Check GPS Fix: 0=no fix, 1=GPS fix, 2=Dif. GPS fix
		if (tmp_words[6][0] == '0') {
			// clear data
			res_fLatitude = 0;
			res_fLongitude = 0;
			m_bFlagDataReady = false;
			// serial_output.printf("*** [FAILED] Parse Message `$GPGGA'\r\n");
			return;
		}
		// parse time
		res_nUTCHour = digit2dec(tmp_words[1][0]) * 10 + digit2dec(tmp_words[1][1]);
		res_nUTCMin = digit2dec(tmp_words[1][2]) * 10 + digit2dec(tmp_words[1][3]);
		res_nUTCSec = digit2dec(tmp_words[1][4]) * 10 + digit2dec(tmp_words[1][5]);
		// serial_output.printf("*** $GPGGA: Time: %02d:%02d:%02d\r\n", res_nUTCHour, res_nUTCMin, res_nUTCSec);

		// parse latitude and longitude in NMEA format
		res_fLatitude = string2float(tmp_words[2]);
		res_fLongitude = string2float(tmp_words[4]);
		// get decimal format
		if (tmp_words[3][0] == 'S') res_fLatitude  *= -1.0;
		if (tmp_words[5][0] == 'W') res_fLongitude *= -1.0;
		float degrees = trunc(res_fLatitude / 100.0f);
		float minutes = res_fLatitude - (degrees * 100.0f);
		res_fLatitude = degrees + minutes / 60.0f;
		degrees = trunc(res_fLongitude / 100.0f);
		minutes = res_fLongitude - (degrees * 100.0f);
		res_fLongitude = degrees + minutes / 60.0f;

		// parse number of satellites
		res_nSatellitesUsed = (int)string2float(tmp_words[7]);
		res_fQuality = string2float(tmp_words[8]);

		// parse altitude
		res_fAltitude = string2float(tmp_words[9]);

		// data ready
		m_bFlagDataReady = true;
	}

	/* $GPRMC
	 * note: a siRF chipset will not support magnetic headers.
	 * $GPRMC,hhmmss.ss,A,llll.ll,a,yyyyy.yy,a,x.x,x.x,ddmmyy,x.x,a*hh
	 * ex: $GPRMC,230558.501,A,4543.8901,N,02112.7219,E,1.50,181.47,230213,,,A*66,
	 *
	 * WORDS:
	 *  1	 = UTC of position fix
	 *  2    = Data status (V=navigation receiver warning)
	 *  3    = Latitude of fix
	 *  4    = N or S
	 *  5    = Longitude of fix
	 *  6    = E or W
	 *  7    = Speed over ground in knots
	 *  8    = Track made good in degrees True, Bearing This indicates the direction that the device is currently moving in,
	 *       from 0 to 360, measured in �azimuth�.
	 *  9    = UT date
	 *  10   = Magnetic variation degrees (Easterly var. subtracts from true course)
	 *  11   = E or W
	 *  12   = Checksum
	 */
	if (mstrcmp(tmp_words[0], "$GNRMC") == 0) {
		// serial_output.printf("*** Parse Message `$GPRMC'\r\n");

		// Check data status: A-ok, V-invalid
		if (tmp_words[2][0] == 'V') {
			// clear data
			res_fLatitude = 0;
			res_fLongitude = 0;
			m_bFlagDataReady = false;
			// serial_output.printf("*** [FAILED] Parse Message `$GPRMC'\r\n");
			return;
		}
		// parse time
		res_nUTCHour = digit2dec(tmp_words[1][0]) * 10 + digit2dec(tmp_words[1][1]);
		res_nUTCMin = digit2dec(tmp_words[1][2]) * 10 + digit2dec(tmp_words[1][3]);
		res_nUTCSec = digit2dec(tmp_words[1][4]) * 10 + digit2dec(tmp_words[1][5]);
		// serial_output.printf("*** $GPRMC: Time: %02d:%02d:%02d\r\n", res_nUTCHour, res_nUTCMin, res_nUTCSec);

		// parse latitude and longitude in NMEA format
		res_fLatitude = string2float(tmp_words[3]);
		res_fLongitude = string2float(tmp_words[5]);
		// get decimal format
		if (tmp_words[4][0] == 'S') res_fLatitude  *= -1.0;
		if (tmp_words[6][0] == 'W') res_fLongitude *= -1.0;
		float degrees = trunc(res_fLatitude / 100.0f);
		float minutes = res_fLatitude - (degrees * 100.0f);
		res_fLatitude = degrees + minutes / 60.0f;
		degrees = trunc(res_fLongitude / 100.0f);
		minutes = res_fLongitude - (degrees * 100.0f);
		res_fLongitude = degrees + minutes / 60.0f;
		//parse speed
		// The knot (pronounced not) is a unit of speed equal to one nautical mile (1.852 km) per hour
		res_fSpeed = string2float(tmp_words[7]);
		res_fSpeed *= 1.852; // convert to km/h
		// parse bearing
		res_fBearing = string2float(tmp_words[8]);
		// parse UTC date
		res_nUTCDay = digit2dec(tmp_words[9][0]) * 10 + digit2dec(tmp_words[9][1]);
		res_nUTCMonth = digit2dec(tmp_words[9][2]) * 10 + digit2dec(tmp_words[9][3]);
		res_nUTCYear = digit2dec(tmp_words[9][4]) * 10 + digit2dec(tmp_words[9][5]);

		// data ready
		m_bFlagDataReady = true;
	}

	// GNGLL
	// $GNGLL,5117.9172,N,00104.2335,E,131659.000,A,A*43
	// 4916.46,N    Latitude 49 deg. 16.45 min. North
	// 12311.12,W   Longitude 123 deg. 11.12 min. West
	// 225444       Fix taken at 22:54:44 UTC
	// A            Data Active or V (void)
	// *iD          checksum data

	if (mstrcmp(tmp_words[0], "$GNGLL") == 0) {
		// serial_output.printf("*** Parse Message `$GNGLL'\r\n");
		res_fLatitude = string2float(tmp_words[1]);
		res_fLongitude = string2float(tmp_words[3]);
		// get decimal format
		if (tmp_words[2][0] == 'S') res_fLatitude  *= -1.0;
		if (tmp_words[4][0] == 'W') res_fLongitude *= -1.0;
		float degrees = trunc(res_fLatitude / 100.0f);
		float minutes = res_fLatitude - (degrees * 100.0f);
		res_fLatitude = degrees + minutes / 60.0f;
		degrees = trunc(res_fLongitude / 100.0f);
		minutes = res_fLongitude - (degrees * 100.0f);
		res_fLongitude = degrees + minutes / 60.0f;

		m_bFlagDataReady = true;
	}
}
/*
 * returns base-16 value of chars '0'-'9' and 'A'-'F';
 * does not trap invalid chars!
 */
int NMEA::digit2dec(char digit) {
	if (int(digit) >= 65)
		return int(digit) - 55;
	else
		return int(digit) - 48;
}

/* returns base-10 value of zero-terminated string
 * that contains only chars '+','-','0'-'9','.';
 * does not trap invalid strings!
 */
float NMEA::string2float(char* s) {
	long  integer_part = 0;
	float decimal_part = 0.0;
	float decimal_pivot = 0.1;
	bool isdecimal = false, isnegative = false;

	char c;
	while ( ( c = *s++) )  {
		// skip special/sign chars
		if (c == '-') { isnegative = true; continue; }
		if (c == '+') continue;
		if (c == '.') { isdecimal = true; continue; }

		if (!isdecimal) {
			integer_part = (10 * integer_part) + (c - 48);
		}
		else {
			decimal_part += decimal_pivot * (float)(c - 48);
			decimal_pivot /= 10.0;
		}
	}
	// add integer part
	decimal_part += (float)integer_part;

	// check negative
	if (isnegative)  decimal_part = - decimal_part;

	return decimal_part;
}

int NMEA::mstrcmp(const char *s1, const char *s2)
{
	while((*s1 && *s2) && (*s1 == *s2))
	s1++,s2++;
	return *s1 - *s2;
}

bool NMEA::isDataReady() {
	return m_bFlagDataReady;
}

int NMEA::getHour() {
	return res_nUTCHour;
}
int NMEA::getMinute() {
	return res_nUTCMin;
}
int NMEA::getSecond() {
	return res_nUTCSec;
}
int NMEA::getDay() {
	return res_nUTCDay;
}
int NMEA::getMonth() {
	return res_nUTCMonth;
}
int NMEA::getYear() {
	return res_nUTCYear;
}

float NMEA::getLatitude() {
	return res_fLatitude;
}

float NMEA::getLongitude() {
	return res_fLongitude;
}

int NMEA::getSatellites() {
	return res_nSatellitesUsed;
}

float  NMEA::getAltitude() {
	return res_fAltitude;
}

float NMEA::getSpeed() {
	return res_fSpeed;
}

float NMEA::getBearing() {
	return res_fBearing;
}

gps_quality_t NMEA::getFixQuality() {
	if (res_fQuality < 1.0)
		return IDEAL;
	if (res_fQuality < 2.0)
		return EXCELLENT;
	if (res_fQuality < 5.0)
		return GODO;
	if (res_fQuality < 10.0)
		return MODERATE;
	if (res_fQuality < 20.0)
		return FAIR;
	return POOR;
}
