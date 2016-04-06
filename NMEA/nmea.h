/*
    File:       nmea.h
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
#ifndef __nmea_h_
#define __nmea_h_

#include "mbed.h"

class NMEA {
public:
		static bool			m_bFlagRead;					// flag used by the parser, when a valid sentence has begun
		static bool			m_bFlagDataReady;				// valid GPS fix and data available, user can call reader functions
		static char			tmp_words[20][80];				//	hold parsed words for one given NMEA sentence
		static char			tmp_szChecksum[15];				//	hold the received checksum for one given NMEA sentence

		// will be set to true for characters between $ and * only
		static bool			m_bFlagComputedCks;				// used to compute checksum and indicate valid checksum interval (between $ and * in a given sentence)
		static int			m_nChecksum;					// numeric checksum, computed for a given sentence
		static bool			m_bFlagReceivedCks;				// after getting  * we start cuttings the received checksum
		static int			index_received_checksum;		// used to parse received checksum

		// word cutting variables
		static int			m_nWordIdx;						// the current word in a sentence
		static int			m_nPrevIdx;						// last character index where we did a cut
		static int			m_nNowIdx;						// current character index

		// globals to store parser results
		static float			res_fLongitude;					// GPRMC and GPGGA
		static float			res_fLatitude;					// GPRMC and GPGGA
		static unsigned char	res_nUTCHour;
		static unsigned char 	res_nUTCMin;
		static unsigned char 	res_nUTCSec;
		static unsigned char	res_nUTCDay	;
		static unsigned char 	res_nUTCMonth;
		static unsigned char 	res_nUTCYear;	// GPRMC
		static int				res_nSatellitesUsed;			// GPGGA
		static float			res_fAltitude;					// GPGGA
		static float			res_fSpeed;						// GPRMC
		static float			res_fBearing;					// GPRMC

		// the parser, currently handling GPRMC and GPGGA, but easy to add any new sentences
		static void			parsedata();
		// aux functions
		static int				digit2dec(char hexdigit);
		static float			string2float(char* s);
		static int				mstrcmp(const char *s1, const char *s2);

		/*
		* The serial data is assembled on the fly, without using any redundant buffers.
		* When a sentence is complete (one that starts with $, ending in EOL), all processing is done on
		* this temporary buffer that we've built: checksum computation, extracting sentence "words" (the CSV values),
		* and so on.
		* When a new sentence is fully assembled using the fusedata function, the code calls parsedata.
		* This function in turn, splits the sentences and interprets the data. Here is part of the parser function,
		* handling both the $GPRMC NMEA sentence:
		*/
		static void			fusedata();

		static void init();


		// READER functions: retrieving results, call isDataReady() first
		static bool				isDataReady();
		static int				getHour();
		static int				getMinute();
		static int				getSecond();
		static int				getDay();
		static int				getMonth();
		static int				getYear();
		static float			getLatitude();
		static float			getLongitude();
		static int				getSatellites();
		static float			getAltitude();
		static float			getSpeed();
		static float			getBearing();
	};
#endif // __nmea_h_
