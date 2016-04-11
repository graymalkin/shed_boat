/*
 * @file HMC5883L.h
 * @author Oskar Lopez de Gamboa
 *
 * @section LICENSE
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 * and associated documentation files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge, publish, distribute,
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
 * BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * @section DESCRIPTION
 *
 * HMC5883L 3-Axis Digital Compas IC
 * The library done by Tyler Weaver with:
 *
 *  *Corrected the XZY order instead of the previous XYZ to match the datasheet.
 *  *Added Declination compensation by a define
 *
 *
 */

#ifndef HMC5883L_H
#define HMC5883L_H

#include "mbed.h"

/*
* Defines
*/

//-----------
// Registers
//-----------
#define CONFIG_A_REG	0x00
#define CONFIG_B_REG	0x01
#define MODE_REG		0x02
#define OUTPUT_REG	  0x03
#define STATUS_REG	  0x09

// configuration register a
#define AVG1_SAMPLES	0x00
#define AVG2_SAMPLES	0x20
#define AVG4_SAMPLES	0x80
#define AVG8_SAMPLES	0xC0

#define OUTPUT_RATE_0_75	0x00
#define OUTPUT_RATE_1_5	 0x04
#define OUTPUT_RATE_3	   0x08
#define OUTPUT_RATE_7_5	 0x0C
#define OUTPUT_RATE_15	  0x10
#define OUTPUT_RATE_30	  0x14
#define OUTPUT_RATE_75	  0x18

#define NORMAL_MEASUREMENT  0x00
#define POSITIVE_BIAS	   0x01
#define NEGATIVE_BIAS	   0x02

// mode register
#define CONTINUOUS_MODE	 0x00
#define SINGLE_MODE		 0x01
#define IDLE_MODE		   0x02

// status register
#define STATUS_LOCK		 0x02
#define STATUS_READY		0x01

// Utility
#ifndef M_PI
#define M_PI 3.1415926535897932384626433832795
#endif

#define PI2		 (2*M_PI)
#define RAD_TO_DEG  (180.0/M_PI)
#define DEG_TO_RAD  (M_PI/180.0)

#define SMOOTHING 0.1


// Once you have your heading, you must then add your 'Declination Angle',
// which is  the 'Error' of the magnetic field in your location.
// Find yours here: http://www.magnetic-declination.com/
// Mine is:  -1° 13' WEST which is -1.2167 Degrees, or (which we need)
// 0,021234839232597676519238237683278  radians, so I would use 0.02123
// If you cannot find your Declination, put 0, your compass will be slightly off.

#ifndef DECLINATION_ANGLE
#define DECLINATION_ANGLE 0
#endif


/**
 * The HMC5883L 3-Axis Digital Compass IC
 */
class HMC5883L
{

public:

	/**
	 * The I2C address that can be passed directly to i2c object (it's already shifted 1 bit left).
	 */
	static const int I2C_ADDRESS = 0x3D;

	/**
	* Constructor that accepts external i2c interface object.
	*
	* Calls init function
	*
	* @param i2c The I2C interface object to use.
	*/
	HMC5883L(I2C &i2c) : i2c_(i2c) {
		init();
	}

	~HMC5883L() {}

	/**
	* Initalize function called by all constructors.
	*
	* Place startup code in here.
	*/
	void init();

	/**
	* Function for setting configuration register A
	*
	* Defined constants should be ored together to create value.
	* Defualt is 0x10 - 1 Sample per output, 15Hz Data output rate, normal measurement mode
	*
	* Refer to datasheet for instructions for setting Configuration Register A.
	*
	* @param config the value to place in Configuration Register A
	*/
	void setConfigurationA(char);

	/**
	* Function for retrieving the contents of configuration register A
	*
	* @returns Configuration Register A
	*/
	char getConfigurationA();

	/**
	* Function for setting configuration register B
	*
	* Configuration Register B is for setting the device gain.
	* Default value is 0x20
	*
	* Refer to datasheet for instructions for setting Configuration Register B
	*
	* @param config the value to place in Configuration Register B
	*/
	void setConfigurationB(char);

	/**
	* Function for retrieving the contents of configuration register B
	*
	* @returns Configuration Register B
	*/
	char getConfigurationB();

	/**
	* Funciton for setting the mode register
	*
	* Constants: CONTINUOUS_MODE, SINGLE_MODE, IDLE_MODE
	*
	* When you send a the Single-Measurement Mode instruction to the mode register
	* a single measurement is made, the RDY bit is set in the status register,
	* and the mode is placed in idle mode.
	*
	* When in Continous-Measurement Mode the device continuously performs measurements
	* and places the results in teh data register.  After being placed in this mode
	* it takes two periods at the rate set in the data output rate before the first
	* sample is avaliable.
	*
	* Refer to datasheet for more detailed instructions for setting the mode register.
	*
	* @param mode the value for setting in the Mode Register
	*/
	void setMode(char);

	/**
	* Function for retrieving the contents of mode register
	*
	* @returns mode register
	*/
	char getMode();

	/**
	* Function for retriaval of the raw data
	* Caution!!  the HMC5883L gives you the data in XZY order
	*
	* @param output buffer that is at least 3 in length
	*/
	void getXYZ(int16_t raw[3]);

	/**
	* Function for retrieving the contents of status register
	*
	* Bit1: LOCK, Bit0: RDY
	*
	* @returns status register
	*/
	char getStatus();

	/**
	* Function for getting radian heading using 2-dimensional calculation.
	*
	* Compass must be held flat and away from an magnetic field generating
	* devices such as cell phones and speakers.
	*
	*
	*
	* @returns heading in radians
	*/
	double getHeadingXY();

	/**
	* Function for getting degree heading using 2-dimensional calculation.
	*
	* Compass must be held flat and away from an magnetic field generating
	* devices such as cell phones and speakers.
	*
	*
	*
	* @returns heading in degrees
	*/
	double getHeadingXYDeg() {
		return (getHeadingXY() * RAD_TO_DEG);
	}

	double smoothedBearing();

private:

	I2C &i2c_;

};

#endif // HMC5883L
