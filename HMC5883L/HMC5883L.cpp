/*
 * @file HMC5883L.cpp
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
 */

#include "HMC5883L.h"
#include <new>

HMC5883L::HMC5883L(PinName sda, PinName scl) : i2c_(*reinterpret_cast<I2C*>(i2cRaw))
{
    // Placement new to avoid additional heap memory allocation.
    new(i2cRaw) I2C(sda, scl);

    init();
}

HMC5883L::~HMC5883L()
{
    // If the I2C object is initialized in the buffer in this object, call destructor of it.
    if(&i2c_ == reinterpret_cast<I2C*>(&i2cRaw))
        reinterpret_cast<I2C*>(&i2cRaw)->~I2C();
}

void HMC5883L::init()
{
    // init - configure your setup here
    setConfigurationA(AVG8_SAMPLES | OUTPUT_RATE_15); // 8 sample average, 15Hz, normal mode
    setConfigurationB(0x20); // default gain
    setMode(CONTINUOUS_MODE); // continuous sample mode
    wait(0.006);//wait 6ms as told in the datasheet
}

void HMC5883L::setConfigurationA(char config)
{
    char cmd[2];
    cmd[0] = CONFIG_A_REG; // register a address
    cmd[1] = config;

    i2c_.write(I2C_ADDRESS, cmd, 2);
}

void HMC5883L::setConfigurationB(char config)
{
    char cmd[2];
    cmd[0] = CONFIG_B_REG; // register b address
    cmd[1] = config;

    i2c_.write(I2C_ADDRESS, cmd, 2);
}

char HMC5883L::getConfigurationA()
{
    char cmd[2];
    cmd[0] = CONFIG_A_REG; // register a address
    i2c_.write(I2C_ADDRESS, cmd, 1, true);
    i2c_.read(I2C_ADDRESS, &cmd[1], 1, false);
    return cmd[1];
}

char HMC5883L::getConfigurationB()
{
    char cmd[2];
    cmd[0] = CONFIG_A_REG; // register b address
    i2c_.write(I2C_ADDRESS, cmd, 1, true);
    i2c_.read(I2C_ADDRESS, &cmd[1], 1, false);
    return cmd[1];
}

void HMC5883L::setMode(char mode = SINGLE_MODE)
{
    char cmd[2];
    cmd[0] = MODE_REG; // mode register address
    cmd[1] = mode;
    i2c_.write(I2C_ADDRESS,cmd,2);
}

char HMC5883L::getMode()
{
    char cmd[2];
    cmd[0] = MODE_REG; // mode register
    i2c_.write(I2C_ADDRESS, cmd, 1, true);
    i2c_.read(I2C_ADDRESS, &cmd[1], 1, false);
    return cmd[1];
}

char HMC5883L::getStatus()
{
    char cmd[2];
    cmd[0] = STATUS_REG; // status register
    i2c_.write(I2C_ADDRESS, cmd, 1, true);
    i2c_.read(I2C_ADDRESS, &cmd[1], 1, false);
    return cmd[1];
}

void HMC5883L::getXYZ(int16_t output[3])
{
    char cmd[2];
    char data[6];
    cmd[0] = 0x03; // starting point for reading
    i2c_.write(I2C_ADDRESS, cmd, 1, true); // set the pointer to the start of x
    i2c_.read(I2C_ADDRESS, data, 6, false);

    for(int i = 0; i < 3; i++) // fill the output variables
        output[i] = int16_t(((unsigned char)data[i*2] << 8) | (unsigned char)data[i*2+1]);
}

double HMC5883L::getHeadingXY()
{
    int16_t raw_data[3];
    getXYZ(raw_data);
    //The  HMC5883L gives X Z Y order
    double heading = atan2(static_cast<double>(raw_data[2]), static_cast<double>(raw_data[0])); // heading = arctan(Y/X)


    heading += DECLINATION_ANGLE;

    if(heading < 0.0) // fix sign
        heading += PI2;

    if(heading > PI2) // fix overflow
        heading -= PI2;

    return heading;
}

double HMC5883L::smoothedBearing() {
    static int historic = getHeadingXYDeg();

    double h = getHeadingXYDeg();

    if(h==historic) return h;

    if( (h<180.0)==(historic<180.0) ) {

        historic = historic*SMOOTHING + h*(1.0-SMOOTHING);

    } else {
        if(h<180) {

            historic = historic*SMOOTHING + (h+360.0)*(1.0-SMOOTHING);

        } else {

            historic = (historic+360.0)*SMOOTHING + h*(1.0-SMOOTHING);

        }
        if(historic>=360.0) {
            historic-=360.0;
        }
    }

    return historic;

}
