#include "mbed.h"
#include "PID.h"
#include "SimonK_I2C_ESC.h"

#include <pb_encode.h>
#include "boat.pb.h"

#define ESC_ADDRESS 0x2B
#define RATE 0.1

I2C i2c(D14, D15);
Serial pc(USBTX, USBRX);

SimonK_I2C_ESC motor_1(i2c, ESC_ADDRESS,6);
PID motor_1_pid(0.5, 1, 0, RATE);

// We are using the nanopb version of protocol buffers.
shedBoat_Telemetry telemetry_message = shedBoat_Telemetry_init_zero;

void Telemetry_Example(){
    telemetry_message.location.latitude = 51.280233;
    telemetry_message.location.longitude = 1.078909;
    telemetry_message.location.number_of_satellites_visible = 4;
    telemetry_message.status = shedBoat_Telemetry_Status_UNDEFINED;
    telemetry_message.motor[0].motor_number = 1;
    telemetry_message.motor[0].rpm = motor_1.rpm();
}    

int main() {    
    i2c.frequency (400);
    motor_1_pid.setInputLimits(0.0,  8913.0);
    motor_1_pid.setOutputLimits(0.0, 32767.0);
    motor_1_pid.setMode(AUTO_MODE);

    motor_1.set(0);
    wait(1);
    motor_1_pid.setSetPoint(3000);
    
    while(true){
        
        motor_1.update(); 
                 
        if(motor_1.isAlive()){
            motor_1_pid.setProcessValue(motor_1.rpm());
            pc.printf("%d",motor_1.rpm());pc.printf(" Actual RPM\t\t");
            float rpm_compensation = motor_1_pid.compute();
            pc.printf("%f",rpm_compensation);pc.printf("Rpm Compensation\t\t");
            pc.printf("\n\r");
            motor_1.set((short)rpm_compensation);
        }
        Telemetry_Example();
        wait(RATE);
    }
}