#include "mbed.h"

#include "gps.h"

Serial usbSerial(USBTX, USBRX);
DigitalOut heartbeat(LED_GREEN);

void beat();

int main() {
    Ticker heartbeat_tkr;
    heartbeat_tkr.attach_us(&beat, 250000);
    while(1)
    {
        ;
    }
}

void beat()
{
    heartbeat = !heartbeat;
}