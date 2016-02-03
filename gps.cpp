#include "gps.h"

void UBX_GPS(int rx, int tx) {
    this.gpsSerial(rx,tx);
}

void ~UBX_GPS() {
    
}

void UBX_GPS:poll(ubx_class_t msg_class, ubx_id_t msg_id, ubx_packet_t * dest) {
    //poll request is `send 0-payload packet and await responce'
    gpsSerial.putc(SYNC_BYTE_A); //sync char 1
    gpsSerial.putc(SYNC_BYTE_B); //sync char 2
    gpsSerial.putc(msg_class);   //class
    gpsSerial.putc(msg_id);      // id
    gpsSerial.putc(0); gpsSerial.putc(0); //16bit length
    //no payload on poll
    uint_8 A = msg_class + msg_id;
    gpsSerial.putc(A);
    gpsSerial.putc(A+msg_class);
    
    waitForSync();
    
    //read 4 bytes to *dest;
    
    
    //read in payload
    char* payload = malloc(dest->msg_length);
    
    
    //read in checksum
    dest->checksum[0] = he
}

void UBX_GPS:waitForSync() {
    char sync = gpsSerial.getc();
    for(;;) {
        while(sync != SYNC_BYTE_A) {
            sync = gpsSerial.getc();
        }
        sync = gpsSerial.getc();
        if(sync == SYNC_BYTE_B)
            return;
    }
}

uint16_t UBX_GPS:calculate_checksum(ubx_packet_t * packet)
{
    uint8_t checksum_a = 0, checksum_b = 0;
    
    checksum_a += packet->msg_id;
    checksum_b += checksum_a;
    
    checksum_a += ((packet->msg_length & 0xFF00) >> 8) + (packet->msg_length & 0x00FF);
    checksum_b += checksum_a;
    
    for(int i = 0; i < packet->msg_length; i++)
    {
        checksum_a += packet->payload[i];
        checksum_b += checksum_a;
    }
    
    return ((checksum_b & 0xFF) << 8) + (checksum_b & 0xFF);
}

void UBX_GPS:debug_print(Serial &out, ubx_packet_t * packet)
{
    out.printf("Class:%d\nID:%d\nLen:%d\n",packet->msg_class,packet->msg_id,packet->msg_length);
    out.printf("Payload:");
    
    if(packet->msg_length==0) {
        out.printf("[]\n");
    } else {
        out.printf("[%02X",packet->payload[0]);
        for(int i=1; i<packet->msg_length; i++) {
            out.printf(", %02X",packet->payload[i]);
        }
        out.printf("]\n");
    }
    
    out.printf("Checksum:%d\n",packet->checksum);
}