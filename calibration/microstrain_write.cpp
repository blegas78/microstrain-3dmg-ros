/*
 Author: Matt Bunting
 Copyright (c) 2023 Vanderbilt
 All rights reserved.
 
 */

/*
 This Documentaion
 */

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <cmath>
#include <iostream>
#include <signal.h>

#include <unistd.h>
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()


#define MICROSTRAIN_NULL_COMMAND (0x00)
#define MICROSTRAIN_SEND_RAW_SENSOR_BITS (0x01)
#define MICROSTRAIN_SEND_GYRO_STABILIZED_VECTORS (0x02)
#define MICROSTRAIN_SEND_INSTANTANEOUS_VECTORS (0x03)
#define MICROSTRAIN_SEND_INSTANTANEOUS_QUATERNION (0x04)
#define MICROSTRAIN_SEND_GYRO_STABILIZED_QUTERNION (0x05)
#define MICROSTRAIN_CAPUTRE_GYRO_BIAS (0x06)
#define MICROSTRAIN_SEND_TEMPERATURE (0x07)
#define MICROSTRAIN_SEND_EEPROM_VALUE (0x08)
#define MICROSTRAIN_PROGRAM_EEPROM_VALUE (0x09)
#define MICROSTRAIN_SEND_INSTANTANEOUS_ORIENTATION_MATRIX (0x0A)
#define MICROSTRAIN_SEND_GYRO_STABILIZED_ORIENTATION_MATRIX (0x0B)
#define MICROSTRAIN_SEND_GYRO_STABILIZED_QUATERNION_AND_VECTORS (0x0C)
#define MICROSTRAIN_SEND_INSTANTANEOUS_EULER_ANGLES (0x0D)
#define MICROSTRAIN_SEND_GYRO_STABILIZED_EULER_ANGLES (0x0E)
#define MICROSTRAIN_SET_CONTINUOUS_MODE (0x10)
#define MICROSTRAIN_SEND_FIRMWARE_VERSION_NUMBER (0xF0)
#define MICROSTRAIN_SEND_DEVICE_SERIAL_NUMBER (0xF1)


int16_t buffToShort(unsigned char* buffer) {
    uint16_t value = ((uint16_t)buffer[0] << 8) + buffer[1];
    return *(int16_t*)&value;
}


int16_t readEEPROM(int serial_port, unsigned char location) {
    unsigned char buffer[10];
    unsigned char command = MICROSTRAIN_SEND_EEPROM_VALUE;
    int responseCount = 2;
    buffer[0] = command;
    buffer[1] = location;
    
    write(serial_port, buffer, 2);
    int num_bytes = 0;
    int readCount = 0;
    while(num_bytes < responseCount) {
        num_bytes += read(serial_port, buffer+num_bytes, responseCount-num_bytes);
        if (readCount++ > 10) {
            printf("Failed to read command MICROSTRAIN_SEND_EEPROM_VALUE!");
            return -1;
        }
    }
    
    // For whater reason there it no error checking when reading EEPROM values...
    return buffToShort(buffer+0);
}

/*
 Catch ctrl-c for cleaner exits
 */
static volatile bool keepRunning = true;
void killPanda(int killSignal) {
    keepRunning = false;
}

int main(int argc, char **argv) {
    if(argc != 4) {
        printf("Usage: %s <serial port> <address> <value>\n", argv[0]);
        return -1;
    }
    
    printf("Microstrain magnetometer hard iron calibration is running...\n");
    signal(SIGINT, killPanda);
    
    int16_t value = atoi(argv[3]);
    unsigned char address = atoi(argv[2]);
    printf("Value: %d, Address: %d\n", value, address);
    
    std::string deviceFilename = argv[1];
    // Serial code thanks to: https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/
    int serial_port = open(deviceFilename.c_str(), O_RDWR);
    if(serial_port < 0) {
        printf("Unable to open port %s\n", deviceFilename.c_str());
        return -1;
    }
    struct termios tty;
    
    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)
    
    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
    
    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
    // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)
    
    tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;
    
    // Set in/out baud rate to be 9600
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);
    // Save tty settings, also checking for error
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        return 1;
    }
    
    unsigned char buffer[100];
    int num_bytes;
    int readCount;
    
    
    
    
    tcflush(serial_port, TCIOFLUSH);
    
    
    // Per the usermanual (in ../docs) value regions only go to 134
    // Hower in testing, values are reported up to 196
    printf("Performing full 3DM-G EEPROM read:\n");
    for( int i = address; i <= address; i+=2) {
        int16_t value = readEEPROM(serial_port, i);
        printf(" |- %03d : %d", i, value);
    }
    printf("\n");
    
    unsigned char command = MICROSTRAIN_PROGRAM_EEPROM_VALUE;
    int responseCount = 2;
    
    buffer[0] = command;
    buffer[1] = 0x71;
    buffer[2] = address;
    buffer[3] = (*(unsigned short*)(&value) & 0xFF00) >> 8;  // MSB
    buffer[4] = (*(unsigned short*)(&value) & 0x00FF);  // MSB
    buffer[5] = 0xAA;
    num_bytes = 6;
    printf("Sending:");
    for(int i = 0; i < num_bytes; i++) {
        printf(" 0x%02X", buffer[i]);
    }
    printf("\n");

    write(serial_port, buffer, num_bytes);
    num_bytes = 0;
    readCount = 0;
    while(num_bytes < responseCount) {
        num_bytes += read(serial_port, buffer+num_bytes, responseCount-num_bytes);
        if (readCount++ > 10) {
            printf("Failed to issue command MICROSTRAIN_PROGRAM_EEPROM_VALUE!\n");
            close(serial_port);
            return -1;
        }
    }
    
    printf("Response: ");
    for(int i = 0; i < num_bytes; i++) {
        printf(" 0x%02X", buffer[i]);
    }
    printf("\n");
    
    
    
    
    close(serial_port);
    
    return 0;
}
