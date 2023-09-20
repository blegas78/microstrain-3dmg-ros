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

#include <ncurses.h>
#include <curses-gfx.h>
#include <curses-gfx-3d.h>

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


void cleanupConsole() {
    clear();
    endwin();

    std::cout << "Console has been cleaned!" << std::endl;
}

void setupTerminal()
{
    
    setlocale(LC_ALL, "");
    
    // Start up Curses window
    initscr();
    cbreak();
    noecho();
    nodelay(stdscr, 1);    // Don't wait at the getch() function if the user hasn't hit a key
    keypad(stdscr, 1); // Allow Function key input and arrow key input

    start_color();
    init_pair(1, COLOR_RED, COLOR_BLACK);
    init_pair(2, COLOR_YELLOW, COLOR_BLACK);
    init_pair(3, COLOR_GREEN, COLOR_BLACK);
    init_pair(4, COLOR_CYAN, COLOR_BLACK);
    init_pair(5, COLOR_BLUE, COLOR_BLACK);
    init_pair(6, COLOR_MAGENTA, COLOR_BLACK);
    init_pair(7, COLOR_WHITE, COLOR_BLACK);

    curs_set(0);    // no cursor

}

void drawRadial(double y, double x, Coordinates2D center, int width, int height, double* data, int count, unsigned char color) {
    Coordinates2D point1 = center;
    Coordinates2D point2 = {.x = 25, .y=0};
    attron(COLOR_PAIR(color));
    for (int i = 0; i < count; i++) {
        drawDotFloat(center.x + ((double)width)*data[i]*sin((double)i/(double)count *2.0*M_PI) + 0.5,
                     center.y - ((double)height)*data[i]*cos((double)i/(double)count *2.0*M_PI) + 0.5);
    }
    attroff(COLOR_PAIR(color));
    
    point1 = center;
    attron(A_BOLD);
    point2.x = point1.x + width*x + 0.5;
    point2.y = point1.y - height*y + 0.5;
    ln2(point1, point2);
    attroff(A_BOLD);
}

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
        if(num_bytes != responseCount) {
//                mvprintw(line++, 0, "Failed read...");
            usleep(10000);
        }
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
    if(argc != 2) {
        printf("Usage: %s <serial port>\n", argv[0]);
        return -1;
    }
    
    printf("Microstrain magnetometer hard iron calibration is running...\n");
    signal(SIGINT, killPanda);
    
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
    
    tty.c_cc[VTIME] = 0;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
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
    
    int16_t GyroGainScale = readEEPROM(serial_port, 130);
    printf("GyroGainScale            = %d\n", GyroGainScale);
    int16_t GyroStabilizationEnabled = readEEPROM(serial_port, 122);
    printf("GyroStabilizationEnabled = %d\n", GyroStabilizationEnabled);
    int16_t ContinuousEnabled = readEEPROM(serial_port, 132);
    printf("ContinuousEnabled        = %d\n", ContinuousEnabled);
    
    double magnetometerOrthogonality[9];
    int magOffsetX;
    int magOffsetY;
    int magOffsetZ;
    int magGainX;
    int magGainY;
    int magGainZ;
    // Per the usermanual (in ../docs) value regions only go to 134
    // Hower in testing, values are reported up to 196
    printf("Performing full 3DM-G EEPROM read:\n");
    for( int i = 2; i <= 134; i+=2) {
        int16_t value = readEEPROM(serial_port, i);
        printf(" |- %03d : %d\n", i, value);
        
        if(i >= 66 && i <= 82) {
            magnetometerOrthogonality[(i-66)/2] = ((double)value)/8192; // COLUMN MAJOR
        }
        
        if(i == 8) { magOffsetX = value;
        } else if (i == 10) { magOffsetY = value;
        } else if (i == 12) { magOffsetZ = value;
        } else if (i == 20) { magGainX = value;
        } else if (i == 22) { magGainY = value;
        } else if (i == 24) { magGainZ = value;
        }
    }
    printf("\n");
    
    static int maxX = -65535, minX = 65535;
    static int maxY = -65535, minY = 65535;
    static int maxZ = -65535, minZ = 65535;
    
    double offsetX;
    double offsetY;
    double offsetZ;
    
    double scaleX;
    double scaleY;
    double scaleZ;
    
    
    // plotting variables:
    int numSamples = 1080;
    double xyMag[numSamples];
    double yzMag[numSamples];
    double zxMag[numSamples];
    for (int i = 0; i < numSamples; i++) {
        xyMag[i] = 0.5;
        yzMag[i] = 0.5;
        zxMag[i] = 0.5;
    }
    
    setupTerminal();
    int screenSizeX, screenSizeY;
    getmaxyx(stdscr, screenSizeY, screenSizeX);
    while(keepRunning == true) {
        int line = 0;
        erase();
        
        mvprintw(line++, 0, "Hello calibration...");
        
        tcflush(serial_port, TCIOFLUSH);
        unsigned char command = MICROSTRAIN_SEND_RAW_SENSOR_BITS;
        int responseCount = 23;
        buffer[0] = command;
        write(serial_port, buffer, 1);
        num_bytes = 0;
        readCount = 0;
        while(num_bytes < responseCount) {
            num_bytes += read(serial_port, buffer+num_bytes, responseCount-num_bytes);
            if(num_bytes != responseCount) {
//                mvprintw(line++, 0, "Failed read...");
                usleep(1000);
            }
            if (readCount++ > 50) {
//                printf("Failed to read command MICROSTRAIN_SEND_RAW_SENSOR_OUPUTS!\n");
                mvprintw(line++, 0, "Failed command.");
                break;
            }
        }

        if(buffer[0] == command && num_bytes == responseCount) {
            unsigned short checksum = buffer[0];
            for (int i = 1; i < (num_bytes-2); i += 2) {
                //                checksum += *((unsigned short*)(buffer+i));
                checksum += ((unsigned short)buffer[i]) << 8;
                checksum += ((unsigned short)buffer[i+1]);
            }
            unsigned short checksumSent = (((unsigned short)buffer[num_bytes-2]) << 8) + buffer[num_bytes-1];
            //            printf("Checksum: %04X vs %04X\n", checksum, checksumSent);
            if(checksumSent == checksum) {

                
                
                int magX = buffToShort(buffer+1);
                int magY = buffToShort(buffer+3);
                int magZ = buffToShort(buffer+5);

                if(magX > maxX) {
                    maxX = magX;
//                    printf("New maxX: %d\n", maxX);
                }
                if(magX < minX) {
                    minX = magX;
//                    printf("New minX: %d\n", minX);
                }
                
                if(magY > maxY) {
                    maxY = magY;
//                    printf("New maxY: %d\n", maxY);
                }
                if(magY < minY) {
                    minY = magY;
//                    printf("New minY: %d\n", minY);
                }
                
                if(magZ > maxZ) {
                    maxZ = magZ;
//                    printf("New maxZ: %d\n", maxZ);
                }
                if(magZ < minZ) {
                    minZ = magZ;
//                    printf("New minZ: %d\n", minZ);
                }
                
//                printf("");mvprintw(line++, 0, "Hello calibration...");
                mvprintw(line++, 0, "Bare:    X: %d   \tY: %d \tZ: %d\n", magX, magY, magZ);
                mvprintw(line++, 0, "Range:   X: %d-%d\tY: %d-%d\tZ: %d-%d\n", minX, maxX, minY, maxY, minZ, maxZ);
                offsetX = (double)(maxX+minX)/2.0;
                offsetY = (double)(maxY+minY)/2.0;
                offsetZ = (double)(maxZ+minZ)/2.0;
                mvprintw(line++, 0, "w/offset X: %.01f  \tY: %.01f  \tZ: %.01f\n", magX-offsetX, magY-offsetY, magZ-offsetZ);
                
                scaleX = (double)(maxX-minX)/2.0;
                scaleY = (double)(maxY-minY)/2.0;
                scaleZ = (double)(maxZ-minZ)/2.0;
                
                mvprintw(line++, 0, "w/EFU:   X: %.03f \tY: %.03f \tZ: %.03f \n", (magX-offsetX)/scaleX, (magY-offsetY)/scaleY, (magZ-offsetZ)/scaleZ);
//                            publisher.publish(imu);
//                            publisherMag.publish(magneticField);
                
                
                double orthoX = magnetometerOrthogonality[0+(3*0)]*(magX-offsetX)/scaleX + magnetometerOrthogonality[0+(3*1)]*(magY-offsetY)/scaleY + magnetometerOrthogonality[0+(3*2)]*(magZ-offsetZ)/scaleZ;
                double orthoY = magnetometerOrthogonality[1+(3*0)]*(magX-offsetX)/scaleX + magnetometerOrthogonality[1+(3*1)]*(magY-offsetY)/scaleY + magnetometerOrthogonality[1+(3*2)]*(magZ-offsetZ)/scaleZ;
                double orthoZ = magnetometerOrthogonality[2+(3*0)]*(magX-offsetX)/scaleX + magnetometerOrthogonality[2+(3*1)]*(magY-offsetY)/scaleY + magnetometerOrthogonality[2+(3*2)]*(magZ-offsetZ)/scaleZ;
                mvprintw(line++, 0, "w/Ortho: X: %.03f \tY: %.03f \tZ: %.03f\n", orthoX, orthoY, orthoZ);
                
//                offsetX = magOffsetX;
//                offsetY = magOffsetY;
//                offsetZ = magOffsetZ;
//                scaleX = magGainX;
//                scaleY = magGainY;
//                scaleZ = magGainZ;
//                orthoX = magnetometerOrthogonality[0+(3*0)]*(magX-offsetX)/scaleX + magnetometerOrthogonality[0+(3*1)]*(magY-offsetY)/scaleY + magnetometerOrthogonality[0+(3*2)]*(magZ-offsetZ)/scaleZ;
//                orthoY = magnetometerOrthogonality[1+(3*0)]*(magX-offsetX)/scaleX + magnetometerOrthogonality[1+(3*1)]*(magY-offsetY)/scaleY + magnetometerOrthogonality[1+(3*2)]*(magZ-offsetZ)/scaleZ;
//                orthoZ = magnetometerOrthogonality[2+(3*0)]*(magX-offsetX)/scaleX + magnetometerOrthogonality[2+(3*1)]*(magY-offsetY)/scaleY + magnetometerOrthogonality[2+(3*2)]*(magZ-offsetZ)/scaleZ;
//                printf("w/Model  : X: %.03f \tY: %.03f \tZ: %.03f\n", orthoX, orthoY, orthoZ);
                
                
                
                double characterAspect = 28.0/12.0;
                
                double width = 30;
                if (screenSizeX < 3.0*screenSizeY*characterAspect) {
                    width = screenSizeX/(3.0*2);    // radial width
                } else {
                    width = screenSizeY/(characterAspect*2.0);
                }
                double height = width/characterAspect;
                Coordinates2D center;
                center.x = width;
                center.y = screenSizeY/2;
                
                double x = ((double)magX-(double)magOffsetX)/(double)magGainX;
                double y = ((double)magY-(double)magOffsetY)/(double)magGainY;
                double z = ((double)magZ-(double)magOffsetZ)/(double)magGainZ;
                int index = floor(atan2(y,x) * numSamples/(2.*3.1415926535897));
                if(index < 0) {
                    index += numSamples;
                }
                if(index >= numSamples) {
                    index -= numSamples;
                }
                double magnitude = sqrt(x*x + y*y);
                if(xyMag[index] < magnitude) {
                    xyMag[index] = magnitude;
                }
                index = floor(atan2(z,y) * numSamples/(2.*3.1415926535897));
                if(index < 0) {
                    index += numSamples;
                }
                if(index >= numSamples) {
                    index -= numSamples;
                }
                magnitude = sqrt(y*y + z*z);
                if(yzMag[index] < magnitude) {
                    yzMag[index] = magnitude;
                }
                index = floor(atan2(x,z) * numSamples/(2.*3.1415926535897));
                if(index < 0) {
                    index += numSamples;
                }
                if(index >= numSamples) {
                    index -= numSamples;
                }
                magnitude = sqrt(z*z + x*x);
                if(zxMag[index] < magnitude) {
                    zxMag[index] = magnitude;
                }
                
                drawRadial(x, y, center, width, height, xyMag, numSamples, 2);
                mvprintw(center.y, center.x, "XY");
                center.x = width + width*2;
                center.y = screenSizeY/2;
                drawRadial(y, z, center, width, height, yzMag, numSamples, 4);
                mvprintw(center.y, center.x, "YZ");
                center.x = width + width*4;
                center.y = screenSizeY/2;
                drawRadial(z, x, center, width, height, zxMag, numSamples, 6);
                mvprintw(center.y, center.x, "ZX");
                
            } else {
                mvprintw(line++, 0, "Bad checksum...\n");

            }
        } else {
            mvprintw(line++, 0, "Bad return message...\n");

        }
        
        
        
        
        int ch;
        if ((ch = getch()) == 0x1B) {    // Escape
            keepRunning = false;
        } else if (ch == KEY_RESIZE) {
            getmaxyx(stdscr, screenSizeY, screenSizeX);
            usleep(1000000);
        }
    }
    
    cleanupConsole();
    
    
    printf("Results:\n");
    printf(" - Offset X: %d\tCurrent: %d\n", (int)offsetX, magOffsetX);
    printf(" - Offset Y: %d\tCurrent: %d\n", (int)offsetY, magOffsetY);
    printf(" - Offset Z: %d\tCurrent: %d\n", (int)offsetZ, magOffsetZ);
    printf(" - Gain X  : %d\tCurrent: %d\n", (int)scaleX, magGainX);
    printf(" - Gain Y  : %d\tCurrent: %d\n", (int)scaleY, magGainY);
    printf(" - Gain Z  : %d\tCurrent: %d\n", (int)scaleZ, magGainZ);
    
    
    close(serial_port);
    
    return 0;
}
