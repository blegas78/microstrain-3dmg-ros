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

#include <cmath>
#include <iostream>

#include <unistd.h>
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()


// ROS headers:
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"

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
            ROS_WARN("Failed to read command MICROSTRAIN_SEND_EEPROM_VALUE!");
            return -1;
        }
    }
    
    // For whater reason there it no error checking when reading EEPROM values...
    return buffToShort(buffer+0);
}


geometry_msgs::Quaternion quaternionMultiplication(const geometry_msgs::Quaternion& p, const geometry_msgs::Quaternion& q ) {
    geometry_msgs::Quaternion result;
    
    // Strange ordering courtesy of https://www.matec-conferences.org/articles/matecconf/pdf/2019/41/matecconf_cscc2019_01060.pdf
    result.w = p.w*q.w - p.x*q.x - p.y*q.y - p.z*q.z;
    result.x = p.w*q.x + p.x*q.w + p.y*q.z - p.z*q.y;
    result.y = p.w*q.y + p.y*q.w + p.z*q.x - p.x*q.z;
    result.z = p.w*q.z + p.z*q.w + p.x*q.y - p.y*q.x;
    
    return result;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "microstrain_reader", ros::init_options::AnonymousName);
    ROS_INFO("microstrain_reader is running...");
    
    ros::NodeHandle nh;
    
    ros::Publisher publisher = nh.advertise<sensor_msgs::Imu>("microstrain/imu", 1000);
    ros::Publisher publisherMag = nh.advertise<sensor_msgs::MagneticField>("microstrain/mag", 1000);
    
    std::string deviceFilename = "/dev/ttyUSB0";
    // Serial code thanks to: https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/
    int serial_port = open(deviceFilename.c_str(), O_RDWR);
    if(serial_port < 0) {
        ROS_ERROR("Unable to open port %s", deviceFilename.c_str());
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
    
    sensor_msgs::Imu imu;
    imu.header.frame_id = "base_link";
    imu.orientation_covariance[0 + (3*0)] = 15*3.1415926535897/180;  // data sheet says accurate to 5 degrees, but more realistically 2
    imu.orientation_covariance[1 + (3*1)] = 15*3.1415926535897/180;
    imu.orientation_covariance[2 + (3*2)] = 15*3.1415926535897/180;
    imu.angular_velocity_covariance[0 + (3*0)] = 0.1;  // complete guess
    imu.angular_velocity_covariance[1 + (3*1)] = 0.1;
    imu.angular_velocity_covariance[2 + (3*2)] = 0.1;
    imu.linear_acceleration_covariance[0 + (3*0)] = 0.2;   // Units are m/s^2.  complete guess
    imu.linear_acceleration_covariance[1 + (3*1)] = 0.2;
    imu.linear_acceleration_covariance[2 + (3*2)] = 0.2;
    
    sensor_msgs::MagneticField magneticField;
    magneticField.header.frame_id = "base_link";
    magneticField.magnetic_field_covariance[0 + (3*0)] = 0; // According to header: A covariance matrix of all zeros will be interpreted as "covariance unknown",
    
    
    
    tcflush(serial_port, TCIOFLUSH);
    
    int16_t GyroGainScale = readEEPROM(serial_port, 130);
    ROS_INFO("GyroGainScale            = %d", GyroGainScale);
    int16_t GyroStabilizationEnabled = readEEPROM(serial_port, 122);
    ROS_INFO("GyroStabilizationEnabled = %d", GyroStabilizationEnabled);
    int16_t ContinuousEnabled = readEEPROM(serial_port, 132);
    ROS_INFO("ContinuousEnabled        = %d", ContinuousEnabled);
    
    double magnetometerOrthogonality[9];
    int magOffsetX;
    int magOffsetY;
    int magOffsetZ;
    int magGainX;
    int magGainY;
    int magGainZ;
    // Per the usermanual (in ../docs) value regions only go to 134
    // Hower in testing, values are reported up to 196
    ROS_INFO("Performing full 3DM-G EEPROM read:");
    for( int i = 2; i <= 134; i+=2) {
        int16_t value = readEEPROM(serial_port, i);
        ROS_INFO(" |- %03d : %d", i, value);
        
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
    
    static int maxX = -65535, minX = 65535;
    static int maxY = -65535, minY = 65535;
    static int maxZ = -65535, minZ = 65535;
    unsigned char command;
    int responseCount;
    
    ros::Rate rate(100);
    while(ros::ok()) {
        ros::spinOnce();
        
//        command = MICROSTRAIN_SEND_RAW_SENSOR_BITS;
//        responseCount = 23;
//        buffer[0] = command;
//        write(serial_port, buffer, 1);
//        num_bytes = 0;
//        readCount = 0;
//        while(num_bytes < responseCount) {
//            num_bytes += read(serial_port, buffer+num_bytes, responseCount-num_bytes);
//            if (readCount++ > 10) {
//                ROS_WARN("Failed to read command MICROSTRAIN_SEND_RAW_SENSOR_OUPUTS!");
//                break;
//            }
//        }
//
//        if(buffer[0] == command && num_bytes == responseCount) {
//            unsigned short checksum = buffer[0];
//            for (int i = 1; i < (num_bytes-2); i += 2) {
//                //                checksum += *((unsigned short*)(buffer+i));
//                checksum += ((unsigned short)buffer[i]) << 8;
//                checksum += ((unsigned short)buffer[i+1]);
//            }
//            unsigned short checksumSent = (((unsigned short)buffer[num_bytes-2]) << 8) + buffer[num_bytes-1];
//            //            printf("Checksum: %04X vs %04X\n", checksum, checksumSent);
//            if(checksumSent == checksum) {
//                imu.header.stamp = ros::Time::now();
//                magneticField.header.stamp = ros::Time::now();
//
//                
//                
//                int magX = buffToShort(buffer+1);
//                int magY = buffToShort(buffer+3);
//                int magZ = buffToShort(buffer+5);
//
//                if(magX > maxX) {
//                    maxX = magX;
//                    ROS_INFO("New maxX: %d", maxX);
//                }
//                if(magX < minX) {
//                    minX = magX;
//                    ROS_INFO("New minX: %d", minX);
//                }
//                
//                if(magY > maxY) {
//                    maxY = magY;
//                    ROS_INFO("New maxY: %d", maxY);
//                }
//                if(magY < minY) {
//                    minY = magY;
//                    ROS_INFO("New minY: %d", minY);
//                }
//                
//                if(magZ > maxZ) {
//                    maxZ = magZ;
//                    ROS_INFO("New maxZ: %d", maxZ);
//                }
//                if(magZ < minZ) {
//                    minZ = magZ;
//                    ROS_INFO("New minZ: %d", minZ);
//                }
//                
//                ROS_INFO("");
//                ROS_INFO("Bare:    X: %d \tY: %d \tZ: %d", magX, magY, magZ);
//                ROS_INFO("X: %d-%d\tY: %d-%d\tZ: %d-%d", minX, maxX, minY, maxY, minZ, maxZ);
//                double offsetX = (double)(maxX+minX)/2.0;
//                double offsetY = (double)(maxY+minY)/2.0;
//                double offsetZ = (double)(maxZ+minZ)/2.0;
//                ROS_INFO("w/offset: X: %.01f\tY: %.01f\tZ: %.01f", magX-offsetX, magY-offsetY, magZ-offsetZ);
//                
//                double scaleX = (double)(maxX-minX)/2.0;
//                double scaleY = (double)(maxY-minY)/2.0;
//                double scaleZ = (double)(maxZ-minZ)/2.0;
//                
//                ROS_INFO("w/EFU   : X: %.03f \tY: %.03f \tZ: %.03f ", (magX-offsetX)/scaleX, (magY-offsetY)/scaleY, (magZ-offsetZ)/scaleZ);
////                            publisher.publish(imu);
////                            publisherMag.publish(magneticField);
//                
//                
//                double orthoX = magnetometerOrthogonality[0+(3*0)]*(magX-offsetX)/scaleX + magnetometerOrthogonality[0+(3*1)]*(magY-offsetY)/scaleY + magnetometerOrthogonality[0+(3*2)]*(magZ-offsetZ)/scaleZ;
//                double orthoY = magnetometerOrthogonality[1+(3*0)]*(magX-offsetX)/scaleX + magnetometerOrthogonality[1+(3*1)]*(magY-offsetY)/scaleY + magnetometerOrthogonality[1+(3*2)]*(magZ-offsetZ)/scaleZ;
//                double orthoZ = magnetometerOrthogonality[2+(3*0)]*(magX-offsetX)/scaleX + magnetometerOrthogonality[2+(3*1)]*(magY-offsetY)/scaleY + magnetometerOrthogonality[2+(3*2)]*(magZ-offsetZ)/scaleZ;
//                ROS_INFO("w/Ortho  : X: %.03f \tY: %.03f \tZ: %.03f ", orthoX, orthoY, orthoZ);
//                
//                offsetX = magOffsetX;
//                offsetY = magOffsetY;
//                offsetZ = magOffsetZ;
//                scaleX = magGainX;
//                scaleY = magGainY;
//                scaleZ = magGainZ;
//                orthoX = magnetometerOrthogonality[0+(3*0)]*(magX-offsetX)/scaleX + magnetometerOrthogonality[0+(3*1)]*(magY-offsetY)/scaleY + magnetometerOrthogonality[0+(3*2)]*(magZ-offsetZ)/scaleZ;
//                orthoY = magnetometerOrthogonality[1+(3*0)]*(magX-offsetX)/scaleX + magnetometerOrthogonality[1+(3*1)]*(magY-offsetY)/scaleY + magnetometerOrthogonality[1+(3*2)]*(magZ-offsetZ)/scaleZ;
//                orthoZ = magnetometerOrthogonality[2+(3*0)]*(magX-offsetX)/scaleX + magnetometerOrthogonality[2+(3*1)]*(magY-offsetY)/scaleY + magnetometerOrthogonality[2+(3*2)]*(magZ-offsetZ)/scaleZ;
//                ROS_INFO("w/Model  : X: %.03f \tY: %.03f \tZ: %.03f ", orthoX, orthoY, orthoZ);
//            } else {
//                ROS_WARN("Bad checksum...");
//
//            }
//        } else {
//            ROS_WARN("Bad return message...");
//
//        }
//        
//        
//        command = MICROSTRAIN_SEND_INSTANTANEOUS_VECTORS;
//        responseCount = 23;
//        buffer[0] = command;
//        write(serial_port, buffer, 1);
//        num_bytes = 0;
//        readCount = 0;
//        while(num_bytes < responseCount) {
//            num_bytes += read(serial_port, buffer+num_bytes, responseCount-num_bytes);
//            if (readCount++ > 10) {
//                ROS_WARN("Failed to read command MICROSTRAIN_SEND_RAW_SENSOR_OUPUTS!");
//                break;
//            }
//        }
//
//        if(buffer[0] == command && num_bytes == responseCount) {
//            unsigned short checksum = buffer[0];
//            for (int i = 1; i < (num_bytes-2); i += 2) {
//                //                checksum += *((unsigned short*)(buffer+i));
//                checksum += ((unsigned short)buffer[i]) << 8;
//                checksum += ((unsigned short)buffer[i+1]);
//            }
//            unsigned short checksumSent = (((unsigned short)buffer[num_bytes-2]) << 8) + buffer[num_bytes-1];
//            //            printf("Checksum: %04X vs %04X\n", checksum, checksumSent);
//            if(checksumSent == checksum) {
//                imu.header.stamp = ros::Time::now();
//                magneticField.header.stamp = ros::Time::now();
//
//                
//                
//                double magX = buffToShort(buffer+1);
//                double magY = buffToShort(buffer+3);
//                double magZ = buffToShort(buffer+5);
//                magX /= 8192;
//                magY /= 8192;
//                magZ /= 8192;
//
//                ROS_INFO("w/Device : X: %.03f \tY: %.03f \tZ: %.03f ", magX, magY, magZ);
//            } else {
//                ROS_WARN("Bad checksum...");
//
//            }
//        } else {
//            ROS_WARN("Bad return message...");
//
//        }
//        
        

    
    
        //        if(0) {
//                    buffer[0] = MICROSTRAIN_SEND_GYRO_STABILIZED_VECTORS;
//                    write(serial_port, buffer, 1);
//                    num_bytes = 0;
//                    readCount = 0;
//                    while(num_bytes < 23) {
//                        num_bytes += read(serial_port, buffer+num_bytes, 23-num_bytes);
//                        if (readCount++ > 10) {
//                            ROS_WARN("Failed to read command MICROSTRAIN_SEND_GYRO_STABILIZED_VECTORS!");
//                            break;
//                        }
//                    }
//
//
//                    //        printf("Vectors: ");
//                    //        for(int i = 0; i < num_bytes; i++) {
//                    //            printf("0x%02X ", buffer[i]);
//                    //        }
//                    //        printf("\n");
//
//                    if(buffer[0] == MICROSTRAIN_SEND_GYRO_STABILIZED_VECTORS && num_bytes == 23) {
//                        unsigned short checksum = buffer[0];
//                        for (int i = 1; i < (num_bytes-2); i += 2) {
//                            //                checksum += *((unsigned short*)(buffer+i));
//                            checksum += ((unsigned short)buffer[i]) << 8;
//                            checksum += ((unsigned short)buffer[i+1]);
//                        }
//                        unsigned short checksumSent = (((unsigned short)buffer[num_bytes-2]) << 8) + buffer[num_bytes-1];
//                        //            printf("Checksum: %04X vs %04X\n", checksum, checksumSent);
//                        if(checksumSent == checksum) {
//                            imu.header.stamp = ros::Time::now();
//                            magneticField.header.stamp = ros::Time::now();
//
//                            double magX = buffToShort(buffer+1);
//                            double magY = buffToShort(buffer+3);
//                            double magZ = buffToShort(buffer+5);
//                            imu.linear_acceleration.x = buffToShort(buffer+7);
//                            imu.linear_acceleration.y = buffToShort(buffer+9);
//                            imu.linear_acceleration.z = buffToShort(buffer+11);
//                            imu.angular_velocity.x = buffToShort(buffer+13);
//                            imu.angular_velocity.y = buffToShort(buffer+15);
//                            imu.angular_velocity.z = buffToShort(buffer+17);
//                            int timerTicks = buffToShort(buffer+19);
//                            double timeInSeconds = 0.0065536 * (double)timerTicks;
//
//                            magX /= 8192;   // Earth field units
//                            magY /= 8192;
//                            magZ /= 8192;
//
//                            imu.linear_acceleration.x /= 8192;  // G
//                            imu.linear_acceleration.y /= 8192;
//                            imu.linear_acceleration.z /= 8192;
//
//                            double GyroGainScale = 64;
//                            imu.angular_velocity.x /= (8192*0.0065536*GyroGainScale);   // rad/s
//                            imu.angular_velocity.y /= (8192*0.0065536*GyroGainScale);
//                            imu.angular_velocity.z /= (8192*0.0065536*GyroGainScale);
//
//                            //                ROS_INFO("\t time: %.02f", timeInSeconds);
//                            //                ROS_INFO("\tMag  : %.02f\t%.02f\t%.02f", magX, magY, magZ);
//                            //                ROS_INFO("\tAccel: %.02f\t%.02f\t%.02f", imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z);
//                            //                ROS_INFO("\tGyro : %.02f\t%.02f\t%.02f", imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z);
//
////                            publisher.publish(imu);
////                            publisherMag.publish(magneticField);
//
//
//                        } else {
//                            ROS_WARN("Bad checksum...");
//
//                        }
//                    } else {
//                        ROS_WARN("Bad return message...");
//
//                    }
//
//                }
        
        //        tcflush(serial_port, TCIOFLUSH);
//                if(0) {
//                    buffer[0] = MICROSTRAIN_SEND_GYRO_STABILIZED_QUTERNION;
//                    write(serial_port, buffer, 1);
//                    num_bytes = 0;
//                    readCount = 0;
//                    while(num_bytes < 13) {
//                        num_bytes += read(serial_port, buffer+num_bytes, 13-num_bytes);
//                        if (readCount++ > 10) {
//                            ROS_WARN("Failed to read command MICROSTRAIN_SEND_GYRO_STABILIZED_QUTERNION!");
//                            break;
//                        }
//                    }
//
//
//                    //        printf("Quats: ");
//                    //        for(int i = 0; i < num_bytes; i++) {
//                    //            printf("0x%02X ", buffer[i]);
//                    //        }
//                    //        printf("\n");
//
//                    if(buffer[0] == MICROSTRAIN_SEND_GYRO_STABILIZED_QUTERNION && num_bytes == 13) {
//                        unsigned short checksum = buffer[0];
//                        for (int i = 1; i < (num_bytes-2); i += 2) {
//                            //                checksum += *((unsigned short*)(buffer+i));
//                            checksum += ((unsigned short)buffer[i]) << 8;
//                            checksum += ((unsigned short)buffer[i+1]);
//                        }
//                        unsigned short checksumSent = (((unsigned short)buffer[num_bytes-2]) << 8) + buffer[num_bytes-1];
//                        //            printf("Checksum: %04X vs %04X\n", checksum, checksumSent);
//                        if(checksumSent == checksum) {
//    imu.header.stamp = ros::Time::now();
//    magneticField.header.stamp = ros::Time::now();
//                            imu.orientation.x = buffToShort(buffer+1);
//                            imu.orientation.y = buffToShort(buffer+3);
//                            imu.orientation.z = buffToShort(buffer+5);
//                            imu.orientation.w = buffToShort(buffer+7);
//                            imu.orientation.x /= 8192;
//                            imu.orientation.y /= 8192;
//                            imu.orientation.z /= 8192;
//                            imu.orientation.w /= 8192;
//
//
//
//                            publisher.publish(imu);
//                        } else {
//                            ROS_WARN("Bad checksum...");
//
//                        }
//                    } else {
//                        ROS_WARN("Bad return message...");
//
//                    }
        //        }
        
        command = MICROSTRAIN_SEND_GYRO_STABILIZED_QUATERNION_AND_VECTORS;   // Send Gyro-Stabilized Quaternion with Vectors
        responseCount = 31;
        buffer[0] = command;
        write(serial_port, buffer, 1);
        num_bytes = 0;
        readCount = 0;
        while(num_bytes < responseCount) {
            num_bytes += read(serial_port, buffer+num_bytes, responseCount-num_bytes);
            if (readCount++ > 10) {
                ROS_WARN("Failed to read command 0x02!");
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
                imu.header.stamp = ros::Time::now();
                magneticField.header.stamp = ros::Time::now();
                
                imu.orientation.w = buffToShort(buffer+1);
                imu.orientation.x = buffToShort(buffer+3);
                imu.orientation.y = -buffToShort(buffer+5);
                imu.orientation.z = -buffToShort(buffer+7);
                imu.orientation.w /= 8192;
                imu.orientation.x /= 8192;
                imu.orientation.y /= 8192;
                imu.orientation.z /= 8192;
                
                geometry_msgs::Quaternion zRotation;
                zRotation.w = 0.7071067812;
                zRotation.x = 0.;
                zRotation.y = 0.;
                zRotation.z = 0.7071067812;
                imu.orientation = quaternionMultiplication(zRotation, imu.orientation);
                
                
                // https://www.kjmagnetics.com/globe.asp
                magneticField.magnetic_field.x = buffToShort(buffer+9);
                magneticField.magnetic_field.y = buffToShort(buffer+11);
                magneticField.magnetic_field.z = buffToShort(buffer+13);
                magneticField.magnetic_field.x /= 8192;   // Earth field units
                magneticField.magnetic_field.y /= 8192;
                magneticField.magnetic_field.z /= 8192;
                magneticField.magnetic_field.x *= 0.00005; // T  (average 50uT for Earth in US)
                magneticField.magnetic_field.y *= 0.00005; // Not accurate, nromalized based on factory calibration measuring Earth
                magneticField.magnetic_field.z *= 0.00005; //

                
                imu.linear_acceleration.x = buffToShort(buffer+15);
                imu.linear_acceleration.y = -buffToShort(buffer+17);
                imu.linear_acceleration.z = -buffToShort(buffer+19);
                imu.linear_acceleration.x /= 8192;  // G
                imu.linear_acceleration.y /= 8192;
                imu.linear_acceleration.z /= 8192;
                imu.linear_acceleration.x *= 9.81;  // m/s^2
                imu.linear_acceleration.y *= 9.81;
                imu.linear_acceleration.z *= 9.81;
                
                
                
                //                    double GyroGainScale = 64;
                imu.angular_velocity.x = buffToShort(buffer+21);
                imu.angular_velocity.y = -buffToShort(buffer+23);
                imu.angular_velocity.z = -buffToShort(buffer+25);
                imu.angular_velocity.x /= (8192*0.0065536*(double)GyroGainScale);   // rad/s
                imu.angular_velocity.y /= (8192*0.0065536*(double)GyroGainScale);
                imu.angular_velocity.z /= (8192*0.0065536*(double)GyroGainScale);
                
                
                int timerTicks = buffToShort(buffer+27);
                double timeInSeconds = 0.0065536 * (double)timerTicks;
                
                
                publisher.publish(imu);
                publisherMag.publish(magneticField);
            } else {
                ROS_WARN("Bad checksum...");
            }
        } else {
            ROS_WARN("Bad return message...");
            
        }
        //        ROS_WARN("On to the next one..");
        
        //        rate.sleep();
        //
        //        buffer[0] = 0xF0;
        //        write(serial_port, buffer, 1);
        //        num_bytes = read(serial_port, &buffer, 5);
        //
        //        printf("Firmware: ");
        //        for(int i = 0; i < num_bytes; i++) {
        //            printf("0x%02X ", buffer[i]);
        //        }
        //        printf("\n");
        
        //        rate.sleep();
        
    }
    
    
    
    close(serial_port);
    
    return 0;
}
