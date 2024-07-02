//-------------------------------MPU6050 Accelerometer and Gyroscope C++ library-----------------------------
#include <iostream>
#include <cstdlib>
#include <stdexcept>
//Copyright (c) 2019, Alex Mous
//Licensed under the CC BY-NC SA 4.0

//Example code

#include <MPU6050.h>

#define DEFAULT_NR_SAMPLES 120

void usage(char* help_text);

MPU6050 device(0x68, 1, true);

int main(int argc, char* argv[]) {
    float ax, ay, az, gr, gp, gy; //Variables to store the accel, gyro and angle values
    int nr_samples = DEFAULT_NR_SAMPLES;

//    std::cout << "0.0;0.0;0.0\n";
    std::cout.flush();
    sleep(1); //Wait for the MPU6050 to stabilize


    // Handle commandline argument(s)
    if (argc == 2) {
        try {
            nr_samples = std::stoi(argv[1]);
        } catch( const std::invalid_argument& e ) {
            std::cerr << "Error: The argument must be an int";
            return 1;
        }  catch( const std::out_of_range& e ) {
            std::cerr << "Error: The argument is out of range for  an int" << std::endl;
            return 1;
        }
    } else if (argc == 1) {
        std::cerr << "Note: No calibration requested" << std::endl;
        nr_samples = 0;
        usage(argv[0]);
    }
    else {
        usage(argv[0]);
    }
    
    // Calculate the offsets

    if (nr_samples > 0) {
        std::cout << "Determining the offsets...\n    Please keep the accelerometer level and still\n    This could take a couple of minutes...";
        device.getOffsets(&ax, &ay, &az, &gr, &gp, &gy, nr_samples);
        std::cout << "Gyroscope Roll ,Pitch ,Yaw offsets: " << gr << "," << gp << "," << gy << "\nAccelerometer X, Y, Z offstes: " << ax << "," << ay << "," << az << "\n";
    }


    //Read the current yaw angle
    device.calc_yaw = true;

    
    // std::cout << "Reading " << nr_samples << " samples\n";
    for (int i = 0; i < nr_samples; i++) {
    	device.getAngle(0, &gr);
    	device.getAngle(1, &gp);
    	device.getAngle(2, &gy);
    	//std::cout << "Current angle around the roll axis: " << gr << "\n";
    	//std::cout << "Current angle around the pitch axis: " << gp << "\n";
    	//std::cout << "Current angle around the yaw axis: " << gy << "\n";
    	std::cout << "" << gr << ";" << gp << ";" << gy << "\n";
            //Get the current actual accelerometer values
            device.getAccel(&ax, &ay, &az);
            //Get the current actual gyroscope values
            device.getGyro(&gr, &gp, &gy);
            //std::cout << "Accelerometer Readings: X: " << ax << ", Y: " << ay << ", Z: " << az << "\n";
            //std::cout << "Gyroscope Readings: X: " << gr << ", Y: " << gp << ", Z: " << gy << "\n";
            //std::cout << "aX: " << ax << ", aY: " << ay << ", aZ: " << az << ", gR: " << gr << ", gP: " << gp << ", gY: " << gy << "\n";
            std::cout.flush();

            std::cout.flush();
            usleep(500000); //0.25sec
	    //usleep(1000000); //1.0sec
	}
	return 0;
}

void usage( char* usage_str ) {
    std::cerr << "Usage: " << usage_str << " <integer>\n" << "<integer> represents the number of samples for calibration (default: " << DEFAULT_NR_SAMPLES << ")";
}
