//-------------------------------MPU6050 Accelerometer and Gyroscope C++ library-----------------------------
//Copyright (c) 2019, Alex Mous
//Licensed under the CC BY-NC SA 4.0

//Example code

#include <MPU6050.h>

#define NR_SAMPLES 120

MPU6050 device(0x68, 1, true);

int main() {
	float ax, ay, az, gr, gp, gy; //Variables to store the accel, gyro and angle values
        std::cout << "0.0;0.0;0.0\n";
        std::cout.flush();
	sleep(1); //Wait for the MPU6050 to stabilize


	//Calculate the offsets


	std::cout << "Determining the offsets...\n    Please keep the accelerometer level and still\n    This could take a couple of minutes...";
	device.getOffsets(&ax, &ay, &az, &gr, &gp, &gy);
	std::cout << "Gyroscope Roll ,Pitch ,Yaw offsets: " << gr << "," << gp << "," << gy << "\nAccelerometer X, Y, Z offstes: " << ax << "," << ay << "," << az << "\n";

	//Read the current yaw angle
	device.calc_yaw = true;

    
	std::cout << "Reading " << NR_SAMPLES << " samples\n";
	for (int i = 0; i < NR_SAMPLES; i++) {
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


