//-------------------------------MPU6050 Accelerometer and Gyroscope C++ library-----------------------------
//Copyright (c) 2019, Alex Mous
//Licensed under the CC BY-NC SA 4.0

//Example code

#include <MPU6050.h>

MPU6050 device(0x68, 1, true);

int main() {
	float ax, ay, az, gr, gp, gy; //Variables to store the accel, gyro and angle values
        std::cout << "0.0;0.0;0.0\n";
        std::cout.flush();
	sleep(1); //Wait for the MPU6050 to stabilize


	//Calculate the offsets
/*
	std::cout << "Calculating the offsets...\n    Please keep the accelerometer level and still\n    This could take a couple of minutes...";
	device.getOffsets(&ax, &ay, &az, &gr, &gp, &gy);
	std::cout << "Gyroscope R,P,Y: " << gr << "," << gp << "," << gy << "\nAccelerometer X,Y,Z: " << ax << "," << ay << "," << az << "\n";
*/


	//Read the current yaw angle
	device.calc_yaw = true;

	for (int i = 0; i < 120; i++) {
		device.getAngle(0, &gr);
		device.getAngle(1, &gp);
		device.getAngle(2, &gy);
		//std::cout << "Current angle around the roll axis: " << gr << "\n";
		//std::cout << "Current angle around the pitch axis: " << gp << "\n";
		//std::cout << "Current angle around the yaw axis: " << gy << "\n";
		std::cout << "" << gr << ";" << gp << ";" << gy << "\n";
                std::cout.flush();
		usleep(500000); //0.25sec
		//usleep(1000000); //1.0sec
	}

	//Get the current accelerometer values
	device.getAccel(&ax, &ay, &az);
	device.getGyro(&gr, &gp, &gy);
	//std::cout << "Accelerometer Readings: X: " << ax << ", Y: " << ay << ", Z: " << az << "\n";

	//Get the current gyroscope values
	//#std::cout << "Gyroscope Readings: X: " << gr << ", Y: " << gp << ", Z: " << gy << "\n";
	std::cout << "aX: " << ax << ", aY: " << ay << ", aZ: " << az << ", gR: " << gr << ", gP: " << gp << ", gY: " << gy << "\n";
        std::cout.flush();

	return 0;
}


