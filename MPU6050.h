//-------------------------------MPU6050 Accelerometer and Gyroscope C++ library-----------------------------
//Copyright (c) 2019, Alex Mous
//Licensed under the CC BY-NC SA 4.0


//-----------------------MODIFY THESE PARAMETERS-----------------------

#define GYRO_RANGE 0 //Select which gyroscope range to use (see the table below) - Default is 0
//	Gyroscope Range
//	0	+/- 250 degrees/second
//	1	+/- 500 degrees/second
//	2	+/- 1000 degrees/second
//	3	+/- 2000 degrees/second
//See the MPU6000 Register Map for more information


#define ACCEL_RANGE 0 //Select which accelerometer range to use (see the table below) - Default is 0
//	Accelerometer Range
//	0	+/- 2g
//	1	+/- 4g
//	2	+/- 8g
//	3	+/- 16g
//See the MPU6000 Register Map for more information


//Offsets: now first call getOffsets to callibrate MPU6050, this should be done
// when robot is level and not moving
//Offsets - supply your own here (calculate offsets with getOffsets function)
// accData: ['aX: -1.123, aY: 0.164, aZ: 2.217, gR: 6.817, gP: -3.634, gY: 0.076']
// Calculating the offsets...
//    Please keep the accelerometer level and still
//    This could take a couple of minutes...Gyroscope R,P,Y: 151.358,-39.2457,-62.6989
//Accelerometer X,Y,Z: 1272.49,-76.8944,11362.3
//Calculating the offsets...
//    Please keep the accelerometer level and still
//    This could take a couple of minutes...Gyroscope R,P,Y: 152.498,-39.1055,-61.9359
//Accelerometer X,Y,Z: 1311.59,-73.5824,11363.6


//     Accelerometer
//#define A_OFF_X 19402
//#define A_OFF_Y -2692
//#define A_OFF_Z -8625
//    Gyroscope
//#define G_OFF_X -733
//#define G_OFF_Y 433
// For now left in offsets I got when board was still and level
//#define G_OFF_Z -75
//     Accelerometer
#define A_OFF_X 1290
#define A_OFF_Y -75
#define A_OFF_Z 11362
//    Gyroscope
// Roll / Pitch / Yaw = X Y Z
#define G_OFF_X 151
#define G_OFF_Y -39
#define G_OFF_Z -63

//Choose how many callibration loops to do, originally 10000, might take quite a while
// We might want to detect when in such a state and do new callibration untill some cleare movement is detected?
//#define CALIBRATION_LOOPS 10000
#define CALIBRATION_LOOPS 100

//-----------------------END MODIFY THESE PARAMETERS-----------------------

#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <time.h>
extern "C" {
	#include <linux/i2c-dev.h>
	#include <i2c/smbus.h>
}
#include <cmath>
#include <thread>

#define _POSIX_C_SOURCE 200809L //Used for calculating time

#define TAU 0.05 //Complementary filter percentage
#define RAD_T_DEG 57.29577951308 //Radians to degrees (180/PI)

//Select the appropriate settings
#if GYRO_RANGE == 1
	#define GYRO_SENS 65.5
	#define GYRO_CONFIG 0b00001000
#elif GYRO_RANGE == 2
	#define GYRO_SENS 32.8
	#define GYRO_CONFIG 0b00010000
#elif GYRO_RANGE == 3
	#define GYRO_SENS 16.4
	#define GYRO_CONFIG 0b00011000
#else //Otherwise, default to 0
	#define GYRO_SENS 131.0
	#define GYRO_CONFIG 0b00000000
#endif
#undef GYRO_RANGE


#if ACCEL_RANGE == 1
	#define ACCEL_SENS 8192.0
	#define ACCEL_CONFIG 0b00001000
#elif ACCEL_RANGE == 2
	#define ACCEL_SENS 4096.0
	#define ACCEL_CONFIG 0b00010000
#elif ACCEL_RANGE == 3
	#define ACCEL_SENS 2048.0
	#define ACCEL_CONFIG 0b00011000
#else //Otherwise, default to 0
	#define ACCEL_SENS 16384.0
	#define ACCEL_CONFIG 0b00000000
#endif
#undef ACCEL_RANGE




class MPU6050 {
	private:
		void _update();

		float _accel_angle[3];
		float _gyro_angle[3];
		float _angle[3]; //Store all angles (accel roll, accel pitch, accel yaw, gyro roll, gyro pitch, gyro yaw, comb roll, comb pitch comb yaw)

		float ax, ay, az, gr, gp, gy; //Temporary storage variables for actual values used in _update()
                float accel_x_off, accel_y_off, accel_z_off, gyro_r_off, gyro_p_off, gyro_y_off; // values that will be updated by getOffsets() calibration method.

		int MPU6050_addr;
		int f_dev; //Device file

		float dt; //Loop time (recalculated with each loop)

		struct timespec start,end; //Create a time structure

		bool _first_run = 1; //Variable for whether to set gyro angle to acceleration angle in compFilter
	public:
		MPU6050(int8_t addr);
		MPU6050(int8_t addr, int8_t i2c_bus_nr);
		MPU6050(int8_t addr, int8_t i2c_bus_nr, bool run_update_thread);
		MPU6050(int8_t addr, bool run_update_thread);
		void getAccelRaw(float *x, float *y, float *z);
		void getGyroRaw(float *roll, float *pitch, float *yaw);
		void getAccel(float *x, float *y, float *z);
		void getGyro(float *roll, float *pitch, float *yaw);
		void getOffsets(float *ax_off, float *ay_off, float *az_off, float *gr_off, float *gp_off, float *gy_off);
		int getAngle(int axis, float *result);
		bool calc_yaw;
};
