#include "Math.h"
#define _USE_MATH_DEFINES
#include <cmath>

DroneMath DMath;

double DroneMath::dpsToRads(double dps){
	return dps*0.01745329252;
}

double DroneMath::radsToDps(double rads){
	return rads*57.29577951;
}

double DroneMath::calculateGravity(double latitude, double height){

	//Simplified internation gravity formula
	double SIFG = 9.780327 * (1 + 0.0053024 * pow(sin(latitude), 2) - 0.0000058 * pow(sin(latitude), 2));

	//Free air correction formula
	double FAC = -0.000003086 * height;

	//Gravity integration
	return SIFG + FAC;
}

droneEulerAngles DroneMath::toEulerAngles(droneQuaternion q) { //TODO, Dronify
    droneEulerAngles angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
        angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}