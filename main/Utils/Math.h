#ifndef MATH_H
#define MATH_H

#define _USE_MATH_DEFINES
#include <cmath>

//TODO, gravity calculator using formula:
// - IGF = 9.780327 (1 + 0.0053024sin^2(latitude) – 0.0000058sin^2(2*latitude))
// - FAC = -3.086 x 10-6 * height
// - gravity = IGF + FAC

#define DRONE_PI  3.14159265358979323846f

typedef struct{
    double x, y, z, w;
} droneQuaternion;

typedef struct{
    double x, y, z, w;
} droneComplexQuaternion;

typedef struct{
    double roll, pitch, yaw;
} droneEulerAngles;

class DroneMath{
	public:

		// Degrees/s - Radiant/s
		double dpsToRads(double dps);
		double radsToDps(double rads);

		double calculateGravity(double latitude, double height);

		droneEulerAngles toEulerAngles(droneQuaternion q);
};
extern DroneMath DMath;
#endif