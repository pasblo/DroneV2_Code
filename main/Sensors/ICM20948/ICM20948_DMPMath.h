#ifndef ICM20948_DMP_MATH_H
#define ICM20948_DMP_MATH_H

#include "ICM20948.h"
#include <math.h>

//! Convert the \a value from float to QN value.
#define DMP_FLT_TO_FXP(value, shift) ((int32_t)  ((float)(value)*(1ULL << (shift)) + ((value>=0)-0.5f))) 

//!	Macro to convert float values from an address into QN values, and copy them to another address.
#define DMP_CONVERT_FLT_TO_FXP(fltptr, fixptr, length, shift) {int i; for(i = 0; i < (length); ++i) (fixptr)[i] = DMP_FLT_TO_FXP((fltptr)[i], shift); }

void DroneICM20948::setChipToBodyAxisQuaternion(float rotationAngle){

	//Define temporal variables
	float rot[9];
	long qcb[4], q_all[4];
	long q_adjust[4];

	//Convert mounting matrix to float
	for (uint8_t i = 0; i < 9; i++)
		rot[i] = (float) mountingMatrix[i];

	// Convert Chip to Body transformation matrix to quaternion
	rotationToQuaternion(rot, qcb);

	// The quaterion generated is the inverse, so take the inverse again.
    qcb[1] = -qcb[1];
    qcb[2] = -qcb[2];
    qcb[3] = -qcb[3];

    // Now rotate by angle, negate angle to rotate other way
    q_adjust[0] = (long)((1L<<30) * cosf(-rotationAngle*(float)M_PI/180.f/2.f));
    q_adjust[1] = 0;
    q_adjust[2] = (long)((1L<<30)*sinf(-rotationAngle*(float)M_PI/180.f/2.f));
    q_adjust[3] = 0;
    convertQuatToMultFXP(q_adjust, qcb, q_all);
    memcpy(quatChipToBody, q_all, sizeof(quatChipToBody));
}

void DroneICM20948::rotationToQuaternion(float Rcb[9], long Qcb[4]){
	float q[4];
	convertMatrixToQuatFLT(Rcb, q);
	DMP_CONVERT_FLT_TO_FXP(q, Qcb, 4, 30);
}

void DroneICM20948::convertMatrixToQuatFLT(float *R, float *q){

	float r11,r12,r13, r21,r22,r23, r31,r32,r33;

	r11 = R[0]; //assume matrix is stored row wise first, that is rot[1] is row 1, col 2
	r12 = R[1];
	r13 = R[2];

	r21 = R[3];
	r22 = R[4];
	r23 = R[5];

	r31 = R[6];
	r32 = R[7];
	r33 = R[8];

	q[0] = (1.f + r11 + r22 + r33) / 4.f;
	q[1] = (1.f + r11 - r22 - r33) / 4.f;
	q[2] = (1.f - r11 + r22 - r33) / 4.f;
	q[3] = (1.f - r11 - r22 + r33) / 4.f;

	if(q[0] < 0.0f) q[0] = 0.0f;
	if(q[1] < 0.0f) q[1] = 0.0f;
	if(q[2] < 0.0f) q[2] = 0.0f;
	if(q[3] < 0.0f) q[3] = 0.0f;
	q[0] = sqrtf(q[0]);
	q[1] = sqrtf(q[1]);
	q[2] = sqrtf(q[2]);
	q[3] = sqrtf(q[3]);

	/* Above paragraph could be reduced in :
	q[0] =(q[0] < 0.0f) ? q[0] = 0.0f : sqrtf(q[0]);
	q[1] =(q[1] < 0.0f) ? q[1] = 0.0f : sqrtf(q[1]);
	q[2] =(q[2] < 0.0f) ? q[2] = 0.0f : sqrtf(q[2]);
	q[3] =(q[3] < 0.0f) ? q[3] = 0.0f : sqrtf(q[3]);
	*/
	
	if(q[0] >= q[1] && q[0] >= q[2] && q[0] >= q[3]) //q[0] is max
	{
		 q[1] = (r23 - r32)/(4.f*q[0]);
		 q[2] = (r31 - r13)/(4.f*q[0]);
		 q[3] = (r12 - r21)/(4.f*q[0]);
	}
	else if(q[1] >= q[0] && q[1] >= q[2] && q[1] >= q[3]) //q[1] is max
	{
		 q[0] = (r23 - r32)/(4.f*q[1]);
		 q[2] = (r12 + r21)/(4.f*q[1]);
		 q[3] = (r31 + r13)/(4.f*q[1]);
	}
	else if(q[2] >= q[0] && q[2] >= q[1] && q[2] >= q[3]) //q[2] is max
	{
		 q[0] = (r31 - r13)/(4.f*q[2]);
		 q[1] = (r12 + r21)/(4.f*q[2]);
		 q[3] = (r23 + r32)/(4.f*q[2]);
	}
	else if(q[3] >= q[0] && q[3] >= q[1] && q[3] >= q[2]) //q[3] is max
	{
		 q[0] = (r12 - r21)/(4.f*q[3]);
		 q[1] = (r31 + r13)/(4.f*q[3]);
		 q[2] = (r23 + r32)/(4.f*q[3]);
	}
}

void DroneICM20948::convertQuatToMultFXP(const long *quat1_q30, const long *quat2_q30, long *quatProd_q30){
	quatProd_q30[0] = convertMultToQ30FXP(quat1_q30[0], quat2_q30[0]) - convertMultToQ30FXP(quat1_q30[1], quat2_q30[1]) -
               convertMultToQ30FXP(quat1_q30[2], quat2_q30[2]) - convertMultToQ30FXP(quat1_q30[3], quat2_q30[3]);

    quatProd_q30[1] = convertMultToQ30FXP(quat1_q30[0], quat2_q30[1]) + convertMultToQ30FXP(quat1_q30[1], quat2_q30[0]) +
               convertMultToQ30FXP(quat1_q30[2], quat2_q30[3]) - convertMultToQ30FXP(quat1_q30[3], quat2_q30[2]);

    quatProd_q30[2] = convertMultToQ30FXP(quat1_q30[0], quat2_q30[2]) - convertMultToQ30FXP(quat1_q30[1], quat2_q30[3]) +
               convertMultToQ30FXP(quat1_q30[2], quat2_q30[0]) + convertMultToQ30FXP(quat1_q30[3], quat2_q30[1]);

    quatProd_q30[3] = convertMultToQ30FXP(quat1_q30[0], quat2_q30[3]) + convertMultToQ30FXP(quat1_q30[1], quat2_q30[2]) -
               convertMultToQ30FXP(quat1_q30[2], quat2_q30[1]) + convertMultToQ30FXP(quat1_q30[3], quat2_q30[0]);
}

long DroneICM20948::convertMultToQ30FXP(long a_q30, long b_q30){
	long long temp;
	long result;
	temp = (long long)a_q30 * b_q30;
	result = (long)(temp >> 30);
	return result;
}

#endif