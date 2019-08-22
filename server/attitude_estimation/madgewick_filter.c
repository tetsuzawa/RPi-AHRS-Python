#include <stdio.h>
#include <stdlib.h>
#include <math.h>
// #define dt (float)0.01 // sampling period in seconds (shown as 10 ms)

// float *qw = 1.0, *qx = 0.0 , *qy = 0.0, *qz = 0.0;      // estimated orientation quaternion elements with initial condition
float beta = 1.0;

void filterUpdate(float wx, float wy, float wz, float ax, float ay, float az, float mx, float my, float mz, float *qw, float *qx, float *qy, float *qz, float dt);

int main(int argc, char *argv[])
{
	float wx, wy, wz, ax, ay, az, mx, my, mz;
	float qw, qx, qy, qz;
	float dt;
	wx = atof(argv[1]);
	wy = atof(argv[2]);
	wz = atof(argv[3]);
	ax = atof(argv[4]);
	ay = atof(argv[5]);
	az = atof(argv[6]);
	mx = atof(argv[7]);
	my = atof(argv[8]);
	mz = atof(argv[9]);
	qw = atof(argv[10]);
	qx = atof(argv[11]);
	qy = atof(argv[12]);
	qz = atof(argv[13]);
	dt = atof(argv[14]);

	// printf("%f\n", wx);
	// printf("%f\n", wy);
	// printf("%f\n", wz);
	// printf("%f\n", ax);
	// printf("%f\n", ay);
	// printf("%f\n", az);
	// printf("%f\n", mx);
	// printf("%f\n", my);
	// printf("%f\n", mz);
	// printf("%f\n", qw);
	// printf("%f\n", qx);
	// printf("%f\n", qy);
	// printf("%f\n", qz);
	// printf("%f\n", dt);
	for(int i=0; i < 1000; i++){
		filterUpdate(wx, wy, wz, ax, ay, az, mx, my, mz, &qw, &qx, &qy, &qz, dt);
	}

	printf("%f %f %f %f\n", qw, qx, qy, qz);
	return 0;
}

// Function to compute one filter iteration (optimized)
void filterUpdate(float wx, float wy, float wz, float ax, float ay, float az, float mx, float my, float mz, float *qw, float *qx, float *qy, float *qz, float dt)
{
	float norm;							  // norm to normalise
	float bx, bz;						  // reference direction of flux in earth frame
	float qew, qex, qey, qez;			  // q_hat_dot_epsilon
	float R11, R12, R13;				  // rotation matrix
	float R21, R22, R23;				  //
	float R31, R32, R33;				  //
	float f1, f2, f3;					  // objective function
	float a1, a2, a3, a4, a5, a6, a7, a8; // auxiliary variables

	// normalise the accelerometer measurement
	norm = sqrt(ax * ax + ay * ay + az * az);
	ax /= norm;
	ay /= norm;
	az /= norm;

	// normalise the magnetometer measurement
	norm = sqrt(mx * mx + my * my + mz * mz);
	mx /= norm;
	my /= norm;
	mz /= norm;

	// compute rotation matrix
	a1 = *qw * *qw - 0.5;
	R11 = a1 + *qx * *qx;
	R22 = a1 + *qy * *qy;
	R33 = a1 + *qz * *qz;
	a1 = *qx * *qy;
	a2 = *qw * *qz;
	R21 = a1 + a2;
	R12 = a1 - a2;
	a1 = *qx * *qz;
	a2 = *qw * *qy;
	R13 = a1 + a2;
	R31 = a1 - a2;
	a1 = *qy * *qz;
	a2 = *qw * *qx;
	R32 = a1 + a2;
	R23 = a1 - a2;
	R11 += R11;
	R12 += R12;
	R13 += R13;
	R21 += R21;
	R22 += R22;
	R23 += R23;
	R31 += R31;
	R32 += R32;
	R33 += R33;

	// rotate m to earth frame and compute b
	a1 = R11 * mx + R12 * my + R13 * mz;
	a2 = R21 * mx + R22 * my + R23 * mz;
	bx = sqrt(a1 * a1 + a2 * a2);
	bz = R31 * mx + R32 * my + R33 * mz;

	// compute J_g^T * f_g to compute qe
	f1 = R31 - ax;
	f2 = R32 - ay;
	f3 = R33 - az;
	a1 = *qx * f3;
	a2 = *qy * f3;
	qew = -*qy * f1 + *qx * f2;
	qex = *qz * f1 + *qw * f2 - a1 - a1;
	qey = -*qw * f1 + *qz * f2 - a2 - a2;
	qez = *qx * f1 + *qy * f2;

	// compute J_b^T * f_b to compute qe
	f1 = R11 * bx + R31 * bz - mx;
	f2 = R12 * bx + R32 * bz - my;
	f3 = R13 * bx + R33 * bz - mz;
	a1 = *qw * bx;
	a2 = *qx * bx;
	a3 = *qy * bx;
	a4 = *qz * bx;
	a5 = *qw * bz;
	a6 = *qx * bz;
	a7 = *qy * bz;
	a8 = *qz * bz;
	qew += -a7 * f1 + (a6 - a4) * f2 + a3 * f3;
	qex += a8 * f1 + (a3 + a5) * f2 + (a4 - a6 - a6) * f3;
	qey += (-a5 - a3 - a3) * f1 + (a2 + a8) * f2 + (a1 - a7 - a7) * f3;
	qez += (a6 - a4 - a4) * f1 + (a7 - a1) * f2 + a2 * f3;

	// normalise qe
	norm = sqrt(qew * qew + qex * qex + qey * qey + qez * qez);
	qew /= norm;
	qex /= norm;
	qey /= norm;
	qez /= norm;

	// compute omega_b
	// ジャイロのバイアスエラーを取り除く場合、ここに処理を記述

	// compute q_dot_omega
	a1 = -*qx * wx - *qy * wy - *qz * wz;
	a2 = *qw * wx - *qz * wy + *qy * wz;
	a3 = *qz * wx + *qw * wy - *qx * wz;
	a4 = -*qy * wx + *qx * wy + *qw * wz;
	a1 /= 2;
	a2 /= 2;
	a3 /= 2;
	a4 /= 2;

	// compute q_dot
	a1 -= beta * qew;
	a2 -= beta * qex;
	a3 -= beta * qey;
	a4 -= beta * qez;

	// compute q
	*qw += a1 * dt;
	*qx += a2 * dt;
	*qy += a3 * dt;
	*qz += a4 * dt;

	// normalise q
	norm = sqrt(*qw * *qw + *qx * *qx + *qy * *qy + *qz * *qz);
	*qw /= norm;
	*qx /= norm;
	*qy /= norm;
	*qz /= norm;
}
