/* *
 * Copyright 1993-2012 NVIDIA Corporation.  All rights reserved.
 *
 * Please refer to the NVIDIA end user license agreement (EULA) associated
 * with this source code for terms and conditions that govern your use of
 * this software. Any use, reproduction, disclosure, or distribution of
 * this software and related documentation outside the terms of the EULA
 * is strictly prohibited.
 */
#include <stdio.h>
#include <stdlib.h>
#include "JY-901B.hpp"

//static const int WORK_SIZE = 256;

/**
 * This macro checks return value of the CUDA runtime call and exits
 * the application if the call failed.
 */
/*#define CUDA_CHECK_RETURN(value) {											\
	cudaError_t _m_cudaStat = value;										\
	if (_m_cudaStat != cudaSuccess) {										\
		fprintf(stderr, "Error %s at line %d in file %s\n",					\
				cudaGetErrorString(_m_cudaStat), __LINE__, __FILE__);		\
		exit(1);															\
	} }

__host__ __device__ unsigned int bitreverse(unsigned int number) {
	number = ((0xf0f0f0f0 & number) >> 4) | ((0x0f0f0f0f & number) << 4);
	number = ((0xcccccccc & number) >> 2) | ((0x33333333 & number) << 2);
	number = ((0xaaaaaaaa & number) >> 1) | ((0x55555555 & number) << 1);
	return number;
}
*/
/**
 * CUDA kernel function that reverses the order of bits in each element of the array.
 */
/*
__global__ void bitreverse(void *data) {
	unsigned int *idata = (unsigned int*) data;
	idata[threadIdx.x] = bitreverse(idata[threadIdx.x]);
}
*/

/**
 * Host function that prepares data array and passes it to the CUDA kernel.
 */
int main(void) {
	printf("JY901 test start!\n");
	CJY901 sensor = CJY901(0, 0x50);
	bool a = sensor.Open();
	//printf("size of this is %i byte",sizeof());

	while (a) {
		sensor.GetAngle();
		printf("x:%f, y:%f, z:%f---",
				(float) sensor.stcAngle.Angle[0] / 32768 * 180,
				(float) sensor.stcAngle.Angle[1] / 32768 * 180,
				(float) sensor.stcAngle.Angle[2] / 32768 * 180);
		sensor.GetLonLat();
		printf("L:%i D %f m L: %i D %f m", sensor.stcLonLat.lLon / 10000000,
				(double) (sensor.stcLonLat.lLon % 10000000) / 100000,
				sensor.stcLonLat.lLat / 10000000,
				(double) (sensor.stcLonLat.lLat % 10000000) / 100000);
		sensor.GetPress();
		printf("Pressure:%i latitude: %.2f\n", sensor.stcPress.lPressure,
				(float) sensor.stcPress.lAltitude / 100);
		usleep(1000);
	}
	/*
	 void *d = NULL;
	 int i;
	 unsigned int idata[WORK_SIZE], odata[WORK_SIZE];

	 for (i = 0; i < WORK_SIZE; i++)
	 idata[i] = (unsigned int) i;

	 CUDA_CHECK_RETURN(cudaMalloc((void**) &d, sizeof(int) * WORK_SIZE));
	 CUDA_CHECK_RETURN(
	 cudaMemcpy(d, idata, sizeof(int) * WORK_SIZE, cudaMemcpyHostToDevice));

	 bitreverse<<<1, WORK_SIZE, WORK_SIZE * sizeof(int)>>>(d);

	 CUDA_CHECK_RETURN(cudaThreadSynchronize());	// Wait for the GPU launched work to complete
	 CUDA_CHECK_RETURN(cudaGetLastError());
	 CUDA_CHECK_RETURN(cudaMemcpy(odata, d, sizeof(int) * WORK_SIZE, cudaMemcpyDeviceToHost));

	 for (i = 0; i < WORK_SIZE; i++)
	 printf("Input value: %u, device output: %u, host output: %u\n",
	 idata[i], odata[i], bitreverse(idata[i]));

	 CUDA_CHECK_RETURN(cudaFree((void*) d));
	 CUDA_CHECK_RETURN(cudaDeviceReset());
	 */
	return 0;
}
