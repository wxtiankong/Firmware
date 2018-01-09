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
	CJY901 sensor = CJY901(1, 0x50);
	bool a = sensor.Open();
	//printf("size of this is %i byte",sizeof());

	while (a) {
		
		sensor.GetAngle();
		usleep(10);
		printf("角度 x:%f, y:%f, z:%f---",
				(float) sensor.stcAngle.Angle[0] / 32768 * 180,
				(float) sensor.stcAngle.Angle[1] / 32768 * 180,
				(float) sensor.stcAngle.Angle[2] / 32768 * 180);
		
		sensor.GetAcc();//输出加速度
		usleep(10);
		printf("加速度 x:%f, y:%f, z:%f---",
				(float) sensor.stcAcc.a[0] / 32768 * 16,
				(float) sensor.stcAcc.a[1] / 32768 * 16,
				(float) sensor.stcAcc.a[2] / 32768 * 16);
		
		sensor.GetGyro();//输出角速度
		usleep(10);
		printf("角速度 :%.3f %.3f %.3f\r\n",
			(float)sensor.stcGyro.w[0]/32768*2000,
			(float)sensor.stcGyro.w[1]/32768*2000,
			(float)sensor.stcGyro.w[2]/32768*2000);

		
		sensor.GetMag();//输出磁场
		usleep(10);
		printf("磁场:%d %d %d\r\n",sensor.stcMag.h[0],sensor.stcMag.h[1],sensor.stcMag.h[2]);
			 	
		
		sensor.GetPress();//输出气压、高度
		usleep(10);
		printf("气压、高度:%ld Height%.2f\r\n",sensor.stcPress.lPressure,(float)sensor.stcPress.lAltitude/100);
			  
		sensor.GetDStatus();//输出端口状态
		usleep(10);
		printf("端口状态:%d %d %d %d\r\n",
			sensor.stcDStatus.sDStatus[0],
			sensor.stcDStatus.sDStatus[1],
			sensor.stcDStatus.sDStatus[2],
			sensor.stcDStatus.sDStatus[3]);
		
		sensor.GetLonLat();//输出经纬度
		usleep(10);
		printf("经纬度 Longitude:%ldDeg%.5fm Lattitude:%ldDeg%.5fm\r\n",sensor.stcLonLat.lLon/10000000,(double)(sensor.stcLonLat.lLon % 10000000)/1e5,sensor.stcLonLat.lLat/10000000,(double)(sensor.stcLonLat.lLat % 10000000)/1e5);
			 
		
		sensor.GetGPSV();//输出地速 
		usleep(10); 
		printf("地速 GPSHeight:%.1fm GPSYaw:%.1fDeg GPSV:%.3fkm/h\r\n",(float)sensor.stcGPSV.sGPSHeight/10,(float)sensor.stcGPSV.sGPSYaw/10,(float)sensor.stcGPSV.lGPSVelocity/1000);
			 
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
