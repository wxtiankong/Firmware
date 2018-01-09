************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file yj901b.cpp
 *
 * Driver for the yj901b.
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_getopt.h>

#include <drivers/device/i2c.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <vector>

#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_range_finder.h>
#include <drivers/device/ringbuffer.h>

#include <uORB/uORB.h> 
#include <uORB/topics/sensor_combined.h>
#include <board_config.h>
 
 
#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif
  
extern "C" __EXPORT int yj901b_main(int argc, char *argv[]); 
 
#include "JY-901B.hpp"
 
int yj901b_main(void) { 
	PX4_INFO("JY901 test start!\n");
	CJY901 sensor = CJY901(1, 0x50);
	bool a = sensor.Open(); 

	sensor_combined data;
	while (a) {
		
		sensor.GetAngle();
		usleep(10);
		printf("角度 x:%f, y:%f, z:%f---",
				(float) sensor.stcAngle.Angle[0] / 32768 * 180,
				(float) sensor.stcAngle.Angle[1] / 32768 * 180,
				(float) sensor.stcAngle.Angle[2] / 32768 * 180);
		orb_publish(ORB_ID(gps_dump), _dump_communication_pub, dump_data);

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
	return 0;
}
