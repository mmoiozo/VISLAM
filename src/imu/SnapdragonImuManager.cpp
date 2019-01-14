/****************************************************************************
 *   Copyright (c) 2016 Ramakrishna Kintada. All rights reserved.
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
 * 3. Neither the name ATLFlight nor the names of its contributors may be
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
#include "SnapdragonImuManager.hpp"
#include "SnapdragonDebugPrint.h"
#include <algorithm>
#include <mvVISLAM.h>
//c stuff
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <iostream>

//quaternion transform
#include <tf2/LinearMath/Matrix3x3.h>
// #include <tf2_ros/transform_broadcaster.h>
// #include <tf2_ros/buffer.h>

//find appropriate cpp way of handeling these vars
#define MAX_BUF 10

int fd_tx = -1;
int fd_rx = -1;
int res = 0;

char *tx_fifo = "/tmp/tx_fifo";
char *rx_fifo = "/tmp/rx_fifo";

int recv_count = 0;
int last_seq = 0;

int64_t imu_time_stamp = 0;

//crc
const uint8_t CRC7_POLY = 0x91;

Snapdragon::ImuManager::ImuManager() {
  thread_stop_ = false;
  initialized_ = false;
  thread_started_ = false;
  imu_api_handle_ptr_ = nullptr;
}

int32_t Snapdragon::ImuManager::Initialize() {
  initialized_ = true;
  thread_started_ = false;
  return 0;
}

int32_t Snapdragon::ImuManager::SetupImuApi() {

     mkfifo(rx_fifo, 0777);
	/* Open, read, and display the message from the FIFO. */
	fd_tx = open(tx_fifo, O_RDONLY | O_NONBLOCK);
	printf("open tx pipe: %d\n", fd_tx);

  return 0;
}

//crc crc_checksum

uint8_t getCRC(uint8_t message[], int length)
{
  uint8_t i, j, crc = 0;

  for (i = 0; i < length; i++)
  {
    crc ^= message[i];
    for (j = 0; j < 8; j++)
    {
      if (crc & 1)
        crc ^= CRC7_POLY;
      crc >>= 1;
    }
  }
  return crc;
}

//use cpp style function
int read_pipe(int32_t *dsp_read_buffer,int buf_len)
{
	char read_buffer[buf_len*4];
	int res = 1;
	int ret = -1;
	while(res > 0)
	{
	res = read(fd_tx, read_buffer, sizeof(read_buffer));

	if(res > 0)
	  {
	    memcpy(dsp_read_buffer,read_buffer,sizeof(read_buffer));
	    //printf("res: %d Received: %d\n",res,dsp_read_buffer[0]);// data_tx[0]);
        if(dsp_read_buffer[9] == 132508){
            //printf("dsp_read_buffer[0]%d\n",dsp_read_buffer[0] );
	           ret = 1;
            }
	  }
	}
	return ret;
}

int32_t Snapdragon::ImuManager::write_pipe( mvVISLAMPose& vislamPose, int64_t vislamFrameId, uint64_t timestamp_ns, float x_cg, float y_cg, float z_cg,float x_cg_vel, float y_cg_vel, float z_cg_vel){

    uint32_t vislam_errorCode = vislamPose.errorCode;
    int16_t vislam_poseQuality = vislamPose.poseQuality;

    int f_length = 30;
    float buffer_f[f_length];

    //use gravityCameraPose instead of bodyPose??

    //position x y z in global frame
    buffer_f[0] = x_cg;//vislamPose.bodyPose.matrix[0][3];
    buffer_f[1] = y_cg;//vislamPose.bodyPose.matrix[1][3];
    buffer_f[2] = z_cg;//vislamPose.bodyPose.matrix[2][3];

    buffer_f[3] = vislamPose.bodyPose.matrix[0][0];
    buffer_f[4] = vislamPose.bodyPose.matrix[0][1];
    buffer_f[5] = vislamPose.bodyPose.matrix[0][2];
    buffer_f[6] = vislamPose.bodyPose.matrix[1][0];
    buffer_f[7] = vislamPose.bodyPose.matrix[1][1];
    buffer_f[8] = vislamPose.bodyPose.matrix[1][2];
    buffer_f[9] = vislamPose.bodyPose.matrix[2][0];
    buffer_f[10] = vislamPose.bodyPose.matrix[2][1];
    buffer_f[11] = vislamPose.bodyPose.matrix[2][2];


    //x y z velocity compensated for cg shift
    buffer_f[12] = x_cg_vel;//vislamPose.velocity[0];
    buffer_f[13] = y_cg_vel;//vislamPose.velocity[1];
    buffer_f[14] = z_cg_vel;//vislamPose.velocity[2];

    // p q r body velocity
    buffer_f[15] = vislamPose.angularVelocity[0];
    buffer_f[16] = vislamPose.angularVelocity[1];
    buffer_f[17] = vislamPose.angularVelocity[2];

    //gyro biases
    buffer_f[18] = vislamPose.wBias[0];
    buffer_f[19] = vislamPose.wBias[1];
    buffer_f[20] = vislamPose.wBias[2];
    //acc biases
    buffer_f[21] = vislamPose.aBias[0];
    buffer_f[22] = vislamPose.aBias[1];
    buffer_f[23] = vislamPose.aBias[2];

    //High-Level commands from ROS to autopilot
    //target_pos_x
    //target_pos_y
    //target_pos_z
    //target_psi
    //target_vel_x
    //target_vel_y
    //target_vel_z
    //target_psi_dot
    //MODE (POS_MODE,VEL_MODE,LANDING, CUT props, MANUAL etc...)


    //pose error covariance(for now only autocovariance)
    for( int16_t i = 0; i < 6; i++ ) {
        buffer_f[24+i] =  i;//vislamPose.errCovPose[i][i];
    }

    //max buffer_f[29] -> f_length = 30

    int f_int_8_length = f_length*sizeof(float);

    //total length of buffer send
    int out_buffer_length = 22+f_int_8_length + 1;//+1 for crc8

    uint8_t out_buffer[out_buffer_length];

    int ptr = 0;
    memcpy(out_buffer,&vislamFrameId,sizeof(vislamFrameId));
    ptr+=8;
    memcpy(out_buffer+ptr,&timestamp_ns,sizeof(timestamp_ns));
    ptr+=8;
    memcpy(out_buffer+ptr,&vislam_errorCode,sizeof(vislam_errorCode));
    ptr+=4;
    memcpy(out_buffer+ptr,&vislam_poseQuality,sizeof(vislam_poseQuality));
    ptr+=2;

    //ptr = 22

    //copy float array to out_buffer
    memcpy(out_buffer+ptr,buffer_f,f_int_8_length);

    //calculate crc checksum
    uint8_t crc_checksum = getCRC(out_buffer,out_buffer_length-2);
    out_buffer[out_buffer_length-1] = crc_checksum;


    if (fd_rx < 0)
		fd_rx = open(rx_fifo, O_WRONLY | O_NONBLOCK);

	int res = write(fd_rx, out_buffer, out_buffer_length);

    return 0;
}

void Snapdragon::ImuManager::ImuThreadMain() {
  if( !initialized_ ) {
    ERROR_PRINT( "Starting IMU thread without initialization. Exiting Thread." );
    thread_started_ = false;
    return;
  }

  if( SetupImuApi() != 0 ) {
    ERROR_PRINT( "Error initializing the IMU API's; exiting thread." );
    thread_started_ = false;
    return;
  }

  int32_t max_imu_samples = 100;
  sensor_imu imu_buffer[ max_imu_samples ];
  int32_t returned_sample_count = 0;
  int16_t api_rc;
  uint32_t prev_sequence_number = 0;
  uint32_t current_seqeunce_number;

  while( !thread_stop_ ) {
    returned_sample_count = 0;
    //api_rc = sensor_imu_attitude_api_get_imu_raw( imu_api_handle_ptr_, imu_buffer, 100, &returned_sample_count );
    int32_t dsp_buffer[MAX_BUF];
    //printf("Before read\n");
    int res = read_pipe(dsp_buffer,MAX_BUF);

    if( res < 0){//api_rc != 0 ) {
     // WARN_PRINT( "WARN: Error getting imu samples from imu api(%d)", api_rc );
    }
    else {
        current_seqeunce_number = dsp_buffer[0];//imu_buffer[0].sequence_number;
        if( prev_sequence_number != 0 && prev_sequence_number + 1 != current_seqeunce_number ) {
        //   WARN_PRINT( "Missed IMU Samples: Expected:(%u) Got(%u) sample count: (%d)",
        //     (prev_sequence_number+1), current_seqeunce_number, returned_sample_count );
        }
        prev_sequence_number = dsp_buffer[0];//imu_buffer[returned_sample_count-1].sequence_number;

        // call the handlers.
        {
          std::lock_guard<std::mutex> lock( sync_mutex_ );
          for( auto h : imu_listeners_ ) {
            //h->Imu_IEventListener_ProcessSamples( imu_buffer, returned_sample_count );
            h->Imu_IEventListener_ProcessSamples( dsp_buffer, 1);
          }
        }
    }
  }
  INFO_PRINT( "Exiting the IMU Manager Thread." );
  sensor_imu_attitude_api_terminate( imu_api_handle_ptr_ );
  thread_started_ = false;
}

int32_t Snapdragon::ImuManager::Start() {
  // start the IMU reader thread.
  if( !thread_started_ ) {
    thread_started_ = true;
    imu_reader_thread_ = std::thread( &Snapdragon::ImuManager::ImuThreadMain, this );
  }
  else {
    WARN_PRINT( "WARN: Imu Reader Thread is already running" );
  }
  return 0;
}

int32_t Snapdragon::ImuManager::Stop() {
  //this is to stop the thread.
  thread_stop_ = true;
  initialized_ = false;
  if( imu_reader_thread_.joinable() ) {
    imu_reader_thread_.join();
  }
  imu_api_handle_ptr_ = nullptr;
  //close pipe
  close(fd_tx);
  close(fd_rx);
  return 0;
}

int32_t Snapdragon::ImuManager::Terminate() {
   Stop();
}


int32_t Snapdragon::ImuManager::AddHandler( Snapdragon::Imu_IEventListener* handler ) {
  int32_t rc = -1;
  std::lock_guard<std::mutex> lock( sync_mutex_ );
  if( std::find( imu_listeners_.begin(), imu_listeners_.end(), handler ) == imu_listeners_.end() ) {
    imu_listeners_.push_back( handler );
    rc = 0;
  }
  return rc;
}

int32_t Snapdragon::ImuManager::RemoveHandler( Snapdragon::Imu_IEventListener* handler ) {
  int32_t rc = -1;
  std::lock_guard<std::mutex> lock( sync_mutex_ );
  auto element = std::find( imu_listeners_.begin(), imu_listeners_.end(), handler );
  if( element != imu_listeners_.end() ) {
    imu_listeners_.erase( element );
    rc = 0;
  }
  return rc;
}
