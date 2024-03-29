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
//c stuff
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>

//find appropriate cpp way of handeling these vars
#define MAX_BUF 10

int fd_tx;
int fd_rx;
int res = 0;

char *tx_fifo = "/tmp/tx_fifo";
char *rx_fifo = "/tmp/rx_fifo";

int recv_count = 0;
int last_seq = 0;

int64_t imu_time_stamp = 0;

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
	printf("open tx pipeline: %d\n", fd_tx);

  // int16_t api_rc = 0;
  // //get the imu_api handle first.
  // imu_api_handle_ptr_ = sensor_imu_attitude_api_get_instance();
  // if( imu_api_handle_ptr_ == nullptr ) {
  //   //error
  //   ERROR_PRINT( "Error getting the IMU API handler." );
  //   return -1;
  // }
  //
  // // check the version is the correct one.
  // char* api_version = sensor_imu_attitude_api_get_version( imu_api_handle_ptr_ );
  // std::string str_version( api_version );
  // std::string str_expected_version( SENSOR_IMU_AND_ATTITUDE_API_VERSION );
  // if( str_version != str_expected_version ) {
  //   ERROR_PRINT( "Error: Imu API version Mismatch.  Make sure to link against the correct version(%s)",
  //     str_expected_version.c_str() );
  //   return -2;
  // }
  //
  // //the API' are good. Call the API's initialize method.
  // api_rc = sensor_imu_attitude_api_initialize( imu_api_handle_ptr_, SENSOR_CLOCK_SYNC_TYPE_MONOTONIC );
  // if( api_rc != 0 ) {
  //   ERROR_PRINT( "Error calling the IMU API's initialize method api_rc(%d)", api_rc );
  //   return api_rc;
  // }
  //
  // api_rc = sensor_imu_attitude_api_wait_on_driver_init( imu_api_handle_ptr_ );
  // if( api_rc != 0 ) {
  //   ERROR_PRINT( "Error calling the IMU API for driver init completion(%d)", api_rc );
  //   return api_rc;
  // }
  return 0;//api_rc;
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
        //ret = 1;
	  }
	}
    //ret = 1;
	return ret;
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
  //close pipe maybe solves crash issue
  close(fd_tx);
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
