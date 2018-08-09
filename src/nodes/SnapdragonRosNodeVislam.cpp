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
#include "SnapdragonRosNodeVislam.hpp"

//for vislam pipe
#include "SnapdragonImuManager.hpp"

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>

// #include <time.h>
//
// //Time
// struct timespec tp;
//
// double delta_time_max = 0;
// double lag_time_max = 0;
// uint64_t timestamp_ns_prev = 0;
// int time_count = 0;

Snapdragon::RosNode::Vislam::Vislam( ros::NodeHandle nh ) : nh_(nh)
{
  pub_vislam_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("vislam/pose",1);
  pub_vislam_odometry_ = nh_.advertise<nav_msgs::Odometry>("vislam/odometry",1);
  pub_vislam_path_ = nh_.advertise<visualization_msgs::Marker>("vislam/path",1);
  image_transport::ImageTransport it(nh_);
  image_transport::ImageTransport it_info(nh_);
  pub_vislam_image_ = it.advertise("/vislam/image",1);
  pub_vislam_camera_info_ = nh_.advertise<sensor_msgs::CameraInfo>("/vislam/camera_info",1);
  vislam_initialized_ = false;
  thread_started_ = false;
  thread_stop_ = false;
  // sleep here so tf buffer can get populated
  ros::Duration(1).sleep(); // sleep for 1 second
}

Snapdragon::RosNode::Vislam::~Vislam()
{
  Stop();
}

int32_t Snapdragon::RosNode::Vislam::Initialize()
{
  vislam_initialized_ = true;
  return 0;
}

int32_t Snapdragon::RosNode::Vislam::Start() {
// start vislam processing thread.
  if( !thread_started_ ) {
    thread_started_ = true;
    vislam_process_thread_ = std::thread( &Snapdragon::RosNode::Vislam::ThreadMain, this );
  }
  else {
    ROS_WARN_STREAM( "Snapdragon::RosNodeVislam::Start() VISLAM Thread already running." );
  }
  return 0;
}

int32_t Snapdragon::RosNode::Vislam::Stop() {
  if( thread_started_ ) {
    thread_stop_ = true;
    if( vislam_process_thread_.joinable() ) {
      vislam_process_thread_.join();
    }
  }
  return 0;
}

void Snapdragon::RosNode::Vislam::ThreadMain() {
 mvCameraConfiguration config;
  // Set up camera configuraiton (snapdragon down facing camera)


//NEW CALIBRATION 0.6pix error
// <?xml version='1.0' encoding='UTF-8'?>
// <Camera>
//     <Parameters>
//         <Intrinsics size="640 480" pp="305.590618 239.961624" fl = "267.324250 267.324250" distModel = "10" dist = "0.066161 -0.040640 0.008189 -0.000463 0.000000 0.000000 0.000000 0.000000"/>
//     </Parameters>
// </Camera>


  memset(&config, 0, sizeof(config));
  config.pixelWidth = 640;
  config.pixelHeight = 480;
  config.memoryStride = 640;

  // config.principalPoint[0] = 320;
  // config.principalPoint[1] = 240;
  config.principalPoint[0] = 305.590618;
  config.principalPoint[1] = 239.961624;

  // config.focalLength[0] = 275;
  // config.focalLength[1] = 275;
  config.focalLength[0] = 267.324250;
  config.focalLength[1] = 267.324250;


  // config.distortion[0] = 0.003908;
  // config.distortion[1] = -0.009574;
  // config.distortion[2] = 0.010173;
  // config.distortion[3] = -0.003329;
  // config.distortion[4] = 0;
  // config.distortion[5] = 0;
  // config.distortion[6] = 0;
  // config.distortion[7] = 0;
  // config.distortionModel = 10;

  //dist = "0.066161 -0.040640 0.008189 -0.000463 0.000000 0.000000 0.000000 0.000000"
  config.distortion[0] = 0.066161;
  config.distortion[1] = -0.040640;
  config.distortion[2] = 0.008189;
  config.distortion[3] = -0.000463;
  config.distortion[4] = 0;
  config.distortion[5] = 0;
  config.distortion[6] = 0;
  config.distortion[7] = 0;
  config.distortionModel = 10;

  Snapdragon::VislamManager::InitParams vislamParams;

  vislamParams.tbc[0] = 0.005;
  vislamParams.tbc[1] = 0.0150;
  vislamParams.tbc[2] = 0.0;

  vislamParams.ombc[0] = 0.0;
  vislamParams.ombc[1] = 0.0;
  vislamParams.ombc[2] = 1.57;

  vislamParams.delta = -0.008;

  vislamParams.std0Tbc[0] = 0.002;
  vislamParams.std0Tbc[1] = 0.002;
  vislamParams.std0Tbc[2] = 0.001;

  vislamParams.std0Ombc[0] = 0.0174532925199433;
  vislamParams.std0Ombc[1] = 0.0174532925199433;
  vislamParams.std0Ombc[2] = 0.0174532925199433;

  vislamParams.std0Delta = 0.001;
  vislamParams.accelMeasRange = 156;
  vislamParams.gyroMeasRange = 34;

  vislamParams.stdAccelMeasNoise = 0.316227766016838; // sqrt(1e-1);
  vislamParams.stdGyroMeasNoise = 1e-2; // sqrt(1e-4);

  vislamParams.stdCamNoise = 100;
  vislamParams.minStdPixelNoise = 0.5;
  vislamParams.failHighPixelNoisePoints = false;

  vislamParams.logDepthBootstrap = -3.2;//0; for ln(0.04) (4cm distance)
  vislamParams.useLogCameraHeight = true;//false;
  vislamParams.logCameraHeightBootstrap = -3.22;
  vislamParams.noInitWhenMoving = true;
  vislamParams.limitedIMUbWtrigger = 35;//35.0;

  Snapdragon::CameraParameters param;
  param.enable_cpa = 1;
  param.camera_config.fps = 30;
  param.camera_config.cam_type = Snapdragon::CameraType::OPTIC_FLOW;
  param.mv_camera_config = config;

  //set the cpa configuration.
  mvCPA_Configuration cpaConfig;
  cpaConfig.cpaType = MVCPA_MODE_COST;
  cpaConfig.legacyCost.startExposure = param.camera_config.exposure;
  cpaConfig.legacyCost.startGain = param.camera_config.gain;
  cpaConfig.legacyCost.filterSize = 1;
  cpaConfig.legacyCost.exposureCost = 1.0f;
  cpaConfig.legacyCost.gainCost = 0.3333f;

  param.mv_cpa_config = cpaConfig;
  Snapdragon::VislamManager vislam_man;
  //object for write_pipe
  Snapdragon::ImuManager imu_man_2;
  if( vislam_man.Initialize( param, vislamParams ) != 0  ) {
    ROS_WARN_STREAM( "Snapdragon::RosNodeVislam::VislamThreadMain: Error initializing the VISLAM Manager " );
    thread_started_ = false;
    return;
  }

// start the VISLAM processing.
  if( vislam_man.Start() != 0 ) {
    ROS_WARN_STREAM( "Snapdragon::RosNodeVislam::VislamThreadMain: Error Starting the VISLAM manager" );
    thread_started_ = false;
    return;
  }

  mvVISLAMPose vislamPose;
  int64_t vislamFrameId;
  uint64_t timestamp_ns;
  thread_stop_ = false;
  int32_t vislam_ret;
  uint8_t* frame_data;
  while( !thread_stop_ ) {
    vislam_ret = vislam_man.GetPose( vislamPose, vislamFrameId, timestamp_ns );
    frame_data = vislam_man.GetFrameData();

    //Check timing
    // clock_gettime(CLOCK_MONOTONIC, &tp);
	// uint64_t current = tp.tv_sec*1000000000 + tp.tv_nsec;
	// double lag_time = (double)(current - timestamp_ns)/1000000.0;//ms
    // double delta_time = (double)(timestamp_ns - timestamp_ns_prev)/1000000.0;//ms
    // timestamp_ns_prev = timestamp_ns;
    // if(delta_time>delta_time_max)delta_time_max=delta_time;
    // if(lag_time>lag_time_max)lag_time_max=lag_time;
    // time_count++;
    // if(time_count>30){
    //     printf("delta_time_max: %f\n", delta_time_max);
    //     printf("lag_time_max: %f\n", lag_time_max);
    //     delta_time_max = 0;
    //     lag_time_max = 0;
    //     time_count = 0;
    // }


    if( vislam_ret == 0 ) {
      //check if the pose quality is good.  If not do not publish the data.
      if( vislamPose.poseQuality != MV_TRACKING_STATE_FAILED  &&
          vislamPose.poseQuality != MV_TRACKING_STATE_INITIALIZING ) {
          // Publish Pose Data
          PublishVislamData( vislamPose, vislamFrameId, timestamp_ns, frame_data );
      }
      //always write to autopilot pipe
      imu_man_2.write_pipe( vislamPose, vislamFrameId, timestamp_ns );
    }
    else {
      ROS_WARN_STREAM( "Snapdragon::RosNodeVislam::VislamThreadMain: Warning Getting Pose Information" );
    }
  }
  thread_started_ = false;
  // the thread is shutting down. Stop the vislam Manager.
  vislam_man.Stop();
  ROS_INFO_STREAM( "Snapdragon::RosNodeVislam::VislamThreadMain: Exising VISLAM Thread" );
  return;
}

int32_t Snapdragon::RosNode::Vislam::PublishVislamData( mvVISLAMPose& vislamPose, int64_t vislamFrameId, uint64_t timestamp_ns, uint8_t* frame_data  ) {
  geometry_msgs::PoseStamped pose_msg;
  ros::Time frame_time;
  frame_time.sec = (int32_t)(timestamp_ns/1000000000UL);
  frame_time.nsec = (int32_t)(timestamp_ns % 1000000000UL);
  pose_msg.header.frame_id = "vislam";
  pose_msg.header.stamp = frame_time;
  pose_msg.header.seq = vislamFrameId;

  // translate vislam pose to ROS pose
  tf2::Matrix3x3 R(
    vislamPose.bodyPose.matrix[0][0],
    vislamPose.bodyPose.matrix[0][1],
    vislamPose.bodyPose.matrix[0][2],
    vislamPose.bodyPose.matrix[1][0],
    vislamPose.bodyPose.matrix[1][1],
    vislamPose.bodyPose.matrix[1][2],
    vislamPose.bodyPose.matrix[2][0],
    vislamPose.bodyPose.matrix[2][1],
    vislamPose.bodyPose.matrix[2][2]);
  tf2::Quaternion q;
  R.getRotation(q);
  pose_msg.pose.position.x = vislamPose.bodyPose.matrix[0][3];
  pose_msg.pose.position.y = vislamPose.bodyPose.matrix[1][3];
  pose_msg.pose.position.z = vislamPose.bodyPose.matrix[2][3];
  pose_msg.pose.orientation.x = q.getX();
  pose_msg.pose.orientation.y = q.getY();
  pose_msg.pose.orientation.z = q.getZ();
  pose_msg.pose.orientation.w = q.getW();
  pub_vislam_pose_.publish(pose_msg);

    //publish the trajectory message.
    path.id = 0;
    path.lifetime=ros::Duration(1);
    path.header.frame_id = "vislam";
    path.header.stamp = frame_time;
    path.action = visualization_msgs::Marker::ADD;
    path.type = visualization_msgs::Marker::LINE_STRIP;
    path.color.r=1.0;
    path.color.g=1.0; // yellow
    path.color.a=1.0;
    path.scale.x=0.001;
    path.pose.orientation.w=1.0;

  //publish the odometry message.
  nav_msgs::Odometry odom_msg;
  odom_msg.header.stamp = frame_time;
  odom_msg.header.frame_id = "vislam";
  odom_msg.pose.pose = pose_msg.pose;
  odom_msg.twist.twist.linear.x = vislamPose.velocity[0];
  odom_msg.twist.twist.linear.y = vislamPose.velocity[1];
  odom_msg.twist.twist.linear.z = vislamPose.velocity[2];
  odom_msg.twist.twist.angular.x = vislamPose.angularVelocity[0];
  odom_msg.twist.twist.angular.y = vislamPose.angularVelocity[1];
  odom_msg.twist.twist.angular.z = vislamPose.angularVelocity[2];

  //set the error covariance for the pose.
  for( int16_t i = 0; i < 6; i++ ) {
    for( int16_t j = 0; j < 6; j++ ) {
      odom_msg.pose.covariance[ i*6 + j ] = vislamPose.errCovPose[i][j];
    }
  }
  pub_vislam_odometry_.publish(odom_msg);

  // get frame
  sensor_msgs::Image img_msg;
  cv::Mat img = cv::Mat(480, 640, CV_8UC1, frame_data);
  cv_bridge::CvImage cvi;
  cvi.header.stamp = ros::Time::now();
  cvi.header.frame_id = "image";
  cvi.image = img;
  cvi.encoding = "mono8";
  cvi.toImageMsg(img_msg);
  img_msg.header.frame_id = "vislam";
  img_msg.header.stamp = frame_time;

  sensor_msgs::CameraInfo c_info;
  c_info.header.frame_id = "vislam";
  c_info.header.stamp = frame_time;

  c_info.height = 480;
  c_info.width = 640;

  // config.principalPoint[0] = 305.590618;
  // config.principalPoint[1] = 239.961624;
  //
  // // config.focalLength[0] = 275;
  // // config.focalLength[1] = 275;
  // config.focalLength[0] = 267.324250;
  // config.focalLength[1] = 267.324250;

   c_info.K[0] =  0.0;//267.324250;
   // c_info.K[1] = 0;
   // c_info.K[2] = 305.590618;
   // c_info.K[3] = 0;
   // c_info.K[4] = 267.324250;
   // c_info.K[5] = 239.961624;
   // c_info.K[6] = 0;
   // c_info.K[7] = 0;
   // c_info.K[8] = 1;

  pub_vislam_image_.publish(img_msg);//,c_info);

  pub_vislam_camera_info_.publish(c_info);


  // compute transforms
  std::vector<geometry_msgs::TransformStamped> transforms;
  geometry_msgs::TransformStamped transform;
  transform.transform.translation.x = pose_msg.pose.position.x;
  transform.transform.translation.y = pose_msg.pose.position.y;
  transform.transform.translation.z = pose_msg.pose.position.z;
  transform.transform.rotation.x = pose_msg.pose.orientation.x;
  transform.transform.rotation.y = pose_msg.pose.orientation.y;
  transform.transform.rotation.z = pose_msg.pose.orientation.z;
  transform.transform.rotation.w = pose_msg.pose.orientation.w;
  transform.child_frame_id = "base_link_vislam";
  transform.header.frame_id = "vislam";
  transform.header.stamp = frame_time;

  // collect transforms
  transforms.push_back(transform);

  // broadcast transforms
  static tf2_ros::TransformBroadcaster br;
  br.sendTransform(transforms);
}
