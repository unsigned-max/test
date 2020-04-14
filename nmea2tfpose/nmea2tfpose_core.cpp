/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "nmea2tfpose_core.h"
#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>

namespace gnss_localizer
{
  long getTimestamp()
  {
    struct timeval tv;
 
    gettimeofday(&tv, NULL);
     
    return (tv.tv_sec*1000 + tv.tv_usec/1000);
  }

string makeFileName()
{
	char buff[128] = {0};
	time_t tt;
	struct tm tr = {0};
	time(&tt);
	localtime_r(&tt, &tr);
	snprintf(buff, sizeof(buff),
		"GPS_DAT_%02d%02d%"
		"02d%02d%02d.csv", 
		tr.tm_mon + 1, tr.tm_mday, 
		tr.tm_hour, tr.tm_min, tr.tm_sec);
	
	string filename = buff;
	return filename;
}

// Constructor
Nmea2TFPoseNode::Nmea2TFPoseNode()
  : private_nh_("~")
  , MAP_FRAME_("map")
  , GPS_FRAME_("gps")
  , roll_(0)
  , pitch_(0)
  , yaw_(0)
  , orientation_time_(0)
  , position_time_(0)
  , current_time_(0)
  , orientation_stamp_(0)
{
  initForROS();
  geo_.set_origin(longitude_, latitude_);
}

// Destructor
Nmea2TFPoseNode::~Nmea2TFPoseNode()
{
  outfile_.close();
}

void Nmea2TFPoseNode::initForROS()
{
  // ros parameter setting

  //log
  longitude_ = 118.5668725;
  latitude_ = 24.97049781;
  private_nh_.getParam("longitude", longitude_);
  private_nh_.getParam("latitude", latitude_);
  ROS_INFO("lon:%lf, lat:%lf", longitude_, latitude_); 
  // setup subscriber
  sub1_ = nh_.subscribe("nmea_sentence", 100, &Nmea2TFPoseNode::callbackFromNmeaSentence, this);

  // setup publisher
  pub1_ = nh_.advertise<geometry_msgs::PoseStamped>("gnss_pose", 10);

  string filename = makeFileName();
  ROS_INFO_STREAM("GPS filename:" << filename); 
  outfile_.open(filename.c_str(), ios::out);
}

void Nmea2TFPoseNode::run()
{
  ros::spin();
}

void Nmea2TFPoseNode::publishPoseStamped()
{
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = MAP_FRAME_;
  pose.header.stamp = current_time_;
  pose.pose.position.x = geo_.x();
  pose.pose.position.y = geo_.y();
  pose.pose.position.z = geo_.z();
  pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll_, pitch_, yaw_);
  pub1_.publish(pose);
  ROS_INFO("Pose: x:%lf, y:%lf, yaw:%lf.", geo_.x(), geo_.y(), yaw_); 
  outfile_ << geo_.x() << "," 
           << geo_.y() << ","
           << geo_.z() << ","
           << roll_ << ","
           << pitch_ << ","
           << yaw_ << endl;
}

void Nmea2TFPoseNode::publishTF()
{
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(geo_.x(), geo_.y(), geo_.z()));
  tf::Quaternion quaternion;
  quaternion.setRPY(roll_, pitch_, yaw_);
  transform.setRotation(quaternion);
  br_.sendTransform(tf::StampedTransform(transform, current_time_, MAP_FRAME_, GPS_FRAME_));
}

void Nmea2TFPoseNode::createOrientation()
{
  yaw_ = atan2(geo_.y(), geo_.x());
  roll_ = 0;
  pitch_ = 0;
}

int Nmea2TFPoseNode::convert(std::vector<std::string> nmea, ros::Time current_stamp)
{
  try
  {
    if (nmea.at(0) == "$PASHR")  //heading pitch roll
    {
      if (stoi(nmea.at(10)) != 1) //丢弃非差分数据       
      {
        ROS_WARN_STREAM("PASHR data was nonRTK.");
        return -1;
      }

      orientation_time_ = stod(nmea.at(1));
      roll_ = stod(nmea.at(4)) * M_PI / 180.;
      pitch_ = -1 * stod(nmea.at(5)) * M_PI / 180.;
      yaw_ = -1 * stod(nmea.at(2)) * M_PI / 180. + M_PI*2;
    }
    else if(nmea.at(0).compare(3, 3, "GGA") == 0) //定位信息
    {
      if (stoi(nmea.at(6)) != 4)  //丢弃非差分数据     
      {
        ROS_WARN_STREAM("GGA data was nonRTK.");
        return -1;
      }

      position_time_ = stod(nmea.at(1));
      double lat = stod(nmea.at(2));
      double lon = stod(nmea.at(4));
      double h = stod(nmea.at(9));
      geo_.set_llh_nmea_degrees(lat, lon, h);
    }
    else if(nmea.at(0) == "$GPRMC")
    {
      if (nmea.at(12).compare("D") != 0)  //丢弃非差分数据     
      {
        ROS_WARN_STREAM("GPRMC data was nonRTK.");
        return -1;
      }

      position_time_ = stod(nmea.at(1));
      double lat = stod(nmea.at(3));
      double lon = stod(nmea.at(5));
      double h = 0.0;
      geo_.set_llh_nmea_degrees(lat, lon, h);
    }
  }catch (const std::exception &e)
  {
    ROS_WARN_STREAM("Message is invalid : " << e.what());
    return -1;
  }
  return 0;
}

void Nmea2TFPoseNode::callbackFromNmeaSentence(const nmea_msgs::Sentence::ConstPtr &msg)
{
  current_time_ = msg->header.stamp;
  int res = 0;
  res = convert(split(msg->sentence), msg->header.stamp) ;
  if ( res < 0)
  {
      ROS_WARN_STREAM("Msg Err : " << msg->sentence);
      return ;
  }
  
  double timeout = 10.0;
  double e = 1e-2;
  if (fabs(orientation_time_ - position_time_) < e)
  {
    double dt = sqrt(pow(geo_.x() - last_geo_.x(), 2) + pow(geo_.y() - last_geo_.y(), 2));
    double threshold = 0.2;
  
    if (dt > threshold)
    {
      //createOrientation();
      publishPoseStamped();
      //publishTF();
      last_geo_ = geo_;
    }
  
    return;
  }
}

std::vector<std::string> split(const std::string &string)
{
  std::vector<std::string> str_vec_ptr;
  std::string token;
  std::stringstream ss(string);

  while (getline(ss, token, ','))
    str_vec_ptr.push_back(token);

  return str_vec_ptr;
}

}  // gnss_localizer

