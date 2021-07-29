/*******************************************************************************************
 *
Copyright (c) 2020 RoboSense
All rights reserved

By downloading, copying, installing or using the software you agree to this license.
If you do not agree to this license, do not download, install, copy or use the software.

License Agreement
For Rslidar Laserscan Tool
(3-clause BSD License)

Redistribution and use in source and binary forms, with or without modification, are
permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of
conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list
of conditions and the following disclaimer in the documentation and/or other materials
provided with the distribution.

3. Neither the names of the RoboSense, nor Suteng Innovation Technology, nor the names
of other contributors may be used to endorse or promote products derived from this software
without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS
OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

********************************************************************************************/

#ifndef _RSLIDAR_LASERSCAN_H_
#define _RSLIDAR_LASERSCAN_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

#include <boost/thread/mutex.hpp>
#include <boost/thread/lock_guard.hpp>

namespace rslidar_laserscan
{
class RslidarLaserScan
{
public:
  RslidarLaserScan(ros::NodeHandle& nh, ros::NodeHandle& nh_priv);

private:
  void connectCb();
  void recvCallback(const sensor_msgs::PointCloud2ConstPtr& msg);

  uint16_t ring_;
  uint16_t height_;
  float range_min_;
  float range_max_;
  std::string sub_topic_;
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  boost::mutex connect_mutex_;
};
}

#endif  // _RSLIDAR_LASERSCAN_H_
