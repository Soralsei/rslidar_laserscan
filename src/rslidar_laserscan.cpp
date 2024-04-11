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

#include "rslidar_laserscan/rslidar_laserscan.h"

#include <sensor_msgs/point_cloud2_iterator.h>

namespace rslidar_laserscan
{
  RslidarLaserScan::RslidarLaserScan(ros::NodeHandle &nh, ros::NodeHandle &nh_priv) : nh_(nh), ring_count(0), reconfig_srv(nh_priv)
  {
    ros::SubscriberStatusCallback connect_cb = boost::bind(&RslidarLaserScan::connectCb, this);
    std::string model = "RS16";

    nh_priv.param("sub_topic", sub_topic_, std::string("/rslidar_points"));
    nh_priv.param("model", model, std::string("RS16"));

    int ring = 1000;
    nh_priv.param("ring", ring, 1000); // 1000 is not set ring

    if (model == "RS16")
    {
      range_min_ = 0.4;
      range_max_ = 150.0;
      default_ring = 7;
    }
    else if (model == "RS32")
    {
      range_min_ = 0.4;
      range_max_ = 200.0;
      default_ring = 9;
    }
    else if (model == "RSHELIOS")
    {
      range_min_ = 0.2;
      range_max_ = 150.0;
      default_ring = 9;
    }
    else if (model == "RSHELIOS_16P")
    {
      range_min_ = 0.4;
      range_max_ = 200.0;
      default_ring = 6;
    }
    else if (model == "RSBP")
    {
      range_min_ = 0.1;
      range_max_ = 30.0;
      default_ring = 31;
    }
    else if (model == "RS80")
    {
      range_min_ = 1.0;
      range_max_ = 230.0;
      default_ring = 74;
    }
    else if (model == "RS128")
    {
      range_min_ = 1.0;
      range_max_ = 250.0;
      default_ring = 118;
    }
    else
    {
      std::cout << "lidar model is bad. please choose right model from: RS16|RS32|RSHELIOS|RSHELIOS_16P|RSBP|RS80|RS128!" << std::endl;
      exit(-1);
    }
    pub_ = nh.advertise<sensor_msgs::LaserScan>("rslidar_laserscan", 10, connect_cb, connect_cb);
    reconfig_srv.setCallback(boost::bind(&RslidarLaserScan::reconfig, this, _1, _2));
  }

  void RslidarLaserScan::connectCb()
  {
    boost::lock_guard<boost::mutex> lock(connect_mutex_);
    if (!pub_.getNumSubscribers())
    {
      sub_.shutdown();
    }
    else if (!sub_)
    {
      sub_ = nh_.subscribe(sub_topic_, 10, &RslidarLaserScan::recvCallback, this);
    }
  }

  void RslidarLaserScan::reconfig(RSLaserScanConfig &config, uint32_t level)
  {
    this->config = config;
  }

  void RslidarLaserScan::recvCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
  {
    // Latch ring count
    if (!ring_count)
    {
      // Check for PointCloud2 field 'ring'
      bool ring_field_found = false;
      for (size_t i = 0; i < msg->fields.size(); i++)
      {
        if (msg->fields[i].datatype == sensor_msgs::PointField::UINT16 && msg->fields[i].name == "ring")
        {
          ring_field_found = true;
          break;
        }
      }
      if (!ring_field_found)
      {
        ROS_ERROR("RslidarLaserScan: Field 'ring' of type 'UINT16' not present in PointCloud2");
        return;
      }
      for (sensor_msgs::PointCloud2ConstIterator<uint16_t> it(*msg, "ring"); it != it.end(); ++it)
      {
        const uint16_t ring = *it;

        if (ring + 1 > ring_count)
        {
          ring_count = ring + 1;
        }
      }
      if (ring_count)
      {
        ROS_INFO("RslidarLaserScan: Latched ring count of %u", ring_count);
      }
      else
      {
        ROS_ERROR("RslidarLaserScan: Field 'ring' of type 'UINT16' not present in PointCloud2");
        return;
      }
    }

    // Select ring to use
    uint16_t ring;

    if ((config.ring < 0) || (config.ring >= ring_count))
    {
      // Default to ring closest to being level for each known sensor
      ring = default_ring;
    }
    else
    {
      ring = config.ring;
    }

    ROS_INFO_ONCE("RslidarLaserScan: Extracting ring %u", ring);

    // field offsets
    int offset_x = -1;
    int offset_y = -1;
    int offset_z = -1;
    int offset_i = -1;
    int offset_r = -1;

    for (size_t i = 0; i < msg->fields.size(); i++)
    {
      if (msg->fields[i].datatype == sensor_msgs::PointField::FLOAT32)
      {
        if (msg->fields[i].name == "x")
        {
          offset_x = msg->fields[i].offset;
        }
        else if (msg->fields[i].name == "y")
        {
          offset_y = msg->fields[i].offset;
        }
        else if (msg->fields[i].name == "z")
        {
          offset_z = msg->fields[i].offset;
        }
        else if (msg->fields[i].name == "intensity")
        {
          offset_i = msg->fields[i].offset;
        }
      }
      else if (msg->fields[i].datatype == sensor_msgs::PointField::UINT16)
      {
        if (msg->fields[i].name == "ring")
        {
          offset_r = msg->fields[i].offset;
        }
      }
    }

    // ROS_INFO_ONCE ("offset x:%u, y:%u, z:%u, intensity:%u",
    // offset_x, offset_y, offset_z, offset_i);

    // laser scan msg
    if ((offset_x == 0) && (offset_y >= 0) && (offset_z >= 0) && (offset_i >= 0))
    {
      const float RESOLUTION = 0.0034906584; // 2.0 * M_PI / 1800 (10Hz)
      const size_t SIZE = 2.0 * M_PI / RESOLUTION;

      sensor_msgs::LaserScanPtr scan(new sensor_msgs::LaserScan());
      scan->header = msg->header;
      scan->angle_increment = RESOLUTION;

      scan->angle_min = -M_PI;
      scan->angle_max = M_PI;

      scan->range_min = range_min_;
      scan->range_max = range_max_;

      scan->scan_time = 0.1;
      scan->time_increment = scan->scan_time / SIZE;
      scan->ranges.resize(SIZE, INFINITY);

      if ((offset_x == 0) &&
          (offset_y == 4) &&
          (offset_i % 4 == 0) &&
          (offset_r % 4 == 0))
      {
        scan->intensities.resize(SIZE);

        const size_t X = 0;
        const size_t Y = 1;
        const size_t I = offset_i / 4;
        const size_t R = offset_r / 4;
        for (sensor_msgs::PointCloud2ConstIterator<float> it(*msg, "x"); it != it.end(); ++it)
        {
          const uint16_t r = *((const uint16_t *)(&it[R])); // ring

          if (r == ring)
          {
            const float x = it[X]; // x
            const float y = it[Y]; // y
            const float i = it[I]; // intensity
            const int bin = (atan2f(y, x) + static_cast<float>(M_PI)) / RESOLUTION;

            if ((bin >= 0) && (bin < static_cast<int>(SIZE)))
            {
              scan->ranges[bin] = sqrtf(x * x + y * y);
              scan->intensities[bin] = i;
            }
          }
        }
      }
      else
      {
        ROS_WARN_ONCE("RslidarLaserScan: PointCloud2 fields in unexpected order. Using slower generic method.");

        if (offset_i >= 0)
        {
          scan->intensities.resize(SIZE);
          sensor_msgs::PointCloud2ConstIterator<uint16_t> iter_r(*msg, "ring");
          sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
          sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
          sensor_msgs::PointCloud2ConstIterator<float> iter_i(*msg, "intensity");
          for (; iter_r != iter_r.end(); ++iter_x, ++iter_y, ++iter_r, ++iter_i)
          {
            const uint16_t r = *iter_r; // ring

            if (r == ring)
            {
              const float x = *iter_x; // x
              const float y = *iter_y; // y
              const float i = *iter_i; // intensity
              const int bin = (atan2f(y, x) + static_cast<float>(M_PI)) / RESOLUTION;

              if ((bin >= 0) && (bin < static_cast<int>(SIZE)))
              {
                scan->ranges[bin] = sqrtf(x * x + y * y);
                scan->intensities[bin] = i;
              }
            }
          }
        }
        else
        {
          sensor_msgs::PointCloud2ConstIterator<uint16_t> iter_r(*msg, "ring");
          sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
          sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");

          for (; iter_r != iter_r.end(); ++iter_x, ++iter_y, ++iter_r)
          {
            const uint16_t r = *iter_r; // ring

            if (r == ring)
            {
              const float x = *iter_x; // x
              const float y = *iter_y; // y
              const int bin = (atan2f(y, x) + static_cast<float>(M_PI)) / RESOLUTION;

              if ((bin >= 0) && (bin < static_cast<int>(SIZE)))
              {
                scan->ranges[bin] = sqrtf(x * x + y * y);
              }
            }
          }
        }
      }

      pub_.publish(scan);
    }
    else
    {
      ROS_ERROR("RslidarLaserScan: PointCloud2 missing one or more required fields! (x,y,ring)");
    }
  }
}
