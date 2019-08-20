/*
 *  Copyright (C) 2010, CCNY Robotics Lab
 *  Ivan Dryanovski <ivan.dryanovski@gmail.com>
 *
 *  http://robotics.ccny.cuny.edu
 *
 *  Based on implementation of Madgwick's IMU and AHRS algorithms.
 *  http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
 *
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "imu_filter_madgwick/imu_filter_ros.h"
 #include <ros/console.h>
 #include "imu_filter_madgwick/ImuDrift.h"

 ImuFilterRos *imu_filter;
//service is added for calling drift calculation
 bool startComp(imu_filter_madgwick::ImuDrift::Request  &req,
           imu_filter_madgwick::ImuDrift::Response &res)
   {
     res.dummy = true;
    
     ROS_INFO("request: duration=%f", (double)req.duration_for_drift);
      imu_filter->setDuration(req.duration_for_drift);
     return true;
   }

int main (int argc, char **argv)
{
  ros::init (argc, argv, "ImuFilter");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  imu_filter = new ImuFilterRos (nh, nh_private);
  ros::ServiceServer service = nh.advertiseService<imu_filter_madgwick::ImuDrift::Request>("startCompensate", startComp);
  ros::spin();
 
  return 0;
}
