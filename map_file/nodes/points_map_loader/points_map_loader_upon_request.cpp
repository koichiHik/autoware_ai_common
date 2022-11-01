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

#include <condition_variable>
#include <queue>
#include <thread>
#include <boost/filesystem.hpp>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/Bool.h>
#include <tf/transform_listener.h>

#include "autoware_msgs/LaneArray.h"

#include "map_file/get_file.h"

namespace
{

ros::Publisher pcd_pub;
ros::Publisher stat_pub;
std_msgs::Bool stat_msg;

sensor_msgs::PointCloud2 create_pcd(const std::vector<std::string>& pcd_paths, int* ret_err = NULL)
{
  sensor_msgs::PointCloud2 pcd, part;
  for (const std::string& path : pcd_paths)
  {
    // Following outputs are used for progress bar of Runtime Manager.
    if (pcd.width == 0)
    {
      if (pcl::io::loadPCDFile(path.c_str(), pcd) == -1)
      {
        std::cerr << "load failed " << path << std::endl;
        if (ret_err)
          *ret_err = 1;
      }
    }
    else
    {
      if (pcl::io::loadPCDFile(path.c_str(), part) == -1)
      {
        std::cerr << "load failed " << path << std::endl;
        if (ret_err)
          *ret_err = 1;
      }
      pcd.width += part.width;
      pcd.row_step += part.row_step;
      pcd.data.insert(pcd.data.end(), part.data.begin(), part.data.end());
    }
    std::cerr << "load " << path << std::endl;
    if (!ros::ok())
      break;
  }

  return pcd;
}

void publish_pcd(sensor_msgs::PointCloud2 pcd, const int* errp = NULL)
{
  if (pcd.width != 0)
  {
    pcd.header.frame_id = "map";
    pcd_pub.publish(pcd);

    if (errp == NULL || *errp == 0)
    {
      stat_msg.data = true;
      stat_pub.publish(stat_msg);
    }
  }
}


}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "points_map_loader");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  pcd_pub = nh.advertise<sensor_msgs::PointCloud2>("points_map", 1, true);
  stat_pub = nh.advertise<std_msgs::Bool>("pmap_stat", 1, true);

  stat_msg.data = false;
  stat_pub.publish(stat_msg);

  int err = 0;
  std::vector<std::string> pcd_file_paths;
  publish_pcd(create_pcd(pcd_file_paths, &err), &err);

  ros::spin();

  return 0;
}
