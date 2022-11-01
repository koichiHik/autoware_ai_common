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


#include <autoware_msgs/String.h>
#include <autoware_msgs/LaneArray.h>

#include "map_file/get_file.h"

namespace
{

sensor_msgs::PointCloud2 create_pcd(const std::string& pcd_path, int* ret_err = NULL)
{
  sensor_msgs::PointCloud2 pcd;

  // Following outputs are used for progress bar of Runtime Manager.
  if (pcd.width == 0) {
    if (pcl::io::loadPCDFile(pcd_path.c_str(), pcd) == -1)
    {
      std::cerr << "load failed " << pcd_path << std::endl;
      if (ret_err) {
        *ret_err = 1;
      }
    }
  }
  std::cerr << "load " << pcd_path << std::endl;

  return pcd;
}

class points_map_loader_upon_request {
public:
  points_map_loader_upon_request(ros::NodeHandle &nh, ros::NodeHandle &pnh);
  
  bool publish_pcd(autoware_msgs::String::Request &req, autoware_msgs::String::Request &res);

  void run();

private:
  ros::Publisher pcd_pub_;
  ros::Publisher stat_pub_;
  ros::ServiceServer service_;

};

points_map_loader_upon_request::points_map_loader_upon_request(ros::NodeHandle &nh, ros::NodeHandle &pnh) {

  pcd_pub_ = nh.advertise<sensor_msgs::PointCloud2>("points_map", 1, true);
  stat_pub_ = nh.advertise<std_msgs::Bool>("pmap_stat", 1, true);

  std_msgs::Bool stat_msg;
  stat_msg.data = false;
  stat_pub_.publish(stat_msg);

  service_ = nh.advertiseService("points_map_load", &points_map_loader_upon_request::publish_pcd, this);
}


bool points_map_loader_upon_request::publish_pcd(autoware_msgs::String::Request &req, autoware_msgs::String::Request &res) {

  int err = 0;
  sensor_msgs::PointCloud2 pcd = create_pcd(req.str, &err);

  if (pcd.width != 0) {
    pcd.header.frame_id = "map";
    pcd_pub_.publish(pcd);

    if (err == 0) {
      std_msgs::Bool stat_msg;
      stat_msg.data = true;
      stat_pub_.publish(stat_msg);
    }
  }

  return true;
}

void points_map_loader_upon_request::run() {
  ros::spin();
}

}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "points_map_loader");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  points_map_loader_upon_request loader(nh, pnh);
  loader.run();

  return 0;
}
