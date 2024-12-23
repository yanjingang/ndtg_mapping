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

/*
 Message counter in subscriber's queue.

 Yuki KITSUKAWA
 */

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/PointCloud2.h>

static int enqueue = 0;
static int dequeue = 0;

static void points_callback(const sensor_msgs::PointCloud2::ConstPtr &input)
{
  enqueue++;
}

static void ndt_map_callback(const sensor_msgs::PointCloud2::ConstPtr &input)
{
  dequeue++;

  std::cout << "(Processed/Input): (" << dequeue << " / " << enqueue << ")" << std::endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "queue_counter");

  ros::NodeHandle nh("~");
  std::string lidar_topic = "/points_raw";
  std::string map_topic = "/ndtg/map";
  nh.getParam("lidar_topic", lidar_topic);
  nh.getParam("map_topic", map_topic);

  ros::NodeHandle n;
  ros::Subscriber points_sub = n.subscribe(lidar_topic, 100000, points_callback);
  ros::Subscriber ndt_map_sub = n.subscribe(map_topic, 100000, ndt_map_callback);

  ros::spin();

  return 0;
}
