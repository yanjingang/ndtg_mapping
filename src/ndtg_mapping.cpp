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
 Localization and mapping program using Normal Distributions Transform

 Yuki KITSUKAWA
 */

#define OUTPUT // If you want to output "position_log.txt", "#define OUTPUT".

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <time.h>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <point_types.h>
#include <velodyne_pointcloud/rawdata.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/ndt.h>

#include <autoware_config_msgs/ConfigNDTMapping.h>
#include <autoware_config_msgs/ConfigNDTMappingOutput.h>

#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <math.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>

struct pose
{
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
  double latitude;
  double longitude;
  double altitude;
};

// global variables
static pose previous_pose, guess_pose, guess_pose_imu, guess_pose_odom, guess_pose_imu_odom, current_pose,
    current_pose_imu, current_pose_odom, current_pose_imu_odom, ndt_pose, added_pose, localizer_pose;

static ros::Time current_scan_time;
static ros::Time previous_scan_time;
static ros::Duration scan_duration;

static double diff = 0.0;
static double diff_x = 0.0, diff_y = 0.0, diff_z = 0.0, diff_yaw; // current_pose - previous_pose
static double offset_imu_x, offset_imu_y, offset_imu_z, offset_imu_roll, offset_imu_pitch, offset_imu_yaw;
static double offset_odom_x, offset_odom_y, offset_odom_z, offset_odom_roll, offset_odom_pitch, offset_odom_yaw;
static double offset_imu_odom_x, offset_imu_odom_y, offset_imu_odom_z, offset_imu_odom_roll, offset_imu_odom_pitch,
    offset_imu_odom_yaw;

static double current_velocity_x = 0.0;
static double current_velocity_y = 0.0;
static double current_velocity_z = 0.0;

static double current_velocity_imu_x = 0.0;
static double current_velocity_imu_y = 0.0;
static double current_velocity_imu_z = 0.0;

static pcl::PointCloud<pcl::PointXYZI> map;

static pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;

// Default values
static int max_iter = 30;       // Maximum iterations
static float resolution = 1.0;  // Resolution
static double step_size = 0.1;  // Step size
static double trans_eps = 0.01; // Transformation epsilon

// Leaf size of VoxelGrid filter.
static double voxel_leaf_size = 2.0;

static ros::Time callback_start, callback_end, t1_start, t1_end, t2_start, t2_end, t3_start, t3_end, t4_start, t4_end,
    t5_start, t5_end;
static ros::Duration d_callback, d1, d2, d3, d4, d5;

static ros::Publisher ndt_map_pub;
static ros::Publisher current_pose_pub;
static ros::Publisher guess_pose_linaer_pub;
static geometry_msgs::PoseStamped current_pose_msg, guess_pose_msg;

static ros::Publisher ndt_stat_pub;
static std_msgs::Bool ndt_stat_msg;

static int initial_scan_loaded = 0;

static Eigen::Matrix4f gnss_transform = Eigen::Matrix4f::Identity();

static double min_scan_range = 5.0;
static double max_scan_range = 200.0;
static double min_add_scan_shift = 1.0;

static double _tf_x, _tf_y, _tf_z, _tf_roll, _tf_pitch, _tf_yaw;
static Eigen::Matrix4f tf_btol, tf_ltob;

static bool _use_imu = false;
static bool _use_odom = false;
static bool _use_gps = false;
static bool _imu_upside_down = false;

static bool _incremental_voxel_update = false;

static std::string _lidar_topic = "/points_raw";
static std::string _imu_topic = "/imu_raw";
static std::string _odom_topic = "/vehicle/odom";
static std::string _gps_topic = "/gps/fix";
static std::string _map_topic = "/ndtg/map";
static std::string _pose_topic = "/ndtg/current_pose";
static std::string _gps_path_topic = "/gps/path";

static std::string _save_pcd_path = "~/autoware_shared_dir/bag/map-6-park/pcd/";

static double fitness_score;
static bool has_converged;
static int final_num_iteration;
static double transformation_probability;

static sensor_msgs::Imu imu;
static nav_msgs::Odometry odom;

static std::ofstream ofs;
static std::string filename;

// // gps path pub
double rad(double d)
{
  return d * 3.1415926 / 180.0;
}
static double EARTH_RADIUS = 6378.137; // 地球半径
ros::Publisher gps_path_pub;
nav_msgs::Path ros_path;

bool init;
pose init_pose;
pose last_pose;

// 发布gps path的tf变换
void publishTFFrames(const geometry_msgs::PoseStamped &current_position)
{
  static tf2_ros::TransformBroadcaster tf_broadcaster;

  // 发布GPS轨迹原点的TF坐标
  geometry_msgs::TransformStamped init_tf;
  init_tf.header.stamp = ros::Time::now();
  init_tf.header.frame_id = "path"; // 假设GPS轨迹原点位于world坐标系
  init_tf.child_frame_id = "gps_origin";
  init_tf.transform.translation.x = 0.0;
  init_tf.transform.translation.y = 0.0;
  init_tf.transform.translation.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, 0);
  init_tf.transform.rotation = tf2::toMsg(q);
  tf_broadcaster.sendTransform(init_tf);

  // 发布当前轨迹位置的TF坐标
  geometry_msgs::TransformStamped current_tf;
  current_tf.header.stamp = ros::Time::now();
  current_tf.header.frame_id = "path"; // 假设当前轨迹位置位于world坐标系
  current_tf.child_frame_id = "current_path_position";
  current_tf.transform.translation.x = current_position.pose.position.x;
  current_tf.transform.translation.y = current_position.pose.position.y;
  current_tf.transform.translation.z = current_position.pose.position.z;
  current_tf.transform.rotation = current_position.pose.orientation;
  q.setRPY(0, 0, 0); // 设置姿态为单位四元数
  current_tf.transform.rotation = tf2::toMsg(q);
  tf_broadcaster.sendTransform(current_tf);
}

/**
 * gps数据处理
 */
static void gps_callback(const sensor_msgs::NavSatFix::Ptr &msg)
{
  ROS_INFO("--------------------- %s ---------------------", __func__);
  if (!init)
  {
    ROS_INFO("init");
    init_pose.latitude = msg->latitude;
    init_pose.longitude = msg->longitude;
    init_pose.altitude = msg->altitude;
    init = true;
  }
  else
  {
    ROS_INFO("calc rad pos");
    // 计算相对源点偏移位置
    double radLat1, radLat2, radLong1, radLong2, delta_lat, delta_long, x, y;
    radLat1 = rad(init_pose.latitude);
    radLong1 = rad(init_pose.longitude);
    radLat2 = rad(msg->latitude);
    radLong2 = rad(msg->longitude);
    // 计算x
    delta_lat = radLat2 - radLat1;
    delta_long = 0;
    if (delta_lat > 0)
      x = -2 * asin(sqrt(pow(sin(delta_lat / 2), 2) + cos(radLat1) * cos(radLat2) * pow(sin(delta_long / 2), 2)));
    else
      x = 2 * asin(sqrt(pow(sin(delta_lat / 2), 2) + cos(radLat1) * cos(radLat2) * pow(sin(delta_long / 2), 2)));
    x = x * EARTH_RADIUS * 1000;

    // 计算y
    delta_lat = 0;
    delta_long = radLong2 - radLong1;
    if (delta_long > 0)
      y = 2 * asin(sqrt(pow(sin(delta_lat / 2), 2) + cos(radLat2) * cos(radLat2) * pow(sin(delta_long / 2), 2)));
    else
      y = -2 * asin(sqrt(pow(sin(delta_lat / 2), 2) + cos(radLat2) * cos(radLat2) * pow(sin(delta_long / 2), 2)));
    y = y * EARTH_RADIUS * 1000;

    // 计算z
    double z = msg->altitude - init_pose.altitude;

    // 发布轨迹
    // 更新当前位置
    geometry_msgs::PoseStamped current_position;
    current_position.header.frame_id = "path";
    current_position.header.stamp = ros::Time::now();
    current_position.pose.position.x = x;
    current_position.pose.position.y = y;
    current_position.pose.position.z = z;

    publishTFFrames(current_position);
    ros_path.header.frame_id = "path";
    ros_path.header.stamp = ros::Time::now();
    geometry_msgs::PoseStamped pose;
    pose.header = ros_path.header;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;
    ros_path.poses.push_back(pose);

    gps_path_pub.publish(ros_path);
    ROS_INFO("GPS Path: %0.6f ,%0.6f ,%0.6f  Move: %0.6f ,%0.6f ,%0.6f", x, y, z, x - last_pose.x, y - last_pose.y, z - last_pose.z);

    last_pose.x = x;
    last_pose.y = y;
    last_pose.z = z;
    last_pose.latitude = msg->latitude;
    last_pose.longitude = msg->longitude;
    last_pose.altitude = msg->altitude;
  }
}

/**
 * 利用里程计与imu联合计算当前初始位置
 *  主要是通过里程计来推算速度和偏移位移距离，imu来推算转角的变化量
 */
static void imu_odom_calc(ros::Time current_time)
{
  static ros::Time previous_time = current_time;
  double diff_time = (current_time - previous_time).toSec();  // 两帧间的时间差
  // imu在该时间差内的角速度变化量
  double diff_imu_roll = imu.angular_velocity.x * diff_time;
  double diff_imu_pitch = imu.angular_velocity.y * diff_time;
  double diff_imu_yaw = imu.angular_velocity.z * diff_time;

  current_pose_imu_odom.roll += diff_imu_roll;  // 累加变化量
  current_pose_imu_odom.pitch += diff_imu_pitch;
  current_pose_imu_odom.yaw += diff_imu_yaw;
  // 里程计在diff_time时间内x轴上变化距离
  double diff_distance = odom.twist.twist.linear.x * diff_time;
  // 变化距离作为imu和里程计的偏差值
  offset_imu_odom_x += diff_distance * cos(-current_pose_imu_odom.pitch) * cos(current_pose_imu_odom.yaw);
  offset_imu_odom_y += diff_distance * cos(-current_pose_imu_odom.pitch) * sin(current_pose_imu_odom.yaw);
  offset_imu_odom_z += diff_distance * sin(-current_pose_imu_odom.pitch);

  offset_imu_odom_roll += diff_imu_roll;
  offset_imu_odom_pitch += diff_imu_pitch;
  offset_imu_odom_yaw += diff_imu_yaw;

  guess_pose_imu_odom.x = previous_pose.x + offset_imu_odom_x;
  guess_pose_imu_odom.y = previous_pose.y + offset_imu_odom_y;
  guess_pose_imu_odom.z = previous_pose.z + offset_imu_odom_z;
  guess_pose_imu_odom.roll = previous_pose.roll + offset_imu_odom_roll;
  guess_pose_imu_odom.pitch = previous_pose.pitch + offset_imu_odom_pitch;
  guess_pose_imu_odom.yaw = previous_pose.yaw + offset_imu_odom_yaw;

  previous_time = current_time;
}

/**
 * 利用里程计来计算ndt配准时的初始值
 *  首先获取两帧的时间差，假设微小时间差内可以认为是匀速运动，来计算微小时间间隔内的里程计角度变化量
 *  ，利用前一帧位置旋转加上变化角度，更新当前odm位置的角度。
 *  然后计算odom的偏差距离和偏差角度，最后对初始位置进行修正=前一帧位置+偏差位置，得到利用odom计算的初始位置。
 */
static void odom_calc(ros::Time current_time)
{
  static ros::Time previous_time = current_time;
  double diff_time = (current_time - previous_time).toSec();  // 获取前后两帧时间差
  // 计算两帧间隔内的里程计旋转角度
  double diff_odom_roll = odom.twist.twist.angular.x * diff_time;
  double diff_odom_pitch = odom.twist.twist.angular.y * diff_time;
  double diff_odom_yaw = odom.twist.twist.angular.z * diff_time;
  // 加上两帧间的角度大小，更新当前里程计位置的角度
  current_pose_odom.roll += diff_odom_roll;
  current_pose_odom.pitch += diff_odom_pitch;
  current_pose_odom.yaw += diff_odom_yaw;

  double diff_distance = odom.twist.twist.linear.x * diff_time; // 表示在diff_time时间内，x方向的变化距离
  // offset表示车身不稳定造成的计算偏差
  offset_odom_x += diff_distance * cos(-current_pose_odom.pitch) * cos(current_pose_odom.yaw);
  offset_odom_y += diff_distance * cos(-current_pose_odom.pitch) * sin(current_pose_odom.yaw);
  offset_odom_z += diff_distance * sin(-current_pose_odom.pitch);

  offset_odom_roll += diff_odom_roll;
  offset_odom_pitch += diff_odom_pitch;
  offset_odom_yaw += diff_odom_yaw;
  // 对初始位置进行修正=前一帧位置+偏差位置
  guess_pose_odom.x = previous_pose.x + offset_odom_x;
  guess_pose_odom.y = previous_pose.y + offset_odom_y;
  guess_pose_odom.z = previous_pose.z + offset_odom_z;
  guess_pose_odom.roll = previous_pose.roll + offset_odom_roll;
  guess_pose_odom.pitch = previous_pose.pitch + offset_odom_pitch;
  guess_pose_odom.yaw = previous_pose.yaw + offset_odom_yaw;

  previous_time = current_time;
}

/**
 * 利用imu来计算初始位置
 *  计算两帧时间差内的角度变化量
 */
static void imu_calc(ros::Time current_time)
{
  static ros::Time previous_time = current_time;  //获取上一帧时间
  double diff_time = (current_time - previous_time).toSec();  //计算前后两帧的时间差

  double diff_imu_roll = imu.angular_velocity.x * diff_time;  //计算在diff_time中角速度的变化量
  double diff_imu_pitch = imu.angular_velocity.y * diff_time;
  double diff_imu_yaw = imu.angular_velocity.z * diff_time;

  current_pose_imu.roll += diff_imu_roll; //当前位置的旋转角度
  current_pose_imu.pitch += diff_imu_pitch;
  current_pose_imu.yaw += diff_imu_yaw;

  // 计算x,y,z方向上的加速度
  double accX1 = imu.linear_acceleration.x;
  double accY1 = std::cos(current_pose_imu.roll) * imu.linear_acceleration.y -
                 std::sin(current_pose_imu.roll) * imu.linear_acceleration.z;
  double accZ1 = std::sin(current_pose_imu.roll) * imu.linear_acceleration.y +
                 std::cos(current_pose_imu.roll) * imu.linear_acceleration.z;

  double accX2 = std::sin(current_pose_imu.pitch) * accZ1 + std::cos(current_pose_imu.pitch) * accX1;
  double accY2 = accY1;
  double accZ2 = std::cos(current_pose_imu.pitch) * accZ1 - std::sin(current_pose_imu.pitch) * accX1;

  double accX = std::cos(current_pose_imu.yaw) * accX2 - std::sin(current_pose_imu.yaw) * accY2;
  double accY = std::sin(current_pose_imu.yaw) * accX2 + std::cos(current_pose_imu.yaw) * accY2;
  double accZ = accZ2;

  // 前后两帧时间内，车辆xyz轴方向为匀加速运动
  //    其中accX,accY,accZ表示的是xyz三个方向上的加速度，分别又有上面的各方向加速度来合成的合加速度
  offset_imu_x += current_velocity_imu_x * diff_time + accX * diff_time * diff_time / 2.0;
  offset_imu_y += current_velocity_imu_y * diff_time + accY * diff_time * diff_time / 2.0;
  offset_imu_z += current_velocity_imu_z * diff_time + accZ * diff_time * diff_time / 2.0;
  // 当前的速度增量
  current_velocity_imu_x += accX * diff_time; //加速度乘以两帧的时间差
  current_velocity_imu_y += accY * diff_time;
  current_velocity_imu_z += accZ * diff_time;
  // offset_imu变化角度=两帧时间内的变化角度
  offset_imu_roll += diff_imu_roll;
  offset_imu_pitch += diff_imu_pitch;
  offset_imu_yaw += diff_imu_yaw;

  // 等于上一帧的变化位置加上两帧间的偏差量
  guess_pose_imu.x = previous_pose.x + offset_imu_x;
  guess_pose_imu.y = previous_pose.y + offset_imu_y;
  guess_pose_imu.z = previous_pose.z + offset_imu_z;
  guess_pose_imu.roll = previous_pose.roll + offset_imu_roll;
  guess_pose_imu.pitch = previous_pose.pitch + offset_imu_pitch;
  guess_pose_imu.yaw = previous_pose.yaw + offset_imu_yaw;

  previous_time = current_time;
}

static double wrapToPm(double a_num, const double a_max)
{
  if (a_num >= a_max)
  {
    a_num -= 2.0 * a_max;
  }
  return a_num;
}

static double wrapToPmPi(double a_angle_rad)
{
  return wrapToPm(a_angle_rad, M_PI);
}

static double calcDiffForRadian(const double lhs_rad, const double rhs_rad)
{
  double diff_rad = lhs_rad - rhs_rad;
  if (diff_rad >= M_PI)
    diff_rad = diff_rad - 2 * M_PI;
  else if (diff_rad < -M_PI)
    diff_rad = diff_rad + 2 * M_PI;
  return diff_rad;
}

/**
 * odom_callback函数以里程计接受到的数据作为函数参数，通过调用odom_calc，通过接受时间戳来求得ndt的初始位姿
 */
static void odom_callback(const nav_msgs::Odometry::ConstPtr &input)
{
  ROS_INFO("--------------------- %s ---------------------", __func__);

  odom = *input;
  odom_calc(input->header.stamp); // 用来计算ndt配准时需要的初始化坐标
}

static void imuUpsideDown(const sensor_msgs::Imu::Ptr input)
{
  double input_roll, input_pitch, input_yaw;

  tf::Quaternion input_orientation;
  tf::quaternionMsgToTF(input->orientation, input_orientation);
  tf::Matrix3x3(input_orientation).getRPY(input_roll, input_pitch, input_yaw);

  input->angular_velocity.x *= -1;
  input->angular_velocity.y *= -1;
  input->angular_velocity.z *= -1;

  input->linear_acceleration.x *= -1;
  input->linear_acceleration.y *= -1;
  input->linear_acceleration.z *= -1;

  input_roll *= -1;
  input_pitch *= -1;
  input_yaw *= -1;

  input->orientation = tf::createQuaternionMsgFromRollPitchYaw(input_roll, input_pitch, input_yaw);
}

/**
 * 另一种定位方式，利用imu来计算位置初值，为Ndt配准提供初始位置
 */
static void imu_callback(const sensor_msgs::Imu::Ptr &input)
{
  // ROS_INFO("--------------------- %s ---------------------", __func__);

  if (_imu_upside_down)
    imuUpsideDown(input);

  const ros::Time current_time = input->header.stamp; // 获取imu此时的时间
  static ros::Time previous_time = current_time;
  const double diff_time = (current_time - previous_time).toSec(); // 计算前后两次消息的时间偏差

  double imu_roll, imu_pitch, imu_yaw;
  tf::Quaternion imu_orientation;                                      // imu旋转四元数
  tf::quaternionMsgToTF(input->orientation, imu_orientation);          // imu四元数消息转化为四元数存入imu_orientation
  tf::Matrix3x3(imu_orientation).getRPY(imu_roll, imu_pitch, imu_yaw); // 获取imu此时的rpy旋转角度

  imu_roll = wrapToPmPi(imu_roll); // 将角度化为弧度
  imu_pitch = wrapToPmPi(imu_pitch);
  imu_yaw = wrapToPmPi(imu_yaw);

  static double previous_imu_roll = imu_roll, previous_imu_pitch = imu_pitch, previous_imu_yaw = imu_yaw;
  const double diff_imu_roll = calcDiffForRadian(imu_roll, previous_imu_roll);
  const double diff_imu_pitch = calcDiffForRadian(imu_pitch, previous_imu_pitch);
  const double diff_imu_yaw = calcDiffForRadian(imu_yaw, previous_imu_yaw);

  imu.header = input->header;
  imu.linear_acceleration.x = input->linear_acceleration.x; // 获取imu在x方向上的线性加速度
  // imu.linear_acceleration.y = input->linear_acceleration.y;
  // imu.linear_acceleration.z = input->linear_acceleration.z;
  imu.linear_acceleration.y = 0;
  imu.linear_acceleration.z = 0;

  if (diff_time != 0) //如果这个微小的时间diff_time不为0，则Imu保持工作状态
  {
    imu.angular_velocity.x = diff_imu_roll / diff_time; // 计算imu的瞬时角速度
    imu.angular_velocity.y = diff_imu_pitch / diff_time;
    imu.angular_velocity.z = diff_imu_yaw / diff_time;
  }
  else  //如果微小的时间为0的话，imu的角速度为0
  {
    imu.angular_velocity.x = 0;
    imu.angular_velocity.y = 0;
    imu.angular_velocity.z = 0;
  }

  imu_calc(input->header.stamp); //利用imu_calc计算位置初值，为ndt提供初始位置

  previous_time = current_time; // 更新前一帧时间
  previous_imu_roll = imu_roll;
  previous_imu_pitch = imu_pitch;
  previous_imu_yaw = imu_yaw;
}

/**
 * 对输入的点云做一些处理
 *    该函数的参数为激光雷达所获取的激光点云数据(ros形式的)
 */
static void points_callback(const sensor_msgs::PointCloud2::ConstPtr &input)
{
  ROS_INFO("--------------------- %s ---------------------", __func__);
  double r;         // 表示激光点云到激光雷达的距离,主要是用于滤除距离车体较近或者较远的点云
  pcl::PointXYZI p; // 表示原始激光点云的点对象
  pcl::PointCloud<pcl::PointXYZI> tmp, scan;  //temp表示临时的原始点云数据，scan表示滤除距离激光雷达过近或者过远的点云数据
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  tf::Quaternion q;

  Eigen::Matrix4f t_localizer(Eigen::Matrix4f::Identity()); //激光雷达相对于map的变换矩阵
  Eigen::Matrix4f t_base_link(Eigen::Matrix4f::Identity()); //车体相对于map的变换矩阵
  static tf::TransformBroadcaster br; //声明一个tf发布者br
  tf::Transform transform;  //声明一个变换对象transform

  current_scan_time = input->header.stamp; // 获取当前点云扫描时间

  pcl::fromROSMsg(*input, tmp); // 将当前点云pcl消息类型转化为ros类型，并存入tmp

  /*注意：
    NDT是将激光雷达获取到的激光点云与地图目标点云进行配准；
    激光点云是相对于激光雷达坐标系的，所以进行NDT配准时求出的是激光雷达相对于全局地图坐标系map的变换矩阵：t_localizer
    因此要想求出车身底盘相对于全局map坐标系的变换关系，需要在t_localizer的基础上补偿一个激光雷达与车身底盘之间的变换矩阵tf_ltob
  */

  // 1. 将tmp点云容器内的点进行逐一处理，去除不符合距离范围内的点云数据
  for (pcl::PointCloud<pcl::PointXYZI>::const_iterator item = tmp.begin(); item != tmp.end(); item++)
  {
    p.x = (double)item->x;
    p.y = (double)item->y;
    p.z = (double)item->z;
    p.intensity = (double)item->intensity;

    r = sqrt(pow(p.x, 2.0) + pow(p.y, 2.0)); // 计算点与激光雷达的距离r，若小于最小距离或大于最大距离则滤除该点
    if (min_scan_range < r && r < max_scan_range)
    {
      scan.push_back(p);
    }
  }

  // 2.然后将初始化点云加入至地图，若点云地图没有初始化载入，则将第一帧图像作为初始图像，然后将配准之后的图像逐帧加入map。通过tf_btol变换矩阵将原始点云进行转化。tf_btol是车辆在起始位置是不在全局地图原点时的变换矩阵。然后对原始输入点云进行体素过滤，选择不同的方法进行参数设置
  pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(scan));

  // 将初始化点云加入至地图 Add initial point cloud to velodyne_map
  if (initial_scan_loaded == 0) // 若点云地图没有初始化
  {
    pcl::transformPointCloud(*scan_ptr, *transformed_scan_ptr, tf_btol); // 通过tf_btol变换矩阵将原始点云进行转化。tf_btol是车辆在起始位置是不在全局地图原点时的变换矩阵
    map += *transformed_scan_ptr;
    initial_scan_loaded = 1;
  }

  // 对原始输入点云进行体素过滤 Apply voxelgrid filter
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
  voxel_grid_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
  voxel_grid_filter.setInputCloud(scan_ptr);
  voxel_grid_filter.filter(*filtered_scan_ptr); //结果保存至filtered_scan_ptr

  pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>(map));
  // 进行ndt参数设置
  ndt.setTransformationEpsilon(trans_eps);  //表示在ndt中，平移向量和旋转向量的临界递增量，收敛条件
  ndt.setStepSize(step_size);
  ndt.setResolution(resolution);
  ndt.setMaximumIterations(max_iter);
  ndt.setInputSource(filtered_scan_ptr);  //设置输入点云是上面过滤掉的filtered_scan_ptr

  static bool is_first_map = true;  //将第一张地图map_ptr设置输入ndt输入点云
  if (is_first_map == true)
  {
    ndt.setInputTarget(map_ptr);  //全局地图初始化之后，将map_ptr作为输入目标点云，而源点云为每一次接收到的降采样过滤原始点云filtered_scan_ptr
    is_first_map = false;
  }

  // 3.guess_pose是ndt配准时候的初始位置，该位置一般由前一帧位置加上微小时间段内的变化，当采用imu或odom时可以利用其进行辅助精确定位初始位置。
  //    如果未使用imu以及odom则使用原来的guess_pose
  //    初始位置 = 前一帧位置+位置的变化
  //    初始位置的偏航角与转弯有关 = 前一帧的偏航角+偏航角的变化
  guess_pose.x = previous_pose.x + diff_x;
  guess_pose.y = previous_pose.y + diff_y;
  guess_pose.z = previous_pose.z + diff_z;
  guess_pose.roll = previous_pose.roll;
  guess_pose.pitch = previous_pose.pitch;
  guess_pose.yaw = previous_pose.yaw + diff_yaw;

  //选择使用初值的计算方法
  if (_use_imu == true && _use_odom == true)  // imu+odom
    imu_odom_calc(current_scan_time);
  if (_use_imu == true && _use_odom == false) // 仅imu 
    imu_calc(current_scan_time);
  if (_use_imu == false && _use_odom == true) // 仅里程计
    odom_calc(current_scan_time);
  // 获取对应方法计算后的初值
  pose guess_pose_for_ndt;
  if (_use_imu == true && _use_odom == true)
    guess_pose_for_ndt = guess_pose_imu_odom;
  else if (_use_imu == true && _use_odom == false)
    guess_pose_for_ndt = guess_pose_imu;
  else if (_use_imu == false && _use_odom == true)
    guess_pose_for_ndt = guess_pose_odom;
  else
    guess_pose_for_ndt = guess_pose;  //未使用里程计或者imu则使用guess_pose

  // 利用guess_pose_for_ndt的位姿旋转量来初始化xyz轴的旋转向量
  Eigen::AngleAxisf init_rotation_x(guess_pose_for_ndt.roll, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf init_rotation_y(guess_pose_for_ndt.pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf init_rotation_z(guess_pose_for_ndt.yaw, Eigen::Vector3f::UnitZ());
  // 利用guess_pose_for_ndt的三维坐标来初始化平移向量
  Eigen::Translation3f init_translation(guess_pose_for_ndt.x, guess_pose_for_ndt.y, guess_pose_for_ndt.z);

  Eigen::Matrix4f init_guess =
      (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix() * tf_btol;

  t3_end = ros::Time::now();
  d3 = t3_end - t3_start;

  t4_start = ros::Time::now();

  pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);

  // 4.进行ndt配准
  ndt.align(*output_cloud, init_guess);       //以init_guess作为初始值迭代优化，将配准结果存入outout_cloud
  fitness_score = ndt.getFitnessScore();      //目标点云与源点云之间的欧式聚类作为适应分数
  t_localizer = ndt.getFinalTransformation(); //通过ndt得到最终激光雷达相对于map的变化矩阵
  has_converged = ndt.hasConverged();         //判断是否收敛
  final_num_iteration = ndt.getFinalNumIteration(); //得到最终迭代次数
  transformation_probability = ndt.getTransformationProbability();

  // 首先求出车体相对于原点的变换矩阵t_base_link
  t_base_link = t_localizer * tf_ltob;
  // 将原始点云经过ndt变换之后输出为转换点云
  pcl::transformPointCloud(*scan_ptr, *transformed_scan_ptr, t_localizer);

  tf::Matrix3x3 mat_l, mat_b; //前者和后者分别表示激光雷达和车体相对于map的旋转矩阵
  //赋值操作
  mat_l.setValue(static_cast<double>(t_localizer(0, 0)), static_cast<double>(t_localizer(0, 1)),
                 static_cast<double>(t_localizer(0, 2)), static_cast<double>(t_localizer(1, 0)),
                 static_cast<double>(t_localizer(1, 1)), static_cast<double>(t_localizer(1, 2)),
                 static_cast<double>(t_localizer(2, 0)), static_cast<double>(t_localizer(2, 1)),
                 static_cast<double>(t_localizer(2, 2)));

  mat_b.setValue(static_cast<double>(t_base_link(0, 0)), static_cast<double>(t_base_link(0, 1)),
                 static_cast<double>(t_base_link(0, 2)), static_cast<double>(t_base_link(1, 0)),
                 static_cast<double>(t_base_link(1, 1)), static_cast<double>(t_base_link(1, 2)),
                 static_cast<double>(t_base_link(2, 0)), static_cast<double>(t_base_link(2, 1)),
                 static_cast<double>(t_base_link(2, 2)));

  // Update localizer_pose.
  localizer_pose.x = t_localizer(0, 3);
  localizer_pose.y = t_localizer(1, 3);
  localizer_pose.z = t_localizer(2, 3);
  //通过mat_l.getRPY()来设置localizer_pose的旋转rpy角度
  mat_l.getRPY(localizer_pose.roll, localizer_pose.pitch, localizer_pose.yaw, 1);

  // 5.将ndt配准的位置作为当前位置，并且以当前位置设置坐标系，并发布坐标变换信息。
  ndt_pose.x = t_base_link(0, 3); // 更新ndt_pose获取ndt配准之后的位置（ndt_pose表示ndt配准后车辆在全局地图中的位姿）
  ndt_pose.y = t_base_link(1, 3);
  ndt_pose.z = t_base_link(2, 3);
  mat_b.getRPY(ndt_pose.roll, ndt_pose.pitch, ndt_pose.yaw, 1);
  // 将此时的ndt_pose作为当前时刻的位姿current_pose
  current_pose.x = ndt_pose.x; 
  current_pose.y = ndt_pose.y;
  current_pose.z = ndt_pose.z;
  current_pose.roll = ndt_pose.roll;
  current_pose.pitch = ndt_pose.pitch;
  current_pose.yaw = ndt_pose.yaw;
  // 以当前位置作为坐标原点
  transform.setOrigin(tf::Vector3(current_pose.x, current_pose.y, current_pose.z)); // 以当前位置作为坐标原点
  q.setRPY(current_pose.roll, current_pose.pitch, current_pose.yaw);                // 根据当前位置旋转角度rpy，设置旋转四元数q
  transform.setRotation(q);                                                         // 利用q来设置旋转

  br.sendTransform(tf::StampedTransform(transform, current_scan_time, "map", "base_link")); // 发布坐标变换信息
  // 计算激光雷达扫描间隔时间
  scan_duration = current_scan_time - previous_scan_time;
  double secs = scan_duration.toSec();

  // 计算相邻帧的位姿偏差 Calculate the offset (curren_pos - previous_pos)
  diff_x = current_pose.x - previous_pose.x;
  diff_y = current_pose.y - previous_pose.y;
  diff_z = current_pose.z - previous_pose.z;
  diff_yaw = calcDiffForRadian(current_pose.yaw, previous_pose.yaw);
  diff = sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z);
  // 利用前后两帧扫描位置偏差与扫描时间间隔计算此时的瞬时速度
  current_velocity_x = diff_x / secs;
  current_velocity_y = diff_y / secs;
  current_velocity_z = diff_z / secs;
  // 同时当前位姿赋予Imu当前位姿，更新矫正
  current_pose_imu.x = current_pose.x;
  current_pose_imu.y = current_pose.y;
  current_pose_imu.z = current_pose.z;
  current_pose_imu.roll = current_pose.roll;
  current_pose_imu.pitch = current_pose.pitch;
  current_pose_imu.yaw = current_pose.yaw;
  // 同时当前位姿赋予里程计当前位姿，更新矫正
  current_pose_odom.x = current_pose.x;
  current_pose_odom.y = current_pose.y;
  current_pose_odom.z = current_pose.z;
  current_pose_odom.roll = current_pose.roll;
  current_pose_odom.pitch = current_pose.pitch;
  current_pose_odom.yaw = current_pose.yaw;
  // 同时当前位姿赋予Imu+里程计当前位姿，更新矫正
  current_pose_imu_odom.x = current_pose.x;
  current_pose_imu_odom.y = current_pose.y;
  current_pose_imu_odom.z = current_pose.z;
  current_pose_imu_odom.roll = current_pose.roll;
  current_pose_imu_odom.pitch = current_pose.pitch;
  current_pose_imu_odom.yaw = current_pose.yaw;

  current_velocity_imu_x = current_velocity_x;
  current_velocity_imu_y = current_velocity_y;
  current_velocity_imu_z = current_velocity_z;

  // 最后将当前位姿赋值给前一帧位姿，为下一帧提供初值 Update position and posture. current_pos -> previous_pos
  previous_pose.x = current_pose.x;
  previous_pose.y = current_pose.y;
  previous_pose.z = current_pose.z;
  previous_pose.roll = current_pose.roll;
  previous_pose.pitch = current_pose.pitch;
  previous_pose.yaw = current_pose.yaw;

  previous_scan_time.sec = current_scan_time.sec;
  previous_scan_time.nsec = current_scan_time.nsec;

  offset_imu_x = 0.0;
  offset_imu_y = 0.0;
  offset_imu_z = 0.0;
  offset_imu_roll = 0.0;
  offset_imu_pitch = 0.0;
  offset_imu_yaw = 0.0;

  offset_odom_x = 0.0;
  offset_odom_y = 0.0;
  offset_odom_z = 0.0;
  offset_odom_roll = 0.0;
  offset_odom_pitch = 0.0;
  offset_odom_yaw = 0.0;

  offset_imu_odom_x = 0.0;
  offset_imu_odom_y = 0.0;
  offset_imu_odom_z = 0.0;
  offset_imu_odom_roll = 0.0;
  offset_imu_odom_pitch = 0.0;
  offset_imu_odom_yaw = 0.0;

  // 计算added_pose与当前位姿之间的距离，added_pose为上一帧的车辆位姿，主要用于判断是否需要更新地图 Calculate the shift between added_pos and current_pos
  double shift = sqrt(pow(current_pose.x - added_pose.x, 2.0) + pow(current_pose.y - added_pose.y, 2.0));
  if (shift >= min_add_scan_shift)  //满足条件，将transformed_scan_ptr加入到map,完成拼接
  {
    map += *transformed_scan_ptr;
    added_pose.x = current_pose.x;
    added_pose.y = current_pose.y;
    added_pose.z = current_pose.z;
    added_pose.roll = current_pose.roll;
    added_pose.pitch = current_pose.pitch;
    added_pose.yaw = current_pose.yaw;
    //最后将当前拼接后的点云数据map_ptr设置为下一次配准的输入目标点云
    ndt.setInputTarget(map_ptr);
  }

  sensor_msgs::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::PointCloud2);  //声明ROS可用的点云类型
  pcl::toROSMsg(*map_ptr, *map_msg_ptr);  //将map_ptr -> map_msg_ptr
  ndt_map_pub.publish(*map_msg_ptr);  //发布出去

  q.setRPY(current_pose.roll, current_pose.pitch, current_pose.yaw);
  current_pose_msg.header.frame_id = "map";
  current_pose_msg.header.stamp = current_scan_time;
  current_pose_msg.pose.position.x = current_pose.x;
  current_pose_msg.pose.position.y = current_pose.y;
  current_pose_msg.pose.position.z = current_pose.z;
  current_pose_msg.pose.orientation.x = q.x();
  current_pose_msg.pose.orientation.y = q.y();
  current_pose_msg.pose.orientation.z = q.z();
  current_pose_msg.pose.orientation.w = q.w();

  current_pose_pub.publish(current_pose_msg);

  // Write log
  if (!ofs)
  {
    std::cerr << "Could not open " << filename << "." << std::endl;
    exit(1);
  }
  ofs << input->header.seq << ","
      << input->header.stamp << ","
      << input->header.frame_id << ","
      << scan_ptr->size() << ","
      << filtered_scan_ptr->size() << ","
      << std::fixed << std::setprecision(5) << current_pose.x << ","
      << std::fixed << std::setprecision(5) << current_pose.y << ","
      << std::fixed << std::setprecision(5) << current_pose.z << ","
      << current_pose.roll << ","
      << current_pose.pitch << ","
      << current_pose.yaw << ","
      << final_num_iteration << ","
      << fitness_score << ","
      << resolution << ","
      << step_size << ","
      << trans_eps << ","
      << max_iter << ","
      << voxel_leaf_size << ","
      << min_scan_range << ","
      << max_scan_range << ","
      << min_add_scan_shift << std::endl;
  //以上，是为了配准地图map并生成ros消息发布出去
  std::cout << "-----------------------------------------------------------------" << std::endl;
  std::cout << "Sequence number: " << input->header.seq << std::endl;
  std::cout << "Number of scan points: " << scan_ptr->size() << " points." << std::endl;
  std::cout << "Number of filtered scan points: " << filtered_scan_ptr->size() << " points." << std::endl;
  std::cout << "transformed_scan_ptr: " << transformed_scan_ptr->points.size() << " points." << std::endl;
  std::cout << "map: " << map.points.size() << " points." << std::endl;
  std::cout << "NDT has converged: " << has_converged << std::endl;
  std::cout << "Fitness score: " << fitness_score << std::endl;
  std::cout << "Number of iteration: " << final_num_iteration << std::endl;
  std::cout << "(x,y,z,roll,pitch,yaw):" << std::endl;
  std::cout << "(" << current_pose.x << ", " << current_pose.y << ", " << current_pose.z << ", " << current_pose.roll
            << ", " << current_pose.pitch << ", " << current_pose.yaw << ")" << std::endl;
  std::cout << "Transformation Matrix:" << std::endl;
  std::cout << t_localizer << std::endl;
  std::cout << "shift: " << shift << std::endl;
  std::cout << "-----------------------------------------------------------------" << std::endl;
}

/**
 * 保存点云地图
 */
static void save_pcd(double leaf_size, const std::string &filename)
{
  std::cout << "leaf_size: " << leaf_size << std::endl;
  std::cout << "filename: " << filename << std::endl;

  pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>(map));
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_filtered(new pcl::PointCloud<pcl::PointXYZI>());
  map_ptr->header.frame_id = "map";
  map_filtered->header.frame_id = "map";
  sensor_msgs::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::PointCloud2);

  // Apply voxelgrid filter
  if (leaf_size == 0.0)
  {
    std::cout << "Original: " << map_ptr->points.size() << " points." << std::endl;
    pcl::toROSMsg(*map_ptr, *map_msg_ptr);
  }
  else
  {
    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
    voxel_grid_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_grid_filter.setInputCloud(map_ptr);
    voxel_grid_filter.filter(*map_filtered);
    std::cout << "Original: " << map_ptr->points.size() << " points." << std::endl;
    std::cout << "Filtered: " << map_filtered->points.size() << " points." << std::endl;
    pcl::toROSMsg(*map_filtered, *map_msg_ptr);
  }

  ndt_map_pub.publish(*map_msg_ptr);

  // Writing Point Cloud data to PCD file
  if (leaf_size == 0.0)
  {
    pcl::io::savePCDFileASCII(filename, *map_ptr);
    std::cout << "Saved " << map_ptr->points.size() << " data points to " << filename << "." << std::endl;
  }
  else
  {
    pcl::io::savePCDFileASCII(filename, *map_filtered);
    std::cout << "Saved " << map_filtered->points.size() << " data points to " << filename << "." << std::endl;
  }
}

// 主函数入口
int main(int argc, char **argv)
{
  // 前一帧点云图车辆的位置
  previous_pose.x = 0.0;
  previous_pose.y = 0.0;
  previous_pose.z = 0.0;
  previous_pose.roll = 0.0;
  previous_pose.pitch = 0.0;
  previous_pose.yaw = 0.0;

  // NDT配准算法得到的车辆位置
  ndt_pose.x = 0.0;
  ndt_pose.y = 0.0;
  ndt_pose.z = 0.0;
  ndt_pose.roll = 0.0;
  ndt_pose.pitch = 0.0;
  ndt_pose.yaw = 0.0;

  // 当前帧点云图车辆的位置
  current_pose.x = 0.0;
  current_pose.y = 0.0;
  current_pose.z = 0.0;
  current_pose.roll = 0.0;
  current_pose.pitch = 0.0;
  current_pose.yaw = 0.0;

  // 当前帧点云车辆的imu坐标
  current_pose_imu.x = 0.0;
  current_pose_imu.y = 0.0;
  current_pose_imu.z = 0.0;
  current_pose_imu.roll = 0.0;
  current_pose_imu.pitch = 0.0;
  current_pose_imu.yaw = 0.0;

  // NDT配准算法所需要的初始位置
  guess_pose.x = 0.0;
  guess_pose.y = 0.0;
  guess_pose.z = 0.0;
  guess_pose.roll = 0.0;
  guess_pose.pitch = 0.0;
  guess_pose.yaw = 0.0;

  added_pose.x = 0.0;
  added_pose.y = 0.0;
  added_pose.z = 0.0;
  added_pose.roll = 0.0;
  added_pose.pitch = 0.0;
  added_pose.yaw = 0.0;

  // 前后两帧的位置与角度变化偏差量
  diff_x = 0.0;
  diff_y = 0.0;
  diff_z = 0.0;
  diff_yaw = 0.0;

  // Imu位置偏差矫正
  offset_imu_x = 0.0;
  offset_imu_y = 0.0;
  offset_imu_z = 0.0;
  offset_imu_roll = 0.0;
  offset_imu_pitch = 0.0;
  offset_imu_yaw = 0.0;

  // 里程计位置偏差矫正
  offset_odom_x = 0.0;
  offset_odom_y = 0.0;
  offset_odom_z = 0.0;
  offset_odom_roll = 0.0;
  offset_odom_pitch = 0.0;
  offset_odom_yaw = 0.0;

  // imu和里程计联合位置偏差矫正
  offset_imu_odom_x = 0.0;
  offset_imu_odom_y = 0.0;
  offset_imu_odom_z = 0.0;
  offset_imu_odom_roll = 0.0;
  offset_imu_odom_pitch = 0.0;
  offset_imu_odom_yaw = 0.0;

  // ROS初始化
  ros::init(argc, argv, "ndtg_mapping");

  // setting parameters
  ros::NodeHandle nh("~");
  nh.getParam("lidar_topic", _lidar_topic);
  nh.getParam("imu_topic", _imu_topic);
  nh.getParam("odom_topic", _odom_topic);
  nh.getParam("gps_topic", _gps_topic);

  nh.getParam("use_odom", _use_odom);
  nh.getParam("use_imu", _use_imu);
  nh.getParam("use_gps", _use_gps);
  nh.getParam("imu_upside_down", _imu_upside_down);
  nh.getParam("incremental_voxel_update", _incremental_voxel_update);

  nh.getParam("tf_x", _tf_x);
  nh.getParam("tf_y", _tf_y);
  nh.getParam("tf_z", _tf_z);
  nh.getParam("tf_yaw", _tf_yaw);
  nh.getParam("tf_pitch", _tf_pitch);
  nh.getParam("tf_roll", _tf_roll);

  nh.getParam("resolution", resolution);
  nh.getParam("step_size", step_size);
  nh.getParam("trans_eps", trans_eps);
  nh.getParam("max_iter", max_iter);
  nh.getParam("voxel_leaf_size", voxel_leaf_size);
  nh.getParam("min_scan_range", min_scan_range);
  nh.getParam("max_scan_range", max_scan_range);
  nh.getParam("min_add_scan_shift", min_add_scan_shift);

  nh.getParam("map_topic", _map_topic);
  nh.getParam("pose_topic", _pose_topic);
  nh.getParam("gps_path_topic", _gps_path_topic);
  nh.getParam("save_pcd_path", _save_pcd_path);

  std::cout << "lidar_topic: " << _lidar_topic << std::endl;
  std::cout << "imu_topic: " << _imu_topic << std::endl;
  std::cout << "odom_topic: " << _odom_topic << std::endl;
  std::cout << "gps_topic: " << _gps_topic << std::endl;

  std::cout << "use_odom: " << _use_odom << std::endl;
  std::cout << "use_imu: " << _use_imu << std::endl;
  std::cout << "use_gps: " << _use_gps << std::endl;
  std::cout << "imu_upside_down: " << _imu_upside_down << std::endl;
  std::cout << "incremental_voxel_update: " << _incremental_voxel_update << std::endl;

  std::cout << "(tf_x,tf_y,tf_z,tf_roll,tf_pitch,tf_yaw): (" << _tf_x << ", " << _tf_y << ", " << _tf_z << ", "
            << _tf_yaw << ", " << _tf_pitch << ", " << _tf_roll << ")" << std::endl;

  std::cout << "resolution: " << resolution << std::endl;
  std::cout << "step_size: " << step_size << std::endl;
  std::cout << "trans_epsilon: " << trans_eps << std::endl;
  std::cout << "max_iter: " << max_iter << std::endl;
  std::cout << "voxel_leaf_size: " << voxel_leaf_size << std::endl;
  std::cout << "min_scan_range: " << min_scan_range << std::endl;
  std::cout << "max_scan_range: " << max_scan_range << std::endl;
  std::cout << "min_add_scan_shift: " << min_add_scan_shift << std::endl;

  std::cout << "map_topic: " << _map_topic << std::endl;
  std::cout << "pose_topic: " << _pose_topic << std::endl;
  std::cout << "gps_path_topic: " << _gps_path_topic << std::endl;
  std::cout << "save_pcd_path: " << _save_pcd_path << std::endl;

  // Set log file name.
  char buffer[80];
  std::time_t now = std::time(NULL);
  std::tm *pnow = std::localtime(&now);
  std::strftime(buffer, 80, "%Y%m%d_%H%M%S", pnow);
  filename = "ndtg_mapping_" + std::string(buffer) + ".csv";
  ofs.open(filename.c_str(), std::ios::app);
  // write header for log file
  if (!ofs)
  {
    std::cerr << "Could not open " << filename << "." << std::endl;
    exit(1);
  }
  ofs << "input->header.seq" << ","
      << "input->header.stamp" << ","
      << "input->header.frame_id" << ","
      << "scan_ptr->size()" << ","
      << "filtered_scan_ptr->size()" << ","
      << "current_pose.x" << ","
      << "current_pose.y" << ","
      << "current_pose.z" << ","
      << "current_pose.roll" << ","
      << "current_pose.pitch" << ","
      << "current_pose.yaw" << ","
      << "final_num_iteration" << ","
      << "fitness_score" << ","
      << "resolution" << ","
      << "step_size" << ","
      << "trans_eps" << ","
      << "max_iter" << ","
      << "voxel_leaf_size" << ","
      << "min_scan_range" << ","
      << "max_scan_range" << ","
      << "min_add_scan_shift" << std::endl;

  /* 以下是为了计算激光雷达相对车身底盘的初始化变换矩阵
    其中tf_x,tf_y,tf_z是激光雷达相对于车身底盘的位置，既欧式坐标距离
    四大坐标系：
      world <- TF -> map <- ndt_mapping -> base_link <- TF -> localizer
    其中传感器坐标系就是localizer
      (1)world与map坐标系重合；
      (2)激光雷达与车身底盘之间的坐标变换关系是不变的；
    因此需要求的是map与base_link之间的坐标变换关系，这部分代码在下方if判断关系之后。
  */
  // 初始化平移向量tl_btol,激光雷达相对于车身底盘之间的的位置关系
  Eigen::Translation3f tl_btol(_tf_x, _tf_y, _tf_z);                // tl: translation
  Eigen::AngleAxisf rot_x_btol(_tf_roll, Eigen::Vector3f::UnitX()); // rot: rotation
  Eigen::AngleAxisf rot_y_btol(_tf_pitch, Eigen::Vector3f::UnitY());  //初始化旋转向量，绕x,y,z旋转
  Eigen::AngleAxisf rot_z_btol(_tf_yaw, Eigen::Vector3f::UnitZ());
  tf_btol = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();  //这是autoware采用的zyx欧拉角旋转方式
  tf_ltob = tf_btol.inverse();

  map.header.frame_id = "map";

  // 发布topic
  ros::NodeHandle n;
  ndt_map_pub = n.advertise<sensor_msgs::PointCloud2>(_map_topic, 1000);         // 发布点云建图
  current_pose_pub = n.advertise<geometry_msgs::PoseStamped>(_pose_topic, 1000); // 发布位姿
  gps_path_pub = n.advertise<nav_msgs::Path>(_gps_path_topic, 10);
  // 订阅topic
  ros::Subscriber points_sub = n.subscribe(_lidar_topic, 100000, points_callback);
  ros::Subscriber odom_sub = n.subscribe(_odom_topic, 100000, odom_callback);
  ros::Subscriber imu_sub = n.subscribe(_imu_topic, 100000, imu_callback);
  ros::Subscriber gps_sub = n.subscribe(_gps_topic, 100000, gps_callback);

  ros::spin();
  // ros::MultiThreadedSpinner spinner(7);
  // spinner.spin();

  // 保存pcd文件
  std::string pcd_file = _save_pcd_path + "ndtg_map_" + std::string(buffer) + ".pcd";
  save_pcd(voxel_leaf_size, pcd_file);

  return 0;
}
