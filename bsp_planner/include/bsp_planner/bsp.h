/* 
 * Copyright (c) 2017, Christos Papachristos, University of Nevada, Reno
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Nevada, Reno nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * The provided code implements the functionalities of the Uncertainty-aware Receding
 * Horizon Exploration and Mapping planner. Part of the implemented functionalities
 * are derived from the Receding Horizon Next-Best View planner.
 */

#ifndef BSP_H_
#define BSP_H_

#include <vector>
#include <fstream>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <octomap_world/octomap_manager.h>

#include <bsp_planner/tree.h>
#include <bsp_planner/rrt.h>

//Planner main service
#include <nav_msgs/Path.h>
#include <bsp_planner/bsp_srv.h>

//Planning initialization states
#define BSP_INITSTATE_NONE -1
#define BSP_INITSTATE_NEEDBBXCLEARANCETOPLAN 0
#define BSP_INITSTATE_PLANNINGINITIALIZED 1

#define SQ(x) ((x)*(x))
#define SQRT2 0.70711

namespace bspPlanning {

template<typename stateVec>
class bspPlanner
{

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Subscriber posClient_;
  ros::Subscriber odomClient_;
  ros::Subscriber pointcloud_0_sub_;
  ros::Subscriber pointcloud_1_sub_;
  ros::Subscriber pointcloud_2_sub_;
  ros::Subscriber pointcloud_3_sub_;

  //Planner main service (called)
  ros::ServiceServer plannerService_;
  //Planner main publisher (responds)
  ros::Publisher plannerPub_;

  Params params_;
  volumetric_mapping::OctomapManager * manager_;

  //Planning state
  int init_state_;
  bool ready_;

  //Track number of successful planning runs
  int num_runs_ = 0;
  int num_reruns_ = 0;

 public:

  //BSP service structures
  bsp_msgs::BSP_SrvSendFilterState::Request bspMsgs_sendFilterStateSrv_request;
  bsp_msgs::BSP_SrvSendFilterState::Response bspMsgs_sendFilterStateSrv_response;

  typedef std::vector<stateVec> vector_t;
  RrtTree * tree_;

  bspPlanner(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  ~bspPlanner();
  bool setParams();
  void posCallback(const geometry_msgs::PoseWithCovarianceStamped& pose);
  void odomCallback(const nav_msgs::Odometry& pose);
  bool plannerCallback(bsp_planner::bsp_srv::Request& req, bsp_planner::bsp_srv::Response& res);
  void insertPointcloudWithTf(const sensor_msgs::PointCloud2::ConstPtr& pointcloud);
  void insertPointcloudWithTfCamUp(const sensor_msgs::PointCloud2::ConstPtr& pointcloud);
  void insertPointcloudWithTfCamDown(const sensor_msgs::PointCloud2::ConstPtr& pointcloud);
};
}

#endif // BSP_H_
