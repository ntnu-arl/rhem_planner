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

#ifndef RRTTREE_H_
#define RRTTREE_H_

#include <ros/ros.h>
#include <ros/package.h>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <kdtree/kdtree.h>
#include <bsp_planner/tree.h>

#define SQ(x) ((x)*(x))
#define SQRT2 0.70711

namespace bspPlanning {

class RrtTree : public TreeBase< Eigen::Vector4d >
{
 public:
  typedef Eigen::Vector4d StateVec;

  RrtTree();
  RrtTree(volumetric_mapping::OctomapManager * manager, ros::NodeHandle * nh_private=NULL);
  ~RrtTree();
  virtual void setStateFromPoseMsg(const geometry_msgs::PoseWithCovarianceStamped& pose);
  virtual void setStateFromOdometryMsg(const nav_msgs::Odometry& pose);

  //Nbvp & Bsp planning, reliant on system belief information
  virtual void initialize(int numRuns, rovio::BSP_SrvSendFilterState::Response& bspSendFilterStateSrv_response);
  virtual void iterate(int numRuns, rovio::BSP_SrvSendFilterState::Response& bspSendFilterStateSrv_response);
  virtual bool resampleBestEdge(int numReRuns, rovio::BSP_SrvSendFilterState::Response& bspSendFilterStateSrv_response);

  virtual std::vector<geometry_msgs::Pose> getBestBranch(std::string targetFrame);
  virtual std::vector<geometry_msgs::Pose> getBestEdge(std::string targetFrame);

  //NBVP_PLANLEVEL vs BSP_PLANLEVEL used to modify markers to add
  virtual void publishPath(bspPlanning::PlanningLevel planninglevel=NBVP_PLANLEVEL);
  virtual void publishBestPath(bspPlanning::PlanningLevel planninglevel=NBVP_PLANLEVEL);

  virtual void clear();
  virtual std::vector<geometry_msgs::Pose> getPathBackToPrevious(std::string targetFrame);
  virtual void memorizeBestBranch();
  void publishNode(Node<StateVec> * node, bspPlanning::PlanningLevel planninglevel=NBVP_PLANLEVEL, int nodeorder=0);
  std::tuple<double, unsigned int, unsigned int, unsigned int, double, double, double> gain(StateVec state);
  std::vector<geometry_msgs::Pose> samplePath(StateVec start, StateVec end, std::string targetFrame, bool useConstraints);

 protected:
  //Nbvp: NBVP-level tree
  kdtree * kdTree_;

  //Bsp: BSP-level nested sub-tree
  kdtree * kdSubTree_; //TODO: Can we live with clear()-ing the 1st-level kdTree ?
  std::vector<Eigen::Vector3d> vecLandmarks_; 

  //Visualization marker id(s)
  int g_ID_S_;
  int g_ID_r_;
  int g_ID_Sr_;
  int n_ID_;
  int n_ID_r_;

  std::stack<StateVec> history_;
  std::vector<StateVec> bestBranchMemory_;
  int g_ID_;
  int iterationCount_;

  //Tf(s)
  tf::TransformListener tfListener_;
};
}

#endif
