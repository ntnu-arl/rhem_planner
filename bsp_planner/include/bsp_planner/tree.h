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

#ifndef TREE_H_
#define TREE_H_

#include <vector>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <octomap_world/octomap_manager.h>

//Bsp: BSP pipeline messages and services
#include <rovio/BSP_Point2fMsg.h>
#include <rovio/BSP_TrajectoryReferenceMsg.h>
#include <rovio/BSP_RobocentricFeatureElementMsg.h>
#include <rovio/BSP_StateMsg.h>
#include <rovio/BSP_FilterStateMsg.h>
#include <rovio/BSP_SrvSendFilterState.h>
#include <rovio/BSP_SrvPropagateFilterState.h>
#include <eigen_conversions/eigen_msg.h>

namespace bspPlanning {

//Planning level: (0) volumetric , (1) nested belief-space
enum PlanningLevel { NBVP_PLANLEVEL, BSP_PLANLEVEL };

struct Params
{

  //BSP service client
  ros::ServiceClient bspServiceClient_;
  
  //BSP params
  bool bspEnable_;
  double bspReplanningDistanceMin_;
  double bspReplanningDistanceRatio_;
  double bspReplanningExtensionRatio_;
  int bspReplanningInitIterations_;
  int bspReplanningCutoffIterations_;
  double bspReplanningGainRange_;
   
  //Visualization params
  double planMarker_lifetime_;
  double bestPlanMarker_lifetime_;
  double replanMarker_lifetime_;
  double bestReplanMarker_lifetime_;
  visualization_msgs::MarkerArray planningPath_markers_;
  visualization_msgs::MarkerArray planningPathStats_markers_;
  visualization_msgs::MarkerArray bestPlanningPath_markers_;
  visualization_msgs::MarkerArray rePlanningPath_markers_;
  visualization_msgs::MarkerArray rePlanningPathStats_markers_;
  visualization_msgs::MarkerArray bestRePlanningPath_markers_;

  std::vector<double> camPitch_;
  std::vector<double> camHorizontal_;
  std::vector<double> camVertical_;
  std::vector<std::vector<Eigen::Vector3d> > camBoundNormals_;

  double igProbabilistic_;
  double igFree_;
  double igOccupied_;
  double igUnmapped_;
  double gainRange_;
  double degressiveCoeff_;
  double zero_gain_;

  double v_max_;
  double dyaw_max_;
  double dOvershoot_;
  double extensionRange_;
  bool exact_root_;
  int initIterations_;
  int cutoffIterations_;
  double dt_;

  double minX_;
  double minY_;
  double minZ_;
  double maxX_;
  double maxY_;
  double maxZ_;
  Eigen::Vector3d boundingBox_;

  //Exploratory behavior modifiers
  double explorationExtensionX_;
  double explorationExtensionY_;
  double explorationMinZ_;
  double explorationMaxZ_;  
  int degressiveSwitchoffLoops_;
  Eigen::Vector3d boundingBoxOffset_;

  //Starting behavior modifiers for limited-FoV systems
  bool softStart_;
  double softStartMinZ_;
  
  //Publishers
  ros::Publisher planningWorkspace_;
  ros::Publisher planningPath_;
  ros::Publisher planningPathStats_;
  ros::Publisher bestPlanningPath_;
  ros::Publisher rePlanningPath_;
  ros::Publisher rePlanningPathStats_;
  ros::Publisher bestRePlanningPath_;
  std::string navigationFrame_;

  double pcl_throttle_;
};

template<typename stateVec>
class Node
{
 public:
  Node();
  ~Node();
  stateVec state_;
  Node * parent_;
  std::vector<Node*> children_;
  double gain_;
  double distance_;

  //Annotated Belief State
  rovio::BSP_FilterStateMsg bsp_belief_state_;

  //Characteristics
  int id_;
  bool leafNode_;

  //Stats
  unsigned int unmapped_cnt_; 
  unsigned int occupied_cnt_;
  unsigned int free_cnt_;
  double unmapped_gain_;
  double occupied_gain_;
  double free_gain_;
};

template<typename stateVec>
class TreeBase
{
 protected:
  Params params_;
  int counter_;
  double bestGain_;

  //BSP nested level re-planning
  double bestReGain_;
  double initReGain_;

  Node<stateVec> * bestNode_;
  Node<stateVec> * rootNode_;
  volumetric_mapping::OctomapManager * manager_;
  stateVec root_;
  stateVec exact_root_;

  //Bsp: Expose to constructing node's parameters (nh is used to carry them over to volumetric_mapping::OctomapManager ctor)
  //NOTE: Work-around of OctomapWorld cont having a "const std::shared_ptr<const octomap::OcTree> getOcTree() const { return octree_; }" for direct manager_->octree_ const access to params
  ros::NodeHandle * nh_private_;

 public:
  TreeBase();
  TreeBase(volumetric_mapping::OctomapManager * manager, ros::NodeHandle * nh_private=NULL);
  ~TreeBase();

  //BSP nested level sub-tree
  TreeBase* subtree_;

  //Ad-hoc clearance
  void getRootState(Eigen::Vector3d& rootState_);
  void clearRootStateBBX(Eigen::Vector3d& boundingBox, const double& minZ = -std::numeric_limits<double>::infinity());

  //BSP planning
  virtual void initialize(int numRuns, rovio::BSP_SrvSendFilterState::Response& bspSendFilterStateSrv_response) = 0;
  virtual void iterate(int numRuns, rovio::BSP_SrvSendFilterState::Response& bspSendFilterStateSrv_response) = 0;
  virtual bool resampleBestEdge(int numReRuns, rovio::BSP_SrvSendFilterState::Response& bspSendFilterStateSrv_response) = 0;
  virtual void clear() = 0;
  virtual void memorizeBestBranch() = 0;

  //Extract tree parts
  virtual std::vector<geometry_msgs::Pose> getBestEdge(std::string targetFrame) = 0;
  virtual std::vector<geometry_msgs::Pose> getBestBranch(std::string targetFrame) = 0;
  virtual std::vector<geometry_msgs::Pose> getPathBackToPrevious(std::string targetFrame) = 0;

  //Publishing
  virtual void publishPath(bspPlanning::PlanningLevel planninglevel=NBVP_PLANLEVEL) = 0;
  virtual void publishBestPath(bspPlanning::PlanningLevel planninglevel=NBVP_PLANLEVEL) = 0;

  int getCounter();
  bool gainFound();
  bool reGainFound();
  void setParams(Params params);

  void insertPointcloudWithTf(const sensor_msgs::PointCloud2::ConstPtr& pointcloud);
};
}

#endif //TREE_H_
