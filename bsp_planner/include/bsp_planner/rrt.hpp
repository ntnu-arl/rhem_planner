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

#ifndef RRTTREE_HPP_
#define RRTTREE_HPP_

#include <bsp_planner/rrt.h>

bspPlanning::RrtTree::RrtTree()
  : bspPlanning::TreeBase<StateVec>::TreeBase()
{
  kdTree_ = kd_create(3);
  iterationCount_ = 0;
}

bspPlanning::RrtTree::RrtTree(volumetric_mapping::OctomapManager * manager, ros::NodeHandle * nh_private)
  : bspPlanning::TreeBase<StateVec>::TreeBase(manager, nh_private)
{
  kdTree_ = kd_create(3);
  iterationCount_ = 0;
}

bspPlanning::RrtTree::~RrtTree()
{
  delete rootNode_;
  kd_free(kdTree_);
}

void bspPlanning::RrtTree::setStateFromPoseMsg(const geometry_msgs::PoseWithCovarianceStamped& pose)
{
  tf::StampedTransform transform;
  try {
    tfListener_.lookupTransform(params_.navigationFrame_, pose.header.frame_id, pose.header.stamp, transform);
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    return;
  }
  tf::Pose poseTF;
  tf::poseMsgToTF(pose.pose.pose, poseTF);
  tf::Vector3 position = poseTF.getOrigin();
  position = transform * position;
  tf::Quaternion quat = poseTF.getRotation();
  quat = transform * quat;
  root_[0] = position.x();
  root_[1] = position.y();
  root_[2] = position.z();
  root_[3] = tf::getYaw(quat);
}

void bspPlanning::RrtTree::setStateFromOdometryMsg(const nav_msgs::Odometry& pose)
{
  tf::StampedTransform transform;
  try {
    tfListener_.lookupTransform(params_.navigationFrame_, pose.header.frame_id, pose.header.stamp, transform);
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    return;
  }
  tf::Pose poseTF;
  tf::poseMsgToTF(pose.pose.pose, poseTF);
  tf::Vector3 position = poseTF.getOrigin();
  position = transform * position;
  tf::Quaternion quat = poseTF.getRotation();
  quat = transform * quat;
  root_[0] = position.x();
  root_[1] = position.y();
  root_[2] = position.z();
  root_[3] = tf::getYaw(quat);
}

void bspPlanning::RrtTree::iterate(int numRuns, rovio::BSP_SrvSendFilterState::Response& bspSendFilterStateSrv_response)
{
  //New configuration is sampled and added to the tree.
  StateVec newState;

  //Nbvp: Sample over a sphere with the radius of the maximum diagonal of the exploration space. Not biasing the tree towards the center of the exploration space.
  double radius = sqrt(SQ(params_.minX_ - params_.maxX_) + SQ(params_.minY_ - params_.maxY_) + SQ(params_.minZ_ - params_.maxZ_));

  //Bsp: Sampling for position Only in Mapped Free space
  bool solutionFound_pos = false;
  int whileThres_pos = 10000;
  while (!solutionFound_pos && whileThres_pos--) {
    for (unsigned int i = 0; i < 3; ++i) {
      newState[i] = 2.0 * radius * (((double) rand()) / ((double) RAND_MAX) - 0.5);
    }
    if (SQ(newState[0]) + SQ(newState[1]) + SQ(newState[2]) > pow(radius, 2.0)){
      continue;
    }
    newState += rootNode_->state_;
    if (!params_.softBounds_) {
      if (newState.x() + params_.boundingBoxOffset_.x() < params_.minX_ + 0.5 * params_.boundingBox_.x()) {
        continue;
      } else if (newState.y() + params_.boundingBoxOffset_.y() < params_.minY_ + 0.5 * params_.boundingBox_.y()) {
        continue;
      } else if (newState.z() + params_.boundingBoxOffset_.z() < params_.minZ_ + 0.5 * params_.boundingBox_.z()) {
        continue;
      } else if (newState.x() + params_.boundingBoxOffset_.x() > params_.maxX_ - 0.5 * params_.boundingBox_.x()) {
        continue;
      } else if (newState.y() + params_.boundingBoxOffset_.y() > params_.maxY_ - 0.5 * params_.boundingBox_.y()) {
        continue;
      } else if (newState.z() + params_.boundingBoxOffset_.z() > params_.maxZ_ - 0.5 * params_.boundingBox_.z()) {
        continue;
      }
    }
    // Bsp: Sample only in explicitly Free (Mapped) space
    if (volumetric_mapping::OctomapManager::CellStatus::kFree != manager_->getCellStatusBoundingBox(Eigen::Vector3d(newState[0],newState[1],newState[2])+params_.boundingBoxOffset_,params_.boundingBox_)){
      continue;
    }
    solutionFound_pos = true;
  }
  if (!solutionFound_pos){
    ROS_WARN("bsp_planner(iterate): newState[POS] random sampling failed to find admissible solution in Mapped Free space.");
    return;
  }

  //Find nearest neighbour
  kdres * nearest = kd_nearest3(kdTree_, newState.x(), newState.y(), newState.z());
  if (kd_res_size(nearest) <= 0) {
    kd_res_free(nearest);
    return;
  }

  bspPlanning::Node<StateVec> * newParent = (bspPlanning::Node<StateVec> *) kd_res_item_data(nearest);
  kd_res_free(nearest);

  //Check for collision of new connection plus some overshoot distance.
  Eigen::Vector3d origin(newParent->state_[0], newParent->state_[1], newParent->state_[2]);
  Eigen::Vector3d direction(newState[0] - origin[0], newState[1] - origin[1], newState[2] - origin[2]);
  if (direction.norm() > params_.extensionRange_) {
    direction = params_.extensionRange_ * direction.normalized();
  }
  newState[0] = origin[0] + direction[0];
  newState[1] = origin[1] + direction[1];
  newState[2] = origin[2] + direction[2];

  //Check collision-free transition
  if ( volumetric_mapping::OctomapManager::CellStatus::kFree == manager_->getLineStatusBoundingBox(origin+params_.boundingBoxOffset_,origin+params_.boundingBoxOffset_+direction+direction.normalized()*params_.dOvershoot_,params_.boundingBox_) ) {

    //Nbvp: Sample new orientation, create node 
    newState[3] = 2.0 * M_PI * (((double) rand()) / ((double) RAND_MAX) - 0.5);
    bspPlanning::Node<StateVec> * newNode = new bspPlanning::Node<StateVec>;
    newNode->state_ = newState;
    newNode->parent_ = newParent;
    newNode->distance_ = newParent->distance_ + direction.norm();
    newParent->children_.push_back(newNode);

    //Bsp: Compute multi-component gain
    auto newNodeGain = gain(newNode->state_);
    if (numRuns<params_.degressiveSwitchoffLoops_)
      newNode->gain_ = newParent->gain_ + std::get<0>(newNodeGain) * exp(-params_.degressiveCoeff_ * newNode->distance_);  //limit behavior to initial runs
    else
      newNode->gain_ = newParent->gain_ + std::get<0>(newNodeGain);
    newNode->unmapped_cnt_ = std::get<1>(newNodeGain);
    newNode->occupied_cnt_ = std::get<2>(newNodeGain);
    newNode->free_cnt_ = std::get<3>(newNodeGain);
    newNode->unmapped_gain_ = std::get<4>(newNodeGain);
    newNode->occupied_gain_ = std::get<5>(newNodeGain);
    newNode->free_gain_ = std::get<6>(newNodeGain);

    //Insert node into tree
    kd_insert3(kdTree_, newState.x(), newState.y(), newState.z(), newNode);

    //Bsp: Increment node id and update node leaf flags
    newNode->id_ = ++n_ID_;
    newNode->leafNode_ = true;
    newNode->parent_->leafNode_ = false;

    //Publish new node (NBVP-level variant of MarkerArray visualization)
    publishNode(newNode, bspPlanning::NBVP_PLANLEVEL);

    //Nbvp: Update best (global) IG and node
    if (newNode->gain_ > bestGain_) {
      bestGain_ = newNode->gain_;
      bestNode_ = newNode;
    }
    counter_++;
  }

}

void bspPlanning::RrtTree::initialize(int numRuns, rovio::BSP_SrvSendFilterState::Response& bspSendFilterStateSrv_response)
{
  //Initialize the tree, including insertion of remainder of previous best branch.
  g_ID_ = 0;
  g_ID_S_ = 0;
  g_ID_r_ = 0;
  g_ID_Sr_ = 0;
  n_ID_ = 0;
  n_ID_r_ = 0;

  kdTree_ = kd_create(3);

  rootNode_ = new Node<StateVec>;
  rootNode_->distance_ = 0.0;
  rootNode_->gain_ = params_.zero_gain_;
  rootNode_->parent_ = NULL;
  //Bsp: Update leaf flags
  rootNode_->leafNode_ = false;

  if (params_.exact_root_) {
    if (iterationCount_ <= 1) {
      exact_root_ = root_;
    }
    rootNode_->state_ = exact_root_;
  } else {
    rootNode_->state_ = root_;
  }

  //Bsp: Initialize system belief state
  rootNode_->bsp_belief_state_ = bspSendFilterStateSrv_response.filterState;
  rootNode_->bsp_belief_state_.header.seq = iterationCount_;
  vecLandmarks_.clear();
  for (unsigned int i=0; i<rootNode_->bsp_belief_state_.landmarks_pcl.size(); ++i){
    geometry_msgs::Point32 landmark_pt = rootNode_->bsp_belief_state_.landmarks_pcl.at(i);
    vecLandmarks_.push_back(Eigen::Vector3d(landmark_pt.x,landmark_pt.y,landmark_pt.z));
  }

  kd_insert3(kdTree_, rootNode_->state_.x(), rootNode_->state_.y(), rootNode_->state_.z(), rootNode_);
  iterationCount_++;

  //Nbvp: Insert all nodes of the remainder of the previous best branch, checking for collisions and recomputing the gain
  for (typename std::vector<StateVec>::reverse_iterator iter = bestBranchMemory_.rbegin(); iter != bestBranchMemory_.rend(); ++iter) {
    StateVec newState = *iter;
    kdres * nearest = kd_nearest3(kdTree_, newState.x(), newState.y(), newState.z());
    if (kd_res_size(nearest) <= 0) {
      kd_res_free(nearest);
      continue;
    }
    bspPlanning::Node<StateVec> * newParent = (bspPlanning::Node<StateVec> *) kd_res_item_data(nearest);
    kd_res_free(nearest);

    //Check for collision of connection
    Eigen::Vector3d origin(newParent->state_[0], newParent->state_[1], newParent->state_[2]);
    Eigen::Vector3d direction(newState[0] - origin[0], newState[1] - origin[1], newState[2] - origin[2]);
    if (direction.norm() > params_.extensionRange_) {
      direction = params_.extensionRange_ * direction.normalized();
    }
    newState[0] = origin[0] + direction[0];
    newState[1] = origin[1] + direction[1];
    newState[2] = origin[2] + direction[2];

    //Check collision-free transition
    if (volumetric_mapping::OctomapManager::CellStatus::kFree == manager_->getLineStatusBoundingBox(origin+params_.boundingBoxOffset_, origin+params_.boundingBoxOffset_+direction+direction.normalized()*params_.dOvershoot_, params_.boundingBox_) ) {

      //Create node
      bspPlanning::Node<StateVec> * newNode = new bspPlanning::Node<StateVec>;
      newNode->state_ = newState;
      newNode->parent_ = newParent;
      newNode->distance_ = newParent->distance_ + direction.norm();
      newParent->children_.push_back(newNode);

      //Bsp: Compute multi-component gain
      auto newNodeGain = gain(newNode->state_);
      if (numRuns<params_.degressiveSwitchoffLoops_){ //limit degressive behavior to initial runs (?)
        newNode->gain_ = newParent->gain_ + std::get<0>(newNodeGain) * exp(-params_.degressiveCoeff_ * newNode->distance_);
      }
      else{
        newNode->gain_ = newParent->gain_ + std::get<0>(newNodeGain);
      }
      newNode->unmapped_cnt_ = std::get<1>(newNodeGain);
      newNode->occupied_cnt_ = std::get<2>(newNodeGain);
      newNode->free_cnt_ = std::get<3>(newNodeGain);
      newNode->unmapped_gain_ = std::get<4>(newNodeGain);
      newNode->occupied_gain_ = std::get<5>(newNodeGain);
      newNode->free_gain_ = std::get<6>(newNodeGain);

      //Insert node into tree
      kd_insert3(kdTree_, newState.x(), newState.y(), newState.z(), newNode);

      //Bsp: Increment node id and update node leaf flags
      newNode->id_ = ++n_ID_;
      newNode->leafNode_ = true;
      newNode->parent_->leafNode_ = false;

      //Publish new node (NBVP-level variant of MarkerArray visualization)
      publishNode(newNode, bspPlanning::NBVP_PLANLEVEL);

      //Nbvp: Update best (global) IG and node
      if (newNode->gain_ > bestGain_) {
        bestGain_ = newNode->gain_;
        bestNode_ = newNode;
      }
      counter_++;
    }
  }

}

bool bspPlanning::RrtTree::resampleBestEdge(int numRuns, rovio::BSP_SrvSendFilterState::Response& bspSendFilterStateSrv_response)
{
//Bsp: This function preforms 2nd-level RRT resampling around the 1st-level best-path's edge 
  bspPlanning::Node<StateVec> * targetNode = bestNode_;
  if (targetNode->parent_ == NULL)
    return false;

  while (targetNode->parent_ != rootNode_ && targetNode->parent_ != NULL) {
    targetNode = targetNode->parent_;
  }
  bspPlanning::Node<StateVec> * sourceNode = targetNode->parent_;
  
  StateVec dState = targetNode->state_-sourceNode->state_;
  Eigen::Vector3f replanWs_ax(0.0,1.0,1.0);
  Eigen::Vector3f replanWs_axSq(SQ(dState[0])+SQ(dState[1])+SQ(dState[2]),SQ(replanWs_ax[1]),SQ(replanWs_ax[2]));
  replanWs_ax[0] = sqrt(replanWs_axSq[0]);
  
  if (replanWs_ax[0]<params_.bspReplanningDistanceMin_)
    return false;
  const int bspReplanningInitInterations_scale = params_.bspReplanningInitIterations_/params_.extensionRange_;
  const int bspReplanningCutoffInterations_scale = params_.bspReplanningCutoffIterations_/params_.extensionRange_;
  int bspReplanningInitIterations = std::max(params_.bspReplanningInitIterations_,(int)(bspReplanningInitInterations_scale*replanWs_ax[0]));
  int bspReplanningCutoffIterations = std::max(params_.bspReplanningCutoffIterations_,(int)(bspReplanningCutoffInterations_scale*replanWs_ax[0]));
    
  tf::Vector3 replanWs_c(0.5*(sourceNode->state_[0]+targetNode->state_[0]),0.5*(sourceNode->state_[1]+targetNode->state_[1]),0.5*(sourceNode->state_[2]+targetNode->state_[2]));
  Eigen::Quaternion<float> q;
  Eigen::Vector3f init(1.0,0.0,0.0);
  Eigen::Vector3f dir(dState[0],dState[1],dState[2]);
  q.setFromTwoVectors(init, dir);
  q.normalize();
  tf::Quaternion replanWs_q(q.x(),q.y(),q.z(),q.w());
  tf::Transform replanWs_tf(replanWs_q,replanWs_c);

  //Bsp: Publish visualization of replanning ellipsoid
  visualization_msgs::Marker p;
  p.header.stamp = ros::Time::now();
  p.header.seq = 0;
  p.header.frame_id = params_.navigationFrame_;
  p.id = 0;
  p.ns = "bsp_workspace";
  p.type = visualization_msgs::Marker::SPHERE;
  p.action = visualization_msgs::Marker::ADD;
  p.pose.position.x = replanWs_c.getX();
  p.pose.position.y = replanWs_c.getY();
  p.pose.position.z = replanWs_c.getZ();
  p.pose.orientation.x = replanWs_q.x();
  p.pose.orientation.y = replanWs_q.y();
  p.pose.orientation.z = replanWs_q.z();
  p.pose.orientation.w = replanWs_q.w();
  p.scale.x = replanWs_ax[0];
  p.scale.y = replanWs_ax[1];
  p.scale.z = replanWs_ax[2];
  p.color.r = 200.0 / 255.0;
  p.color.g = 200.0 / 255.0;
  p.color.b = 0;
  p.color.a = 0.25;
  p.lifetime = ros::Duration(10.0);
  p.frame_locked = false;
  params_.planningWorkspace_.publish(p);
  
//Bsp: Reset criteria since replanning is performed
counter_ = 0;
//bestReGain_ = std::numeric_limits::infinity(); //should not need to
iterationCount_ = 0;

kdSubTree_ = kd_create(3);
kd_insert3(kdSubTree_, sourceNode->state_.x(), sourceNode->state_.y(), sourceNode->state_.z(), sourceNode);

//Bsp: Flags variables manipulated in iterative re-planning loop 
bool gotBspMap = false;
ros::Time lastBspMap_stamp = ros::Time(0);
bool firstLoop = true;
bool lastLoop = false;
int loopCount = 0;

std::vector< bspPlanning::Node<StateVec>* > leafNode_vec;
size_t leafNode_vecIt;

//Bsp: reGainFound evaluates minimum gain (RHEM planner's propagation pipeline uses uncertainty D-optimality metric)
while ((lastLoop || (!reGainFound() || loopCount < bspReplanningInitIterations)) && ros::ok()) {
  if (lastLoop){
    for (;leafNode_vecIt<leafNode_vec.size();){ if ((leafNode_vec.at(leafNode_vecIt++))->leafNode_){ break; } }
    if (leafNode_vecIt>=leafNode_vec.size())
      break;
  }
  if (loopCount > bspReplanningCutoffIterations && !lastLoop) {
    ROS_WARN("bsp_planner(resample): No gain found, stopping...");
    if (leafNode_vec.empty())
      break;
    lastLoop = true;
    leafNode_vecIt = 0;
    continue;
  }
  if (loopCount > 1000 * (getCounter() + 1) && !lastLoop) {
    ROS_WARN("bsp_planner(resample): Exceeding maximum failed iterations, stopping...");
    if (leafNode_vec.empty())
      break;
    lastLoop = true;
    leafNode_vecIt = 0;
    continue;
  }
  loopCount++;

  StateVec newState;
  bspPlanning::Node<StateVec> * newParent = NULL;
  Eigen::Vector3d direction;
  bool finalConnection = false;

  if (!lastLoop){
    bool solutionFound_pos = false;
    unsigned int whileThres_pos = 10000;
    while (!solutionFound_pos && whileThres_pos--) {
      for (unsigned int i = 0; i < 3; ++i) {
        newState[i] = replanWs_ax[i] * (((double) rand()) / ((double) RAND_MAX) - 0.5);
      }
      if (SQ(newState[0])/replanWs_axSq[0] + SQ(newState[1])/replanWs_axSq[1] + SQ(newState[2])/replanWs_axSq[2] > 1.0)
        continue;
      //Bsp: Transform new state by (best) edge
      tf::Vector3 newState_tfed = replanWs_tf * tf::Vector3(newState[0],newState[1],newState[2]);
      newState[0] = newState_tfed.x();
      newState[1] = newState_tfed.y();
      newState[2] = newState_tfed.z();
      if (!params_.softBounds_) {
        if (newState.x() + params_.boundingBoxOffset_.x() < params_.minX_ + 0.5 * params_.boundingBox_.x()) {
          continue;
        } else if (newState.y() + params_.boundingBoxOffset_.y() < params_.minY_ + 0.5 * params_.boundingBox_.y()) {
          continue;
        } else if (newState.z() + params_.boundingBoxOffset_.z() < params_.minZ_ + 0.5 * params_.boundingBox_.z()) {
          continue;
        } else if (newState.x() + params_.boundingBoxOffset_.x() > params_.maxX_ - 0.5 * params_.boundingBox_.x()) {
          continue;
        } else if (newState.y() + params_.boundingBoxOffset_.y() > params_.maxY_ - 0.5 * params_.boundingBox_.y()) {
          continue;
        } else if (newState.z() + params_.boundingBoxOffset_.z() > params_.maxZ_ - 0.5 * params_.boundingBox_.z()) {
          continue;
        }
      }
      // Bsp: Sample only in explicitly Free (Mapped) space
      if (volumetric_mapping::OctomapManager::CellStatus::kFree != manager_->getCellStatusBoundingBox(Eigen::Vector3d(newState[0],newState[1],newState[2])+params_.boundingBoxOffset_,params_.boundingBox_))
        continue;
      solutionFound_pos = true;
    }
    //Bsp: Force resampling (return control to top-level while) of origin (iteration-limited random sampling failed to find a valid position)
    if (!solutionFound_pos){
      ROS_WARN("bsp_planner(resample): newState[POS] random sampling failed to find admissible solution.");
      continue;
    }

    // Find nearest neighbour
    kdres * nearest = kd_nearest3(kdSubTree_, newState.x(), newState.y(), newState.z());
    if (kd_res_size(nearest) <= 0) {
      kd_res_free(nearest);
      continue;
    }
    newParent = (bspPlanning::Node<StateVec> *) kd_res_item_data(nearest);
 
    //Bsp: Handle extension range accounting for potential connection to final waypoint
    double bsp_replanning_extension_range = params_.bspReplanningExtensionRatio_*replanWs_ax[0];
    if (Eigen::Vector3d(targetNode->state_[0]-newParent->state_[0], targetNode->state_[1]-newParent->state_[1], targetNode->state_[2]-newParent->state_[2]).norm() < bsp_replanning_extension_range ){  //TODO Avoid any implicit conversion Vec3d<-Vec4d TODO//
      ROS_INFO("bsp_planner(resample): Wiring distance threshold to target reached! Connecting to final waypoint...");
      finalConnection = true;
    }
    else{
      direction << newState[0]-newParent->state_[0], newState[1]-newParent->state_[1], newState[2]-newParent->state_[2];
      if (!firstLoop && direction.norm()>bsp_replanning_extension_range) {
        direction = bsp_replanning_extension_range * direction.normalized();
      }

      if (newParent->distance_ + direction.norm() > params_.bspReplanningDistanceRatio_*replanWs_ax[0]){
        ROS_INFO("bsp_planner(resample): Wiring distance exceeded! Connecting to final waypoint...");
        finalConnection = true;
      }
      kd_res_free(nearest);
    }
  }
  else{ 
    //Bsp: Last allowed loop iteration
    finalConnection = true;
    newParent = leafNode_vec.at(leafNode_vecIt);
  }  

  //Bsp: Handle extension / connection to final waypoint
  if (firstLoop || finalConnection){
    newState[0] = targetNode->state_[0];
    newState[1] = targetNode->state_[1];
    newState[2] = targetNode->state_[2];
  }
  else{
    newState[0] = newParent->state_[0] + direction[0];
    newState[1] = newParent->state_[1] + direction[1];
    newState[2] = newParent->state_[2] + direction[2];
  }
  direction << newState[0]-newParent->state_[0], newState[1]-newParent->state_[1], newState[2]-newParent->state_[2];
  Eigen::Vector3d origin(newState[0], newState[1], newState[2]);

  //Check collision-free transition
  if ( volumetric_mapping::OctomapManager::CellStatus::kFree == manager_->getLineStatusBoundingBox(origin+params_.boundingBoxOffset_,origin+params_.boundingBoxOffset_+direction+direction.normalized()*params_.dOvershoot_,params_.boundingBox_) ) {
 
    //Bsp: Sample new orientation accounting for potential connection to final waypoint
    if (firstLoop || finalConnection){
      newState[3] = targetNode->state_[3];
    }
    else{
      //Bsp: Sample the new orientation w.r.t. vecLandmarks_ (landmarks acquired from slam pipeline)
      const double disc = manager_->getResolution();
      double rangeSq = SQ(params_.bspReplanningGainRange_);
      const unsigned int numFea_tracked_thres = 3;
      unsigned int numOutOfRange = 0;
      bool solutionFound_orient = false;
      int whileThres_orient = 1000;
      while ( (!solutionFound_orient && whileThres_orient--) && numOutOfRange < vecLandmarks_.size() ){ 
        int numFea_tracked = 0;
        newState[3] = 2.0 * M_PI * (((double) rand()) / ((double) RAND_MAX) - 0.5);
        for (unsigned int i=0; i<vecLandmarks_.size(); ++i){
          Eigen::Vector3d dir = vecLandmarks_.at(i) - origin;
          double dir_norm = dir.norm();
          if (dir_norm > rangeSq) {
            ++numOutOfRange;
            continue;
          }
          for (typename std::vector<std::vector<Eigen::Vector3d>>::iterator itCBN = params_.camBoundNormals_.begin(); itCBN != params_.camBoundNormals_.end(); itCBN++) {
            bool inFoV = true;
            for (typename std::vector<Eigen::Vector3d>::iterator itSingleCBN = itCBN->begin(); itSingleCBN != itCBN->end(); itSingleCBN++) {
              Eigen::Vector3d normal = Eigen::AngleAxisd(newState[3], Eigen::Vector3d::UnitZ()) * (*itSingleCBN);
              double val = dir.dot(normal.normalized());
              if (val < SQRT2 * disc) {
                inFoV = false;
                break;
              }
            }
            if (inFoV) {
              ++numFea_tracked;
              break;
            }
          }
        }   
        if ( numFea_tracked >= numFea_tracked_thres )
          solutionFound_orient = true;
      }
      //Bsp: Force resampling (return caontrol to top-level while) of origin (iteration-limited random sampling failed to find a valid position)
      if (!solutionFound_orient){
        ROS_WARN("bsp_planner(resample): newState[ORIENT] random sampling failed to find admissible solution.");
        continue;
      }
      //Bsp: Force resampling (return control to top-level while) of origin (it is outofRange w.r.t entire vecLandmarks_)
      if ( numOutOfRange >= vecLandmarks_.size() ){
        ROS_WARN("bsp_planner(resample): All landmarks out of visible range! Resampling...");
        continue; 
      }
    }

    //Create node
    bspPlanning::Node<StateVec> * newNode = new bspPlanning::Node<StateVec>;
    newNode->state_ = newState;
    newNode->parent_ = newParent;
    newNode->distance_ = newParent->distance_ + direction.norm();
    newParent->children_.push_back(newNode);

    //Bsp: uncertainty propagation transactions, transfer filter (belief) state to propagation pipeline, re-acquire new (belief) state as a response and store, response also includes optimality metric
    if (ros::service::exists("bsp_filter/propagate_filter_state", false)){
      rovio::BSP_SrvPropagateFilterState::Request rovio_bspPropagateFilterStateSrv_request;
      rovio::BSP_SrvPropagateFilterState::Response rovio_bspPropagateFilterStateSrv_response;
      //Bsp: (Optional) octomap population
      octomap_msgs::GetOctomap::Request octomapWorld_getMapSrv_request;
      static octomap_msgs::GetOctomap::Response octomapWorld_getMapSrv_response;
      if (!gotBspMap){
	octomapWorld_getMapSrv_response.map.data.clear();
        //NOTE: In OctomapManager (octomap_manager.h) the following would be more efficient
        //      "bool OctomapManager::getOctomapBinaryCallback(octomap_msgs::GetOctomap::Request& request,octomap_msgs::GetOctomap::Response& response)
        //       {  response.map.header.frame_id = world_frame_; response.map.header.stamp = octree_data_stamp_; response.map.binary = true; return getOctomapBinaryMsg(&response.map);  }"
        //      with "octree_data_stamp_ = stamp;" updated at the end of OctomapWorld::insertPointcloudIntoMapImpl and OctomapWorld::insertProjectedDisparityIntoMapImpl
        if ( manager_->getOctomapCallback(octomapWorld_getMapSrv_request, octomapWorld_getMapSrv_response) ){
	  gotBspMap = true;
          //NOTE: The following line is necessary only if the above modifications are not in place 
          octomapWorld_getMapSrv_response.map.header.stamp = ros::Time::now();  
        }
	else
          ROS_WARN("bsp_planner(resample): Failed to acquire current octomap from manager...");
      }
      //Bsp: No need to fill out on every re-planning call (last available map from slam pipeline remains the same), propagation pipepline is configured to handle working with last available map 
      else if (octomapWorld_getMapSrv_response.map.header.stamp > lastBspMap_stamp){
        rovio_bspPropagateFilterStateSrv_request.filterStateMap = octomapWorld_getMapSrv_response.map; 
      }
std::cout<< octomapWorld_getMapSrv_response.map.header.stamp <<std::endl;
      //Bsp: Populate filter (belief) state messages with values
      rovio_bspPropagateFilterStateSrv_request.filterStateMsgInit = newNode->parent_->bsp_belief_state_;
      //Bsp: Populate trajectory reference messages with values
      rovio::BSP_TrajectoryReferenceMsg bspTrajectoryReference_msg;
      bspTrajectoryReference_msg.header.frame_id = "body";
      bspTrajectoryReference_msg.header.stamp = ros::Time(newNode->parent_->bsp_belief_state_.t);
      geometry_msgs::Pose poseReference;
      poseReference.position.x = newNode->state_[0];
      poseReference.position.y = newNode->state_[1];
      poseReference.position.z = newNode->state_[2];
      tf::Quaternion qReference; qReference.setRPY(0.0,0.0,newNode->state_[3]);
      poseReference.orientation.x = qReference.x();
      poseReference.orientation.y = qReference.y();
      poseReference.orientation.z = qReference.z();
      poseReference.orientation.w = qReference.w();
      bspTrajectoryReference_msg.pose = poseReference;
      std::vector<rovio::BSP_TrajectoryReferenceMsg> vecBspTrajectoryReferenceMsg;
      vecBspTrajectoryReferenceMsg.push_back(bspTrajectoryReference_msg);
      rovio_bspPropagateFilterStateSrv_request.vecTrajectoryReferenceMsg = vecBspTrajectoryReferenceMsg;
      //Bsp: Call propagation service
      if (params_.bspServiceClient_.call(rovio_bspPropagateFilterStateSrv_request, rovio_bspPropagateFilterStateSrv_response)){
        //ROS_INFO("bsp_planner(resample): service call /bsp_filter/propagate_filter_state success");
        newNode->bsp_belief_state_ = rovio_bspPropagateFilterStateSrv_response.filterStateMsgFinal;
        lastBspMap_stamp = rovio_bspPropagateFilterStateSrv_response.filterStateMap_stamp;
      }
      else{
        ROS_WARN("bsp_planner(resample): service call /bsp_filter/propagate_filter_state fail");
      }
    }
    else{
      ROS_WARN("bsp_planner(resample): service call /bsp_filter/propagate_filter_state not found or not available");
    }

    //Bsp: gain is any metric calculated by propagation pipeline (RHEM planner's propagation pipeline uses uncertainty D-optimality metric)
    newNode->gain_ = newNode->bsp_belief_state_.opt_metric;
    // Increment node id
    newNode->id_ = ++n_ID_r_;
    //Bsp: Update leaf flags
    newNode->leafNode_ = true;
    newNode->parent_->leafNode_ = false;

    //Bsp: Fist & Final evaluation of direct connection is not inserted into kdTree, to avoid connecting subsequent nodes close to them as part of the tree
    if (!firstLoop && !finalConnection){
      kd_insert3(kdSubTree_, newState.x(), newState.y(), newState.z(), newNode);
      leafNode_vec.push_back(newNode);
    }

    // Display new node
    int nodeorder;
    if (finalConnection) nodeorder=2;
    else if (firstLoop) nodeorder=1;
    else nodeorder=0;
    publishNode(newNode, bspPlanning::BSP_PLANLEVEL, nodeorder);

    //Bsp: Update best IG and node only when 1st-level's targetNode is reached
    if (firstLoop){
      bestReGain_ = newNode->gain_;
      bestNode_ = newNode;
      initReGain_ = bestReGain_;
    } 
    else if (finalConnection){
      if (newNode->gain_ < bestReGain_) {
        bestReGain_ = newNode->gain_;
        bestNode_ = newNode;
      }
    }
    
    firstLoop = false;
    counter_++;
  }
}

if (reGainFound())
  return true;
else 
  return false;

}

std::vector<geometry_msgs::Pose> bspPlanning::RrtTree::getBestBranch(std::string targetFrame)
{
  //Bsp: Returns the complete best branch
  std::vector<geometry_msgs::Pose> ret, ret_edge;
  bspPlanning::Node<StateVec> * current = bestNode_;
  if (current->parent_ != NULL) {
    while (current->parent_ != rootNode_ && current->parent_ != NULL) {
      ret_edge = samplePath(current->parent_->state_, current->state_, targetFrame, false);
      ret.insert(ret.begin(),ret_edge.begin(),ret_edge.end());
      current = current->parent_;
    }
    ret_edge = samplePath(current->parent_->state_, current->state_, targetFrame, false);
    ret.insert(ret.begin(),ret_edge.begin(),ret_edge.end());
    history_.push(current->parent_->state_);
    exact_root_ = current->state_;
  }
  return ret;
}

std::vector<geometry_msgs::Pose> bspPlanning::RrtTree::getBestEdge(std::string targetFrame)
{
  //Nbvp: Returns the first edge of the best branch
  std::vector<geometry_msgs::Pose> ret;
  bspPlanning::Node<StateVec> * current = bestNode_;
  if (current->parent_ != NULL) {
    while (current->parent_ != rootNode_ && current->parent_ != NULL) {
      current = current->parent_;
    }
    ret = samplePath(current->parent_->state_, current->state_, targetFrame, true);
    history_.push(current->parent_->state_);
    exact_root_ = current->state_;
  }
  return ret;
}

std::tuple<double, unsigned int, unsigned int, unsigned int, double, double, double> bspPlanning::RrtTree::gain(StateVec state)
{
  //Bsp: Compute multi-component gain
  double gain = 0;
  unsigned int unmapped_cnt = 0;
  unsigned int occupied_cnt = 0;
  unsigned int free_cnt = 0;
  double unmapped_gain = 0;
  double occupied_gain = 0;
  double free_gain = 0;

  //Bsp: Probabilistic gain scaling based on octree mapping values
  //NOTE: In OctomapWorld (octomap_world.h) an inline: "const std::shared_ptr<const octomap::OcTree> getOcTree() const { return octree_; }" would suffice for direct manager_->octree_ const access
  //      lacking this forces work-around to derive from same NodeHandle used to instantiate OctomapManager, exposed by modified TreeBase and derived RrtTree ctors
  double prob_hit = 0.5; 
  double prob_miss = 0.4;
  double clamping_thres_min = 0.12;
  double clamping_thres_max = 0.97;
  double occupancy_thres = 0.7;
  if (nh_private_){
    nh_private_ -> getParam("probability_hit", prob_hit);
    nh_private_ -> getParam("probability_miss", prob_miss);
    nh_private_ -> getParam("threshold_min", clamping_thres_min);
    nh_private_ -> getParam("threshold_max", clamping_thres_max);
    nh_private_ -> getParam("threshold_occupancy", occupancy_thres);
  }
  const double occupied_occupancy_scale = clamping_thres_max - occupancy_thres;
  const double free_occupancy_scale = occupancy_thres - clamping_thres_min;

  const double disc = manager_->getResolution();
  const double volume_scale = pow(disc, 3.0);
  Eigen::Vector3d origin(state[0], state[1], state[2]);
  Eigen::Vector3d vec;
  double rangeSq = pow(params_.gainRange_, 2.0);
  //Nbvp: Iterate over all nodes within the allowed distance
  for (vec[0] = std::max(state[0] - params_.gainRange_, params_.minX_-params_.explorationExtensionX_); vec[0] < std::min(state[0] + params_.gainRange_, params_.maxX_+params_.explorationExtensionX_); vec[0] += disc) {
    for (vec[1] = std::max(state[1] - params_.gainRange_, params_.minY_-params_.explorationExtensionY_); vec[1] < std::min(state[1] + params_.gainRange_, params_.maxY_+params_.explorationExtensionY_); vec[1] += disc) {
      for (vec[2] = std::max(state[2] - params_.gainRange_, params_.explorationMinZ_); vec[2] < std::min(state[2] + params_.gainRange_, params_.maxZ_+params_.explorationMaxZ_); vec[2] += disc) {
        Eigen::Vector3d dir = vec - origin;
        //Nbvp: Skip if distance is too large
        if (dir.transpose().dot(dir) > rangeSq) {
          continue;
        }
        bool insideAFieldOfView = false;
        //Nbvp: Check that voxel center is inside one of the fields of view.
        for (typename std::vector<std::vector<Eigen::Vector3d>>::iterator itCBN = params_.camBoundNormals_.begin(); itCBN != params_.camBoundNormals_.end(); itCBN++) {
          bool inThisFieldOfView = true;
          for (typename std::vector<Eigen::Vector3d>::iterator itSingleCBN = itCBN->begin(); itSingleCBN != itCBN->end(); itSingleCBN++) {
            Eigen::Vector3d normal = Eigen::AngleAxisd(state[3], Eigen::Vector3d::UnitZ()) * (*itSingleCBN);
            double val = dir.dot(normal.normalized());
            if (val < SQRT2 * disc) {
              inThisFieldOfView = false;
              break;
            }
          }
          if (inThisFieldOfView) {
            insideAFieldOfView = true;
            break;
          }
        }
        if (!insideAFieldOfView) {
          continue;
        }

        //Nbvp: Check cell status and add to the gain considering the corresponding factor.
        double probability;
        volumetric_mapping::OctomapManager::CellStatus node_cellstatus = manager_->getCellProbabilityPoint(vec, &probability);	

	//Bsp: Nodes counting and separate gain calculations (exploration gain for unknown, probabilistic gain for occupied, probabilistic gain for free)
        if (node_cellstatus == volumetric_mapping::OctomapManager::CellStatus::kUnknown) {
          if (volumetric_mapping::OctomapManager::CellStatus::kOccupied != this->manager_->getVisibility(origin, vec, false)) {
            ++unmapped_cnt;
	    unmapped_gain += volume_scale * params_.igUnmapped_ * 1;
          }
        } else if (node_cellstatus == volumetric_mapping::OctomapManager::CellStatus::kOccupied) {
          if (volumetric_mapping::OctomapManager::CellStatus::kOccupied != this->manager_->getVisibility(origin, vec, false)) {
            ++occupied_cnt;
	    //Bsp: For Unknown nodes probability is -1.0
	    if (probability > clamping_thres_min){ 
              occupied_gain += volume_scale * params_.igOccupied_*(params_.igProbabilistic_/occupied_occupancy_scale) * (clamping_thres_max-probability);
            }
          }
        } else {
          if (volumetric_mapping::OctomapManager::CellStatus::kOccupied != this->manager_->getVisibility(origin, vec, false)) {
	    ++free_cnt;
            free_gain += volume_scale * params_.igFree_*(params_.igProbabilistic_/free_occupancy_scale) * (1.0-probability-clamping_thres_min);
          }
        }
      }//for (vec[0]...)
    }//for (vec[1]...)
  }//for (vec[2]...)

  //Bsp: Scaled gains sum
  gain = unmapped_gain + occupied_gain + free_gain;

  //Bsp: Return gains tuple contains all components
  return std::make_tuple(gain, unmapped_cnt, occupied_cnt, free_cnt, unmapped_gain, occupied_gain, free_gain);
}

std::vector<geometry_msgs::Pose> bspPlanning::RrtTree::getPathBackToPrevious(std::string targetFrame)
{
  std::vector<geometry_msgs::Pose> ret;
  if (history_.empty()) {
    return ret;
  }
  ret = samplePath(root_, history_.top(), targetFrame, true);
  history_.pop();
  return ret;
}

void bspPlanning::RrtTree::memorizeBestBranch()
{
  bestBranchMemory_.clear();
  Node<StateVec> * current = bestNode_;
  while (current->parent_ && current->parent_->parent_) {
    bestBranchMemory_.push_back(current->state_);
    current = current->parent_;
  }
}

void bspPlanning::RrtTree::clear()
{
  delete rootNode_;
  rootNode_ = NULL;

  counter_ = 0;
  bestNode_ = NULL;

  //Bsp: NBVP gain (information gain): 0
  bestGain_ = params_.zero_gain_;
  //Bsp: BSP gain (uncertainty D-optimality metric): Inf
  bestReGain_ = std::numeric_limits<double>::infinity();

  kd_free(kdTree_);
  
  //Clear visualization MarkerArrays
  params_.planningPath_markers_.markers.clear();
  params_.planningPathStats_markers_.markers.clear();
  params_.bestPlanningPath_markers_.markers.clear();
  params_.rePlanningPath_markers_.markers.clear();
  params_.rePlanningPathStats_markers_.markers.clear();
  params_.bestRePlanningPath_markers_.markers.clear();
}

//Visualization helper for node annotation
template <typename T>
std::string to_string_with_precision(const T a_value, const int n = 6)
{
    std::ostringstream out;
    out << std::setprecision(n) << std::fixed << a_value;
    return out.str();
}

void bspPlanning::RrtTree::publishPath(bspPlanning::PlanningLevel planninglevel)
{
  if (planninglevel==bspPlanning::PlanningLevel::NBVP_PLANLEVEL){
    params_.planningPath_.publish(params_.planningPath_markers_);
    params_.planningPathStats_.publish(params_.planningPathStats_markers_);
  }
  else if (planninglevel==bspPlanning::PlanningLevel::BSP_PLANLEVEL){
    params_.rePlanningPath_.publish(params_.rePlanningPath_markers_);
    params_.rePlanningPathStats_.publish(params_.rePlanningPathStats_markers_);
  }
  else{
    ROS_WARN("bsp_planner(publishPath): Invalid bspPlanning::PlanningLevel.");
  }
}

void bspPlanning::RrtTree::publishNode(Node<StateVec> * node, bspPlanning::PlanningLevel planninglevel, int nodeorder)
{
  const ros::Duration lifetime(params_.planMarker_lifetime_);
  visualization_msgs::Marker p;

  p.header.stamp = ros::Time::now();
  if (planninglevel == bspPlanning::NBVP_PLANLEVEL){
    p.header.seq = g_ID_;
  }
  else if (planninglevel == bspPlanning::BSP_PLANLEVEL){
    p.header.seq = g_ID_r_;
  }
  else{
    ROS_WARN("bsp_planner(publishNode): Invalid bspPlanning::PlanningLevel.");
    return;
  }
  p.header.frame_id = params_.navigationFrame_;

  unsigned int markerlevel  = static_cast<unsigned int>(planninglevel);

  //Orientations
  if (planninglevel == bspPlanning::NBVP_PLANLEVEL){
    p.id = g_ID_;
    g_ID_++;
  }
  else if (planninglevel == bspPlanning::BSP_PLANLEVEL){
    p.id = g_ID_r_;
    g_ID_r_++;
  }
  p.ns = "vp_orientations";
  p.type = visualization_msgs::Marker::ARROW;
  p.action = visualization_msgs::Marker::ADD;
  p.pose.position.x = node->state_[0];
  p.pose.position.y = node->state_[1];
  p.pose.position.z = node->state_[2];
  tf::Quaternion quat;
  quat.setEuler(0.0, 0.0, node->state_[3]);
  p.pose.orientation.x = quat.x();
  p.pose.orientation.y = quat.y();
  p.pose.orientation.z = quat.z();
  p.pose.orientation.w = quat.w();
  p.scale.x = 0.3;
  p.scale.y = 0.03/(0.35*markerlevel+1);
  p.scale.z = 0.03/(0.35*markerlevel+1);
  p.color.a = 1.0;
  p.lifetime = lifetime;
  p.frame_locked = false;
  if (planninglevel == bspPlanning::NBVP_PLANLEVEL){
  p.color.r = 0.75;
  p.color.g = 0.75;
  p.color.b = 0.0;
  params_.planningPath_markers_.markers.push_back(p);
  }
  else if (planninglevel == bspPlanning::BSP_PLANLEVEL){
  p.color.r = 0.0;
  p.color.g = 0.75;
  p.color.b = 0.0;
  params_.rePlanningPath_markers_.markers.push_back(p);
  }

  double p_text_scale = 0.1;
  if (planninglevel == bspPlanning::NBVP_PLANLEVEL){
    //Numbers
    unsigned int p_tot = node->unmapped_cnt_+node->occupied_cnt_+node->free_cnt_;

    p.id = g_ID_S_;
    g_ID_S_++;
    p.ns = "vp_connection"; //connection:=WhiteAlpha
    p.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    p.action = visualization_msgs::Marker::ADD;
    p.scale.z = p_text_scale;
    p.pose.position.x = node->state_[0];
    p.pose.position.y = node->state_[1];
    p.pose.position.z = node->state_[2];
    p.color.r = 1.0;
    p.color.g = 1.0;
    p.color.b = 1.0;
    p.color.a = 0.0;
    p.lifetime = lifetime;
    p.frame_locked = false;
    p.text = std::to_string(nodeorder)+":";
    if (!node->parent_)
      p.text += std::string("-1");
    else
      p.text += std::to_string(node->parent_->id_);
    p.text += std::string(",")+std::to_string(node->id_);
    params_.planningPathStats_markers_.markers.push_back(p);

    p.id = g_ID_S_;
    g_ID_S_++;
    p.ns = "vp_gain"; //gain:=Black
    p.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    p.action = visualization_msgs::Marker::ADD;
    p.scale.z = 1.25*p_text_scale;
    p.pose.position.x = node->state_[0];
    p.pose.position.y = node->state_[1];
    p.pose.position.z = node->state_[2]+p.scale.z*2.5;
    p.color.r = 0.0;
    p.color.g = 0.0;
    p.color.b = 0.0;
    p.color.a = 1.0;
    p.lifetime = lifetime;
    p.frame_locked = false;
    p.text = to_string_with_precision((float)node->gain_,2);
    params_.planningPathStats_markers_.markers.push_back(p);

    p.id = g_ID_S_;
    g_ID_S_++;
    p.ns = "vp_unmapped_gain"; //kUnmapped:=Blue
    p.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    p.action = visualization_msgs::Marker::ADD;
    p.scale.z = p_text_scale;
    p.pose.position.x = node->state_[0];
    p.pose.position.y = node->state_[1];
    p.pose.position.z = node->state_[2]+p.scale.z*2;
    p.color.r = 0.0;
    p.color.g = 0.0;
    p.color.b = 0.75;
    p.color.a = 1.0;
    p.lifetime = lifetime;
    p.frame_locked = false;
    p.text = to_string_with_precision((float)node->unmapped_gain_,2);
    params_.planningPathStats_markers_.markers.push_back(p);

    p.id = g_ID_S_;
    g_ID_S_++;
    p.ns = "vp_occupied_gain"; //kOccupied:=Green
    p.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    p.action = visualization_msgs::Marker::ADD;
    p.scale.z = p_text_scale;
    p.pose.position.x = node->state_[0];
    p.pose.position.y = node->state_[1];
    p.pose.position.z = node->state_[2]+p.scale.z*1;
    p.color.r = 0.0;
    p.color.g = 0.75;
    p.color.b = 0.0;
    p.color.a = 1.0;
    p.lifetime = lifetime;
    p.frame_locked = false;
    p.text = to_string_with_precision((float)node->occupied_gain_,2);
    params_.planningPathStats_markers_.markers.push_back(p);

    p.id = g_ID_S_;
    g_ID_S_++;
    p.ns = "vp_unmapped_cnt"; //visibility:=Alpha
    p.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    p.action = visualization_msgs::Marker::ADD;
    p.scale.z = p_text_scale;
    p.pose.position.x = node->state_[0];
    p.pose.position.y = node->state_[1];
    p.pose.position.z = node->state_[2]-p.scale.z*1;
    p.color.r = 0.0;
    p.color.g = 0.75;
    p.color.b = 0.75;
    p.color.a = 1.0;
    p.lifetime = lifetime;
    p.frame_locked = false;
    p.text = std::to_string(node->unmapped_cnt_);
    params_.planningPathStats_markers_.markers.push_back(p);

    p.id = g_ID_S_;
    g_ID_S_++;
    p.ns = "vp_occupied_cnt"; //kOccupied:=Yellow
    p.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    p.action = visualization_msgs::Marker::ADD;
    p.scale.z = p_text_scale;
    p.pose.position.x = node->state_[0];
    p.pose.position.y = node->state_[1];
    p.pose.position.z = node->state_[2]-p.scale.z*2;
    p.color.r = 0.0;
    p.color.g = 0.75;
    p.color.b = 0.0;
    p.color.a = 1.0;
    p.lifetime = lifetime;
    p.frame_locked = false;
    p.text = std::to_string(node->occupied_cnt_);
    params_.planningPathStats_markers_.markers.push_back(p);

  }
  else if (planninglevel == bspPlanning::BSP_PLANLEVEL){
    p.id = g_ID_Sr_;
    g_ID_Sr_++;
    p.ns = "vp_reconnection"; //reconnection:=WhiteAlpha
    p.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    p.action = visualization_msgs::Marker::ADD;
    p.scale.z = p_text_scale;
    p.pose.position.x = node->state_[0];
    p.pose.position.y = node->state_[1];
    p.pose.position.z = node->state_[2];
    p.color.r = 1.0;
    p.color.g = 1.0;
    p.color.b = 1.0;
    p.color.a = 0.0;
    p.lifetime = lifetime;
    p.frame_locked = false;
    p.text = std::to_string(nodeorder)+":";
    if (!node->parent_)
      p.text += std::string("-1");
    else
      p.text += std::to_string(node->parent_->id_);
    p.text += std::string(",")+std::to_string(node->id_);
    params_.rePlanningPathStats_markers_.markers.push_back(p);

    p.id = g_ID_Sr_;
    g_ID_Sr_++;
    p.ns = "vp_dopt"; 
    p.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    p.action = visualization_msgs::Marker::ADD;
    p.scale.z = 1.25*0.75*p_text_scale;
    p.pose.position.x = node->state_[0];
    p.pose.position.y = node->state_[1];
    p.pose.position.z = node->state_[2]+1.25*p.scale.z;
    p.color.r = 0.0;
    p.color.g = 0.0;
    p.color.b = 0.0;
    p.color.a = 1.0;
    p.lifetime = lifetime;
    p.frame_locked = false;
    p.text = std::to_string(node->bsp_belief_state_.opt_metric);
    params_.rePlanningPathStats_markers_.markers.push_back(p);

    p.id = g_ID_Sr_;
    g_ID_Sr_++;
    p.ns = "vp_distance"; 
    p.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    p.action = visualization_msgs::Marker::ADD;
    p.scale.z = 0.75*p_text_scale;
    p.pose.position.x = node->state_[0];
    p.pose.position.y = node->state_[1];
    p.pose.position.z = node->state_[2]-p.scale.z*1;
    p.color.r = 1.0;
    p.color.g = 1.0;
    p.color.b = 1.0;
    p.color.a = 0.0;
    p.lifetime = lifetime;
    p.frame_locked = false;
    p.text = std::to_string(node->distance_);
    params_.rePlanningPathStats_markers_.markers.push_back(p);

  }

  if (!node->parent_)
    return;

  //Positions
  if (planninglevel == bspPlanning::NBVP_PLANLEVEL){
    p.id = g_ID_;
    g_ID_++;
  }
  else if (planninglevel == bspPlanning::BSP_PLANLEVEL){
    p.id = g_ID_r_;
    g_ID_r_++;
  }
  p.ns = "vp_positions";
  p.type = visualization_msgs::Marker::ARROW;
  p.action = visualization_msgs::Marker::ADD;
  p.pose.position.x = node->parent_->state_[0];
  p.pose.position.y = node->parent_->state_[1];
  p.pose.position.z = node->parent_->state_[2];
  Eigen::Quaternion<float> q;
  Eigen::Vector3f init(1.0, 0.0, 0.0);
  Eigen::Vector3f dir(node->state_[0] - node->parent_->state_[0],
                      node->state_[1] - node->parent_->state_[1],
                      node->state_[2] - node->parent_->state_[2]);
  q.setFromTwoVectors(init, dir);
  q.normalize();
  p.pose.orientation.x = q.x();
  p.pose.orientation.y = q.y();
  p.pose.orientation.z = q.z();
  p.pose.orientation.w = q.w();
  p.scale.x = dir.norm();
  p.scale.y = 0.03/(0.35*planninglevel+1);
  p.scale.z = 0.03/(0.35*planninglevel+1);
  p.color.a = 1.0;
  p.lifetime = lifetime;
  p.frame_locked = false;
  if (planninglevel == bspPlanning::NBVP_PLANLEVEL){
    p.color.r = 0.0;
    p.color.g = 0.0;
    p.color.b = 0.75;
    params_.planningPath_markers_.markers.push_back(p);
  }
  else if (planninglevel == bspPlanning::BSP_PLANLEVEL){
    p.color.r = 0.75;
    p.color.g = 0.0;
    p.color.b = 0.0;
    params_.rePlanningPath_markers_.markers.push_back(p);
  }

}

void bspPlanning::RrtTree::publishBestPath(bspPlanning::PlanningLevel planninglevel)
{
  bspPlanning::Node<StateVec> * node = bestNode_;
  while (node != rootNode_ && node->parent_ != NULL) {

    const ros::Duration lifetime(params_.bestPlanMarker_lifetime_);
    visualization_msgs::Marker p;

    p.header.stamp = ros::Time::now();
    if (planninglevel == bspPlanning::NBVP_PLANLEVEL){
      p.header.seq = g_ID_;
    }
    else if (planninglevel == bspPlanning::BSP_PLANLEVEL){
      p.header.seq = g_ID_r_;
    }
    else{
      ROS_WARN("bsp_planner(publishNode): Invalid bspPlanning::PlanningLevel.");
      return;
    }
    p.header.frame_id = params_.navigationFrame_;

    unsigned int markerlevel  = static_cast<unsigned int>(planninglevel);

    //Orientations
    if (planninglevel == bspPlanning::NBVP_PLANLEVEL){
      p.id = g_ID_;
      g_ID_++;
    }
    else if (planninglevel == bspPlanning::BSP_PLANLEVEL){
      p.id = g_ID_r_;
      g_ID_r_++;
    }
    p.ns = "vp_orientations_best";
    p.type = visualization_msgs::Marker::ARROW;
    p.action = visualization_msgs::Marker::ADD;
    p.pose.position.x = node->state_[0];
    p.pose.position.y = node->state_[1];
    p.pose.position.z = node->state_[2];
    tf::Quaternion quat;
    quat.setEuler(0.0, 0.0, node->state_[3]);
    p.pose.orientation.x = quat.x();
    p.pose.orientation.y = quat.y();
    p.pose.orientation.z = quat.z();
    p.pose.orientation.w = quat.w();
    p.scale.x = 0.3;
    //p.scale.x = std::max(node->gain_ / 200.0, 0.15);
    p.scale.y = 1.25*0.03/(0.35*markerlevel+1);
    p.scale.z = 1.25*0.03/(0.35*markerlevel+1);
    p.color.a = 1.0;
    p.lifetime = lifetime;
    p.frame_locked = false;
    if (planninglevel == bspPlanning::NBVP_PLANLEVEL){
      p.color.r = 0.75;
      p.color.g = 0.75;
      p.color.b = 0.0;
      params_.bestPlanningPath_markers_.markers.push_back(p);
    }
    else if (planninglevel == bspPlanning::BSP_PLANLEVEL){
      p.color.r = 0.0;
      p.color.g = 0.75;
      p.color.b = 0.0;
      params_.bestRePlanningPath_markers_.markers.push_back(p);
    }

    if (!node->parent_)
      break;

    //Positions
    if (planninglevel == bspPlanning::NBVP_PLANLEVEL){
      p.id = g_ID_;
      g_ID_++;
    }
    else if (planninglevel == bspPlanning::BSP_PLANLEVEL){
      p.id = g_ID_r_;
      g_ID_r_++;
    }
    p.ns = "vp_positions_best";
    p.type = visualization_msgs::Marker::ARROW;
    p.action = visualization_msgs::Marker::ADD;
    p.pose.position.x = node->parent_->state_[0];
    p.pose.position.y = node->parent_->state_[1];
    p.pose.position.z = node->parent_->state_[2];
    Eigen::Quaternion<float> q;
    Eigen::Vector3f init(1.0, 0.0, 0.0);
    Eigen::Vector3f dir(node->state_[0] - node->parent_->state_[0],
                        node->state_[1] - node->parent_->state_[1],
                        node->state_[2] - node->parent_->state_[2]);
    q.setFromTwoVectors(init, dir);
    q.normalize();
    p.pose.orientation.x = q.x();
    p.pose.orientation.y = q.y();
    p.pose.orientation.z = q.z();
    p.pose.orientation.w = q.w();
    p.scale.x = dir.norm();
    p.scale.y = 1.25*0.03/(0.35*markerlevel+1);
    p.scale.z = 1.25*0.03/(0.35*markerlevel+1);
    p.color.a = 1.0;
    p.lifetime = lifetime;
    p.frame_locked = false;
    if (planninglevel == bspPlanning::NBVP_PLANLEVEL){
      p.color.r = 0.0;
      p.color.g = 0.75;
      p.color.b = 0.75;
      params_.bestPlanningPath_markers_.markers.push_back(p);
    }
    else if (planninglevel == bspPlanning::BSP_PLANLEVEL){
      p.color.r = 0.75;
      p.color.g = 0.0;
      p.color.b = 0.75;
      params_.bestRePlanningPath_markers_.markers.push_back(p);
    }
 
    node = node->parent_;
  }

  if (planninglevel == bspPlanning::NBVP_PLANLEVEL){
    params_.bestPlanningPath_.publish(params_.bestPlanningPath_markers_);
  }
  else if (planninglevel == bspPlanning::BSP_PLANLEVEL){
    params_.bestRePlanningPath_.publish(params_.bestRePlanningPath_markers_);
  }
}

std::vector<geometry_msgs::Pose> bspPlanning::RrtTree::samplePath (StateVec start, StateVec end, std::string targetFrame, bool useConstraints)
{
  std::vector<geometry_msgs::Pose> ret;
  static tf::TransformListener listener;
  tf::StampedTransform transform;
  try {
    listener.lookupTransform(targetFrame, params_.navigationFrame_, ros::Time(0), transform);
  } catch (tf::TransformException ex) {
    ROS_ERROR("bsp_planner(samplePath): %s", ex.what());
    return ret;
  }
  Eigen::Vector3d distance(end[0] - start[0], end[1] - start[1], end[2] - start[2]);
  double yaw_direction = end[3] - start[3];
  if (yaw_direction > M_PI) {
    yaw_direction -= 2.0 * M_PI;
  }
  if (yaw_direction < -M_PI) {
    yaw_direction += 2.0 * M_PI;
  }
  
  if (useConstraints){
    geometry_msgs::Pose pose;
    StateVec state = start;
    for (unsigned int i=0; i<2; ++i, state=end){
      tf::Vector3 origin(state[0],state[1],state[2]);
      double yaw = state[3];
      if (yaw > M_PI)
        yaw -= 2.0 * M_PI;
      else if (yaw < -M_PI)
        yaw += 2.0 * M_PI;
      tf::Quaternion quat;
      quat.setEuler(0.0, 0.0, yaw);
      origin = transform * origin;
      quat = transform * quat;
      tf::Pose poseTF(quat, origin);
      tf::poseTFToMsg(poseTF, pose);
      ret.push_back(pose);
    }
    tf::Vector3 origin_end(end[0],end[1],end[2]);
  }
  else{
    double disc = std::min(params_.dt_ * params_.v_max_ / distance.norm(), params_.dt_ * params_.dyaw_max_ / abs(yaw_direction));
    assert(disc > 0.0);
    for (double it = 0.0; it <= 1.0; it += disc) {
      tf::Vector3 origin((1.0 - it) * start[0] + it * end[0], (1.0 - it) * start[1] + it * end[1], (1.0 - it) * start[2] + it * end[2]);
      double yaw = start[3] + yaw_direction * it;
      if (yaw > M_PI)
        yaw -= 2.0 * M_PI;
      if (yaw < -M_PI)
        yaw += 2.0 * M_PI;
      tf::Quaternion quat;
      quat.setEuler(0.0, 0.0, yaw);
      origin = transform * origin;
      quat = transform * quat;
      tf::Pose poseTF(quat, origin);
      geometry_msgs::Pose pose;
      tf::poseTFToMsg(poseTF, pose);
      ret.push_back(pose);
    }
  }

  return ret;
}

#endif // RRTTREE_HPP_
