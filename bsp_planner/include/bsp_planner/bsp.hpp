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

#ifndef BSP_HPP_
#define BSP_HPP_

#include <bsp_planner/bsp.h>

#include <visualization_msgs/Marker.h>

using namespace Eigen;

template<typename stateVec>
bspPlanning::bspPlanner<stateVec>::bspPlanner(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    				  	     : nh_(nh),
      					       nh_private_(nh_private)
{
  manager_ = new volumetric_mapping::OctomapManager(nh_, nh_private_);

  //Set up NBVP & BSP planning and best path extraction topics and services
  params_.planningWorkspace_ = nh_.advertise<visualization_msgs::Marker>("planningWorkspace", 1000);
  params_.planningPath_ = nh_.advertise<visualization_msgs::MarkerArray>("planningPath", 1000);
  params_.planningPathStats_ = nh_.advertise<visualization_msgs::MarkerArray>("planningPathStats", 1000);
  params_.bestPlanningPath_ = nh_.advertise<visualization_msgs::MarkerArray>("bestPlanningPath", 1000);
  params_.rePlanningPath_ = nh_.advertise<visualization_msgs::MarkerArray>("rePlanningPath", 1000);
  params_.rePlanningPathStats_ = nh_.advertise<visualization_msgs::MarkerArray>("rePlanningPathStats", 1000);
  params_.bestRePlanningPath_ = nh_.advertise<visualization_msgs::MarkerArray>("bestRePlanningPath", 1000);
  plannerService_ = nh_.advertiseService("bsp_planner", &bspPlanning::bspPlanner<stateVec>::plannerCallback, this);

  //Bsp: BSP service client creation
  while ( !(params_.bspServiceClient_ = nh_.serviceClient<bsp_msgs::BSP_SrvPropagateFilterState> ("bsp_filter/propagate_filter_state", false)) ){ //true for persistent
    ROS_WARN("bsp_planner(ctor): ServiceClient \"bsp_filter/propagate_filter_state\" creation failed, re-attempting");
    sleep(1);
  }
  ROS_INFO("bsp_planner(ctor): Created ServiceClient \"bsp_filter/propagate_filter_state\"");

  //Published path to follow 
  plannerPub_ = nh_.advertise<nav_msgs::Path>("planner_path", 1000);

  //Subscribers
  posClient_ = nh_.subscribe("pose", 10, &bspPlanning::bspPlanner<stateVec>::posCallback, this);
  odomClient_ = nh_.subscribe("odometry", 10, &bspPlanning::bspPlanner<stateVec>::odomCallback, this);
  pointcloud_sub_ = nh_.subscribe("pointcloud_throttled", 1, &bspPlanning::bspPlanner<stateVec>::insertPointcloudWithTf, this);

  if (!setParams()) {
    ROS_ERROR("bsp_planner(ctor): Could not start the planner, parameters missing...");
  }

  //Nbvp: Compute camera FoV
  for (int i = 0; i < params_.camPitch_.size(); i++) {
    double pitch = M_PI * params_.camPitch_[i] / 180.0;
    double camTop = (pitch - M_PI * params_.camVertical_[i] / 360.0) + M_PI / 2.0;
    double camBottom = (pitch + M_PI * params_.camVertical_[i] / 360.0) - M_PI / 2.0;
    double side = M_PI * (params_.camHorizontal_[i]) / 360.0 - M_PI / 2.0;
    Vector3d bottom(cos(camBottom), 0.0, -sin(camBottom));
    Vector3d top(cos(camTop), 0.0, -sin(camTop));
    Vector3d right(cos(side), sin(side), 0.0);
    Vector3d left(cos(side), -sin(side), 0.0);
    AngleAxisd m = AngleAxisd(pitch, Vector3d::UnitY());
    Vector3d rightR = m * right;
    Vector3d leftR = m * left;
    rightR.normalize();
    leftR.normalize();
    std::vector<Eigen::Vector3d> camBoundNormals;
    camBoundNormals.push_back(bottom);
    camBoundNormals.push_back(top);
    camBoundNormals.push_back(rightR);
    camBoundNormals.push_back(leftR);
    params_.camBoundNormals_.push_back(camBoundNormals);
  }

  //Bsp: Initialize the tree, with workaround to expose to constructing node's parameters
  tree_ = new RrtTree(manager_, &nh_private_);
  tree_->setParams(params_);

  //Bsp: Uninitialized state
  init_state_ = BSP_INITSTATE_NONE;
  ready_ = false;
  num_runs_ = 0;
  num_reruns_ = 0;
}

template<typename stateVec>
bspPlanning::bspPlanner<stateVec>::~bspPlanner()
{
  if (manager_) {
    delete manager_;
  }
}

template<typename stateVec>
void bspPlanning::bspPlanner<stateVec>::posCallback(const geometry_msgs::PoseWithCovarianceStamped& pose)
{
  tree_->setStateFromPoseMsg(pose);
  
  //Bsp: Setup state in intial freespace-clearance-around-agent mode.
  if (!ready_) {
    init_state_ = BSP_INITSTATE_NEEDBBXCLEARANCETOPLAN;
  }

  //Nbvp: Planner is now ready to plan.
  ready_ = true;
}

template<typename stateVec>
void bspPlanning::bspPlanner<stateVec>::odomCallback(const nav_msgs::Odometry& pose)
{
  tree_->setStateFromOdometryMsg(pose);

  //Bsp: Setup state in intial freespace-clearance-around-agent mode.
  if (!ready_) {
    init_state_ = BSP_INITSTATE_NEEDBBXCLEARANCETOPLAN;
  }

  //Nbvp: Planner is now ready to plan.
  ready_ = true;
}

template<typename stateVec>
bool bspPlanning::bspPlanner<stateVec>::plannerCallback(bsp_planner::bsp_srv::Request& req,
                                                        bsp_planner::bsp_srv::Response& res)
{
  ros::Time computationTime = ros::Time::now();
  //Nbvp: Check that planner is ready to compute path.
  if (!ros::ok()) {
    ROS_INFO_THROTTLE(1, "Exploration finished. Not planning any further moves.");
    return true;
  }

  if (!ready_) {
    ROS_ERROR_THROTTLE(1, "Planner not set up: Planner not ready!");
    return true;
  }
  if (manager_ == NULL) {
    ROS_ERROR_THROTTLE(1, "Planner not set up: No octomap available!");
    return true;
  }
  if (manager_->getMapSize().norm() <= 0.0) {
    ROS_ERROR_THROTTLE(1, "Planner not set up: Octomap is empty!");
    return true;
  }
  res.path.clear();

  //Bsp: If BSP planning is required, call odometry filter to get initial belief state
  if (params_.bspEnable_){
    if (ros::service::exists("slam_filter/send_filter_state", false)){
      bspMsgs_sendFilterStateSrv_request.header.stamp = ros::Time::now();
      bspMsgs_sendFilterStateSrv_request.header.frame_id = params_.navigationFrame_;
      if (ros::service::call("slam_filter/send_filter_state", bspMsgs_sendFilterStateSrv_request, bspMsgs_sendFilterStateSrv_response)){
        //ROS_INFO("bsp_planner: service call /slam_filter/send_filter_state success");
      }
      else{
        ROS_WARN("bsp_planner: service call /slam_filter/send_filter_state fail");
        return false;
      }
    }
    else{
      ROS_ERROR("bsp_planner: service call /slam_filter/send_filter_state not found or not available");
      return false;
    }
  }

  //Nbvp: Clear old tree and reinitialize
  tree_->clear();
  tree_->initialize(num_runs_, bspMsgs_sendFilterStateSrv_response);
  vector_t path;

  //Nbvp: Iterate the tree construction method
  bool brkflag = false;
  int loopCount = 0;
  while ((!tree_->gainFound() || tree_->getCounter() < params_.initIterations_) && ros::ok()) {
    if (tree_->getCounter() > params_.cutoffIterations_) {
      ROS_INFO("No gain found, breaking...");
      brkflag = true;
      break;
    }
    if (loopCount > 1000 * (tree_->getCounter() + 1)) {
      ROS_WARN("bsp_planner(iterate): Exceeding maximum failed iterations, breaking and returning to previous point...");
      res.path = tree_->getPathBackToPrevious(req.header.frame_id);
      brkflag = true;
      break;
    }
    tree_->iterate(num_runs_, bspMsgs_sendFilterStateSrv_response);
    loopCount++;
  }
  if (brkflag){
    return false;
  }

  //Bsp: NBVP planning level done, publish entire MarkerArray (full tree)
  tree_->publishPath(NBVP_PLANLEVEL);
  res.path = tree_->getBestEdge(req.header.frame_id);
  if (!res.path.empty()){
    ++num_runs_; //successful NBVP planning steps
  }
  //Nbvp: Publish extracted path (best branch)
  tree_->publishBestPath(NBVP_PLANLEVEL);

  //Bsp: Invoke nested bsp replanning level on best branch's 1st edge
  if (params_.bspEnable_){ 
    bool res_resampleBestEdge = tree_->resampleBestEdge(num_reruns_, bspMsgs_sendFilterStateSrv_response);

    //Bsp: BSP planning level done, publish entire MarkerArray (full nested replanning tree)
    tree_->publishPath(BSP_PLANLEVEL);
    if (res_resampleBestEdge){
      //Bsp: Extract the best branch
      res.path = tree_->getBestBranch(req.header.frame_id);
      if (!res.path.empty()){
        ++num_reruns_; //successful BSP planning steps
      }
      //Bsp: Publish extracted path (best nested branch)
      tree_->publishBestPath(BSP_PLANLEVEL);
    }
  }

  tree_->memorizeBestBranch();

  ROS_INFO("bsp_planner(plannerCallback): Path computation lasted %2.3fs", (ros::Time::now() - computationTime).toSec());

  nav_msgs::Path planner_path;
  planner_path.header.stamp = ros::Time::now();
  planner_path.header.frame_id = params_.navigationFrame_;
  for (int i=0;i<res.path.size();++i){
    geometry_msgs::PoseStamped posestamped_i;
    posestamped_i.header.stamp = ros::Time::now()+ros::Duration(1.0);
    posestamped_i.header.frame_id = params_.navigationFrame_;
    posestamped_i.pose = res.path.at(i);
    planner_path.poses.push_back(posestamped_i);
  }

  if (!planner_path.poses.empty()){

    //Bsp: Set planning state to initialized
    if (init_state_ < BSP_INITSTATE_PLANNINGINITIALIZED){
      ROS_INFO("bsp_planner(plannerCallback): Planner performed first successful iteration.");
      init_state_ = BSP_INITSTATE_PLANNINGINITIALIZED;
    }
    
    //ROS_INFO("nbvPlanner: Publishing path...");
    plannerPub_.publish( planner_path );
  }
  else{
    ROS_WARN("bsp_planner(plannerCallback): No path computed...");
  }

  return true;
}

template<typename stateVec>
bool bspPlanning::bspPlanner<stateVec>::setParams()
{
  std::string ns = ros::this_node::getName();
  bool ret = true;
  params_.v_max_ = 0.25;
  if (!ros::param::get(ns + "/system/v_max", params_.v_max_)) {
    ROS_WARN("No maximal system speed specified. Looking for %s. Default is 0.25.", (ns + "/system/v_max").c_str());
  }
  params_.dyaw_max_ = 0.5;
  if (!ros::param::get(ns + "/system/dyaw_max", params_.dyaw_max_)) {
    ROS_WARN("No maximal yaw speed specified. Looking for %s. Default is 0.5.", (ns + "/system/yaw_max").c_str());
  }
  params_.camPitch_ = {15.0};
  if (!ros::param::get(ns + "/system/camera/pitch", params_.camPitch_)) {
    ROS_WARN("No camera pitch specified. Looking for %s. Default is 15deg.", (ns + "/system/camera/pitch").c_str());
  }
  params_.camHorizontal_ = {90.0};
  if (!ros::param::get(ns + "/system/camera/horizontal", params_.camHorizontal_)) {
    ROS_WARN("No camera horizontal opening specified. Looking for %s. Default is 90deg.", (ns + "/system/camera/horizontal").c_str());
  }
  params_.camVertical_ = {60.0};
  if (!ros::param::get(ns + "/system/camera/vertical", params_.camVertical_)) {
    ROS_WARN("No camera vertical opening specified. Looking for %s. Default is 60deg.", (ns + "/system/camera/vertical").c_str());
  }
  if(params_.camPitch_.size() != params_.camHorizontal_.size() ||params_.camPitch_.size() != params_.camVertical_.size() ){
    ROS_WARN("Specified camera fields of view unclear: Not all parameter vectors have same length! Setting to default.");
    params_.camPitch_.clear();
    params_.camPitch_ = {15.0};
    params_.camHorizontal_.clear();
    params_.camHorizontal_ = {90.0};
    params_.camVertical_.clear();
    params_.camVertical_ = {60.0};
  }
  params_.bspEnable_ = false;
  if (!ros::param::get(ns + "/bsp/enable", params_.bspEnable_)) {
    ROS_WARN("Not specified whether Belief-Space Planner is to be called. Looking for %s. Default is false", (ns + "/bsp/enable").c_str());
  }
  params_.bspReplanningDistanceMin_ = 0.25;
  if (!ros::param::get(ns + "/bsp/replanning_distance_min", params_.bspReplanningDistanceMin_)) {
    ROS_WARN("No min distance threshold for Replanning step. Looking for %s. Default is 0.25.", (ns + "/bsp/replanning_distance_min").c_str());
  }
  params_.bspReplanningDistanceRatio_ = 1.5;
  if (!ros::param::get(ns + "/bsp/replanning_distance_ratio", params_.bspReplanningDistanceRatio_)) {
    ROS_WARN("No ratio for max distance travelled in Replanning step. Looking for %s. Default is 1.5.", (ns + "/bsp/replanning_distance_ratio").c_str());
  }
  params_.bspReplanningExtensionRatio_ = 0.5;
  if (!ros::param::get(ns + "/bsp/replanning_extension_ratio", params_.bspReplanningExtensionRatio_)) {
    ROS_WARN("No ratio for max extension range in Replanning step. Looking for %s. Default is 0.5.", (ns + "/bsp/replanning_extension_ratio").c_str());
  }
  params_.bspReplanningInitIterations_ = 15;
  if (!ros::param::get(ns + "/bsp/replanning_initial_iterations", params_.bspReplanningInitIterations_)) {
    ROS_WARN("No initial iterations defined for Replanning step. Looking for %s. Default is 15.", (ns + "/bsp/replanning_initial_iterations").c_str());
  }
  params_.bspReplanningCutoffIterations_ = 200;
  if (!ros::param::get(ns + "/bsp/replanning_cutoff_iterations", params_.bspReplanningCutoffIterations_)) {
    ROS_WARN("No cutoff iterations defined for Replanning step. Looking for %s. Default is 200.", (ns + "/bsp/replanning_cutoff_iterations").c_str());
  }
  params_.bspReplanningGainRange_ = 5.0;
  if (!ros::param::get(ns + "/bsp/replanning_gain_range", params_.bspReplanningGainRange_)) {
    ROS_WARN("No Replanning gain range for feature visibility checking specified. Looking for %s. Default is 5.0m.", (ns + "/bsp/replanning_gain_range").c_str());
  }
  params_.planMarker_lifetime_ = 15.0;
  if (!ros::param::get(ns + "/markers/planMarker_lifetime", params_.planMarker_lifetime_)) {
    ROS_WARN("No lifetime for planning markers specified. Looking for %s. Default is 15.0s.", (ns + "/markers/planMarker_lifetime").c_str());
  }
  params_.bestPlanMarker_lifetime_ = 15.0;
  if (!ros::param::get(ns + "/markers/bestPlanMarker_lifetime", params_.bestPlanMarker_lifetime_)) {
    ROS_WARN("No lifetime for Best planning markers specified. Looking for %s. Default is 15.0s.", (ns + "/markers/bestPlanMarker_lifetime").c_str());
  }
  params_.replanMarker_lifetime_ = 15.0;
  if (!ros::param::get(ns + "/markers/replanMarker_lifetime", params_.replanMarker_lifetime_)) {
    ROS_WARN("No lifetime for replanning markers specified. Looking for %s. Default is 15.0s.", (ns + "/markers/replanMarker_lifetime").c_str());
  }
  params_.bestReplanMarker_lifetime_ = 15.0;
  if (!ros::param::get(ns + "/markers/bestReplanMarker_lifetime", params_.bestReplanMarker_lifetime_)) {
    ROS_WARN("No lifetime for Best replanning markers specified. Looking for %s. Default is 15.0s.", (ns + "/markers/bestReplanMarker_lifetime").c_str());
  }
  params_.igProbabilistic_ = 0.0;
  if (!ros::param::get(ns + "/nbvp/gain/probabilistic", params_.igProbabilistic_)) {
    ROS_WARN("No gain coefficient for probability of cells specified. Looking for %s. Default is 0.0.", (ns + "/nbvp/gain/probabilistic").c_str());
  }
  params_.igFree_ = 0.0;
  if (!ros::param::get(ns + "/nbvp/gain/free", params_.igFree_)) {
    ROS_WARN("No gain coefficient for free cells specified. Looking for %s. Default is 0.0.", (ns + "/nbvp/gain/free").c_str());
  }
  params_.igOccupied_ = 0.0;
  if (!ros::param::get(ns + "/nbvp/gain/occupied", params_.igOccupied_)) {
    ROS_WARN("No gain coefficient for occupied cells specified. Looking for %s. Default is 0.0.", (ns + "/nbvp/gain/occupied").c_str());
  }
  params_.igUnmapped_ = 1.0;
  if (!ros::param::get(ns + "/nbvp/gain/unmapped", params_.igUnmapped_)) {
    ROS_WARN("No gain coefficient for unmapped cells specified. Looking for %s. Default is 1.0.", (ns + "/nbvp/gain/unmapped").c_str());
  }
  params_.degressiveCoeff_ = 0.25;
  if (!ros::param::get(ns + "/nbvp/gain/degressive_coeff", params_.degressiveCoeff_)) {
    ROS_WARN("No degressive factor for gain accumulation specified. Looking for %s. Default is 0.25.", (ns + "/nbvp/gain/degressive_coeff").c_str());
  }
  params_.degressiveSwitchoffLoops_ = 5;
  if (!ros::param::get(ns + "/nbvp/gain/degressive_switchoffLoops", params_.degressiveSwitchoffLoops_)) {
    ROS_WARN("No Loop counts for degressive factor (for gain accumulation) switchoff specified. Looking for %s. Default is 5.", (ns + "/nbvp/gain/degressive_switchoffLoops").c_str());
  }
  params_.extensionRange_ = 1.0;
  if (!ros::param::get(ns + "/nbvp/tree/extension_range", params_.extensionRange_)) {
    ROS_WARN("No value for maximal extension range specified. Looking for %s. Default is 1.0m.", (ns + "/nbvp/tree/extension_range").c_str());
  }
  params_.initIterations_ = 15;
  if (!ros::param::get(ns + "/nbvp/tree/initial_iterations", params_.initIterations_)) {
    ROS_WARN("No number of initial tree iterations specified. Looking for %s. Default is 15.", (ns + "/nbvp/tree/initial_iterations").c_str());
  }
  params_.dt_ = 0.1;
  if (!ros::param::get(ns + "/nbvp/dt", params_.dt_)) {
    ROS_WARN("No sampling time step specified. Looking for %s. Default is 0.1s.", (ns + "/nbvp/dt").c_str());
  }
  params_.gainRange_ = 1.0;
  if (!ros::param::get(ns + "/nbvp/gain/range", params_.gainRange_)) {
    ROS_WARN("No gain range specified. Looking for %s. Default is 1.0m.", (ns + "/nbvp/gain/range").c_str());
  }
  params_.cutoffIterations_ = 200;
  if (!ros::param::get(ns + "/nbvp/tree/cutoff_iterations", params_.cutoffIterations_)) {
    ROS_WARN("No cutoff iterations value specified. Looking for %s. Default is 200.", (ns + "/nbvp/tree/cutoff_iterations").c_str());
  }
  params_.zero_gain_ = 0.0;
  if (!ros::param::get(ns + "/nbvp/gain/zero", params_.zero_gain_)) {
    ROS_WARN("No zero gain value specified. Looking for %s. Default is 0.0.", (ns + "/nbvp/gain/zero").c_str());
  }
  params_.exact_root_ = true;
  if (!ros::param::get(ns + "/nbvp/tree/exact_root", params_.exact_root_)) {
    ROS_WARN("No option for exact root selection specified. Looking for %s. Default is true.", (ns + "/nbvp/tree/exact_root").c_str());
  }
  if (!ros::param::get(ns + "/bbx/minX", params_.minX_)) {
    ROS_WARN("No x-min value specified. Looking for %s", (ns + "/bbx/minX").c_str());
    ret = false;
  }
  if (!ros::param::get(ns + "/bbx/minY", params_.minY_)) {
    ROS_WARN("No y-min value specified. Looking for %s", (ns + "/bbx/minY").c_str());
    ret = false;
  }
  if (!ros::param::get(ns + "/bbx/minZ", params_.minZ_)) {
    ROS_WARN("No z-min value specified. Looking for %s", (ns + "/bbx/minZ").c_str());
    ret = false;
  }
  if (!ros::param::get(ns + "/bbx/maxX", params_.maxX_)) {
    ROS_WARN("No x-max value specified. Looking for %s", (ns + "/bbx/maxX").c_str());
    ret = false;
  }
  if (!ros::param::get(ns + "/bbx/maxY", params_.maxY_)) {
    ROS_WARN("No y-max value specified. Looking for %s", (ns + "/bbx/maxY").c_str());
    ret = false;
  }
  if (!ros::param::get(ns + "/bbx/maxZ", params_.maxZ_)) {
    ROS_WARN("No z-max value specified. Looking for %s", (ns + "/bbx/maxZ").c_str());
    ret = false;
  }
  if (!ros::param::get(ns + "/bbx/explorationExtensionX", params_.explorationExtensionX_)) {
    ROS_WARN("No BBx x-extension value for IG collection (exploration space) specified. Looking for %s", (ns + "/bbx/explorationExtensionX").c_str());
    ret = false;
  }
  if (!ros::param::get(ns + "/bbx/explorationExtensionY", params_.explorationExtensionY_)) {
    ROS_WARN("No BBx y-extension value for IG collection (exploration space) specified. Looking for %s", (ns + "/bbx/explorationExtensionY").c_str());
    ret = false;
  }
  if (!ros::param::get(ns + "/bbx/explorationMinZ", params_.explorationMinZ_)) {
    ROS_WARN("No BBx z-min value for IG collection (exploration space) specified. Looking for %s", (ns + "/bbx/explorationMinZ").c_str());
    ret = false;
  }
  if (!ros::param::get(ns + "/bbx/explorationMaxZ", params_.explorationMaxZ_)) {
    ROS_WARN("No BBx z-max value for IG collection (exploration space) specified. Looking for %s", (ns + "/bbx/explorationMaxZ").c_str());
    ret = false;
  }
  params_.boundingBox_[0] = 0.5;
  if (!ros::param::get(ns + "/system/bbx/x", params_.boundingBox_[0])) {
    ROS_WARN("No x size value specified. Looking for %s. Default is 0.5m.", (ns + "/system/bbx/x").c_str());
  }
  params_.boundingBox_[1] = 0.5;
  if (!ros::param::get(ns + "/system/bbx/y", params_.boundingBox_[1])) {
    ROS_WARN("No y size value specified. Looking for %s. Default is 0.5m.", (ns + "/system/bbx/y").c_str());
  }
  params_.boundingBox_[2] = 0.3;
  if (!ros::param::get(ns + "/system/bbx/z", params_.boundingBox_[2])) {
    ROS_WARN("No z size value specified. Looking for %s. Default is 0.3m.", (ns + "/system/bbx/z").c_str());
  }
  params_.boundingBoxOffset_[0] = 0.0;
  if (!ros::param::get(ns + "/system/bbx/x_offset", params_.boundingBoxOffset_[0])) {
    ROS_WARN("No x offset value specified. Looking for %s. Default is 0.0m.", (ns + "/system/bbx/x_offset").c_str());
  }
  params_.boundingBoxOffset_[1] = 0.0;
  if (!ros::param::get(ns + "/system/bbx/y_offset", params_.boundingBoxOffset_[1])) {
    ROS_WARN("No y offset value specified. Looking for %s. Default is 0.0m.", (ns + "/system/bbx/y_offset").c_str());
  }
  params_.boundingBoxOffset_[2] = 0.0;
  if (!ros::param::get(ns + "/system/bbx/z_offset", params_.boundingBoxOffset_[2])) {
    ROS_WARN("No z offset value specified. Looking for %s. Default is 0.0m.", (ns + "/system/bbx/z_offset").c_str());
  }
  params_.dOvershoot_ = 0.5;
  if (!ros::param::get(ns + "/system/bbx/overshoot", params_.dOvershoot_)) {
    ROS_WARN("No estimated overshoot value for collision avoidance specified. Looking for %s. Default is 0.5m.", (ns + "/system/bbx/overshoot").c_str());
  }
  params_.navigationFrame_ = "world";
  if (!ros::param::get(ns + "/tf_frame", params_.navigationFrame_)) {
    ROS_WARN("No navigation frame specified. Looking for %s. Default is 'world'.", (ns + "/tf_frame").c_str());
  }
  params_.pcl_throttle_ = 0.25;
  if (!ros::param::get(ns + "/pcl_throttle", params_.pcl_throttle_)) {
    ROS_WARN("No throttle time constant for the point cloud insertion specified. Looking for %s. Default is 0.25.", (ns + "/pcl_throttle").c_str());
  }
  params_.softStart_ = true;
  if (!ros::param::get(ns + "/init/softStart", params_.softStart_)) {
    ROS_WARN("No option for soft starting (clearing a flight space of free cells until first successful iteration) specified. Looking for %s. Default is true.", (ns + "/init/softStart").c_str());
  }
  params_.softStartMinZ_ = 0.0;
  if (!ros::param::get(ns + "/init/softStartMinZ", params_.softStartMinZ_)) {
    ROS_WARN("No option for soft starting MinZ (clearing a flight space of free cells only over this MinZ value) specified. Looking for %s. Default is 0.0.", (ns + "/init/softStartMinZ").c_str());
  }
  return ret;
}

template<typename stateVec>
void bspPlanning::bspPlanner<stateVec>::insertPointcloudWithTf(const sensor_msgs::PointCloud2::ConstPtr& pointcloud)
{
  static double last = ros::Time::now().toSec();
  if (last + params_.pcl_throttle_ < ros::Time::now().toSec()) {
    tree_->insertPointcloudWithTf(pointcloud);
    last += params_.pcl_throttle_;
 
    //Bsp: Free planning space until 1st planning success (5*overshoot should be enough)
    if (params_.softStart_ && init_state_ <= BSP_INITSTATE_NEEDBBXCLEARANCETOPLAN){
      Eigen::Vector3d boundingBoxSize(params_.boundingBox_[0]+params_.dOvershoot_*5,params_.boundingBox_[1]+params_.dOvershoot_*5,params_.boundingBox_[2]+params_.dOvershoot_*5);
      tree_->clearRootStateBBX(boundingBoxSize,params_.softStartMinZ_);
    }
  }

}

#endif // BSP_HPP_
