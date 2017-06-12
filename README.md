# **rhem_planner**
Uncertainty-aware Receding Horizon Exploration and Mapping ROS stack repository.

The Uncertainty-aware RHEM planner performs planning for autonomous exploration, while ensuring that viewpoint-to-viewpoint trajectories minimize localization & mapping uncertainty. Its operation depends on closing the loop between two pipelines:

# *Planning pipeline*:

Implemented in the **bsp_planner** ("belief-space propagation" planner) package, it is responsible for maintaining and updating the probabilistic [volumetric map](https://github.com/ethz-asl/volumetric_mapping) of the environment, and for performing a *2-layer* planning optimization:

* The first planning layer gives the *next-best-viewpoint* that leads towards an increased mapping knowledge (volumetric exploration & mapping probability improvement).
* The second level gives the *best-trajectory* to reach that selected viewpoint (in terms of localization & mapping uncertainty).

# *Estimation & Propagation pipeline*:

The rhem_planner stack provides a localization and mapping pipeline, augmented to perform both tasks. The included pipeline is based on a *modified* version of the [Robust Visual-Inertial Odometry](https://github.com/ethz-asl/rovio) pipeline. It is responsible for:

* Calculating and maintaining the real-time localization & mapping state and statistics (Estimation).
* Forward-simulation of the process given an initial state and statistics and a sample trajectory (Propagation).

# *Loop-Closure scheme*:

The *Planning* pipeline performs RRT-based sampling to reach the 1st layer's next-best-viewpoint. Each RRT vertex carries the *Estimation & Propagation* pipeline state (current pose, landmarks, and associated statistics). The communication between the two pipelines is performed with ROS service-based data transactions:

 * The *Planning* pipeline requests and acquires from the *Estimation* pipeline the current state, which is used to populate the root vertex "belief".
 * For each RRT vertex, the *Planning* pipeline transfers its parent's vertex "belief" to the *Propagation* pipeline. During the first request, also the current 3D occupancy map is tranferred (used by the *Propagation* pipeline to evaluate feature visibility). 
 * The *Propagation* pipeline performs forward-simulation of the localization and mapping process, and responds with the resulting "belief" state at the target vertex. The *Planning* pipeline stores this  and re-uses it in subsequent propagation calls.
 * The process is repeated, until the 1-st layer's next-best-viewpoint is reached via a trajectory that gives better localization & mapping statistics than the "straight-path" motion towards it.

# Staging the stack

First, download the **rhem_planner** stack, which contains the packages above. Navigate to the source folder of your ros catkin workspace and:

```sh
git clone https://github.com/unr-arl/rhem_planner.git
cd rhem_planner
git submodule init --
git submodule sync --recursive
git submodule update --init --recursive
```
This framework employs [rovio_bsp](https://github.com/unr-arl/rovio_bsp), a *modified* version of the [Robust Visual-Inertial Odometry](https://github.com/ethz-asl/rovio) pipeline (included as a submodule). 

Make sure you have the following dependencies:

* [ROS](http://wiki.ros.org/)
* [Octomap](http://wiki.ros.org/octomap)
* [catkin_tools](http://catkin-tools.readthedocs.io/en/latest/installing.html)

Navigate back to your ros catkin workspace folder and proceed with a Release build:

```sh
catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

# Testing the planner

To launch the RHEM pipeline directly you can use the following launch file:

```sh
roslaunch rhem_planner rhem_exploration.launch
```

This will bring up:

* **bsp_planner**: The planning pipeline node.
* **rovio_node**: The estimation pipeline node.
* **rovio_bsp_node**: The propagation pipeline node.

To trigger a planning iteration a call to the respective *bsp_planner* service is required. This can be directly performed from the terminal:

```sh
rosservice call /bsp_planner '{header: {stamp: now, frame_id: world}}'
```
If you use this software in a scientific publication, please cite the following paper:
```
@inproceedings{papachristos2017uncertainty,
  title={Uncertainty-aware Receding Horizon Exploration and Mapping Using Aerial Robots},
  author={Papachristos, Christos and Khattak, Shehryar and Alexis, Kostas},
  booktitle={2017 IEEE International Conference on Robotics and Automation (ICRA)},
  pages={to appear},
  year={2017},
  organization={IEEE}
}
```

# Contact

You can contact us for any question or remark:
* [Christos Papachristos](mailto:cpapachristos@unr.edu)
* [Shehryar Khattak](mailto:shehryar@nevada.unr.edu)
* [Kostas Alexis](mailto:kalexis@unr.edu)
