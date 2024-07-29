# Mapping a 500 m² Indoor Environment Using SlamToolbox and TurtleBot: 2D SLAM

In this project, we utilized [SlamToolbox](https://github.com/SteveMacenski/slam_toolbox) and [TurtleBot3 Waffle Pi](https://emanual.robotis.com/docs/en/platform/turtlebot3/features/) to achieve 2D SLAM (Simultaneous Localization and Mapping) in a 500 m² indoor environment. The process involved recording bag files as the robot moved around the space, followed by parameter tuning to optimize the quality of the generated maps.

## Map Results Without Parameter Tuning

The resulting map clearly demonstrates significant inaccuracies. However, these experiments provided valuable insights into the specific conditions under which the algorithm struggled and which parameters should be better tuned.

![Alt text](https://github.com/RonaldoCD/slam_group_project_ecn/blob/main/group_project/imgs/without_tunning.png)

* Poor scan correlation accuracy during complex robot movements, fast turns and abrupt maneuvers caused significant inaccuracies in scan matching.
* Error accumulation in feature-scarce areas. 
* Loop closure recognition was not working properly, parameter tuning needed.
   
## Parameter Tuning

### General Parameters
General parameter tuning involves adjusting settings influenced by the robot's sensor capabilities, movement conditions, and the environment features.
To tune these parameters, we conducted a series of small experiments where the robot followed simple trajectories such as straight lines, circles, and loops. This approach allowed us to ensure fast iteration in the tuning process while obtaining reliable results under various movement conditions.
<table>
  <tr>
    <td><img src="https://github.com/RonaldoCD/slam_group_project_ecn/blob/main/group_project/imgs/line_with_objects.png" alt="Image 1" width="300"></td>
    <td><img src="https://github.com/RonaldoCD/slam_group_project_ecn/blob/main/group_project/imgs/line_without_objects.png" alt="Image 2" width="300"></td>
  </tr>
  <tr>
    <td><img src="https://github.com/RonaldoCD/slam_group_project_ecn/blob/main/group_project/imgs/circle.png" alt="Image 3" width="300"></td>
    <td><img src="https://github.com/RonaldoCD/slam_group_project_ecn/blob/main/group_project/imgs/one_loop.png" alt="Image 4" width="300"></td>
  </tr>
</table>

#### Results after general parameter tuning

![Alt text](https://github.com/RonaldoCD/slam_group_project_ecn/blob/main/group_project/imgs/after_general_params_tuning.jpg)

The map generated for one of the rooms demonstrates significant improvement. However, due to incomplete loop closure settings, some areas that should overlap are misaligned.

### Loop Closure Parameters

As it is illustrated in the [main diagram](https://github.com/SteveMacenski/slam_toolbox/blob/ros2/images/slam_toolbox_sync.png) of the Slam Toolbox package, the loop closure identication is performed by the funtion [TryCloseLoop](https://github.com/SteveMacenski/slam_toolbox/blob/ros2/lib/karto_sdk/src/Mapper.cpp#L1500-L1560). 

With an understanding of how the TryCloseLoop function operates, we decided to fine-tune the following parameters, which primarily serve as thresholds. Parameters related to scan correlation and matching were set to the default recommended values provided by Slam Toolbox.

* loop_match_minimum_chain_size
* loop_match_maximum_variance_coarse
* loop_match_minimum_response_coarse
* loop_match_minimum_response_fine

The `TryCloseLoop` function performs the following steps to identify and confirm loop closures:

1. **Candidate Chain Evaluation:** 
   The function first generates a `candidateChain`—a vector of potential nodes in the pose graph that could establish a loop closure with the current node. If the size of this chain is smaller than the parameter `loop_match_minimum_chain_size`, the loop closure is discarded.

2. **Initial Scan Matching:**
   Next, the `MatchScan` function is called to identify the node within the `candidateChain` that best matches the current node. This matching process is performed at a coarse or low resolution. For the loop closure identification to proceed, the computed variance and coarse response of the best candidate must be below and above the thresholds set by the parameters `loop_match_maximum_variance_coarse` and `loop_match_minimum_response_coarse`, respectively.

3. **Refined Scan Matching:**
   The `MatchScan` function is then called again, but this time at a finer resolution to achieve more precise correlation and improve pose estimation. The fine response value must exceed the threshold defined by the `loop_match_minimum_response_fine` parameter.

4. **Loop Closure Confirmation:**
   If all the specified conditions are met, a loop closure is confirmed. The best candidate node is then linked to the current node, and the poses in the pose graph are optimized to incorporate the new loop closure information.

#### Loop Closure Parameter Tunning Process

By printing the computed values for the candidate chain size, covariance, and coarse and fine responses to the terminal, we were able to tune the parameters effectively in order to generate loop closures where they were expected and avoid undesired ones. 

![Alt text](path/to/your/image.png)

#### Results of Loop Closure Parameter Tuning

In this example, a correct loop closure is identified which aligns the walls and obstables of the previous map.

![Alt text](https://github.com/RonaldoCD/slam_group_project_ecn/blob/main/group_project/imgs/after_loop_closure_tuning.jpg)

## Mapping Results

We recorded a ROS2 bag file for each room. Using the functionalities of SlamToolbox, we merged the generated maps into a single, unified map.

<table>
  <tr>
    <td><img src="path/to/your/image1.png" alt="Image 1" width="300"></td>
    <td><img src="path/to/your/image2.png" alt="Image 2" width="300"></td>
  </tr>
</table>

![Alt text](path/to/your/image.png)

![Alt text](path/to/your/image.png)

https://github.com/user-attachments/assets/b1e987fc-6042-4b13-85b0-baf47fd0ea3d

## Navigation

https://github.com/user-attachments/assets/a0b4e7d9-3fde-4a3b-bfda-23ecbb3080f9










