# Mapping a 500 m² Indoor Environment Using SlamToolbox and TurtleBot: 2D SLAM

In this project, we utilized [SlamToolbox](https://github.com/SteveMacenski/slam_toolbox) and [TurtleBot3 Waffle Pi](https://emanual.robotis.com/docs/en/platform/turtlebot3/features/) to achieve 2D SLAM (Simultaneous Localization and Mapping) in a 500 m² indoor environment. The process involved recording bag files as the robot traversed the space, followed by parameter tuning to optimize the quality of the generated maps.

## Map Results Without Parameter Tuning

The resulting map clearly demonstrates significant inaccuracies. However, these experiments provided valuable insights into the specific conditions under which the algorithm struggled and which parameters should be better tuned.

![Alt text](path/to/your/image.png)

* Bad loop closure identification. 
* Poor scan correlation accuracy during complex robot movements, fast turns and abrupt maneuvers caused significant inaccuracies in scan matching.
* Error accumulation in feature-scarce areas. 

## Parameter Tuning

### General Parameters
General parameter tuning involves adjusting settings influenced by the robot's sensor capabilities, movement conditions, and the environment features.
To tune these parameters, we conducted a series of small experiments where the robot followed simple trajectories such as straight lines, circles, and loops. This approach allowed us to ensure fast iteration in the tuning process while obtaining reliable results under various movement conditions.
<table>
  <tr>
    <td><img src="path/to/your/image1.png" alt="Image 1" width="300"></td>
    <td><img src="path/to/your/image2.png" alt="Image 2" width="300"></td>
  </tr>
  <tr>
    <td><img src="path/to/your/image3.png" alt="Image 3" width="300"></td>
    <td><img src="path/to/your/image4.png" alt="Image 4" width="300"></td>
  </tr>
</table>

#### Results after general parameter tuning

![Alt text](path/to/your/image.png)

The map generated for one of the rooms demonstrates significant improvement. However, due to incomplete loop closure settings, some areas that should overlap are misaligned.

### Loop Closure Parameters



