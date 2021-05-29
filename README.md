# MutiverseOdometry
## Simple But Effective Redundant Odometry for Autonomous Vehicles
**Authors: **
[Andrzej Reinke](https://scholar.google.pl/citations?user=WiY7oFIAAAAJ&hl=pl),
[Xieyuanli Chen](https://scholar.google.com/citations?hl=en&user=DvrngV4AAAAJ),
[Cyrill Stachniss](https://scholar.google.com/citations?user=8vib2lAAAAAJ&hl=enchen)

**Related Papers**

* **Simple But Effective Redundant Odometry for Autonomous Vehicles**,  Andrzej Reinke, Xieyuanli Chen, Lei Tai,Cyrill Stachniss, IEEE International Conference on Robotics and Automation (ICRA) 2021. [pdf]((https://arxiv.org/abs/2105.11783)

<!-- ----------------------------------------------------------- -->
### 1. Prerequisites
1.1 **Ubuntu**

The code was tested under Ubuntu 64-bit 18.04.

1.2. **Open3D**

Installation process[here](http://www.open3d.org/docs/release/cpp_project.html)

1.3. **OpenCV 4.3**

Installation process [here](https://docs.opencv.org/4.3.0/d7/d9f/tutorial_linux_install.html)

1.4. **catkin_tools**

Installation process [here](https://catkin-tools.readthedocs.io/en/latest/installing.html)

1.4.  **Clone the repo**

```
mkdir MutiverseOdometry_ws
cd MutiverseOdometry
git clone git@github.com:PRBonn/MutiverseOdometry.git
catkin build 
```
Before running setup paths in ```demos_standalone/config/config.yaml```
With the new release the parameters are read from txt kitti file
Go to 

```
build/standalone/
```

Run (as you can see the initial name was a bit different)

```
./run_VLO_ComboVLO_pipeline
```

<!-- ----------------------------------------------------------- -->


### 6. Notes/TODO: 
- New release with more clean code is coming soon since this release is a cherry pick from semester master project.
- The plan is also to make it real time and ROS-based.
- Of course other research will be added on-top of this work.
<!-- ----------------------------------------------------------- -->
### 7. Licence
The source code is released under [GPLv3](http://www.gnu.org/licenses/) license.

 For any technical issues, please contact Andrzej Reinke <andreinkfemust@gmail.com> with tag **[MutiverseOdometry]**.


