# MutiverseOdometry
## Simple But Effective Redundant Odometry for Autonomous Vehicles
**Authors:**
[Andrzej Reinke](https://scholar.google.pl/citations?user=WiY7oFIAAAAJ&hl=pl),
[Xieyuanli Chen](https://scholar.google.com/citations?hl=en&user=DvrngV4AAAAJ),
[Cyrill Stachniss](https://scholar.google.com/citations?user=8vib2lAAAAAJ&hl=enchen)

**Related Papers**

* **Simple But Effective Redundant Odometry for Autonomous Vehicles**,  Andrzej Reinke, Xieyuanli Chen, Lei Tai,Cyrill Stachniss, IEEE International Conference on Robotics and Automation (ICRA) 2021. [pdf](https://arxiv.org/abs/2105.11783)
```
@inproceedings{reinke2021icra,
title={{Simple But Effective Redundant Odometry for Autonomous Vehicles}},
author={A. Reinke and X. Chen and C. Stachniss},
booktitle={Proc. of the IEEE Intl. Conf. on Robotics \& Automation (ICRA)},
year=2021,
url = {http://www.ipb.uni-bonn.de/pdfs/reinke2021icra.pdf}
}
```

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

### 2. How to run
2.1.  **Clone the repo**

```
mkdir MutiverseOdometry_ws
cd MutiverseOdometry
git clone git@github.com:PRBonn/MutiverseOdometry.git
catkin build 
```
2.1.  **Setup config file**

Before running setup paths in ```demos_standalone/config/config.yaml```
With the new release the parameters are read from txt kitti file

2.1.1 **Odometries**
Here are available odometries that you can setup (in a default mode all odometries are available)

```
odometries:
  colorbased_vlo: null
  gicp_lo: null
  huang_vlo:
    sift:
      contrastThreshold: 0.04
      edgeThreshold: 10.0
      nOctaveLayers: 3
      nfeatures: 0
      sigma: 1.6
  ndt_lo: null
  point2plane_lo: null
```
null are necessary in order to yaml work
In the future relase there will be more things to play with since there will be a chance to pick matcher and feature descriptor for the image-based odometry for Huang's method (also with deep learning methods and semantic/panoptic information)

2.2.  **Run**

Go to  ```build/standalone/```

Run (as you can see the initial name was a bit different)

```./run_VLO_ComboVLO_pipeline```

<!-- ----------------------------------------------------------- -->


### 3 Notes/TODO: 
- New release with more clean code is coming soon since this release is a cherry pick from semester master project.
- The plan is also to make it real time and ROS-based.
- Of course other research will be added on-top of this work.
<!-- ----------------------------------------------------------- -->
### 4. Licence
The source code is released under [GPLv3](http://www.gnu.org/licenses/) license.

 For any technical issues, please contact Andrzej Reinke <andreinkfemust@gmail.com> with tag **[MutiverseOdometry]**.

