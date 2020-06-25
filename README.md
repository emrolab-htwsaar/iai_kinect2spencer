# kinect2_adapt

Middleware for use between [iai_kinect2](https://github.com/code-iai/iai_kinect2) and [spencer_people_tracking](https://github.com/spencer-project).

## Maintainer:
- Philip Hoffmann <<philip.hoffmann@htwsaar.de>>, [Embedded Robotics Lab](http://emrolab.htw-saarland.de/), htwsaar - University of Applied Science

### Dependencies:
* [Robot Operating System](http://www.ros.org/), please check the version needed inside the SPENCER and iai_kinect2 README.

### Installation:
1) Follow the installation instructions of [ROS](http://www.ros.org/install/), [iai_kinect2](https://github.com/code-iai/iai_kinect2#install) and [spencer_people_tracking](https://github.com/spencer-project/spencer_people_tracking#installation)

2) Clone this repository to your catkin_workspace (where you installed the above packages)

3) Compile using `catkin_make -DCMAKE_BUILD_TYPE=Release`

### Usage:
`roslaunch spencer_launch spencer_launch.launch`

This will launch iai_kinect2, image_resampler and spencer_people_tracking.

### RVIZ Visualization:
There is a preconfigured RVIZ config file under iai_kinect2_adapt/spencer_launch/viz/rviz_config.rviz

You can either load this config by hand or alter the visualization setting in the **spencer_launch.launch** file to **true**.

To start RVIZ manually use `rosrun rviz rviz`

## Package: spencer_launch
This package launches the whole SPENCER Detection Pipeline including iai_kinect2, image_resampler and spencer_people_tracking.

## Package: image_resapmpler
This package takes a depth image (provided by iai_kinect2), changes the resolution, adapts its encoding and republishes it to an user specified topic.

In case you want to run this package only (no iai_kinect2, no SPENCER) you can use the following command:

`roslaunch image_resampler image_resampler.launch`

### Configuration
The settings are preconfigured to work with iai_kinect2 and spencer_people_tracking

Source and target topics as well as target resolution can be configured via settings.yaml

### Current Restrictions:
* source_topic.encoding must be 16UC1
* target_topic.encoding will be 32FC1
* BigEndian encoded images are not supported

### Config locations:
image_resampler: iai_kinect2_adapt/config/settings.yaml

RVIZ: iai_kinect2_adapt/spencer_launch/viz/rviz_config.rviz

## Credits & License
This project is licensed under [BSD-3-Clause](BSD-3-Clause). Please see the included LICENSE file for details.

This application uses or is based of Open Source components. You can find the source code of their open source projects along with license information below. We acknowledge and are grateful to these developers for their contributions to open source.
   
### SPENCER
Project:  spencer_people_tracking/launch/spencer_people_tracking_launch https://github.com/spencer-project/spencer_people_tracking/blob/master/launch/spencer_people_tracking_launch

Copyright (c) 2014-2015 Timm Linder, Social Robotics Laboratory, University of Freiburg

Copyright (c) 2014-2015 Stefan Breuers, Computer Vision Group, RWTH Aachen University

License (3-Clause BSD) https://github.com/spencer-project/spencer_people_tracking/blob/master/launch/spencer_people_tracking_launch/LICENSE
   
### iai_kinect2
Project:  iai_kinect2 https://github.com/code-iai/iai_kinect2

Copyright (c) 2014 Thiemo Wiedemeyer, Institute for Artificial Intelligence - University of Bremen

License ( Version 2.0) https://github.com/code-iai/iai_kinect2/blob/master/LICENSE
