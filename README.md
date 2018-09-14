# AGRICOLMAP: Aerial-Ground Collaborative 3D Mapping for Precision Farming #

This repository contains **AGRICOLMAP**,  an  open,  research-oriented 3D map registration system for multi-robot in farming scenarios [Flourish Sapienza Datasets Website](http://www.dis.uniroma1.it/~labrococo/fds/). 

## Requirements ##

The code is tested on Ubuntu 18.04. **AGRICOLMAP** requires different tools and libraries. To install them on Ubuntu, use the terminal command:

- Install ROS Melodic by following this [guide](http://wiki.ros.org/kinetic/Installation);

```bash
sudo apt-get install ros-melodic-qt-build libyaml-cpp-dev python-catkin-tools
```

## Building ##

To build **AGRICOLMAP** on Ubuntu, type in a terminal the following command sequence.

```bash
## Creating the workspace 
mkdir -p ~/agricolmap_ws/src
cd ~/agricolmap_ws/
catkin init && catkin build
```

### Tutorial ###

In this brief tutorial, we show how to use the AGRICOLMAP to register 3D maps gathered by aerial and ground robots.
The files you need to download are:

- https://drive.google.com/uc?id=1nUYH5ofifw7xielyptZ8d2Z8G6AhiecA&export=download (Soybean Dataset)

The "Soybean Dataset" contains UAV and UGV datasets registered in a soybean farm. Other datasets are freely available on [Sapienza Collaborative Mapping Datasets](http://www.dis.uniroma1.it/~labrococo/fsd/collaborativemapping.html).

```bash
rosrun uav_ugv_collaboration_module registration_node src/agricolmap/params/aligner_soybean_params_row3.yaml 10 250 50 2
```

The 5 parameters are, respectively:

  * the .yaml param file
  * the initial scale error magnitude
  * the translational error magnitude
  * the heading error magnitude
  * the ID number for storing the resulting transform

In this case, we are registering the soybean third row, with an initial scale error magnitude of 10%, an traslational error magnitude of 2.5 metres, an heading error magnitude of 5 degrees, and an ID of 2.