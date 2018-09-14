# AGRICOLMAP: Aerial-Ground Collaborative 3D Mapping for Precision Farming #

This repository contains **AGRICOLMAP**,  an  open,  research-oriented 3D map registration system for multi-robot in farming scenarios [Flourish Sapienza Datasets Website](http://www.dis.uniroma1.it/~labrococo/fds/). 

# References ##

Paper Describing the Approach:

Marco Imperoli*, Ciro Potena*, Daniele Nardi, Giorgio Grisetti, Alberto Pretto: An Effective Multi-Cue Positioning System for Agricultural Robotics, In arXiv preprint arXiv:1803.00954 ([PDF](https://arxiv.org/abs/1803.00954))


```bash
@article{ipngp_arxiv2018,
    title={An Effective Multi-Cue Positioning System for Agricultural Robotics},
    author={Imperoli, Marco and Potena, Ciro and Nardi, Daniele and Grisetti, Giorgio and Pretto, Alberto},
    journal={IEEE Robotics and Automation Letters},
    volume = {3},
    number = {4},   
    year = {October 2018},
    pages = {3685--3692},
    doi={10.1109/LRA.2018.2855052}
} 
```

Please also check out our video:

[Youtube Video link](https://youtu.be/l2CxYKS3tkgk)   

## Requirements ##

The code is tested on Ubuntu 16.04. **MCAPS** requires different tools and libraries. To install them on Ubuntu, use the terminal command:

- Install ROS Kinetic by following this [guide](http://wiki.ros.org/kinetic/Installation);
- Install g2o by downloading this [repo](https://github.com/Imperoli/g2o) and following the guide.

```bash
sudo apt-get install ros-kinetic-qt-build ros-kinetic-gps-common ros-kinetic-velodyne libyaml-cpp-dev libpcap0.8-dev qtdeclarative5-dev python-catkin-tools
```

## Building ##

To build **MCAPS** on Ubuntu, type in a terminal the following command sequence.

```bash
## Creating the workspace 
mkdir -p ~/mcaps_ws/src
cd ~/mcaps_ws/
catkin init && catkin build

## Cloning and compiling the required ROS packages
cd src
git clone https://github.com/catkin/catkin_simple
git clone https://github.com/ethz-asl/geodetic_utils
git clone https://bitbucket.org/cirpote/pose_graph
git clone https://bitbucket.org/Imperoli/flourish_mapping
cd .. && catkin build
```

### Tutorial ###

In this brief tutorial, we show how to use the MCAPS toolbox to retrieve the global pose of a ground robot within an agricultural field, while building a map of the environment: 
The files you need to download are:

- https://drive.google.com/open?id=191boDPpW75dgxBqWySAHFU4lxaxkoAFM (Config files)
- https://drive.google.com/open?id=1OGii-FrMfA1-pDJrrpXbe_kXSWFfTcUJ (DatasetA Calib)

The "DatasetA Calib" is a reduced and calibrated version of DatasetA, freely available on [Sapienza Mapping Datasets](http://www.dis.uniroma1.it/~labrococo/fds/mappingdatasets.html).
The Config files econde all parameters and paths requried to run toolbox.
Now, change the absolute paths in all the *.yaml files, and then call:

```bash
rosrun flourish_mapping flourish_mapping data/gui_params.yaml data/opt_params.yaml
```

  </export>
</package>
