# AgriColMap: Aerial-Ground Collaborative 3D Mapping for Precision Farming #

This repository contains **AgriColMap**,  an  open,  research-oriented 3D map registration system for multi-robot in farming scenarios. This software has been tested using the [UAV-UGV Collaborative Mapping Dataset](http://www.dis.uniroma1.it/~labrococo/fds/collaborativemapping.html) distributed within the [Flourish Sapienza Datasets](http://www.dis.uniroma1.it/~labrococo/fds/) collection. 

## Requirements ##

The code is tested on Ubuntu 18.04. **AgriColMap** requires requires different tools and libraries. To install them on Ubuntu, use the terminal command:

- Install ROS Melodic by following this [guide](https://wiki.ros.org/melodic/Installation/Ubuntu);

```bash
sudo apt-get install libyaml-cpp-dev python-catkin-tools
```

## Building ##

To build **AgriColMap** on Ubuntu, type in a terminal the following command sequence.

```bash
## Creating the workspace 
mkdir -p ~/agricolmap_ws/src
git clone https://bitbucket.org/cirpote/agricolmap
cd ~/agricolmap_ws/
catkin init && catkin build
```

### Tutorial ###

In this brief tutorial, we briefly show how to use the AgriColMap to register 3D maps gathered by aerial and ground robots.
The files you need to download are:

- https://drive.google.com/uc?id=1nUYH5ofifw7xielyptZ8d2Z8G6AhiecA&export=download (Soybean Dataset)

Uncompress the downloaded file into: ~/agricolmap_ws/src/agricolmap/maps/. The "Soybean Dataset" contains UAV and UGV datasets registered in a soybean farm. Other datasets are freely available on [Sapienza Collaborative Mapping Datasets](http://www.dis.uniroma1.it/~labrococo/fsd/collaborativemapping.html).

```bash
rosrun uav_ugv_collaboration_module registration_node src/agricolmap/params/aligner_soybean_params_row3.yaml 10 250 50 2
```

The 5 parameters are, respectively:

  * the .yaml param file
  * the initial scale error magnitude
  * the translational error magnitude
  * the heading error magnitude
  * the ID number for storing the resulting transform

In this case, we are registering the third row of the soybean dataset with an initial scale error magnitude of 10%, an traslational error magnitude of 2.5 metres, an heading error magnitude of 5 degrees, and an ID of 2.

### License ###

AgriColMap is licensed under the GPL2 License. However, some libraries are available under different license terms. See below.

The following parts are licensed under GPL3:

CPM

The following parts are licensed under GPL2:

cpd

AgriColMap is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the licenses for more details.

### Citing ###

Please cite the [following paper](https://arxiv.org/abs/1810.00457) when using AgriColMap for your research:

```bibtex
@article{pknnp_arxiv2018,
  title={{A}gri{C}ol{M}ap: {A}erial-Ground Collaborative {3D} Mapping
         for Precision Farming},
  author={Potena, Ciro and Khanna, Raghav and Nieto, Juan and
          Siegwart, Roland and Nardi, Daniele and Pretto, Alberto},
  journal={arXiv:1810.00457},
  year={2018}
}
```