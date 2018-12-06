# AgriColMap: Aerial-Ground Collaborative 3D Mapping for Precision Farming #

This repository contains **AgriColMap**,  an  open,  research-oriented 3D map registration system for multi-robot in farming scenarios. This software has been tested using the [UAV-UGV Collaborative Mapping Dataset](http://www.dis.uniroma1.it/~labrococo/fds/collaborativemapping.html) distributed within the [Flourish Sapienza Datasets](http://www.dis.uniroma1.it/~labrococo/fds/) collection. Please also check out our video:

## Installation with OpenCV > 3.2.0 (with extra modules) and PCL 1.8.1 ##

```bash
sudo apt-get install libyaml-cpp-dev python-catkin-tools
## Creating the workspace 
git clone https://bitbucket.org/cirpote/agricolmap
git submodule update --init --recursive
cd /agricolmap && mkdir build && cd build
cmake ..
make -j8
```

### Tutorial ###

In this tutorial, we briefly show how to use the AgriColMap to register 3D maps gathered by aerial and ground robots.
The files you need to download are:

- https://drive.google.com/uc?id=1nUYH5ofifw7xielyptZ8d2Z8G6AhiecA&export=download (Soybean Dataset)

Uncompress the downloaded file into: ${PATH_TO_AGRICOLMAP}/maps/. The "Soybean Dataset" contains UAV and UGV datasets registered in a soybean farm. Other datasets are freely available on [Sapienza Collaborative Mapping Datasets](http://www.dis.uniroma1.it/~labrococo/fsd/collaborativemapping.html).

```bash
cd bin
./registration_node ../params/aligner_soybean_params_row3.yaml  10 250 50 2
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
