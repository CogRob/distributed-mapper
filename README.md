Distributed-Mapper
===================================================
This library is an implementation of the algorithm described in Distributed Trajectory Estimation with Privacy and Communication Constraints: 
a Two-Stage Distributed Gauss-Seidel Approach. The core library is developed in C++
language.

Distributed-Mapper is developed by [Siddharth Choudhary](http://www.cc.gatech.edu/~choudhar/), 
[Luca Carlone](http://www.lucacarlone.com/), [Carlos Nieto](https://scholar.google.com/citations?user=p3S78jsAAAAJ&hl=en) and [John Rogers](https://scholar.google.com/citations?user=uH_LDocAAAAJ&hl=en) as part of the collaboration between Georgia Tech, MIT and Army Research Lab. 

Prerequisites
------

- CMake (Ubuntu: `sudo apt-get install cmake`), compilation configuration tool.
- [Boost](http://www.boost.org/)  (Ubuntu: `sudo apt-get install libboost-all-dev`), portable C++ source libraries.
- [GTSAM](https://bitbucket.org/gtborg/gtsam) develop branch, a C++ library that implement smoothing and mapping (SAM) framework in robotics and vision. Here we use factor graph implementations and inference/optimization tools provided by GTSAM. To install a particular commit of GTSAM follow the following instructions: 

```
$ git clone https://bitbucket.org/gtborg/gtsam
$ git checkout b7c695fa71efd43b40972eec154df265617fc07d -b dist-mapper
$ mkdir build
$ cmake ..
$ make -j8
$ sudo make install
```

Compilation & Installation
------

In the ```cpp``` folder excute:

```
$ mkdir build
$ cd build
$ cmake ..
$ make -j3
$ make check  # optonal, run unit tests
$ make install
```

Run Experiments On Simulated Block World data
------
In the ```cpp/build``` folder, run:
```
$ make testDistributedMapper.run
```

To plot per-robot graph errors, convergence and other statistics: change the variable ```nrRobots``` and ```filename``` in ```matlab/plotTrace.m``` and run it. ```filename``` corresponds to the tracefile name mentioned in the unit test. See ```plotTrace.m``` for an example.

Run Distributed Mapper on a dataset
------
In the ```cpp/build/``` folder, run:
```
$ ./runDistributedMapper --nrRobots <num_robots> --dataDir <data_directory>
```
For example:
```
$ ./runDistributedMapper --nrRobots 2 --dataDir ../../data/example_2robots/
```
OR
```
$ ./runDistributedMapper --nrRobots 4 --dataDir ../../data/example_4robots/
$ ./runDistributedMapper --nrRobots 9 --dataDir ../../data/example_9robots/
$ ./runDistributedMapper --nrRobots 16 --dataDir ../../data/example_16robots/
$ ./runDistributedMapper --nrRobots 25 --dataDir ../../data/example_25robots/
$ ./runDistributedMapper --nrRobots 36 --dataDir ../../data/example_36robots/
$ ./runDistributedMapper --nrRobots 49 --dataDir ../../data/example_49robots/
```

Data Format
-----
Each robot's graph is written in [g2o](https://github.com/RainerKuemmerle/g2o/wiki/File-Format) format and is indexed from 0. For example, for a 4 robot scenario, the directory will contain ```0.g2o```, ```1.g2o```, ```2.g2o``` and ```3.g2o```. An example dataset for 4 robots is given in ```data/example_4robots```. Each robot is specified using a character prefix symbol like 'a', 'b', 'c', 'd' for 4 robot case.

### Vertices ###
All the vertices corresponding to the first robot will be prefixed using 'a' using ```gtsam.Symbol``` like  ```gtsam.Symbol('a', 1)```, ```gtsam.Symbol('a',2)``` etc. Similarly the second robot will be prefixed using 'b' like ```gtsam.Symbol('b', 1)```, ```gtsam.Symbol('b',2)``` etc. 
For example, the vertices for the first robot (in ```0.g2o```) in 4 robot scenario is written as,  
```
VERTEX_SE3:QUAT 6989586621679009792 0.324676 0.212487 0.042821 0.00270783 0.0121983 0.00760222 0.999893
VERTEX_SE3:QUAT 6989586621679009793 0.0716917 2.00724 -0.0729262 -0.00363348 0.00166876 0.00765756 0.999963
VERTEX_SE3:QUAT 6989586621679009796 1.99449 0.184786 -0.0642561 0.0125092 0.0130271 0.00220908 0.999834
VERTEX_SE3:QUAT 6989586621679009797 1.93989 1.89999 0.143294 0.00127135 0.0167209 0.0057457 0.999843
VERTEX_SE3:QUAT 6989586621679009808 0.487317 0.0604815 2.01166 0.00685618 0.00528219 -0.00553837 0.999947
VERTEX_SE3:QUAT 6989586621679009809 -0.351155 1.94853 2.14991 -0.000449498 -0.00170132 -0.00501099 0.999986
VERTEX_SE3:QUAT 6989586621679009812 1.95844 0.179349 2.08246 -0.0015633 -0.00662232 0.00133146 0.999976
VERTEX_SE3:QUAT 6989586621679009813 2.01223 2.04334 1.72038 -0.0031675 0.00127498 0.00478204 0.999983
VERTEX_SE3:QUAT 6989586621679009824 0.375458 0.121848 3.97795 -0.00206729 0.00230435 0.00676051 0.999972
VERTEX_SE3:QUAT 6989586621679009825 0.0907182 1.8793 3.8406 0.00151304 0.000763284 0.00242012 0.999996
VERTEX_SE3:QUAT 6989586621679009828 1.78298 -0.141432 3.93967 -0.00459483 -0.0122962 -0.00755554 0.999885
VERTEX_SE3:QUAT 6989586621679009829 1.80735 2.22278 3.68278 0.00296836 -0.000200073 -0.0104751 0.999941
VERTEX_SE3:QUAT 6989586621679009840 0.157452 -0.255627 5.88308 0.00907326 0.0300276 0.0308352 0.999032
VERTEX_SE3:QUAT 6989586621679009841 0.021695 1.93194 5.81483 0.00423999 0.00464847 -0.00542489 0.999965
VERTEX_SE3:QUAT 6989586621679009844 1.8204 -0.431461 6.01878 0.00230292 -0.00198441 0.00720772 0.999969
VERTEX_SE3:QUAT 6989586621679009845 1.92566 2.23857 6.19073 0.0152246 0.0101054 -0.00845407 0.999797
```
### Intra-Robot Edges ###
 All the non-communication edges corresponding to the first robot is written in g2o format in ```0.g2o``` and likewise for the other robots in ```1.g2o```, ```2.g2o```, ```3.g2o``` respectively. For example, the intra-robot edges for the first robot (in ```0.g2o```) in 4 robot scenario is written as,  
 
```
EDGE_SE3:QUAT 6989586621679009792 6989586621679009793 -0.390787 2.11308 0.0155589 0.00042254 -0.00328797 -0.010647 0.999938 1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1
EDGE_SE3:QUAT 6989586621679009792 6989586621679009796 2.30676 0.546708 0.0334878 -0.00546279 -0.00389958 0.00693602 0.999953 1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1
EDGE_SE3:QUAT 6989586621679009793 6989586621679009797 2.05794 0.233082 -0.181741 0.00845604 -0.0123624 -0.00407352 0.99988 1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1
EDGE_SE3:QUAT 6989586621679009796 6989586621679009797 -0.110238 1.98252 -0.0695641 0.0111376 -0.00568388 0.00216275 0.999919 1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1
EDGE_SE3:QUAT 6989586621679009792 6989586621679009808 0.0306595 -0.250865 1.76544 0.0014922 0.00916009 0.00622605 0.999938 1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1
EDGE_SE3:QUAT 6989586621679009808 6989586621679009809 -0.322983 2.44921 0.147856 -0.00149178 -0.0108401 -0.0127671 0.999859 1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1
EDGE_SE3:QUAT 6989586621679009793 6989586621679009809 -0.342576 -0.0919789 2.2182 -0.00111701 -0.0100567 -0.0180511 0.999786 1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1
EDGE_SE3:QUAT 6989586621679009808 6989586621679009812 2.09196 -0.0435013 0.159225 0.00238566 -0.00721739 -0.0031607 0.999966 1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1
EDGE_SE3:QUAT 6989586621679009796 6989586621679009812 0.104281 -0.198205 1.94937 -0.0269272 -0.009021 -0.0134929 0.999506 1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1
EDGE_SE3:QUAT 6989586621679009809 6989586621679009813 1.83031 -0.0480742 0.120586 -0.00384726 0.000411292 0.00880903 0.999954 1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1
EDGE_SE3:QUAT 6989586621679009812 6989586621679009813 -0.284145 2.13387 0.136647 0.00681912 -0.000686726 -0.013227 0.999889 1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1
EDGE_SE3:QUAT 6989586621679009797 6989586621679009813 0.161651 0.115788 2.15257 0.00367834 -0.0131576 -0.00764767 0.999877 1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1
EDGE_SE3:QUAT 6989586621679009808 6989586621679009824 0.0574064 -0.0929209 2.07686 -0.00229244 0.0244892 0.0109378 0.999638 1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1
EDGE_SE3:QUAT 6989586621679009824 6989586621679009825 -0.169446 1.88482 -0.0151826 0.0141175 -0.000934893 -0.00330174 0.999894 1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1
EDGE_SE3:QUAT 6989586621679009809 6989586621679009825 -0.19564 -0.128205 2.1041 0.0177383 0.0125375 -0.00137839 0.999763 1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1
EDGE_SE3:QUAT 6989586621679009824 6989586621679009828 2.28126 0.0993667 0.0165661 0.0171145 0.00232563 0.00442653 0.999841 1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1
EDGE_SE3:QUAT 6989586621679009812 6989586621679009828 -0.309201 0.0866565 2.02059 0.00138903 0.0162411 -0.0135298 0.999776 1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1
EDGE_SE3:QUAT 6989586621679009825 6989586621679009829 1.73039 -0.35086 -0.0727618 -0.00615263 0.00433336 -0.00495048 0.999959 1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1
EDGE_SE3:QUAT 6989586621679009828 6989586621679009829 -0.0416471 1.69897 0.361948 -0.0130913 0.0039124 -0.00542152 0.999892 1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1
EDGE_SE3:QUAT 6989586621679009813 6989586621679009829 0.181719 -0.230844 2.25151 0.00432985 0.010696 -0.00106693 0.999933 1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1
EDGE_SE3:QUAT 6989586621679009824 6989586621679009840 0.0508859 -0.0305667 1.88852 0.00665235 -0.00531186 0.0022339 0.999961 1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1
EDGE_SE3:QUAT 6989586621679009840 6989586621679009841 -0.0479563 2.1842 -0.181376 0.0040515 0.00743545 0.00587749 0.999947 1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1
EDGE_SE3:QUAT 6989586621679009825 6989586621679009841 -0.168767 0.388688 1.95062 -0.00167067 0.0121836 0.00128056 0.999924 1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1
EDGE_SE3:QUAT 6989586621679009840 6989586621679009844 1.90164 -0.19386 -0.0948216 -0.0140169 0.0117721 -0.00722882 0.999806 1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1
EDGE_SE3:QUAT 6989586621679009828 6989586621679009844 -0.164132 0.0202259 2.2168 -0.00906032 0.00422716 -0.00577513 0.999933 1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1
EDGE_SE3:QUAT 6989586621679009841 6989586621679009845 2.12955 0.209913 0.264619 -0.00740909 0.0158267 -0.0160603 0.999718 1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1
EDGE_SE3:QUAT 6989586621679009844 6989586621679009845 -0.121132 2.05434 0.0717287 -0.00141595 0.000218583 -0.00451449 0.999989 1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1
EDGE_SE3:QUAT 6989586621679009829 6989586621679009845 -0.00151322 0.142061 1.90749 0.0233327 0.00108301 -0.00310063 0.999722 1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1
```

### Inter-Robot Communication Edges ###
Communication edges between the two robots are written in the g2o files corresponding to both the robots.  For example, the communication edges between the first and second robot (in ```0.g2o``` and ```1.g2o```) in 4 robot scenario is written as,  

```
EDGE_SE3:QUAT 6989586621679009793 7061644215716937730 0.160437 1.90624 -0.00931847 0.00910267 0.0069412 0.000641041 0.999934 1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1
EDGE_SE3:QUAT 6989586621679009797 7061644215716937734 -0.00807974 2.13944 0.0337605 -0.0165965 -0.0143173 0.00265868 0.999756 1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1
EDGE_SE3:QUAT 6989586621679009809 7061644215716937746 -0.111132 1.72969 0.0728423 -0.00874126 -0.000574261 -0.0014352 0.999961 1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1
EDGE_SE3:QUAT 6989586621679009813 7061644215716937750 -0.227448 2.1286 -0.00255201 0.0131718 0.00877742 0.001412 0.999874 1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1
EDGE_SE3:QUAT 6989586621679009825 7061644215716937762 0.111804 2.11273 -0.0676793 0.0143307 0.0173408 -0.00356963 0.999741 1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1
EDGE_SE3:QUAT 6989586621679009829 7061644215716937766 -0.0236693 1.74212 0.0115127 0.00212283 0.00521578 0.00242397 0.999981 1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1
EDGE_SE3:QUAT 6989586621679009841 7061644215716937778 0.0934036 2.00091 0.0652181 0.00253681 -0.00700303 0.00387134 0.999965 1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1
EDGE_SE3:QUAT 6989586621679009845 7061644215716937782 0.145824 2.18015 -0.265128 0.000747624 0.00244069 -0.00381022 0.999989 1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1
```


Questions & Bug reporting
-----

Please use Github issue tracker to report bugs. For other questions please contact [Siddharth Choudhary](mailto:siddharth.choudhary@gatech.edu).

Acknowledgements
----
This work was partially funded by the ARL MAST CTA
Project 1436607 “Autonomous Multifunctional Mobile Microsystems”.

Citing
-----

If you use this work, please cite any of the following publications:

```
@inproceedings{Choudhary16icra,
  author    = {Siddharth Choudhary and
               Luca Carlone and
	       Carlos Nieto and
	       John Rogers and
               Henrik I. Christensen and
               Frank Dellaert},
  title     = {Distributed Trajectory Estimation with Privacy and Communication Constraints: 
a Two-Stage Distributed Gauss-Seidel Approach},
  booktitle = {IEEE International Conference on Robotics and Automation 2016},
  year      = {2016}
}
```
```
@article{Choudhary17arXiv,
  author    = {Siddharth Choudhary and
               Luca Carlone and
               Carlos Nieto{-}Granda and
               John G. Rogers III and
               Henrik I. Christensen and
               Frank Dellaert},
  title     = {Distributed Mapping with Privacy and Communication Constraints: Lightweight
               Algorithms and Object-based Models},
  journal   = {CoRR},
  volume    = {abs/1702.03435},
  year      = {2017},
  url       = {http://arxiv.org/abs/1702.03435},
}
```


License
-----
Distributed-Mapper is released under the BSD license, reproduced in the file LICENSE in this directory.

<br><br>
![IRIM](imgs/irim.jpg)
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
![LIDS](imgs/mit_logo.png?raw=true "LIDS")
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
![ARL](imgs/ARL_logo.png?raw=true "ARL")
