# GICI-LIB

## GNSS/INS/Camera Integrated Navigation Library

GNSS/INS/Camera Integrated Navigation Library (GICI-LIB) is an open-source software package for Global Navigation Satellite System (GNSS), Inertial Navigation System (INS), and Camera integrated navigation. The features of GICI-LIB are:

a) It is built under the Factor Graph Optimization (FGO) framework. It contains most of the possible GNSS loose and tight integration factors, INS factors, visual factors, and motion constraints, together with reliable initialization, measurement sparsification, and outlier rejection algorithms. The GNSS formulations are implemented towards four constellations and full frequencies.

b) For ease of use, the software is developed under object-oriented programming features, and the graph is designed to enable the flexible addition of sensors. By simple instantiation, one can easily form any kind of multi-sensor fusion algorithm with considerable robustness.

c) It supports multiple algorithms, including GNSS Single Point Positioning (SPP), Real-Time Differential (RTD), Single-Differenced GNSS (SDGNSS), Real-Time Kinematic (RTK), Precise Point Positioning (PPP), SPP-based loosely coupled (LC) and tightly coupled (TC) GNSS/Inertial Navigation System (GINS), SPP-based Solution/Raw/Raw (SRR) and Raw/Raw/Raw (RRR) GNSS/Visual/Inertial Navigation System (GVINS), RTK-based LC GINS, TC GINS, SSR GVINS, and RRR GVINS. Moreover, other integration algorithms can be instantiated by users.

d) It supports multiple I/O ports, including serial, TCP/IP, NTRIP, V4L2, file, and ROS topics.

e) It supports multiple message de/encoders, including RTCM2, RTCM3, Ublox raw, Septentrio raw, Novatel raw, Tersus raw, NMEA, DCB-file, ATX-file for GNSS, image-pack, image-v4l2 for image, and IMU-pack for IMU. 

f) It supports multiple stream and multi-algorithm processing. No maximum quantity is limited. 

**Authors:** Cheng Chi, Xin Zhang, Jiahui Liu, Yulong Sun, Zihao Zhang, and Xingqun Zhan.

**Contact:** chichengcn@sjtu.edu.cn

**Related Papers** 

C. Chi, X. Zhang, J. Liu, Y. Sun, Z. Zhang and X. Zhan, "GICI-LIB: A GNSS/INS/Camera Integrated Navigation Library," in IEEE Robotics and Automation Letters, vol. 8, no. 12, pp. 7970-7977, Dec. 2023, doi: 10.1109/LRA.2023.3324825.

**Dataset** [GICI-dataset](https://github.com/chichengcn/gici-open-dataset.git)

**Documentation** [GICI-manual](./doc/manual.pdf)

## 1. Dependencies

#### 1.1 Ubuntu

We are developing our code on Ubuntu 20.04, and tested on Ubuntu 18.04 and Ubuntu 22.04. We recommend you to use the same or similar environment if you are not familiar with cross-compiling.

#### 1.2 Eigen 3.3 or later. REQUIRED.

Eigen is a C++ template library for linear algebra. You can find the releases on [Eigen][eigen].

[eigen]: https://eigen.tuxfamily.org/index.php?title=Main_Page

#### 1.3 OpenCV 4.2.0 or later. REQUIRED.

OpenCV is a computer vision library. You can find the releases on [OpenCV][opencv].

[opencv]: https://opencv.org/releases/

#### 1.4 Yaml-cpp 0.6.0 or later. REQUIRED.

Yaml-cpp is a decoder and encoder for YAML formats. We use YAML file to configure our workflow. You can find the releases on [yaml-cpp][yaml].

[yaml]: https://github.com/jbeder/yaml-cpp

#### 1.5 Glog 0.6.0 or later. REQUIRED.
Glog is a logging control library. You can find the releases on [Glog][glog_]. You should install Glog together with [Gflags][gflags]. We suggest you install Glog from source code, rather than apt-get. Because installing from apt-get may make GICI fail to find the Glog library during compiling.

[glog_]: https://github.com/google/glog
[gflags]: https://github.com/gflags/gflags

#### 1.6 Ceres-Solver 2.1.0 or later. REQUIRED.

Ceres-Solver is a nonlinear optimization library. You can find the releases on [Ceres-Solver][ceres].

[ceres]: http://ceres-solver.org/

#### 1.7 ROS. OPTIONAL.

ROS is a library for robot applications. We provide a ROS wrapper to enable GICI to handle some ROS messages. If you want to build GICI with ROS, you should install ROS. You can find the instructions on [ROS][ros].

[ros]: http://wiki.ros.org/Documentation

## 2. Build GICI

#### 2.1 Normal Build

```
cd <gici-root-directory>
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j8 
```

Now you can run GICI via 

```
./gici_main <gici-config-file>
```

#### 2.2 Build with ROS

```
cd <gici-root-directory>/ros_wrapper
catkin_make -DCMAKE_BUILD_TYPE=Release
source ./devel/setup.bash
```

Now you can run GICI ROS wrapper via 

```
rosrun gici_ros gici_ros_main <gici-config-file>
```

## 3. Run GICI

We provide various example YAML configuration files for non-ROS and ROS interfaces. See [non-ROS](./option) and [ROS](./ros_wrapper/src/gici/option). You can modify them according to your requirements to enable real-time estimation, pseudo-real-time, data transfer, and data storage. See our [documentation](./doc/manual.pdf) for details. Here we only illustrate how to run GICI with our datasets.

You can download our datasets on [GICI-dataset](https://github.com/chichengcn/gici-open-dataset.git). We provide example YAML configuration files to run different algorithms with our dataset. The usage of these configuration files are illustrated in [GICI-dataset](https://github.com/chichengcn/gici-open-dataset.git). 

We support both ROS and non-ROS visualization.

The following video is an example of ROS visualization.

<a href="https://youtu.be/dAczU-7r85U" target="_blank"><img src="https://github.com/chichengcn/gici-open-dataset/blob/master/figures/run/4.1_ros.png" 
alt="ros" width="500" height="290" border="10" /></a>

The following video is an example of non-ROS visualization using [RTKLIB](https://rtklib.com/).

<a href="https://youtu.be/8TP0We9lOEQ" target="_blank"><img src="https://github.com/chichengcn/gici-open-dataset/blob/master/figures/run/4.1_non_ros.png" 
alt="non_ros" width="500" height="425" border="10" /></a>

## 4. Acknowledgment

Many of the GNSS tools, I/O handlers, and message de/encoders are inherited from [RTKLIB](https://rtklib.com/). The basic FGO management and the visual and IMU factors are partly inherited from [OKVIS](https://github.com/ethz-asl/okvis). The feature handler is partly inherited from [SVO 2.0](https://github.com/uzh-rpg/rpg_svo_pro_open).

## 5. License

The GICI-LIB software package is distributed under [GPL v3](https://www.gnu.org/licenses/gpl-3.0.html) license. Users are freedom to modify and distribute the software as they see fit, provided that they adhere to the terms and conditions set forth in the license. This includes the ability to incorporate or use GICI-LIB with other software, whether for non-commercial or commercial purposes. However, any modifications or derivative works must also be distributed under the GPL v3 license, ensuring that the software remains free and accessible to all users.

