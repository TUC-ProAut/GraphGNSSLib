# gnss_converter
### An Open-source ROS Package for GNSS Preprocessing

This ROS package contains a simple RINEX-to-Rosbag converter written in C++. It decodes observations from a multi-constellation GNSS receiver and augments them with ephemeris, atmospheric corrections and the individual satellite clock bias.

The package is used to generate a Rosbag file containing raw GNSS pseudoranges for further research. In addition, it contains a rover position-fix, base position-fix, and doppler velocity data calculated by the RTKLIB. A Matlab script can ultimately be used to convert the Rosbag file to a .mat file.

Please not that this repository holds just a stripped-down version [GraphGNSSLib](https://github.com/weisongwen/GraphGNSSLib), which provides a full factor graph back-end. We created this fork for a simpler maintainability of the GNSS preprocessor.


## 1. Install ROS

Install [ROS](http://wiki.ros.org/ROS/Installation) on a Ubuntu machine. The package is tested under Ubuntu 20.04 and ROS noetic.

## 2. Build gnss_converter

Clone the repository to a desired folder and build the package:
```
mkdir -p gnss_converter/src
cd gnss_converter/src
git clone https://github.com/TUC-ProAut/gnss_converter.git
cd ..
catkin_make
```
Add the setup.bash to your ~.bashrc:
```
source ~/gnss_converter/devel/setup.bash
```

## 3. Configuration

Yaml files are used for the configuration of the gnss preprocessor. Many preconfigured examples are given, but new configurations can also be created.

### 3.1 Preconfigured

Example configurations of many different datasets can be found [here](/gnss_preprocessor/config).

### 3.2 Custom

The following parameters have to be adapted to fit to the considered dataset. They are internally used by the RTKLIB and GraphGNSSLib scripts:

- mode: GNSS positioning mode (0: single, 1: DGPS/DGNSS, 2: kinematic)
- frequency: all used frequencies of the GNSS receiver (1: L1, 2: L1+L2, 3: L1+L2+L5)
- soltype: type of the solution (0: forward, 1: backward, 2: combined)
- satellites: list of all GNSS-systems to be used (GPS, GLONASS, Galileo, BeiDou, QZSS, SBAS)
- shared_ephemeris: use only one RINEX file for the broadcast ephemeris (true/false) --> please see [Getting GNSS relates files](docs/gnss_related_files.md)
- precise_ephemeris: use the precise satellite orbit solution (true/false) --> please see [Getting GNSS relates files](docs/gnss_related_files.md)
- ionex_correction: use a IONEX TEC correction file for ionosphere correction
- custom_atx: use a custom antenna model file --> please see [Getting GNSS relates files](docs/gnss_related_files.md)
- elevationmask: minimal elevation angle of satellites to be used in degrees 

Please see the [documentation of the RTKLIB](http://www.rtklib.com/rtklib_document.htm) for further explanations regarding some parameters.

### 3.3 Launchfiles

ROS launchfiles are used to start the gnss preprocessor. The paths to the config file and all the other GNSS-related files have to be specified here.

Many examples are given inside the [launch](/gnss_preprocessor/launch) folder. 

## 4. Getting GNSS related files

All information about downloading the GNSS-related files is separated to ["Getting GNSS relates files"](docs/gnss_related_files.md).

## 5. Usage

### 5.1 Run GNSS preprocessor with your own RINEX file

Run the desired launchfile: 
```
roslaunch gnss_preprocessor <datasetname>.launch
```

While running you can see the number of satellites used for each system:
```
[ INFO] [1647857601.020952311]: GPS_cnt [1]   9
[ INFO] [1647857601.020979743]: SBS_cnt [2]   0
[ INFO] [1647857601.020996915]: GLO_cnt [4]   7
[ INFO] [1647857601.021008216]: GAL_cnt [8]   0
[ INFO] [1647857601.021029226]: QZS_cnt [16]  0
[ INFO] [1647857601.021037431]: CMP_cnt [32]  0
```

The processed data is saved in a rosbag file inside the folder dataset/processed. 

### 5.2 Messages

Different messages are published while the package is running:

- **gnss_raw** (gnss_msgs::GNSS_Raw_Array): custom message containing the raw GNSS information (pseudoranges and additional information regarding the satellites)
- **gnss_fix** (sensor_msgs::NavSatFix): receiver position fix calculated by the RTKLIB (reference frame: WGS 84)
- **gnss_vel** (geometry_msgs::TwistStamped): receiver velocity information calculated by the RTKLIB from doppler data
- **gnss_raw_base** (gnss_msgs::GNSS_Raw_Array): base GNSS information (only used with relative positioning modes) 

### 5.3 Conversion to a Matlab-file

The file [convertRosbags.m](/gnss_preprocessor/matlab/) can be used to convert the generated Rosbag file to a .mat file for further evaluation.
Please note that currently only a few specific datasets can be directly used. For own datasets, the filenames have to be manually entered.

## 6. Acknowledgments

Since this package is just a stripped-down version of the GraphGNSSLib from [Weisong Wen](https://weisongwen.wixsite.com/weisongwen), all credits for creating this helpful converter belong to him.
The [RTKLIB](http://www.rtklib.com/) is used internally for GNSS data decoding and atmospheric corrections.

## 7. License
The source code is released under [GPLv3](http://www.gnu.org/licenses/) license. 
