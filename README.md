# GraphGNSSLib - Converter
### An Open-source Package for GNSS preprocessing

This package contains a simple RINEX-to-Rosbag converter written in C++. It decodes observations from a multi-constellation GNSS receiver and augments them with ephemeris, atmospheric corrections and the individual satellite clock bias.

Please not that this repository holds just a stripped-down version [GraphGNSSLib](https://github.com/weisongwen/GraphGNSSLib), which provides a full factor graph back-end. We created this fork for a simpler maintainability of the GNSS preprocessor.



## 1. Install Dependencies
todo...

## 2. Build GraphGNSSLib
todo...

## 3. Run GNSS preprocessor with your own RINEX file

todo...

## 5. Acknowledgments

Since this package is just a stripped-down version of the GraphGNSSLib from [Weisong Wen](https://weisongwen.wixsite.com/weisongwen), all credits for creating this helpful converter belong to him.
The [RTKLIB](http://www.rtklib.com/) is used internally for GNSS data decoding and atmospheric corrections.

## 6. License
The source code is released under [GPLv3](http://www.gnu.org/licenses/) license. 