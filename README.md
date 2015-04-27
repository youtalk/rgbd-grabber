rgbd-grabber
============

RGB-D camera grabber with OpenCV and PCL currently for the following devices.

* UVC
* [Creative Senz3D](http://us.creative.com/p/web-cameras/creative-senz3d) / [SoftKinetic DS325](http://www.softkinetic.com/Store/tabid/579/ProductID/6/language/en-US/Default.aspx)
* [pmd CamBoard nano](http://www.pmdtec.com/products_services/reference_design.php)
* [iDS uEye LE](http://jp.ids-imaging.com/store/produkte/kameras/usb-2-0-kameras/ueye-le.html)

Installation
------------

Dependencies:
* [Boost](https://github.com/boostorg/boost) 1.46 or newer
* [OpenCV](https://github.com/Itseez/opencv) 2.3 or newer
* [PCL](https://github.com/PointCloudLibrary/pcl) 1.7 or newer
* [gflags](https://github.com/gflags/gflags) 2.1 or newer

~~~ sh
$ git clone https://github.com/gflags/gflags
$ cd gflags
$ cmake .
$ make
$ sudo make install
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt-get update
$ sudo apt-get install libboost-all-dev libopencv-dev libpcl-1.7-all
~~~

### UVC
~~~ sh
$ cmake .
$ make
$ bin/UVCameraCapture --id=0
~~~

### Creative Senz3D / SoftKinetic DS325
~~~ sh
$ cmake -DUSE_DS=ON .
$ make
$ bin/DS325Capture --id=0
~~~

### pmd CamBoard nano
~~~ sh
$ cmake -DUSE_PMD=ON .
$ make
$ bin/PMDNanoCapture --pap=/path/to/camboardnano.L64.pap --ppp=/path/to/camboardnanoproc.L64.ppp
~~~

### iDS uEye LE
See also the examples of configuration file `data/ueye-conf.ini` and calibration data `data/ueye-calib.xml` for an uEye camera.

**Single camera**
~~~ sh
$ cmake -DUSE_UEYE=ON .
$ make
$ bin/UEyeCalibration --id=0 --conf=/path/to/conf.ini --intrinsics=/path/to/calib.xml
# Enter 'c' to capture an image.
# Enter 'q' to quit capture and start calibration.
$ bin/UEyeCapture --id=0 --conf=/path/to/conf.ini --intrinsics=/path/to/calib.xml
~~~

**Stereo camera**
~~~ sh
$ cmake -DUSE_UEYE=ON .
$ make
$ bin/StereoUEyeGetCalibrationData --left_id=0 --right_id=1 --left_conf=/path/to/conf.ini --right_conf=/path/to/conf.ini
$ bin/StereoCameraCalibration --intrinsics=/path/to/intrinsics.xml --extrinsics=/path/to/extrinsics.xml
$ bin/StereoUEyeCapture --left_id=0 --right_id=1 --left_conf=/path/to/conf.ini --right_conf=/path/to/conf.ini --intrinsics=/path/to/intrinsics.xml --extrinsics=/path/to/extrinsics.xml
~~~
