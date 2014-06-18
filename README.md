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
* **Boost 1.46** or newer
* **OpenCV 2.3** or newer
* **PCL 1.7** or newer

### UVC
~~~ sh
$ cmake .
$ make
$ bin/UVCameraCapture 0
~~~

### Creative Senz3D / SoftKinetic DS325
See also the example of calibration data `data/ds325-calib-example.yml` for a DS325 camera.
~~~ sh
$ cmake -DUSE_DS=ON .
$ make
$ bin/DS325Capture 0 data/ds325-calib-example.yml
~~~

### pmd CamBoard nano
~~~ sh
$ cmake -DUSE_PMD=ON .
$ make
$ bin/PMDNanoCapture /path/to/camboardnano.L64.pap /path/to/camboardnanoproc.L64.ppp
~~~

### iDS uEye LE
See also the examples of configuration file `data/ueye-conf-example.ini` and calibration data `data/ueye-calib-example.ini` for an uEye camera.
~~~ sh
$ cmake -DUSE_UEYE=ON .
$ make
$ bin/UEyeCalibration 0 /path/to/conf.ini /path/to/calib.xml
$ bin/UEyeCapture 0 /path/to/conf.ini /path/to/calib.xml
~~~
