PCL People Detector 2

This directory contains nodes required to run the people detector.

Currently, the flow works like this:
 
ground_based_detector ----> upperbody_filter ----> human_markers_publisher

1. ground_based_detector:
	Basically, pcl people detection by M. Munaro, F. Basso and E. Menegatti
	in “Tracking  people  within  groups  with  RGB­D  data” (2012).
	The node publishes humanArray messages.
	
	***N.-B.: the node begins by estimating the floor plane coefficients, it will crash
	if it cannot when launched (e.g. if one stands in front of the camera (occlusing the floor cloud)
	while launching)
	Also, you may have to adjust y axis filtering limits for this estimation in the launch file,
	depending on the real height of the camera comparating to the ground.
	At this point, coordinate system is optical : z point front, x pointing right, y pointing to the 
	ground i.e.
													  ___				x
													 |Z. |____________\
													 |___|------------/  
													  ||
													  ||
												Y	  ||
													  ||
													 \  /
													  \/


2. upperbody_filter:
	This node wraps an OpenCV Haar Cascade Classifier trained for Upper Body detection.
	The node basically subscribe to humanArray from ground_based_detector and the 
	camears's rgb image message in order to filter false positives by removing human
	messages which don't present an upperbody.
	Steps are:
	1. receive humanArray;
	2. for each human in humanArray :
		-	crop RGB Image corresponding to human region;
		- detect upper bodies in it;
		- check if human theorical top (from PCL) is contained in a detected upper body rectangle;
		- if so publish the human message;
		- else don't

	***N.-B.: the opencv version is a custom build, because IT DOES require Intel TBB for
	computation. You can find a simple shell scipt to show how it was compiled.
	It was still linked with catkin using the CMakeLists.txt available in the upperbody_filter
	directory. Also note that CMakeLists.txt must be edited if you don't install the custom built
	OpenCV in /usr/local

3. human_markers_publisher:
	This node subscibe to humanArray message from upperbody_filter and publish cylinder
	markers for visualization (e.g. in rviz).

4. pcl_ppl_detector2_launcher:
	This package is only used to launch the whole system.

5. ground_estimator is work in progress, but is not used.

6. upperbody_detector is not used either. It was a try to detect all upperbodies w/o cropping
	per human. This is not quick enough, even with TBB. So it could be removed.

7. region_normalizer is not used, it's a work in progress to estimate a torso orientation.
	The implementaion of RegionNormalizer class has worked before, but to use it, a ROS Node that
	subscribes to humanArray messages must be written.

LAUNCHING THE SYSTEM:

to launch the whole system:
$ roslaunch pcl_ppl_detector2_launcher pcl_ppl_detector2.launch


EXTERNAL (NON-ROS) REQUIREMENTS:

	- OpenCV built with Intel TBB support.
	- PCL "people"


ACKNOWLEDGMENT

The packages where built and run under ROS Groovy, but nothing in my knowledge lets me think
it couldn't be run under ROS Hydro or later.