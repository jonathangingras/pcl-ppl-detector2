#ifndef __C4_BRIDGE__
#define __C4_BRIDGE__

#include <time.h>

#include <sstream>
#include <string>

#include <pcl/visualization/pcl_visualizer.h> 
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>

#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl/io/pcd_io.h>

#include <boost/thread.hpp>

#include <ground_based_detector/human.h>
#include <ground_based_detector/humanArray.h>

#include "Pixel.h"

#define HUMAN_height 108
#define HUMAN_width 36
#define HUMAN_xdiv 9
#define HUMAN_ydiv 4

class UpperBodyDetector {
private:
    typedef boost::shared_ptr<cv::Mat> cvMatPtr;
    cv::Mat croppedMatrix;

    cv::CascadeClassifier upperBodyClassifier;
    cvMatPtr matrix;
    boost::mutex& image_mutex;

    std::vector<cv::Rect> results;

    inline void preventBordersOverflow(cv::Point& topLeft, cv::Point& bottomRight) const;

    inline cv::Rect getUpperBodyRoi(ground_based_detector::human&) const;

public:
	inline UpperBodyDetector(cvMatPtr _matrix, std::string classifierFilePath, boost::mutex& _image_mutex) : matrix(_matrix), image_mutex(_image_mutex) {
        //cv::namedWindow("openCV Haar-Cascade Upper Body Classifier results");

        upperBodyClassifier.load(classifierFilePath);
    }

	virtual inline ~UpperBodyDetector() {}

	bool isHuman(ground_based_detector::human&);

};

#include "impl/UpperBodyDetector.hpp"

#endif /*HEADER GUARD*/