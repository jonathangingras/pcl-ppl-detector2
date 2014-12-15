#ifndef __UPPERBODY_DPM_FILTER_H__
#define __UPPERBODY_DPM_FILTER_H__

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
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl/io/pcd_io.h>

#include <boost/thread.hpp>

#include <ground_based_detector/human.h>
#include <ground_based_detector/humanArray.h>

#include <object_recognition_by_parts/CandidateArray.h>

#include "Pixel.h"

#define HUMAN_height 108
#define HUMAN_width 36
#define HUMAN_xdiv 9
#define HUMAN_ydiv 4

class UpperBodyDPMFilter {
private:
    boost::mutex& dpm_mutex;

    std::vector<cv::Rect> results;

    inline void preventBordersOverflow(cv::Point& topLeft, cv::Point& bottomRight) const;

    inline cv::Rect getUpperBodyRoi(ground_based_detector::human&) const;

public:
    typedef object_recognition_by_parts::CandidateArray CandidateArray;
    typedef object_recognition_by_parts::Candidate PartCenterArray;
    typedef object_recognition_by_parts::PartCenter PartCenter;

	inline UpperBodyDPMFilter(boost::mutex& _dpm_mutex) : dpm_mutex(_dpm_mutex) {}

	bool humanMatchesDPMParts(ground_based_detector::human&, CandidateArray&);
};

#include "impl/UpperBodyDPMFilter.hpp"

#endif /*HEADER GUARD*/
