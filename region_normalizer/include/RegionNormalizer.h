#ifndef __REGION_NORMALIZER_HPP__
#define __REGION_NORMALIZER_HPP__

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

#include "Histogram.h"

//http://pointclouds.org/documentation/tutorials/normal_estimation.php#normal-estimation

inline pcl::Normal projectOn(Eigen::Vector3f planeNormal, const pcl::Normal& vector);
inline float getAngle(const pcl::Normal& vector);

template <typename PointT>
class RegionNormalizer {
protected:
  unsigned int id;
  typedef pcl::PointCloud<PointT> PointCloudT;

  Eigen::Vector3f center;
  float cubeRadius;
  typename PointCloudT::ConstPtr inCloud;
  typename PointCloudT::ConstPtr cubeCloud;
  pcl::PointCloud<pcl::Normal>::Ptr cloudNormals;
  
  bool normalsComputed;
  pcl::Normal computedAverageNormal;

  inline typename PointCloudT::Ptr rangeFilteredCloud();
  inline std::string getZoneId() const;
  inline std::string getNormalsId() const;
  inline void drawNormalToViewer(pcl::visualization::PCLVisualizer& viewer, const pcl::Normal& normal) const;

public:
  typedef boost::shared_ptr<RegionNormalizer> Ptr;
  typedef boost::shared_ptr<const RegionNormalizer> ConstPtr;
  
  virtual ~RegionNormalizer() {}
  inline RegionNormalizer(unsigned int _id, Eigen::Vector3f _center, typename PointCloudT::ConstPtr _cloud, float _radius = 0.04);

  void computeNormals();
  inline void setRadius(float _radius);
  inline pcl::Normal computeAverageNormal();
  inline float computeHistogramPeekAngle(size_t);
  inline void drawNormalsCloudToViewer(pcl::visualization::PCLVisualizer& viewer) const;
  inline void drawAverageNormalToViewer(pcl::visualization::PCLVisualizer& viewer);
  inline void drawProjectedAverageNormalToViewer(pcl::visualization::PCLVisualizer& viewer);

};

#include "impl/RegionNormalizer.hpp"

#endif