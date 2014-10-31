inline pcl::Normal projectOn(Eigen::Vector3f planeNormal, const pcl::Normal& vector) {
  float x, y, z;
  x = vector.normal_x * pow(planeNormal(0), 2);
  y = vector.normal_y * pow(planeNormal(1), 2);
  z = vector.normal_z * pow(planeNormal(2), 2);

  return pcl::Normal(vector.normal_x - x, vector.normal_y - y, vector.normal_z - z);
}

inline float getAngle(const pcl::Normal& vector) {
  double angle = atan2(vector.normal_x, vector.normal_z);
  if(angle < 0) angle += M_PI * 2;
  angle -= (M_PI * 0.5);
  return isnan(angle) ? 0 : angle;
}

template <typename PointT>
inline RegionNormalizer<PointT>::RegionNormalizer(unsigned int _id, Eigen::Vector3f _center, typename pcl::PointCloud<PointT>::ConstPtr _cloud, float _radius): 
  id(_id), inCloud(_cloud), center(_center), cubeRadius(_radius), normalsComputed(false), cloudNormals(new pcl::PointCloud<pcl::Normal>) {}

template <typename PointT>
inline std::string RegionNormalizer<PointT>::getZoneId() const {
  std::ostringstream oss;
  oss << "normal zone cloud no " << id;
  return oss.str();
}

template <typename PointT>
inline std::string RegionNormalizer<PointT>::getNormalsId() const {
  std::ostringstream oss;
  oss << "Normals " << id;
  return oss.str();
}

template <typename PointT>
inline void RegionNormalizer<PointT>::setRadius(float _radius) {
  cubeRadius = _radius;
}

template <typename PointT>
inline typename pcl::PointCloud<PointT>::Ptr RegionNormalizer<PointT>::rangeFilteredCloud() {
  typename PointCloudT::Ptr xFilteredCloud(new PointCloudT);
  pcl::PassThrough<PointT> xFilter;
  xFilter.setInputCloud (inCloud);
  xFilter.setFilterFieldName ("x");
  xFilter.setFilterLimits (center(0) - 2*cubeRadius, center(0) + 2*cubeRadius);
  xFilter.filter(*xFilteredCloud);
  
  typename PointCloudT::ConstPtr const_cloud2(xFilteredCloud);
  typename PointCloudT::Ptr zFilteredCloud(new PointCloudT);
  pcl::PassThrough<PointT> zFilter;
  zFilter.setInputCloud (const_cloud2);
  zFilter.setFilterFieldName ("z");
  zFilter.setFilterLimits (center(2) - 2*cubeRadius, center(2) + 2*cubeRadius);
  zFilter.filter(*zFilteredCloud);

  typename PointCloudT::ConstPtr const_cloud3(zFilteredCloud);
  typename PointCloudT::Ptr yFilteredCloud(new PointCloudT);
  pcl::PassThrough<PointT> yFilter;
  yFilter.setInputCloud (const_cloud3);
  yFilter.setFilterFieldName ("y");
  yFilter.setFilterLimits (center(1) - cubeRadius, center(1) + cubeRadius);
  yFilter.filter(*yFilteredCloud);

  return yFilteredCloud;
}

template <typename PointT>
inline void RegionNormalizer<PointT>::drawNormalToViewer(pcl::visualization::PCLVisualizer& viewer, const pcl::Normal& normal) const {
  std::string zoneId(getZoneId());
  std::string normalsId(getNormalsId());

  typename PointCloudT::Ptr singleCloud(new PointCloudT);
  singleCloud->push_back(cubeCloud->at(cubeCloud->size() / 2));

  pcl::PointCloud<pcl::Normal>::Ptr normalCloud(new pcl::PointCloud<pcl::Normal>);
  normalCloud->push_back(normal);

  pcl::visualization::PointCloudColorHandlerCustom<PointT> green_color(cubeCloud, 0, 255, 0);
  viewer.addPointCloudNormals<PointT, pcl::Normal> (singleCloud, normalCloud, 10, 0.5, normalsId);
  viewer.addPointCloud<PointT>(cubeCloud, green_color, zoneId);
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, zoneId);
}

template <typename PointT>
void RegionNormalizer<PointT>::computeNormals() {
  normalsComputed = false;
  cubeCloud = rangeFilteredCloud();

  if(cubeCloud->size() > 100) {
    pcl::NormalEstimationOMP<PointT, pcl::Normal> ne;
    ne.setInputCloud (cubeCloud);

    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
    ne.setSearchMethod (tree);

    ne.setRadiusSearch (0.1);

    ne.compute(*cloudNormals);
    normalsComputed = true;
  }
}

template <typename PointT>
inline pcl::Normal RegionNormalizer<PointT>::computeAverageNormal() {
  float x = 0.0, y = 0.0, z = 0.0;
  if(!normalsComputed) computeNormals();

  size_t nbNormals = cloudNormals->size();
  for(size_t i = 0; i < nbNormals; ++i) {
    x += cloudNormals->at(i).normal_x;
    y += cloudNormals->at(i).normal_y;
    z += cloudNormals->at(i).normal_z;
  }
  x /= (float)nbNormals;
  y /= (float)nbNormals;
  z /= (float)nbNormals;
  
  computedAverageNormal = pcl::Normal(x, y, z);
  return computedAverageNormal;
}

template <typename PointT>
inline float RegionNormalizer<PointT>::computeHistogramPeekAngle(size_t _nbBins) {
  if(!normalsComputed) computeNormals();
  if(!normalsComputed) return 0;

  Histogram<float> histogram(_nbBins, -M_PI, M_PI);

  size_t nbNormals = cloudNormals->size();
  for(size_t i = 0; i < nbNormals; ++i) {
    histogram.push( -getAngle( cloudNormals->at(i) ) );
  }
  
  return histogram.getPeek();
}

template <typename PointT>
inline void RegionNormalizer<PointT>::drawNormalsCloudToViewer(pcl::visualization::PCLVisualizer& viewer) const {
  if(normalsComputed) {
    std::string zoneId(getZoneId());
    std::string normalsId(getNormalsId());
    pcl::visualization::PointCloudColorHandlerCustom<PointT> green_color(cubeCloud, 0, 255, 0);
    viewer.addPointCloudNormals<PointT, pcl::Normal> (cubeCloud, cloudNormals, 10, 0.5, normalsId);
    viewer.addPointCloud<PointT>(cubeCloud, green_color, zoneId);
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, zoneId);
  }
}

template <typename PointT>
inline void RegionNormalizer<PointT>::drawAverageNormalToViewer(pcl::visualization::PCLVisualizer& viewer) {
  if(normalsComputed) {
    drawNormalToViewer(viewer, computedAverageNormal);
  }
}

template <typename PointT>
inline void RegionNormalizer<PointT>::drawProjectedAverageNormalToViewer(pcl::visualization::PCLVisualizer& viewer) {
  if(normalsComputed) {
    pcl::Normal normal(projectOn(Eigen::Vector3f(0.0, 1.0, 0.0), computedAverageNormal));
    drawNormalToViewer(viewer, normal);
  }
}