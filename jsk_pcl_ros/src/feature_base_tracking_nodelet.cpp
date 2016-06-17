// -*- mode: C++ -*-

#define BOOST_PARAMETER_MAX_ARITY 7
#include "jsk_pcl_ros/feature_base_tracking.h"
#include <pcl/gpu/features/features.hpp>


namespace jsk_pcl_ros
{
  void FeatureBaseTracking::onInit()
  {
   DiagnosticNodelet::onInit();

  }

  void FeatureBaseTracking::renewModel()
  {
    boost::mutex::scoped_lock lock(mutex_);

  }

  void FeatureBaseTracking::cloudCallback()
  {
    boost::mutex::scoped_lock lock(mutex_);

  }

  void FeatureBaseTracking::extractFeatures(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud)
  {

  }

  void FeatureBaseTracking::subscribe()
  {
    sub_points_ = pnh_->subscribe("input/points", 1, &FeatureBaseTracking::cloudCallback, this);
    sub_box_ = pnh_->subscribe("input/box", 1, &FeatureBaseTracking::renewModel, this);
  }

  void FeatureBaseTracking::unsubscribe()
  {
    sub_points_->shutdown();
  }
}


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::FeatureBaseTracking, nodelet::Nodelet);
