// -*- mode: c++ -*-


#ifndef JSK_PCL_ROS_FEATURE_BASE_TRACKING_H_
#define JSK_PCL_ROS_FEATURE_BASE_TRACKING_H_


#include <ros/ros.h>
#include <ros/names.h>


#include "sensor_msgs/PointCloud2.h"
#include <dynamic_reconfigure/server.h>
#include <pcl_ros/pcl_nodelet.h>


#include <jsk_topic_tools/vital_checker.h>
#include "jsk_topic_tools/diagnostic_nodelet.h"

namespace jsk_pcl_ros
{
  class FeatureBaseTracking: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    FeatureBaseTracking(): DiagnosticNodelet("FeatureBaseTracking"){ }

  protected:
    virtual void onInit();
    virtual void renewModel();
    virtual void cloudCallback();
    virtual void computeRegionGrowing();
    virtual void updateModel();
    virtual void estimateHiddenArea();
    virtual void extractFeatures(
     const pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud
     );
    virtual void detection();
    virtual void subscribe();
    virtual void unsubscribe();

    boost::mutex mutex_;


  private:
  };
}

#endif
