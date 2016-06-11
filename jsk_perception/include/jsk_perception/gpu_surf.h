// -*- mode: c++ -*-

#ifndef JSK_PERCEPTION_GPU_SURF_H_
#define JSK_PERCEPTION_GPU_SURF_H_

#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <ros/node_handle.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <cv_bridge/cv_bridge.h>

namespace jsk_perception
{
  class GpuSURF: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    GpuSURF(): DiagnosticNodelet("GpuSURF") {}
  protected:
    ros::Publisher img_pub_;
    image_transport::Subscriber img_sub_;
    boost::shared_ptr<image_transport::ImageTransport> it_;
    boost::mutex mutex_;
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    virtual void updateDiagnostic(
      diagnostic_updater::DiagnosticStatusWrapper &stat);
  private:
  };
}

#endif
