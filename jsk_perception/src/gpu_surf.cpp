// -*- mode: c++ -*-
#include <vector>

#include <ros/ros.h>
#include <jsk_topic_tools/log_utils.h>
#include "jsk_perception/gpu_surf.h"

#include "opencv2/features2d/features2d.hpp"

#if CV_MAJOR_VERSION >= 3
#include "opencv2/cudafeatures2d.hpp"
#include "opencv2/xfeatures2d/cuda.hpp"
#else
#include "opencv2/nonfree/gpu.hpp"
#endif

using namespace cv;
namespace enc = sensor_msgs::image_encodings;

namespace jsk_perception
{
  void GpuSURF::onInit() {
    DiagnosticNodelet::onInit();
    img_pub_  = advertise<sensor_msgs::Image>(*pnh_, "surf_image", 1);
  }
  
  void GpuSURF::subscribe()
  {
    it_.reset(new image_transport::ImageTransport(*pnh_));
    img_sub_ = it_->subscribe("input", 1, &GpuSURF::imageCallback, this);
  }

  void GpuSURF::unsubscribe()
  {
    img_sub_.shutdown();
  }

  void GpuSURF::imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    JSK_ROS_INFO("callback");
    vital_checker_->poke();
    boost::mutex::scoped_lock lock(mutex_);
    cv::Mat input_img = cv_bridge::toCvShare(msg, msg->encoding)->image;
    cv::Mat tmp_img = input_img.clone();
    std::vector<KeyPoint> keypoints;
    std::vector<float> descriptors;
// #if CV_MAJOR_VERSION >= 3
//     cuda::SURF_CUDA surf;
//     cuda::GpuMat g_img(input_img);
//     cuda::GpuMat keypointsGPU, descriptorsGPU;
//     surf(g_img, cuda::GpuMat(), keypointsGPU, descriptorsGPU);
// #else
//     gpu::SURF_GPU surf;
//     gpu::GpuMat g_img(input_img);
//     gpu::GpuMat keypointsGPU, descriptorsGPU;
//     surf(g_img, gpu::GpuMat(), keypointsGPU, descriptorsGPU);
// #endif
//     surf.downloadKeypoints(keypointsGPU, keypoints);
//     surf.downloadDescriptors(descriptorsGPU, descriptors);
//     for (size_t i = 0 ; i < keypoints.size() ; i++)
//       {
//     cv::Point pt;
//     pt = keypoints[i].pt;
//     cv::circle(input_img, pt, 2 ,cv::Scalar(255,0,0), 1);
//   }
//     sensor_msgs::Image::Ptr output_msg = cv_bridge::CvImage(msg->header, enc::MONO8, input_img).toImageMsg();
//     img_pub_.publish(output_msg);
  }

  void GpuSURF::updateDiagnostic(
    diagnostic_updater::DiagnosticStatusWrapper &stat)
  {
    if (vital_checker_->isAlive()) {
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK,
                   "GpuSURF running");
    }
    else {
      jsk_topic_tools::addDiagnosticErrorSummary(
        "GpuSURF", vital_checker_, stat);
    }
  }

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(jsk_perception::GpuSURF, nodelet::Nodelet);
