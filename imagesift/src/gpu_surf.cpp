#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <jsk_topic_tools/log_utils.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "opencv2/features2d/features2d.hpp"

#if CV_MAJOR_VERSION >= 3
#include "opencv2/cudafeatures2d.hpp"
#include "opencv2/xfeatures2d/cuda.hpp"
#else
#include "opencv2/nonfree/gpu.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#endif

using namespace cv;

namespace enc = sensor_msgs::image_encodings;

namespace imagesift {

  class GpuSURF: public nodelet::Nodelet
  {
  private:
    image_transport::Publisher img_pub_;
    image_transport::Subscriber img_sub_;

    boost::shared_ptr<image_transport::ImageTransport> it_;
    ros::NodeHandle nh_;
    int subscriber_count_;
    bool initialized_;

    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
      try
	{
	  cv::Mat input_img = cv_bridge::toCvShare(msg, msg->encoding)->image;
	  cv::Mat tmp_img = input_img.clone();
	  vector<KeyPoint> keypoints;
	  vector<float> descriptors;

#if CV_MAJOR_VERSION >= 3
	  cuda::SURF_CUDA surf;
	  cuda::GpuMat g_img(input_img);
	  cuda::GpuMat keypointsGPU, descriptorsGPU;
	  surf(g_img, cuda::GpuMat(), keypointsGPU, descriptorsGPU);
#else
	  gpu::SURF_GPU surf;
	  gpu::GpuMat g_img(input_img);
	  gpu::GpuMat keypointsGPU, descriptorsGPU;
	  surf(g_img, gpu::GpuMat(), keypointsGPU, descriptorsGPU);
#endif

	  surf.downloadKeypoints(keypointsGPU, keypoints);
	  surf.downloadDescriptors(descriptorsGPU, descriptors);

	  for (size_t i = 0 ; i < keypoints.size() ; i++)
	    {
	      cv::Point pt;
	      pt = keypoints[i].pt;
	      cv::circle(input_img, pt, 2 ,cv::Scalar(255,0,0), 1);
	    }
	  // Publish the image.
	  if (initialized_) {
	    sensor_msgs::Image::Ptr output_msg = cv_bridge::CvImage(msg->header, enc::MONO8, input_img).toImageMsg();
	    img_pub_.publish(output_msg);
	  }
	}
      catch (cv::Exception &e)
	{
	  JSK_NODELET_ERROR("Image processing error: %s %s %s %i", e.err.c_str(), e.func.c_str(), e.file.c_str(), e.line);
	}
    }

    void subscribe()
    {
      JSK_NODELET_DEBUG("Subscribing to image topic.");
      img_sub_ = it_->subscribe("image", 3, &GpuSURF::imageCallback, this);
    }

    void unsubscribe()
    {
      JSK_NODELET_DEBUG("Unsubscribing from image topic.");
      img_sub_.shutdown();
    }

    void connectCb(const image_transport::SingleSubscriberPublisher& ssp)
    {
        if (subscriber_count_++ == 0) {
            subscribe();
        }
    }

    void disconnectCb(const image_transport::SingleSubscriberPublisher&)
    {
        subscriber_count_--;
        if (subscriber_count_ == 0) {
            unsubscribe();
        }
    }
    
  public:
    void onInit() {
#if CV_MAJOR_VERSION >= 3
      cv::cuda::printShortCudaDeviceInfo(cv::cuda::getDevice());
#else
    int cudaDevices = cv::gpu::getCudaEnabledDeviceCount();
    if (!(cudaDevices > 0)){
      JSK_ROS_ERROR("cannot use gpu");
    }
#endif
      initialized_ = false;
      nh_ = getNodeHandle();
      subscriber_count_ = 0;
      it_.reset(new image_transport::ImageTransport(nh_));
      image_transport::SubscriberStatusCallback connect_cb    = boost::bind(&GpuSURF::connectCb, this, _1);
      image_transport::SubscriberStatusCallback disconnect_cb = boost::bind(&GpuSURF::disconnectCb, this, _1);
      img_pub_ = image_transport::ImageTransport(ros::NodeHandle(nh_, "gpu_surf")).advertise("image", 1, connect_cb, disconnect_cb);
      initialized_ = true;
    }
  };
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(imagesift::GpuSURF, nodelet::Nodelet);
