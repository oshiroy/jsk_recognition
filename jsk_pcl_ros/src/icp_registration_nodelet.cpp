// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include "jsk_pcl_ros/icp_registration.h"
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include "jsk_pcl_ros/pcl_conversion_util.h"
#include <eigen_conversions/eigen_msg.h>
#include <pcl/common/transforms.h>
#include <eigen_conversions/eigen_msg.h>
#include "jsk_pcl_ros/transform_pointcloud_in_bounding_box.h"

namespace jsk_pcl_ros
{
  void ICPRegistration::onInit()
  {
    PCLNodelet::onInit();
    tf_listener_.reset(new tf::TransformListener());
    ////////////////////////////////////////////////////////
    // Dynamic Reconfigure
    ////////////////////////////////////////////////////////
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (
        &ICPRegistration::configCallback, this, _1, _2);
    srv_->setCallback (f);

    bool align_box;
    pnh_->param("align_box", align_box, false);
    
    pub_result_pose_ = pnh_->advertise<geometry_msgs::PoseStamped>(
      "output_pose", 1);
    pub_result_cloud_ = pnh_->advertise<sensor_msgs::PointCloud2>(
      "output", 1);
    pub_debug_source_cloud_ = pnh_->advertise<sensor_msgs::PointCloud2>(
      "debug/source", 1);
    pub_debug_target_cloud_ = pnh_->advertise<sensor_msgs::PointCloud2>(
      "debug/target", 1);
    pub_debug_flipped_cloud_ = pnh_->advertise<sensor_msgs::PointCloud2>(
      "debug/flipped", 1);
    pub_debug_result_cloud_ = pnh_->advertise<sensor_msgs::PointCloud2>(
      "debug/result", 1);

    ////////////////////////////////////////////////////////
    // subscription
    ////////////////////////////////////////////////////////
    sub_reference_ = pnh_->subscribe("input_reference", 1,
                                       &ICPRegistration::referenceCallback,
                                       this);
    if (align_box) {
      sub_input_.subscribe(*pnh_, "input", 1);
      sub_box_.subscribe(*pnh_, "input_box", 1);
      sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
      sync_->connectInput(sub_input_, sub_box_);
      sync_->registerCallback(boost::bind(
                                &ICPRegistration::alignWithBox,
                                this, _1, _2));
    }
    else {
      sub_ = pnh_->subscribe("input", 1,
                             &ICPRegistration::align,
                             this);
    }
  }

  void ICPRegistration::publishDebugCloud(
      ros::Publisher& pub,
      const pcl::PointCloud<PointT>& cloud)
  {
    sensor_msgs::PointCloud2 ros_cloud;
    pcl::toROSMsg(cloud, ros_cloud);
    ros_cloud.header.frame_id = "base_link";
    ros_cloud.header.stamp = ros::Time::now();
    pub.publish(ros_cloud);
  }
  
  void ICPRegistration::configCallback(Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    algorithm_ = config.algorithm;
    use_flipped_initial_pose_ = config.use_flipped_initial_pose;
    max_iteration_ = config.max_iteration;
    correspondence_distance_ = config.correspondence_distance;
    transform_epsilon_ = config.transform_epsilon;
    euclidean_fittness_epsilon_ = config.euclidean_fittness_epsilon;
    rotation_epsilon_ = config.rotation_epsilon;
    maximum_optimizer_iterations_ = config.maximum_optimizer_iterations;
  }

  void ICPRegistration::alignWithBox(
      const sensor_msgs::PointCloud2::ConstPtr& msg,
      const BoundingBox::ConstPtr& box_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    if (!reference_cloud_) {
      NODELET_FATAL("no reference is specified");
      return;
    }
    try
    {
      Eigen::Affine3f offset;
      pcl::PointCloud<PointT>::Ptr output (new pcl::PointCloud<PointT>);
      transformPointcloudInBoundingBox<PointT>(
        *box_msg, *msg,
        *output, offset,
        *tf_listener_);
      Eigen::Affine3f inversed_offset = offset.inverse();
      alignPointcloud(output, inversed_offset, msg->header);
    }
    catch (tf2::ConnectivityException &e)
    {
      NODELET_ERROR("Transform error: %s", e.what());
    }
    catch (tf2::InvalidArgumentException &e)
    {
      NODELET_ERROR("Transform error: %s", e.what());
    }

    
  }

  void ICPRegistration::align(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    if (!reference_cloud_) {
      NODELET_FATAL("no reference is specified");
      return;
    }
    
    pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*msg, *cloud);
    Eigen::Affine3f offset = Eigen::Affine3f::Identity();
    alignPointcloud(cloud, offset, msg->header);
  }

  double ICPRegistration::alignPointcloud(
    pcl::PointCloud<PointT>::Ptr& cloud,
    const Eigen::Affine3f& offset,
    pcl::PointCloud<PointT>::Ptr& output_cloud,
    Eigen::Affine3d& output_transform)
  {
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    // icp.setInputSource(cloud);
    // icp.setInputTarget(reference_cloud_);
    if (algorithm_ == 1) {
      pcl::GeneralizedIterativeClosestPoint<PointT, PointT> gicp;
      gicp.setRotationEpsilon(rotation_epsilon_);
      gicp.setCorrespondenceRandomness(correspondence_randomness_);
      gicp.setMaximumOptimizerIterations(maximum_optimizer_iterations_);
      icp = gicp;
    }
    icp.setInputSource(reference_cloud_);
    icp.setInputTarget(cloud);
    icp.setMaxCorrespondenceDistance (correspondence_distance_);
    icp.setMaximumIterations (max_iteration_);
    icp.setTransformationEpsilon (transform_epsilon_);
    icp.setEuclideanFitnessEpsilon (euclidean_fittness_epsilon_);
    pcl::PointCloud<PointT> final;
    icp.align(final);
    pcl::transformPointCloud(final, *output_cloud, offset);
    // NODELET_INFO_STREAM("ICP converged: " << icp.hasConverged());
    // NODELET_INFO_STREAM("ICP score: " << icp.getFitnessScore());
        Eigen::Matrix4f transformation = icp.getFinalTransformation ();
    Eigen::Matrix4d transformation_d;
    convertMatrix4<Eigen::Matrix4f, Eigen::Matrix4d>(
      transformation, transformation_d);
    output_transform = Eigen::Affine3d(transformation_d);
    return icp.getFitnessScore();
  }
  
  void ICPRegistration::alignPointcloud(pcl::PointCloud<PointT>::Ptr& cloud,
                                        const Eigen::Affine3f& offset,
                                        const std_msgs::Header& header) {
    pcl::PointCloud<PointT>::Ptr transformed_cloud
      (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr transformed_cloud_for_debug_result
      (new pcl::PointCloud<PointT>);
    Eigen::Affine3d transform_result;
    double score = alignPointcloud(cloud, offset,
                                   transformed_cloud, transform_result);
    pcl::transformPointCloud(
      *transformed_cloud, *transformed_cloud_for_debug_result,
      offset.inverse());
    NODELET_INFO("score: %f", score);
    if (use_flipped_initial_pose_) {
      pcl::PointCloud<PointT>::Ptr flipped_transformed_cloud
        (new pcl::PointCloud<PointT>);
    Eigen::Affine3d flipped_transform_result;
    Eigen::Affine3f flipped_offset
      = offset * Eigen::AngleAxisf(M_PI, Eigen::Vector3f(0, 0, 1));
      pcl::PointCloud<PointT>::Ptr flipped_cloud (new pcl::PointCloud<PointT>);
      pcl::transformPointCloud(
        *cloud, *flipped_cloud,
        Eigen::Affine3f(Eigen::AngleAxisf(M_PI, Eigen::Vector3f(0, 0, 1))));
      publishDebugCloud(pub_debug_flipped_cloud_, *flipped_cloud);
      double flipped_score
        = alignPointcloud(flipped_cloud, flipped_offset,
                          flipped_transformed_cloud,
                          flipped_transform_result);
      NODELET_INFO("flipped score: %f", flipped_score);
      if (flipped_score < score) {
        transformed_cloud = flipped_transformed_cloud;
        transform_result = flipped_transform_result;
        pcl::transformPointCloud(
          *transformed_cloud, *transformed_cloud_for_debug_result,
          flipped_offset.inverse());
      }
    }
    sensor_msgs::PointCloud2 ros_final;
    pcl::toROSMsg(*transformed_cloud, ros_final);
    ros_final.header = header;
    pub_result_cloud_.publish(ros_final);
    
    publishDebugCloud(pub_debug_source_cloud_, *reference_cloud_);
    publishDebugCloud(pub_debug_target_cloud_, *cloud);
    publishDebugCloud(pub_debug_result_cloud_, *transformed_cloud_for_debug_result);
    
  }
  
  void ICPRegistration::referenceCallback(
    const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*msg, *cloud);
    {
      boost::mutex::scoped_lock lock(mutex_);
      reference_cloud_ = cloud;
    }
  }
  
}

#include <pluginlib/class_list_macros.h>
typedef jsk_pcl_ros::ICPRegistration ICPRegistration;
PLUGINLIB_DECLARE_CLASS (jsk_pcl, ICPRegistration, ICPRegistration, nodelet::Nodelet);