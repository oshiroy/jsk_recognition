// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, JSK Lab
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
 *   * Neither the name of the JSK Lab nor the names of its
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
#define BOOST_PARAMETER_MAX_ARITY 7

#include "jsk_pcl_ros/organized_euclidean_clustering.h"
#include <pcl_conversions/pcl_conversions.h>
#include "jsk_recognition_utils/pcl_conversion_util.h"
#include "jsk_pcl_ros/pcl_util.h"
#include <jsk_topic_tools/rosparam_utils.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>

namespace jsk_pcl_ros
{
  void OrganizedEuclideanClustering::onInit()
  {
    DiagnosticNodelet::onInit();
    pnh_->param("use_exclude_indices", use_exclude_indices_, false);
    pub_cluster_indices_ = advertise<jsk_recognition_msgs::ClusterPointIndices> (*pnh_,"output",1);
    cluster_num_pub_ = advertise<jsk_recognition_msgs::Int32Stamped> (*pnh_, "cluster_num", 1);
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&OrganizedEuclideanClustering::configCallback, this, _1, _2);
    srv_->setCallback (f);
  }

  void OrganizedEuclideanClustering::subscribe()
  {
    if(use_exclude_indices_){
      sub_cloud_.subscribe(*pnh_, "input", 1);
      sub_cluster_indices_.subscribe(*pnh_, "input/indices", 1);
      sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
      sync_->connectInput(sub_cluster_indices_, sub_cloud_);
      sync_->registerCallback(boost::bind(&OrganizedEuclideanClustering::extract, this, _1, _2));
    }
    else{
      sub_ = pnh_->subscribe("input", 1, &OrganizedEuclideanClustering::extract, this);
    }
  }

  void OrganizedEuclideanClustering::unsubscribe()
  {
    if(use_exclude_indices_){
      sub_cloud_.unsubscribe();
      sub_cluster_indices_.unsubscribe();
    }
    else{
      sub_.shutdown();
    }
  }

  void OrganizedEuclideanClustering::extract(
    const jsk_recognition_msgs::ClusterPointIndices::ConstPtr cluster_msg,
    const sensor_msgs::PointCloud2ConstPtr& input)
  {
    input_cluster_  = pcl_conversions::convertToPCLPointIndices(cluster_msg->cluster_indices);
    OrganizedEuclideanClustering::extract(input);
  }

  void OrganizedEuclideanClustering::extract(
    const sensor_msgs::PointCloud2ConstPtr& input)
  {
    boost::mutex::scoped_lock lock(mutex_);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input, *cloud);
    pcl::EuclideanClusterComparator<pcl::PointXYZ, pcl::Normal, pcl::Label>::Ptr comparator
      (new pcl::EuclideanClusterComparator<pcl::PointXYZ, pcl::Normal, pcl::Label>);
    comparator->setInputCloud(cloud);
    comparator->setDistanceThreshold(tolerance_,false);
    pcl::PointCloud<pcl::Label>::Ptr labels
      (new pcl::PointCloud<pcl::Label>(cloud->width, cloud->height));
    std::vector<bool> exclude_labels;
    for (size_t i = 0; i < cloud->points.size(); i++){
      pcl::Label t;
      if (pcl_isfinite(cloud->points[i].x) && pcl_isfinite(cloud->points[i].y) &&s pcl_isfinite(cloud->points[i].z))
    	t.label = 0;
      else
    	t.label = 1;
      labels->push_back(t);
    }
    if(use_exclude_indices_){
      exclude_labels.resize(2 + input_cluster_.size());
      for(size_t i = 0 ; i < input_cluster_.size() ; i ++){
    	if(input_cluster_[i]->indices.size() > exclude_minsize_){
    	  for(size_t j = 0; j < input_cluster_[i]->indices.size(); j++){
    	    labels->points[input_cluster_[i]->indices[j]].label = i + 2;
    	  }
    	  exclude_labels[i + 2] = true;
    	}
      }
    }
    else{
      exclude_labels.resize(2);
    }
    exclude_labels[0] = false;
    exclude_labels[1] = true;
    comparator->setLabels(labels);
    comparator->setExcludeLabels(exclude_labels);
    pcl::PointCloud<pcl::Label> euc_labels;
    std::vector<pcl::PointIndices> euc_label_indices;
    pcl::OrganizedConnectedComponentSegmentation<pcl::PointXYZ, pcl::Label> occs(comparator);
    occs.setInputCloud(cloud);
    occs.segment(euc_labels, euc_label_indices);
    std::vector<pcl::PointIndices> cluster_indices;
    for(size_t i = 0; i < euc_label_indices.size(); i++){
      if(euc_label_indices[i].indices.size() > minsize_){
	cluster_indices.push_back(euc_label_indices[i]);
      }
    }
    jsk_recognition_msgs::ClusterPointIndices result;
    result.cluster_indices.resize(cluster_indices.size());
    result.header = input->header;
    for (size_t i = 0; i < cluster_indices.size(); i++) {
      result.cluster_indices[i].header = pcl_conversions::fromPCL(cluster_indices[i].header);
      result.cluster_indices[i].indices = cluster_indices[i].indices;
    }
    pub_cluster_indices_.publish(result);

    jsk_recognition_msgs::Int32Stamped::Ptr cluster_num_msg (new jsk_recognition_msgs::Int32Stamped);
    cluster_num_msg->header = input->header;
    cluster_num_msg->data = cluster_indices.size();
    cluster_num_pub_.publish(cluster_num_msg);
    }

  void OrganizedEuclideanClustering::configCallback (Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    tolerance_ = config.tolerance;
    maxsize_ = config.max_size;
    minsize_ = config.min_size;
    exclude_minsize_ = config.exclude_min_size;
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::OrganizedEuclideanClustering, nodelet::Nodelet);
