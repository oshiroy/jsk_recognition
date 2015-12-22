// -*- mode: C++ -*-
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

#ifndef JSK_PCL_ROS_ORGANIZED_EUCLIDEAN_CLUSTERING_NODELET_H_
#define JSK_PCL_ROS_ORGANIZED_EUCLIDEAN_CLUSTERING_NODELET_H_

#include "jsk_recognition_msgs/ClusterPointIndices.h"
#include "jsk_recognition_msgs/Int32Stamped.h"

#include "jsk_recognition_utils/pcl_util.h"
#include "jsk_recognition_utils/pcl_conversion_util.h"
#include <jsk_topic_tools/vital_checker.h>

#include "jsk_topic_tools/diagnostic_nodelet.h"
#include "jsk_recognition_utils/pcl_conversion_util.h"

#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <dynamic_reconfigure/server.h>
#include "jsk_pcl_ros/OrganizedEuclideanClusteringConfig.h"

namespace jsk_pcl_ros
{
  class OrganizedEuclideanClustering: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    typedef message_filters::sync_policies::ExactTime<
      jsk_recognition_msgs::ClusterPointIndices,
      sensor_msgs::PointCloud2 > SyncPolicy;
    typedef jsk_pcl_ros::OrganizedEuclideanClusteringConfig Config;

    OrganizedEuclideanClustering(): DiagnosticNodelet("OrganizedEuclideanClustering") { }

  protected:
    ////////////////////////////////////////////////////////
    // methods
    ////////////////////////////////////////////////////////
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void extract(
      const jsk_recognition_msgs::ClusterPointIndices::ConstPtr cluster_msg,
      const sensor_msgs::PointCloud2ConstPtr& input);
    virtual void extract(
      const sensor_msgs::PointCloud2ConstPtr& input);
    virtual void configCallback(Config& config, uint32_t level);
    ////////////////////////////////////////////////////////
    // ros variables
    ////////////////////////////////////////////////////////
    ros::Subscriber sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_cloud_;
    message_filters::Subscriber<jsk_recognition_msgs::ClusterPointIndices> sub_cluster_indices_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> >sync_;
    boost::shared_ptr <dynamic_reconfigure::Server<Config> > srv_;
    ros::Publisher pub_cluster_indices_;
    ros::Publisher cluster_num_pub_;
    boost::mutex mutex_;
    ////////////////////////////////////////////////////////
    // parameters
    ////////////////////////////////////////////////////////
    std::vector<pcl::PointIndices::Ptr> input_cluster_;
    bool use_exclude_indices_;
    bool approximate_sync_;
    double tolerance_;
    double exclude_minsize_;
    int minsize_;
    int maxsize_;

  private:
    
  };
}
#endif
