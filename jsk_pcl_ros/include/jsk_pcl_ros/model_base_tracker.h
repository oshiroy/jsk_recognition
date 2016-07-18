// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, JSK Lab
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


#ifndef JSK_PCL_ROS_MODEL_BASE_TRACKER_H_
#define JSK_PCL_ROS_MODEL_BASE_TRACKER_H_

#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <boost/random.hpp>
#include <dynamic_reconfigure/server.h>
#include "jsk_pcl_ros/tf_listener_singleton.h"
#include "jsk_recognition_utils/geo_util.h"
#include "jsk_recognition_utils/pcl_conversion_util.h"

#include <jsk_pcl_ros/ModelBaseTrackerConfig.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_msgs/ModelCoefficients.h>
#include <geometry_msgs/PolygonStamped.h>
#include <std_srvs/Empty.h>

#include <pcl/tracking/tracking.h>
#include "jsk_pcl_ros/pcl/simple_particle_filter.h"

namespace jsk_pcl_ros
{
  template <class Config>
  double closestPointLikelihood(const pcl::tracking::ParticleXYZRPY& p,
                                pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                                pcl::PointCloud<pcl::PointXYZ>::ConstPtr ref_cloud,
                                const Config& config)
  {
    return 1.0;
  }

  template <class Config>
  double distanceFromPlaneLikelihood(const pcl::tracking::ParticleXYZRPY& p,
                                     const pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                                     const pcl::PointCloud<pcl::PointXYZ>::ConstPtr ref_cloud,
                                     const geometry_msgs::PolygonStamped polygon,
                                     const Config& config)
  {
    if (config.use_distance_from_plane_likelihood) {
      return 1.0;
    }
    else {
      return 1.0;
    }
  }

  template <class Config>
  double computeLikelihood(const pcl::tracking::ParticleXYZRPY& p,
                           pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                           const pcl::PointCloud<pcl::PointXYZ>::ConstPtr ref_cloud,
                           const geometry_msgs::PolygonStamped polygon,
                           const Eigen::Vector3f& viewpoint,
                           const Config& config)
  {
    return (closestPointLikelihood(p,cloud,ref_cloud,config)
            * distanceFromGroundLikelihood(p, config);
  }


  class ModelBaseTracker: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    typedef pcl::tracking::ParticleXYZRPY Particle;
    typedef pcl::PointCloud<Particle> ParticleCloud;
    typedef boost::shared_ptr<ModelBaseTracker> Ptr;
    typedef ModelBaseTrackerConfig Config;
    typedef message_filters::sync_policies::ExactTime<
      geometry_msgs::PolygonStamped,
      pcl_msgs::ModelCoefficients> PolygonSyncPolicy;

    ModelBaseTracker(): DiagnosticNodelet("ModelBaseTracker") {}

  protected:
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void computeTracking();
    virtual void cloudCallback(
      const sensor_msgs::PointCloud2::ConstPtr& msg);
    virtual void renewInteractiveModel(
      const visualization_msgs::Marker &marker);
    virtual void updateInteractiveModelRegionGrowing();
    virtual void updateModelWeight();
    virtual void checkSanityModelRestraint();
    virtual void likelihood();
    virtual bool resetCallback(std_srvs::EmptyRequest& req,
                               std_srvs::EmptyResponse& res);
    virtual void ModelBaseTracker::polygonCallback(
      const geometry_msgs::PolygonStamped::ConstPtr& polygon_msg,
      const pcl_msgs::ModelCoefficients:ConstPtr& coef_msg);
    virtual void configCallback();

    boost::mutex mutex_;
    ros::Subscriber sub_cloud_;
    ros::Publisher pub_result_;
    ros::Publisher pub_particles_;
    ros::Publisher pub_result_pose_;
    ros::ServiceServer srv_reset_;
    message_filters::Subscriber<geometry_msgs::PolygonStamped> sub_polygon_;
    message_filters::Subscriber<pcl_msgs::ModelCoefficients> sub_coefficients_;
    boost::shared_ptr<message_filters::Synchronizer<PolygonSyncPolicy> > sync_polygon_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr candidate_cloud_;

    geometry_msgs::PolygonStamped::ConstPtr latest_polygon_msg_;
    pcl_msgs::ModelCoefficients::ConstPtr latest_coefficients_msg_;
    boost::shared_ptr<dynamic_reconfigure::Server<Config> > srv_;
    Config config_;

    double init_dx_mean_;
    double init_dx_variance_;
    double init_dy_mean_;
    double init_dy_variance_;
    double init_dz_mean_;
    double init_dz_variance_;

    double step_x_variance_;
    double step_y_variance_;
    double step_z_variance_;
    double step_roll_variance_;
    double step_pitch_variance_;
    double step_yaw_variance_;
    double step_dx_variance_;
    double step_dy_variance_;
    double step_dz_variance_;

    int particle_num_;
    std::string sensor_frame_;
    bool interactive_mode_;
    bool ref_model_set_flg_;
    tf::TransformListener* tf_;
    Eigen::Vector3f viewpoint_;
    boost::mt19937 random_generator_;
    pcl::tracking::ROSCollaborativeParticleFilterTracker<pcl::PointXYZ,
                                                         pcl::tracking::ParticleXYZRPY>::Ptr tracker_;
    pcl::PointCloud<PointXYZ> reference_cloud_;
  private:
  };
}

#endif
