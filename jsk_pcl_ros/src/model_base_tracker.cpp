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

#include "jsk_pcl_ros/model_base_tracker.h"
#include "jsk_recognition_utils/pcl_conversion_util.h"
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>

namespace jsk_pcl_ros
{
  void ModelBaseTracker::onInit()
  {
    DiagnosticNodelet::onInit();
    pnh_->param("interactive_mode", interactive_mode_, false);
    ref_model_set_flg_ = false;
    // model should be set if not interactive model
    if (!interactive_mode_)
      {
        std::vector<std::string> model_files;
        if (!jsk_topic_tools::readVectorParameter(*pnh_, "models", model_files)
            || model_files.size() == 0)
          {
            JSK_NODELET_FATAL("no models is specified");
            return;
          }
        // number of models file should be one
        if (model_files.size() != 1)
          {
            JSK_NODELET_FATAL("number of  models is not one");
            return;
          }
        else
          {
            setReferenceCloudModel(model_files[0]);
            ref_model_set_flg_ = true;
          }
      }
    pnh_->param("plane_supported_", plane_supported_, false);
    
    pub_result_ = pnh_->advertise<sensor_msgs::PointCloud2>("output/result", 1);
    pub_result_pose_ = pnh_->advertise<geometry_msgs::PoseStamped>("output/result_pose", 1);
    pub_particles_ = pnh_->advertise<sensor_msgs::PointCloud2>("output/particles", 1);

    sub_polygon_.subscribe(*pnh_, "input/polygon", 10);
    sub_coefficients_.subscribe(*pnh_, "input/coefficients", 10);

    sync_polygon_ = boost::make_shared<message_filters::Synchronizer<PolygonSyncPolicy> >(100);
    sync_polygon_->connectInput(sub_polygon_, sub_coefficients_);
    sync_polygon_->registerCallback(boost::bind(&ModelBaseTracker::polygonCallback, this, _1, _2));

    srv_reset_ = pnh_->advertiseService("reset", &ModelBaseTracker::resetCallback, this);
  }

  void ModelBaseTracker::subscribe()
  {
  }

  void ModelBaseTracker::unsubscribe()
  {
  }

  void ModelBaseTracker::computeTracking(
    const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    try {
    // viewpoint
      tf::StampedTransform transform
        = lookupTransformWithDuration(tf_, sensor_frame_, msg->header.frame_id,
                                      ros::Time(0.0),
                                      ros::Duration(0.0));
      tf::vectorTFToEigen(transform.getOrigin(), viewpoint_);
      if (!tracker_) {
        JSK_NODELET_INFO("initialize Tracker");
        pcl::PointCloud<pcl::tracking::ParticleXYZRPY>::Ptr particles = initParticles();
        tracker_.reset(new pcl::tracking::ROSCollaborativeParticleFilterTracker
                       <pcl::PointXYZ, pcl::tracking::ParticleModel>);
        tracker_->setCustomSampleFunc(boost::bind(&ModelBaseTracker::sample, this, _1));
        tracker_->setLikelihoodFunc(boost::bind(&ModelBaseTracker::likelihood, this, _1, _2));
        tracker_->setParticles(particles);
        tracker_->setParticleNum(particle_num_);
      }
      else {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);
        // TODO extract plane pointcloud when plane_supported_ is true
        tracker_->setInputCloud(cloud);
        candidate_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
        JSK_ROS_INFO("start tracker_->compute()");
        tracker_->compute();
        JSK_ROS_INFO("done tracker_->compute()");
        Particle result = tracker_->getResult();
        geometry_msgs::PoseStamped pose;
        pose.pose.x = result.x;
        pose.pose.y = result.y;
        pose.pose.z = result.z;
        pose.header = msg->header;
        pub_result_pose_.publish(pose);
      }
    }
    ParticleCloud::Ptr particles = tracker_->getParticles();
    // Publish particles
    sensor_msgs::PointCloud2 ros_particles;
    pcl::toROSMsg(*particles, ros_particles);
    ros_particles.header = msg->header;
    pub_particles_.publish(ros_particles);
    }
    catch (tf2::TransformException& e) {
      JSK_ROS_ERROR("tf exception");
    }
  }

  void ModelBaseTracker::cloudCallback(
    const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    NODELET_INFO("cloudCallback");
    if (!tracker_) {
      JSK_NODELET_INFO("initialize Tracker");
      // viewpoint
      tf::StampedTransform transform
        = lookupTransformWithDuration(tf_, sensor_frame_, msg->header.frame_id,
                                      ros::Time(0.0),
                                      ros::Duration(0.0));
      tf::vectorTFToEigen(transform.getOrigin(), viewpoint_);

      // intialize Particles
      ParticleCloud::Ptr particles = initParticles();
      tracker_.reset(new pcl::tracking::ROSCollaborativeParticleFilterTracker
                     <pcl::PointXYZ, pcl::tracking::ParticleModel>);
      tracker_->setCustomSampleFunc(boost::bind(&ModelBaseTracker::sample, this, _1));
      tracker_->setLikelihoodFunc(boost::bind(&ModelBaseTracker::likelihood, this, _1, _2));
      tracker_->setParticles(particles);
      tracker_->setParticleNum(particle_num_);
      // Publish particles
      sensor_msgs::PointCloud2 ros_particles;
      pcl::toROSMsg(*particles, ros_particles);
      ros_particles.header = msg->header;
      pub_particles_.publish(ros_particles);
    }
    else {
      ParticleCloud::Ptr particles = tracker_->getParticles();
      Eigen::Vector4f center;
      pcl::compute3DCentroid(*particles, center);
      if (center.norm() > fast_cloud_threshold_) {
        computeTracking(msg);
      }
    }
  }

  void ModelBaseTracker::setReferenceCloudModel(const std::string fname):
  {
    std::string ext;
    pcl::PolygonMesh mesh;
    std::string stem;
    pcl::PolygonMesh mesh;
    refarence_cloud_.reset(new pcl::PointCloud<PointXYZ>);
    boost::filesystem::path path(fname);
    ext = path.extension().string();
    stem = path.stem().string();
    if (ext == ".stl")
      {
        pcl::io::loadPolygonFileSTL(fname, mesh);
        pcl::fromPCLPointCloud2(mesh.cloud, *reference_cloud_);
      }
    else
      {
        pcl::io::loadPCDFile(fname, *reference_cloud_);
      }
  }

  void ModelBaseTracker::renewInteractiveModel(const visualization_msgs::Marker &marker)
  {

  }

  void ModelBaseTracker::updateInteractiveModelRegionGrowing()
  {

  }

  void ModelBaseTracker::updateModelWeight()
  {

  }

  void ModelBaseTracker:: checkSanityModelRestraint()
  {

  }

  pcl::tracking::ParticleXYZRPY ModelBaseTracker::sample(const pcl::tracking::ParticleXYZRPY& p)
  {
    pcl::tracking::ParticleXYZRPY sampled_particle;
    // Motion model
    // considering restraint condition of ground or not
    if (ground_restraint_) {
      // particle satisfy below conditions
      // 1. hoge
      // 2. fuga
      // TODO
    }
    else {
      sampled_particle.x = randomGaussian(p.x, step_x_variance_, random_generator_);
      sampled_particle.y = randomGaussian(p.y, step_y_variance_, random_generator_);
      sampled_particle.z = randomGaussian(p.z, step_z_variance_, random_generator_);
      sampled_particle.roll = randomGaussian(p.roll, step_roll_variance_, random_generator_);
      sampled_particle.pitch = randomGaussian(p.pitch, step_pitch_variance_, random_generator_);
      sampled_particle.yaw = randomGaussian(p.yaw, step_yaw_variance_, random_generator_);
      sampled_particle.weight = p.weight;
    }
    return sampled_particle;
  }

  void ModelBaseTracker::likelihood(pcl::PointCloud<pcl::PointCloudXYZ>::ConstPtr input,
                                    pcl::tracking::ParticleXYZRPY& p)
  {
    p.weight = computeLikelihood(p, candidate_cloud_, viewpoint_,
                                 config_);
  }

  pcl::PointCloud<pcl::tracking::ParticleXYZRPY>::Ptr ModelBaseTracker::initParticles()
  {
    pcl::PointCloud<pcl::tracking::ParticleXYZRPY>::Ptr particles (new pcl::PointCloud<pcl::tracking::ParticleXYZRPY>);
    particles->points.resize(particle_num_);

    for (size_t i = 0; i < Particle_num_; i++) {
      while (true) {
        pcl::tracking::ParticleXYZRPY p_local;
        p_local.x = randomGaussian(init_x_mean_, init_x_variance_, random_generator_);
        p_local.y = randomGaussian(init_y_mean_, init_y_variance_, random_generator_);
        p_local.z = randomGaussian(init_z_mean_, init_z_variance_, random_generator_);
        p_local.roll = randomGaussian(init_local_orientation_roll_mean_,
                                      init_local_orientation_roll_variance_,
                                      random_generator_);
        p_local.pitch = randomGaussian(init_local_orientation_pitch_mean_,
                                     init_local_orientation_pitch_variance_,
                                       random_generator_);
        p_local.yaw = randomGaussian(init_local_orientation_yaw_mean_,
                                     init_local_orientation_yaw_variance_,
                                     random_generator_);

        if (plane_contraint_) {
          //TODO
        }
        if(disable_roll_) {
          p_global.roll = 0;
        }
        if(disable_pitch_) {
          p_global.pitch = 0;
        }
        if(disable_yaw_) {
          p_global.yaw = 0;
        }
        particles->points[i] = p_global;
        break;
      }
    }
    return particles;
  }

  void ModelBaseTracker::polygonCallback(
    const geometry_msgs::PolygonStamped::ConstPtr& polygon_msg,
    const pcl_msgs::ModelCoefficients:ConstPtr& coef_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    latest_polygon_msg_ = polygon_msg;
    latest_coefficients_msg_ = coef_msg;
    ground_plane_updated_ = true;
  }

  void ModelBaseTracker::configCallback(Config& config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    config_ = config;
    particle_num_ = config.particle_num;
    disable_roll_ = config.disable_roll_;
    disable_pitch_ = config.disable_pitch_;
    disable_yaw_ = config.disable_yaw_;
    init_x_mean_ = config.init_x_mean_;
    init_y_mean_ = config.init_y_mean_;
    init_z_mean_ = config.init_z_mean_;
    step_x_variance_ = config.step_x_variance;
    step_y_variance_ = config.step_y_variance;
    step_z_variance_ = config.step_z_variance;
    step_roll_variance_ = config.step_roll_variance;
    step_pitch_variance_ = config.step_pitch_variance;
    step_yaw_variance_ = config.step_yaw_variance;
    step_dx_variance_ = config.step_dx_variance;
    step_dy_variance_ = config.step_dy_variance;
    step_dz_variance_ = config.step_dz_variance;
  }

  bool ModelBaseTracker::resetCallback(std_srvs::EmptyRequest& req,
                                       std_srvs::EmptyResponse& res)
  {
    boost::mutex::scoped_lock lock(mutex_);
    latest_polygon_msg_ = geometry_msgs::PolygonStamped::ConstPtr();
    latest_coefficients_msg_ = pcl_msgs::ModelCoefficients::ConstPtr();
    tracker_.reset();
    return true;
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::ModelBaseTracker, nodelet::Nodelet);
