/* Copyright (c) 2015, Stefan Isler, islerstefan@bluewin.ch
*
This file is part of dense_reconstruction, a ROS package for...well,
***********************************************************************
 *
 * based on the OctomapDRServer from
 * 
 * Copyright (c) 2010-2013, A. Hornung, University of Freiburg
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once
#include <octomap_server/OctomapServer.h>
#include <image_geometry/pinhole_camera_model.h>
#include <dense_reconstruction/ViewInformation.h>
#include <dense_reconstruction/ViewInformationRequest.h>
#include <dense_reconstruction/ViewInformationReturn.h>
#include <visualization_msgs/Marker.h>


namespace octomap_server {
class OctomapDRServer:public OctomapServer{
public:
  struct InformationRetrievalStructure;
  class InformationMetric;
  
  OctomapDRServer(ros::NodeHandle private_nh_ = ros::NodeHandle("~"));
  
  /**
   * service that gathers information related to specific views
   */
  bool informationService( dense_reconstruction::ViewInformationReturn::Request& _req, dense_reconstruction::ViewInformationReturn::Response& _res );
  
  /// loads all necessary information about the camera from the camera info topic (default:"/camera/camera_info", can be defined using the 'camera_info_topic' parameter)
  void cameraInfoCallback( const sensor_msgs::CameraInfoConstPtr& _caminfo );
private:
  image_geometry::PinholeCameraModel cam_model_;
  
  ros::ServiceServer view_information_server_;
  ros::Subscriber camera_info_subscriber_;
  ros::Publisher marker_visualizer_;
  
  std::string camera_info_topic_;
  
  bool display_rays_;
  
  /**
   * creates and publishes a message for RVIZ to visualize the ray
   * @param _origin line origin
   * @param _direction line direction
   * @param _length line length [m], default is 4m
   */
  void displayRay( octomap::point3d& _origin, octomap::point3d& _direction, double _length=4 );
  
  /**
   * possibly recursive function call to retrieve informations for a view that may possibly be further in the future
   */
  void informationRetrieval( InformationRetrievalStructure& _info  );
  
  /**
   * retrieves the information for the last view in the _info structure
   */
  void retrieveInformationForView( InformationRetrievalStructure& _info );
  
  /** retrieves the information for a specific ray
   * @param _octree OcTree in which the ray will be cast and the information calculated
   * @param _metrics set of metrics to use for the calculation
   * @param _origin origin of the ray
   * @param _direction direction of the ray
   * @param _min_ray_depth minimal length of the ray
   * @param _max_ray_depth maximal length of the ray (if zero it isn't considered)
   * @param _occupied_passthrough_threshold if endpoints have an occupancy likelihood lower than this, then they're not considered as endpoint and the ray is continued
   * @param _ray_step_size each _ray_step_size-th voxel on the ray is used for metric calculations (if zero, default is 1)
   */
  void retrieveInformationForRay( octomap::OccupancyOcTreeBase<octomap::DROcTreeNode>* _octree, std::vector<boost::shared_ptr<InformationMetric> >& _metrics, octomap::point3d& _origin, octomap::point3d& _direction, double _min_ray_depth, double _max_ray_depth, double _occupied_passthrough_threshold, unsigned int _ray_step_size=1 );
};

/// handy structure to bundle function arguments
struct OctomapDRServer::InformationRetrievalStructure
{
  unsigned int iteration_idx; /// which pose is to be processed
  dense_reconstruction::ViewInformationReturn::Request* request;
  dense_reconstruction::ViewInformationReturn::Response* response;
  std::vector<octomap::point3d>* origins;
  std::vector< Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond> >* orientations;
  std::vector< Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >* ray_directions;
  std::vector< Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >* forecast_ray_directions;
  octomap::OccupancyOcTreeBase<octomap::DROcTreeNode>* octree; /// current octree on which the function should be operating on
};

/// abstract base class (interface) for information metrics @TODO: factory?
class OctomapDRServer::InformationMetric
{
public:
    InformationMetric():traversedVoxelCount_(0){};
  /**
   * tells the metric on which octree to perform the measurements, won't do anything if this isn't set
   */
  inline void setOcTreeTarget( octomap::OccupancyOcTreeBase<octomap::DROcTreeNode>* _octree )
  {
    octree_ = _octree;
  }
  
  virtual bool isTotalTreeMetric(){ return false; }
  
  /**
   * returns the name of the method
   */
  virtual std::string type()=0;
  
  /**
   * calculates the information for the data added so far (for all rays)
   */
  virtual double getInformation()=0;
  
  /**
   * clears all ray specific data, gets ready for a new ray
   */
  virtual void makeReadyForNewRay(){};
  
  /**
   * includes a measurement for a point on a ray
   */
  virtual void includeRayMeasurement( octomap::OcTreeKey& _to_measure )=0;
  
  /**
   * includes a measurement for an endpoint
   */
  virtual void includeEndPointMeasurement( octomap::OcTreeKey& _to_measure )=0;
  
  /**
   * informs the metric that a complete ray was cast through empty space without
   * retrieving any measurements
   */
  virtual void informAboutVoidRay(){};
  
  /** returns the number of traversed voxels */
  virtual unsigned int voxelCount(){ return traversedVoxelCount_; }
  
protected:
  unsigned int traversedVoxelCount_; // total nr of traversed voxels
  octomap::OccupancyOcTreeBase<octomap::DROcTreeNode>* octree_;
};

}