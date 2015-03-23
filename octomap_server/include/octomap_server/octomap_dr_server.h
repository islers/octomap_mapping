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
  
  ros::Subscriber camera_info_subscriber_;
  
  std::string camera_info_topic_;
  
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
   */
  void retrieveInformationForRay( octomap::OccupancyOcTreeBase<octomap::ColorOcTreeNode>* _octree, std::vector<boost::shared_ptr<InformationMetric> >& _metrics, octomap::point3d& _origin, octomap::point3d& _direction, double _min_ray_depth, double _max_ray_depth, double _occupied_passthrough_threshold );
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
  octomap::OccupancyOcTreeBase<octomap::ColorOcTreeNode>* octree; /// current octree on which the function should be operating on
};

/// abstract base class (interface) for information metrics @TODO: factory?
class OctomapDRServer::InformationMetric
{
public:
  /**
   * tells the metric on which octree to perform the measurements, won't do anything if this isn't set
   */
  inline void setOcTreeTarget( octomap::OccupancyOcTreeBase<octomap::ColorOcTreeNode>* _octree )
  {
    octree_ = _octree;
  }
  
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
  virtual void makeReadyForNewRay()=0;
  
  /**
   * includes a measurement for a point on a ray
   */
  virtual void includeRayMeasurement( octomap::OcTreeKey& _to_measure )=0;
  
  /**
   * includes a measurement for an endpoint
   */
  virtual void includeEndPointMeasurement( octomap::OcTreeKey& _to_measure )=0;
  
protected:
  octomap::OccupancyOcTreeBase<octomap::ColorOcTreeNode>* octree_;
};

/** information metric for rays:
 * counts the number of unknown voxels hit by the ray
 */
class NrOfUnknownVoxels: public OctomapDRServer::InformationMetric
{
  inline std::string type()
  {
    return "NrOfUnknownVoxels";
  }
  double getInformation();
  void makeReadyForNewRay();
  void includeRayMeasurement( octomap::OcTreeKey& _to_measure );
  void includeEndPointMeasurement( octomap::OcTreeKey& _to_measure );
};

/** information metric for rays:
 * calculates the average uncertainty of the measurements (basically how close the occupancy likelihood is to 0.5)
 * max uncertainty: 1, min uncertainty 0
 */
class AverageUncertainty: public OctomapDRServer::InformationMetric
{
  inline std::string type()
  {
    return "AverageUncertainty";
  }
  double getInformation();
  void makeReadyForNewRay();
  void includeRayMeasurement( octomap::OcTreeKey& _to_measure );
  void includeEndPointMeasurement( octomap::OcTreeKey& _to_measure );
};

/** information metric for rays:
 * calculates the average uncertainty for found end points for the rays (uncertainty defined as for the AverageUncertainty metric)
 */
class AverageEndPointUncertainty: public OctomapDRServer::InformationMetric
{
  inline std::string type()
  {
    return "AverageEndPointUncertainty";
  }
  double getInformation();
  void makeReadyForNewRay();
  void includeRayMeasurement( octomap::OcTreeKey& _to_measure );
  void includeEndPointMeasurement( octomap::OcTreeKey& _to_measure );
};

/** information metric for rays:
 * counts how often an unknown voxel is followed by an occupied one (this is thus an alternative version of the common 'frontier' metric)
 */
class UnknownObjectSideFrontier: public OctomapDRServer::InformationMetric
{
  inline std::string type()
  {
    return "UnknownObjectSideFrontier";
  }
  double getInformation();
  void makeReadyForNewRay();
  void includeRayMeasurement( octomap::OcTreeKey& _to_measure );
  void includeEndPointMeasurement( octomap::OcTreeKey& _to_measure );
};

/** information metric for rays:
 * counts the total number of successive unknown voxels directly in front of an occupied one, thus targeted at finding
 * hidden volume of the object
 */
class UnknownObjectVolumeFrontier: public OctomapDRServer::InformationMetric
{
  inline std::string type()
  {
    return "UnknownObjectVolumeFrontier";
  }
  double getInformation();
  void makeReadyForNewRay();
  void includeRayMeasurement( octomap::OcTreeKey& _to_measure );
  void includeEndPointMeasurement( octomap::OcTreeKey& _to_measure );
};

/** information metric for rays:
 * counts how often a free voxel is followed by an unknown, a metric often described in exploration tasks
 */
class ClassicFrontier: public OctomapDRServer::InformationMetric
{
  inline std::string type()
  {
    return "ClassicFrontier";
  }
  double getInformation();
  void makeReadyForNewRay();
  void includeRayMeasurement( octomap::OcTreeKey& _to_measure );
  void includeEndPointMeasurement( octomap::OcTreeKey& _to_measure );
};

}

