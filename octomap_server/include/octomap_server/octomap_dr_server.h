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
  void retrieveInformationForRay( octomap::OccupancyOcTreeBase<octomap::ColorOcTreeNode>* _octree, std::vector<boost::shared_ptr<InformationMetric> >& _metrics, octomap::point3d& _origin, octomap::point3d& _direction, double _min_ray_depth, double _max_ray_depth, double _occupied_passthrough_threshold, unsigned int _ray_step_size=1 );
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
  
protected:
  octomap::OccupancyOcTreeBase<octomap::ColorOcTreeNode>* octree_;
};

class IgnorantTotalIG: public OctomapDRServer::InformationMetric
{
public:
  IgnorantTotalIG():ig_(0.0){};
  inline std::string type()
  {
    return "IgnorantTotalIG";
  }
  
  /**
   * returns the occupancy likelihodd for the voxel
   */
  virtual double getOccupancy( octomap::OcTreeKey& _to_measure );
  
  /**
   * calculates the information gain for the voxel
   */
  virtual double calcIG( double _p_occ );
  
  virtual double getInformation();
  virtual void makeReadyForNewRay();
  virtual void includeRayMeasurement( octomap::OcTreeKey& _to_measure );
  virtual void includeEndPointMeasurement( octomap::OcTreeKey& _to_measure );
  
  static double unknown_p_prior_;
  static double unknown_lower_bound_;
  static double unknown_upper_bound_;
  
private:
  double ig_;
  
  void includeMeasurement( octomap::OcTreeKey& _to_measure );
};

class OccupancyAwareTotalIG: public IgnorantTotalIG
{
public:
  OccupancyAwareTotalIG():ig_(0.0),p_vis_(1.0){};
  inline std::string type()
  {
    return "OccupancyAwareTotalIG";
  }
  
  virtual double getInformation();
  virtual void makeReadyForNewRay();
  virtual void includeRayMeasurement( octomap::OcTreeKey& _to_measure );
  virtual void includeEndPointMeasurement( octomap::OcTreeKey& _to_measure );
  
  static double unknown_p_prior_;
  static double unknown_lower_bound_;
  static double unknown_upper_bound_;
  
private:
  double ig_;
  double p_vis_;
  
  void includeMeasurement( octomap::OcTreeKey& _to_measure );
};

/** information metric for rays:
 * counts how often a free voxel is followed by an unknown, a metric often described in exploration tasks
 */
class ClassicFrontier: public OctomapDRServer::InformationMetric
{
public:
  ClassicFrontier():frontier_voxel_count_(0),previous_voxel_free_(false),already_counts_(false){};
  inline std::string type()
  {
    return "ClassicFrontier";
  }
  double getInformation();
  void makeReadyForNewRay();
  void includeRayMeasurement( octomap::OcTreeKey& _to_measure );
  void includeEndPointMeasurement( octomap::OcTreeKey& _to_measure );
private:
  unsigned int frontier_voxel_count_;
  bool previous_voxel_free_;
  bool already_counts_;
};

class TotalUnknownIG: public IgnorantTotalIG
{
public:
  TotalUnknownIG():ig_(0.0),current_ray_ig_(0.0),p_vis_(1.0),previous_voxel_free_(false),already_counts_(false){};
  inline std::string type()
  {
    return "TotalUnknownIG";
  }
  
  virtual double getInformation();
  virtual void makeReadyForNewRay();
  virtual void includeRayMeasurement( octomap::OcTreeKey& _to_measure );
  virtual void includeEndPointMeasurement( octomap::OcTreeKey& _to_measure );
  
  /**
   * whether the ray passed a frontier
   * @arg _p_occ occupancy likelihood of the current voxel
   */
  virtual bool isFrontierPassRay( double _p_occ );
  
  /**
   * whtether the voxel is an unknown one or not
   */
  virtual bool isUnknown( double _p_occ );
  
  static double unknown_p_prior_;
  static double unknown_lower_bound_;
  static double unknown_upper_bound_;
  
private:
  double ig_; //total information gain
  double current_ray_ig_; //information gain for current ray
  double p_vis_;
  bool previous_voxel_free_;
  bool already_counts_;
  
  void includeMeasurement( octomap::OcTreeKey& _to_measure );
};

/** information metric for rays:
 * counts how often an unknown voxel is followed by an occupied one (this is thus an alternative version of the common 'frontier' metric)
 */
class UnknownObjectSideFrontier: public OctomapDRServer::InformationMetric
{
public:
  UnknownObjectSideFrontier():front_voxel_count_(0),previous_voxel_unknown_(false){};
  inline std::string type()
  {
    return "UnknownObjectSideFrontier";
  }
  double getInformation();
  void makeReadyForNewRay();
  void includeRayMeasurement( octomap::OcTreeKey& _to_measure );
  void includeEndPointMeasurement( octomap::OcTreeKey& _to_measure );
private:
  unsigned int front_voxel_count_;
  bool previous_voxel_unknown_;
  void includeMeasurement( octomap::OcTreeKey& _to_measure );
};

class UnknownObjectVolumeIG: public IgnorantTotalIG
{
public:
  UnknownObjectVolumeIG():ig_(0.0),current_ray_ig_(0.0),p_vis_(1.0),previous_voxel_unknown_(false){};
  inline std::string type()
  {
    return "UnknownObjectVolumeIG";
  }
  
  bool isUnknownVoxel( double _p_occ );
  
  virtual double getInformation();
  virtual void makeReadyForNewRay();
  virtual void includeRayMeasurement( octomap::OcTreeKey& _to_measure );
  virtual void includeEndPointMeasurement( octomap::OcTreeKey& _to_measure );
  
  static double unknown_p_prior_;
  static double unknown_lower_bound_;
  static double unknown_upper_bound_;
  
private:
  double ig_;
  double current_ray_ig_;
  double p_vis_;
  bool previous_voxel_unknown_;
  
  void includeMeasurement( octomap::OcTreeKey& _to_measure );
};

/** information metric for rays:
 * counts the number of unknown voxels hit by the ray
 */
class NrOfUnknownVoxels: public OctomapDRServer::InformationMetric
{
public:
  NrOfUnknownVoxels():unknown_voxel_count(0){};
  inline std::string type()
  {
    return "NrOfUnknownVoxels";
  }
  double getInformation();
  void makeReadyForNewRay();
  void includeRayMeasurement( octomap::OcTreeKey& _to_measure );
  void includeEndPointMeasurement( octomap::OcTreeKey& _to_measure );
private:
  unsigned int unknown_voxel_count;
};

/** information metric for rays:
 * calculates the average uncertainty of the measurements (basically how close the occupancy likelihood is to 0.5)
 * max uncertainty: 1, min uncertainty 0
 */
class AverageUncertainty: public OctomapDRServer::InformationMetric
{
public:
  AverageUncertainty(): certainty_sum_(0), nr_of_measurements_(0){};
  inline std::string type()
  {
    return "AverageUncertainty";
  }
  virtual double getInformation();
  virtual void makeReadyForNewRay();
  virtual void includeRayMeasurement( octomap::OcTreeKey& _to_measure );
  virtual void includeEndPointMeasurement( octomap::OcTreeKey& _to_measure );
protected:
  double certainty_sum_;
  unsigned int nr_of_measurements_;
  void includeMeasurement( octomap::OcTreeKey& _to_measure );
};

/** information metric for rays:
 * calculates the average uncertainty for found end points for the rays (uncertainty defined as for the AverageUncertainty metric)
 */
class AverageEndPointUncertainty: public AverageUncertainty
{
public:
  inline std::string type()
  {
    return "AverageEndPointUncertainty";
  }
  void includeRayMeasurement( octomap::OcTreeKey& _to_measure );
};

/** information metric for rays:
 * counts the total number of successive unknown voxels directly in front of an occupied one, thus targeted at finding
 * volumes of the object that haven't been seen so far
 */
class UnknownObjectVolumeFrontier: public OctomapDRServer::InformationMetric
{
public:
  UnknownObjectVolumeFrontier():volume_count_(0), running_count_(0){};
  inline std::string type()
  {
    return "UnknownObjectVolumeFrontier";
  }
  double getInformation();
  void makeReadyForNewRay();
  void includeRayMeasurement( octomap::OcTreeKey& _to_measure );
  void includeEndPointMeasurement( octomap::OcTreeKey& _to_measure );
private:
  unsigned int volume_count_;
  unsigned int running_count_;
  void includeMeasurement( octomap::OcTreeKey& _to_measure );
};

/** information metric for rays:
 * sums up the likelihoods over all end nodes
 */
class EndNodeOccupancySum: public OctomapDRServer::InformationMetric
{
public:
  EndNodeOccupancySum():sum_(0){};
  inline std::string type()
  {
    return "EndNodeOccupancySum";
  }
  double getInformation();
  void makeReadyForNewRay();
  void includeRayMeasurement( octomap::OcTreeKey& _to_measure );
  void includeEndPointMeasurement( octomap::OcTreeKey& _to_measure );
private:
  double sum_;
};

class TotalTreeMetric: public OctomapDRServer::InformationMetric
{
public:
  virtual void calculateOnTree( octomap::OccupancyOcTreeBase<octomap::ColorOcTreeNode>* _octree )=0;
  bool isTotalTreeMetric(){ return true;}
};

/** total sum over the occupancy of all nodes in the whole tree that are occupied - doesn't calculate on ray
 */
class TotalOccupancyCertainty: public TotalTreeMetric
{
public:
  TotalOccupancyCertainty():sum_(0){};
  inline std::string type()
  {
    return "TotalOccupancyCertainty";
  }
  void calculateOnTree( octomap::OccupancyOcTreeBase<octomap::ColorOcTreeNode>* _octree );
  double getInformation();
  void makeReadyForNewRay(){};
  void includeRayMeasurement( octomap::OcTreeKey& _to_measure ){};
  void includeEndPointMeasurement( octomap::OcTreeKey& _to_measure ){};
private:
  double sum_;
};

/** total number of occupieds in the whole tree
 */
class TotalNrOfOccupieds: public TotalTreeMetric
{
public:
  TotalNrOfOccupieds():sum_(0){};
  inline std::string type()
  {
    return "TotalNrOfOccupieds";
  }
  void calculateOnTree( octomap::OccupancyOcTreeBase<octomap::ColorOcTreeNode>* _octree );
  double getInformation();
  void makeReadyForNewRay(){};
  void includeRayMeasurement( octomap::OcTreeKey& _to_measure ){};
  void includeEndPointMeasurement( octomap::OcTreeKey& _to_measure ){};
private:
  unsigned int sum_;
};

/** total number of free nodes in the whole tree
 */
class TotalNrOfFree: public TotalTreeMetric
{
public:
  TotalNrOfFree():sum_(0){};
  inline std::string type()
  {
    return "TotalNrOfFree";
  }
  void calculateOnTree( octomap::OccupancyOcTreeBase<octomap::ColorOcTreeNode>* _octree );
  double getInformation();
  void makeReadyForNewRay(){};
  void includeRayMeasurement( octomap::OcTreeKey& _to_measure ){};
  void includeEndPointMeasurement( octomap::OcTreeKey& _to_measure ){};
private:
  unsigned int sum_;
};

/** total entropy in the whole tree
 */
class TotalEntropy: public TotalTreeMetric
{
public:
  TotalEntropy():sum_(0){};
  inline std::string type()
  {
    return "TotalEntropy";
  }
  void calculateOnTree( octomap::OccupancyOcTreeBase<octomap::ColorOcTreeNode>* _octree );
  double getInformation();
  void makeReadyForNewRay(){};
  void includeRayMeasurement( octomap::OcTreeKey& _to_measure ){};
  void includeEndPointMeasurement( octomap::OcTreeKey& _to_measure ){};
private:
  double sum_;
};

/** total number of occupieds in the whole tree
 */
class TotalNrOfNodes: public TotalTreeMetric
{
public:
  TotalNrOfNodes():sum_(0){};
  inline std::string type()
  {
    return "TotalNrOfNodes";
  }
  void calculateOnTree( octomap::OccupancyOcTreeBase<octomap::ColorOcTreeNode>* _octree );
  double getInformation();
  void makeReadyForNewRay(){};
  void includeRayMeasurement( octomap::OcTreeKey& _to_measure ){};
  void includeEndPointMeasurement( octomap::OcTreeKey& _to_measure ){};
private:
  unsigned int sum_;
};

}

