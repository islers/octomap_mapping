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

#include <octomap_server/octomap_dr_server.h>
#include "boost/foreach.hpp"


namespace octomap_server {
  
OctomapDRServer::OctomapDRServer(ros::NodeHandle _private_nh)
  :OctomapServer()
  ,camera_info_topic_("/camera/camera_info")
{
  
  double bbx_min_x;
  double bbx_min_y;
  double bbx_min_z;
  double bbx_max_x;
  double bbx_max_y;
  double bbx_max_z;
  
  if( _private_nh.getParam("update_volume/min/x",bbx_min_x) &&
    _private_nh.getParam("update_volume/max/x",bbx_max_x) )
  {
    ROS_INFO("Set update limits for x.");
    
    m_update_volume_min.x() = bbx_min_x;
    m_update_volume_max.x() = bbx_max_x;
    
    m_use_update_volume_x = true;
  }
  if( 
    _private_nh.getParam("update_volume/min/y",bbx_min_y) &&
    _private_nh.getParam("update_volume/max/y",bbx_max_y) )
  {
    ROS_INFO("Set update limits for y.");
    
    m_update_volume_min.y() = bbx_min_y;
    m_update_volume_max.y() = bbx_max_y;
    
    m_use_update_volume_y = true;
  }
  if( 
    _private_nh.getParam("update_volume/min/z",bbx_min_z) &&
    _private_nh.getParam("update_volume/max/z",bbx_max_z) )
  {
    ROS_INFO("Set update limits for z.");
    
    m_update_volume_min.z() = bbx_min_z;
    m_update_volume_max.z() = bbx_max_z;
  }
  
  _private_nh.param("camera_info_topic", camera_info_topic_, camera_info_topic_);
  
  camera_info_subscriber_ = m_nh.subscribe(camera_info_topic_,1,&OctomapDRServer::cameraInfoCallback, this );
}

bool OctomapDRServer::informationService( dense_reconstruction::ViewInformationReturn::Request& _req, dense_reconstruction::ViewInformationReturn::Response& _res )
{
  // test if camera info has been initialized
  if( !cam_model_.initialized() )
  {
    ROS_ERROR("OctomapDRServer::informationService called but no camera information has been received yet. Cannot serve the request.");
    return false;
  }
  if( _req.call.ray_resolution_x==0 || _req.call.ray_resolution_y )
  {
    ROS_ERROR("OctomapDRServer::informationService called with ray_resolution_x or ray_resolution_y zero which is not allowed. Cannot serve the request.");
    return false;
  }
  if( _req.call.poses.size()==0 )
  {
    ROS_WARN("OctomapDRServer::informationService called without any view poses given.");
    return true;
  }
  
  // build data needed for the rays
  std::vector<octomap::point3d> origins; // camera positions
  std::vector< Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond> > orientations; // camera orientations
  std::vector< Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > ray_directions; // directions of the rays
  std::vector< Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > forecast_ray_directions; // directions of the rays for forecasting
  
  for( unsigned int i=0;i<_req.call.poses.size();i++ )
  {
    origins.push_back( octomap::point3d( _req.call.poses[i].position.x, _req.call.poses[i].position.y, _req.call.poses[i].position.z ) );
    orientations.push_back( Eigen::Quaterniond( _req.call.poses[i].orientation.w, _req.call.poses[i].orientation.x, _req.call.poses[i].orientation.y, _req.call.poses[i].orientation.z ) );
  }
  
  // build ray directions for information retrieval, relative to camera coordinate frame
  double px_x_step = 1.0/_req.call.ray_resolution_x;
  double px_y_step = 1.0/_req.call.ray_resolution_y;
  for( double x=0; x<cam_model_.fullResolution().width; x+=px_x_step )
  {
    for( double y=0; y<cam_model_.fullResolution().height; y+=px_y_step )
    {
      cv::Point2d pixel(x,y);
      cv::Point3d ray_direction = cam_model_.projectPixelTo3dRay(pixel);
      Eigen::Vector3d direction(ray_direction.x, ray_direction.y, ray_direction.z );
      direction.normalize();
      ray_directions.push_back(direction);
    }
  }
  
  if( _req.call.poses.size()!=1 ) // for predictions the "artificial raytracing grid" ray directions need to be built as well
  {
    if( _req.call.forecast_ray_resolution_x==0 )
    {
      ROS_WARN("OctomapDRServer::informationService: forecast_ray_resolution_x not specified or zero, using same resolution as for information retrieval.");
      _req.call.forecast_ray_resolution_x = _req.call.ray_resolution_x;
    }
    if( _req.call.forecast_ray_resolution_y==0 )
    {
      ROS_WARN("OctomapDRServer::informationService: forecast_ray_resolution_y not specified or zero, using same resolution as for information retrieval.");
      _req.call.forecast_ray_resolution_y = _req.call.ray_resolution_y;
    }
    double px_forecast_x_step = 1.0/_req.call.forecast_ray_resolution_x;
    double px_forecast_y_step = 1.0/_req.call.forecast_ray_resolution_y;
    for( double x=0; x<cam_model_.fullResolution().width; x+=px_forecast_x_step )
    {
      for( double y=0; y<cam_model_.fullResolution().height; y+=px_forecast_y_step )
      {
	cv::Point2d pixel(x,y);
	cv::Point3d ray_direction = cam_model_.projectPixelTo3dRay(pixel);
	Eigen::Vector3d direction(ray_direction.x, ray_direction.y, ray_direction.z );
	direction.normalize();
	forecast_ray_directions.push_back(direction);
      }
    }
  }
  
  InformationRetrievalStructure infos;
  infos.iteration_idx = 0;
  infos.request = &_req;
  infos.response = &_res;
  infos.origins = &origins;
  infos.orientations = &orientations;
  infos.ray_directions = &ray_directions;
  infos.forecast_ray_directions = &forecast_ray_directions;
  infos.octree = m_octree;
  
  informationRetrieval(infos);
  
  return true;
}

void OctomapDRServer::cameraInfoCallback( const sensor_msgs::CameraInfoConstPtr& _caminfo )
{
  cam_model_.fromCameraInfo(_caminfo);
  camera_info_subscriber_.shutdown();
}

void OctomapDRServer::informationRetrieval( InformationRetrievalStructure& _info  )
{
  if( _info.iteration_idx==(_info.origins->size()-1) ) // that's the view for which information shall be received
  {
    retrieveInformationForView( _info );
    return;
  }
  else // simulating the future, if different hypothesis techniques were available, they'd be used here (TODO), now assuming that each upcoming observation will simply support what has already been seen
  {/*
    octomap::OccupancyOcTreeBase<octomap::ColorOcTreeNode>* octree;
    if( _info.iteration_idx==0 ) // build hypothetical octree to include synthetic measurements. TODO: Could be optimized e.g. by caching some of the results since very similar calls might come from planner
    {
      if (m_stereoModel)
      {
	octree = new octomap::ColorOcTreeStereo(_info.octree); TODO (deep copy)
      }
      else
      {
	octree = new octomap::ColorOcTree(_info.octree); TODO (deep copy)
      }
    }
    else
    {
      octree = _info.octree;
    }
    // retrieve information... TODO
    // new InformationRetrievalStructure
    // recursive call to informationRetrieval(...)
    
    if( _info.iteration_idx==0 )
      delete octree;*/
  }
}

void OctomapDRServer::retrieveInformationForView( InformationRetrievalStructure& _info )
{
  // build vector with all ray metrics
  std::vector<boost::shared_ptr<InformationMetric> > metrics;

  BOOST_FOREACH( std::string metric, _info.request->call.metric_names )
  {
    if( metric=="NrOfUnknownVoxels" )
    {
      metrics.push_back( boost::shared_ptr<InformationMetric>( new NrOfUnknownVoxels() ) );
    }
    else if( metric=="AverageUncertainty" )
    {
      metrics.push_back( boost::shared_ptr<InformationMetric>( new AverageUncertainty() ) );
    }
    else if( metric=="AverageEndPointUncertainty" )
    {
      metrics.push_back( boost::shared_ptr<InformationMetric>( new AverageEndPointUncertainty() ) );
    }
    else if( metric=="UnknownObjectSideFrontier" )
    {
      metrics.push_back( boost::shared_ptr<InformationMetric>( new UnknownObjectSideFrontier() ) );
    }
    else if( metric=="UnknownObjectVolumeFrontier" )
    {
      metrics.push_back( boost::shared_ptr<InformationMetric>( new UnknownObjectVolumeFrontier() ) );
    }
    else if( metric=="ClassicFrontier" )
    {
      metrics.push_back( boost::shared_ptr<InformationMetric>( new ClassicFrontier() ) );
    }
    metrics.back()->setOcTreeTarget(_info.octree);
  }
  
  // retrieve information for each ray
  for( unsigned int i=0; i<_info.ray_directions->size(); ++i )
  {
    BOOST_FOREACH( boost::shared_ptr<InformationMetric> metric, metrics )
    {
      metric->makeReadyForNewRay();
    }
    // transform direction from camera coordinates to octomap coordinates, given orientation of the view
    Eigen::Vector3d dir_oct = _info.orientations->back()*(*_info.ray_directions)[i];
    octomap::point3d direction( dir_oct.x(), dir_oct.y(), dir_oct.z() );
    retrieveInformationForRay( _info.octree, metrics, _info.origins->back(), direction, _info.request->call.min_ray_depth, _info.request->call.max_ray_depth, _info.request->call.occupied_passthrough_threshold );
  }
  
  _info.response->expected_information.metric_names = _info.request->call.metric_names;
  
  std::vector<double> information;
  BOOST_FOREACH( boost::shared_ptr<InformationMetric> metric, metrics )
  {
    information.push_back( metric->getInformation() );
  }
  _info.response->expected_information.values = information;
  
  return;
}

void OctomapDRServer::retrieveInformationForRay( octomap::OccupancyOcTreeBase<octomap::ColorOcTreeNode>* _octree, std::vector<boost::shared_ptr<InformationMetric> >& _metrics, octomap::point3d& _origin, octomap::point3d& _direction, double _min_ray_depth, double _max_ray_depth, double _occupied_passthrough_threshold )
{
  octomap::point3d end_point; // calculate endpoint (if any)
  
  double max_range = (_max_ray_depth>0)?_max_ray_depth:10.0;
  double log_odd_passthrough_threshold = std::log(_occupied_passthrough_threshold);
  
  double min_ray_depth = (_min_ray_depth==0)?0.005:_min_ray_depth; //default for min ray depth [m]
  
  bool found_endpoint = _octree->castRay( _origin, _direction, end_point, true, max_range ); // ignore unknown cells
  
  if( found_endpoint ) // check that endpoint satisfies constraints and move startpoint once if on occupied voxel
  {
    octomap::point3d offset_origin = _origin;
    double max_range_for_offset = max_range;
    bool search = true;
    
    if( end_point==offset_origin ) // try an offset once
    {
      offset_origin = offset_origin + _direction*min_ray_depth;
      max_range_for_offset -= min_ray_depth;
      found_endpoint = _octree->castRay( offset_origin, _direction, end_point, true, max_range_for_offset );
            
      search = true;
    }
    
    if( found_endpoint )
    {
      octomap::ColorOcTreeNode* end_node = _octree->search(end_point);
      double occ_likelihood = end_node->getLogOdds();
      
      if( occ_likelihood<log_odd_passthrough_threshold ) // doesn't satisfy, keep recasting rays until either satisfying end point is found or none
      {
	octomap::point3d offset_origin = end_point;
	octomap::point3d dist_vec = end_point-_origin;
	double current_length = dist_vec.norm();
	max_range_for_offset = max_range - current_length;
	
	do // look for different end point for as long as none satisfies the threshold
	{
	  found_endpoint=false; // no endpoint found that satisfies
	  
	  offset_origin = offset_origin + _direction*min_ray_depth;
	  max_range_for_offset -= min_ray_depth;
	  
	  if( max_range_for_offset<=0 ) // no space left for iteration
	  {
	    found_endpoint=false;
	    break;
	  }
	  
	  found_endpoint = _octree->castRay( offset_origin, _direction, end_point, true, max_range_for_offset );
	  
	  if( !found_endpoint )
	    break;
	  
	  end_node = _octree->search(end_point);
	  occ_likelihood = end_node->getLogOdds();
	}while( occ_likelihood<log_odd_passthrough_threshold );
      }
    }
  }
  
  // calculate metrics for all points on ray, calculate points on ray
  if( !found_endpoint ) // *artificial end point*
    end_point = _origin + _direction*max_range;
  
  octomap::KeyRay ray;
  _octree->computeRayKeys( _origin, end_point, ray );
  
  for( octomap::KeyRay::iterator it = ray.begin(); it!=ray.end(); ++it )
  {
    BOOST_FOREACH( boost::shared_ptr<InformationMetric> metric, _metrics )
    {
      metric->includeRayMeasurement( *it );
    }
  }
  
  // calculate metric for end point of ray if it exists
  if( found_endpoint )
  {
    octomap::OcTreeKey end_key = _octree->coordToKey(end_point);
    BOOST_FOREACH( boost::shared_ptr<InformationMetric> metric, _metrics )
    {
      metric->includeEndPointMeasurement( end_key );
    }
  }
  
  return;
}

double NrOfUnknownVoxels::getInformation()
{
  return unknown_voxel_count;
}

void NrOfUnknownVoxels::makeReadyForNewRay()
{
  // void
}

void NrOfUnknownVoxels::includeRayMeasurement( octomap::OcTreeKey& _to_measure )
{
  octomap::ColorOcTreeNode* added = octree_->search(_to_measure);
  if( added==NULL )
    ++unknown_voxel_count;
}

void NrOfUnknownVoxels::includeEndPointMeasurement( octomap::OcTreeKey& _to_measure )
{
  // void - an end point must be occupied and thus can't be unknown
}

double AverageUncertainty::getInformation()
{
  return 2*(0.5-certainty_sum_/nr_of_measurements_);
}

void AverageUncertainty::makeReadyForNewRay()
{
  // has no effect
}

void AverageUncertainty::includeRayMeasurement( octomap::OcTreeKey& _to_measure )
{
  includeMeasurement(_to_measure);
}

void AverageUncertainty::includeEndPointMeasurement( octomap::OcTreeKey& _to_measure )
{
  includeMeasurement(_to_measure);
}

void AverageUncertainty::includeMeasurement( octomap::OcTreeKey& _to_measure )
{
  double occupany_likelihood;
  octomap::ColorOcTreeNode* added = octree_->search(_to_measure);
  if( added==NULL )
  {
    occupany_likelihood=0.5; // default for unknown
  }
  else
  {
    occupany_likelihood = added->getOccupancy();
  }
  certainty_sum_+=std::fabs(0.5-occupany_likelihood);
  ++nr_of_measurements_;
}

void AverageEndPointUncertainty::includeRayMeasurement( octomap::OcTreeKey& _to_measure )
{
  // void
}

double UnknownObjectSideFrontier::getInformation()
{
  return front_voxel_count_;
}

void UnknownObjectSideFrontier::makeReadyForNewRay()
{
  previous_voxel_unknown_ = false;
}

void UnknownObjectSideFrontier::includeRayMeasurement( octomap::OcTreeKey& _to_measure )
{
  includeMeasurement(_to_measure);
}

void UnknownObjectSideFrontier::includeEndPointMeasurement( octomap::OcTreeKey& _to_measure )
{
  includeMeasurement(_to_measure);
}

void UnknownObjectSideFrontier::includeMeasurement( octomap::OcTreeKey& _to_measure )
{
  octomap::ColorOcTreeNode* added = octree_->search(_to_measure);
  if( added==NULL )
  {
    previous_voxel_unknown_=true;
  }
  else
  {
    if( octree_->isNodeOccupied( added ) && previous_voxel_unknown_ ) // object side frontier!
    {
      ++front_voxel_count_;
    }
    previous_voxel_unknown_=false;
  }
}

double UnknownObjectVolumeFrontier::getInformation()
{
  return volume_count_;
}

void UnknownObjectVolumeFrontier::makeReadyForNewRay()
{
  running_count_ = 0;
}

void UnknownObjectVolumeFrontier::includeRayMeasurement( octomap::OcTreeKey& _to_measure )
{
  includeMeasurement(_to_measure);
}

void UnknownObjectVolumeFrontier::includeEndPointMeasurement( octomap::OcTreeKey& _to_measure )
{
  includeMeasurement(_to_measure);
}

void UnknownObjectVolumeFrontier::includeMeasurement( octomap::OcTreeKey& _to_measure )
{
  octomap::ColorOcTreeNode* to_measure = octree_->search(_to_measure);
  if( to_measure==NULL )
  {
    ++running_count_;
  }
  else
  {
    if( octree_->isNodeOccupied( to_measure ) ) // object side frontier!
    {
      volume_count_+=running_count_;
    }
    running_count_=0;
  }
}

double ClassicFrontier::getInformation()
{
  return frontier_voxel_count_;
}

void ClassicFrontier::makeReadyForNewRay()
{
  previous_voxel_free_=false;
}

void ClassicFrontier::includeRayMeasurement( octomap::OcTreeKey& _to_measure )
{
  octomap::ColorOcTreeNode* to_measure = octree_->search(_to_measure);
  if( to_measure==NULL )
  {
    if( previous_voxel_free_ )
    {
      ++frontier_voxel_count_;
    }
    previous_voxel_free_ = false;
  }
  else
  {
    if( !octree_->isNodeOccupied( to_measure ) ) // frontier
    {
      previous_voxel_free_ = true;
    }
    else
    {
      previous_voxel_free_ = false;
    }
  }
}

void ClassicFrontier::includeEndPointMeasurement( octomap::OcTreeKey& _to_measure )
{
  // void since end points are occupied by definition
}


}

