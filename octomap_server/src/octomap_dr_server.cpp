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
  ,display_rays_(false)
{
  
  double bbx_min_x;
  double bbx_min_y;
  double bbx_min_z;
  double bbx_max_x;
  double bbx_max_y;
  double bbx_max_z;
  
  _private_nh.getParam("display_rays",display_rays_);
  
  if( _private_nh.getParam("update_volume/min/x",bbx_min_x) &&
    _private_nh.getParam("update_volume/max/x",bbx_max_x) )
  {
    ROS_INFO("Set update limits for: x.");
    
    m_update_volume_min.x() = bbx_min_x;
    m_update_volume_max.x() = bbx_max_x;
    
    m_use_update_volume_x = true;
  }
  if( 
    _private_nh.getParam("update_volume/min/y",bbx_min_y) &&
    _private_nh.getParam("update_volume/max/y",bbx_max_y) )
  {
    ROS_INFO("Set update limits for: y.");
    
    m_update_volume_min.y() = bbx_min_y;
    m_update_volume_max.y() = bbx_max_y;
    
    m_use_update_volume_y = true;
  }
  if( 
    _private_nh.getParam("update_volume/min/z",bbx_min_z) &&
    _private_nh.getParam("update_volume/max/z",bbx_max_z) )
  {
    ROS_INFO("Set update limits for: z.");
    
    m_update_volume_min.z() = bbx_min_z;
    m_update_volume_max.z() = bbx_max_z;
  }
  
  _private_nh.param("camera_info_topic", camera_info_topic_, camera_info_topic_);
  
  camera_info_subscriber_ = m_nh.subscribe(camera_info_topic_,1,&OctomapDRServer::cameraInfoCallback, this );
  view_information_server_ = m_nh.advertiseService("/dense_reconstruction/3d_model/information", &OctomapDRServer::informationService, this);
  marker_visualizer_ = m_nh.advertise<visualization_msgs::Marker>("/dense_reconstruction/octomap/visualization", 10);
}

bool OctomapDRServer::informationService( dense_reconstruction::ViewInformationReturn::Request& _req, dense_reconstruction::ViewInformationReturn::Response& _res )
{
  ROS_INFO("informationService called, calculating information for view.");
  
  // test if camera info has been initialized
  if( !cam_model_.initialized() )
  {
    ROS_ERROR("OctomapDRServer::informationService called but no camera information has been received yet. Cannot serve the request.");
    return false;
  }
  if( _req.call.ray_resolution_x==0 || _req.call.ray_resolution_y==0 )
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
  _req.call.ray_step_size = (_req.call.ray_step_size==0)?1:_req.call.ray_step_size;
  
  unsigned int max_x = (_req.call.max_x==0)?cam_model_.fullResolution().width:_req.call.max_x;
  unsigned int min_x = _req.call.min_x;
  unsigned int max_y = (_req.call.max_y==0)?cam_model_.fullResolution().height:_req.call.max_y;
  unsigned int min_y = _req.call.min_y;
  
  if( max_x<=min_x || max_y<=min_y )
  {
    ROS_ERROR("OctomapDRServer::informationService called with invalid max_x/min_x or max_y/min_y combination. Cannot serve the request.");
    return false;
  }
  
  for( double x=min_x; x<=max_x; x+=px_x_step ) // just to test the directions
  {
    for( double y=min_y; y<=max_y; y+=px_y_step )
    {
      cv::Point2d pixel(x,y);
      cv::Point3d ray_direction = cam_model_.projectPixelTo3dRay(pixel);
      Eigen::Vector3d direction(ray_direction.x, ray_direction.y, ray_direction.z );
      direction.normalize();
      ray_directions.push_back(direction);
    }
  }
  
  ROS_INFO_STREAM("A total number of "<<ray_directions.size()<<" rays will be cast per view.");
  
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
  
  ROS_INFO("Service finished.");
  
  return true;
}

void OctomapDRServer::cameraInfoCallback( const sensor_msgs::CameraInfoConstPtr& _caminfo )
{
  cam_model_.fromCameraInfo(_caminfo);
  camera_info_subscriber_.shutdown();
}

void OctomapDRServer::displayRay( octomap::point3d& _origin, octomap::point3d& _direction, double _length )
{
  visualization_msgs::Marker line;
  line.header.frame_id = "dr_origin";
  line.header.stamp = ros::Time::now();
  line.ns = "information_score_ray";
  line.action = visualization_msgs::Marker::ADD;
  line.pose.orientation.w = 1.0;
  line.id = 0;
  
  line.type = visualization_msgs::Marker::LINE_STRIP;
  
  line.lifetime = ros::Duration(0.01);
  
  line.scale.x = 0.02;
  line.color.g = 1;
  line.color.r = 1;
  line.color.a = 1;
  
  geometry_msgs::Point origin,end;
  origin.x = _origin.x();
  origin.y =_origin.y();
  origin.z = _origin.z();
  end.x = _origin.x()+_length*_direction.x();
  end.y = _origin.y()+_length*_direction.y();
  end.z = _origin.z()+_length*_direction.z();
    
  line.points.push_back(origin);
  line.points.push_back(end);
  marker_visualizer_.publish(line);
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
    if( metric=="IgnorantTotalIG" )
    {
      metrics.push_back( boost::shared_ptr<InformationMetric>( new IgnorantTotalIG() ) );
    }
    else if( metric=="OccupancyAwareTotalIG" )
    {
      metrics.push_back( boost::shared_ptr<InformationMetric>( new OccupancyAwareTotalIG() ) );
    }
    else if( metric=="ClassicFrontier" )
    {
      metrics.push_back( boost::shared_ptr<InformationMetric>( new ClassicFrontier() ) );
    }
    else if( metric=="TotalUnknownIG" )
    {
      metrics.push_back( boost::shared_ptr<InformationMetric>( new TotalUnknownIG() ) );
    }
    else if( metric=="UnknownObjectSideFrontier" )
    {
      metrics.push_back( boost::shared_ptr<InformationMetric>( new UnknownObjectSideFrontier() ) );
    }
    else if( metric=="UnknownObjectVolumeIG" )
    {
      metrics.push_back( boost::shared_ptr<InformationMetric>( new UnknownObjectVolumeIG() ) );
    }
    else if( metric=="TotalOccupancyCertainty" )
    {
      metrics.push_back( boost::shared_ptr<InformationMetric>( new TotalOccupancyCertainty() ) );
    }
    else if( metric=="TotalNrOfOccupieds" )
    {
      metrics.push_back( boost::shared_ptr<InformationMetric>( new TotalNrOfOccupieds() ) );
    }
    else if( metric=="TotalNrOfFree" )
    {
      metrics.push_back( boost::shared_ptr<InformationMetric>( new TotalNrOfFree() ) );
    }
    else if( metric=="TotalEntropy" )
    {
      metrics.push_back( boost::shared_ptr<InformationMetric>( new TotalEntropy() ) );
    }
    else if( metric=="TotalNrOfNodes" )
    {
      metrics.push_back( boost::shared_ptr<InformationMetric>( new TotalNrOfNodes() ) );
    }
    metrics.back()->setOcTreeTarget(_info.octree);
  }
  
  // retrieve total tree informations
  BOOST_FOREACH( boost::shared_ptr<InformationMetric> metric, metrics )
  {
    if( metric->isTotalTreeMetric() )
    {
      boost::shared_ptr<TotalTreeMetric> cast = boost::dynamic_pointer_cast<TotalTreeMetric>(metric);
      cast->calculateOnTree(_info.octree);
    }
  }
  //ros::Time start = ros::Time::now();
  // retrieve information for each ray
  for( unsigned int i=0; i<_info.ray_directions->size(); ++i )
  {
    if( i%1000==0 )
      ROS_INFO_STREAM("Retrieving information from ray "<<i<<"/"<<_info.ray_directions->size()<<"...");
    BOOST_FOREACH( boost::shared_ptr<InformationMetric> metric, metrics )
    {
      metric->makeReadyForNewRay();
    }
    // transform direction from camera coordinates to octomap coordinates, given orientation of the view
    Eigen::Vector3d dir_oct = _info.orientations->back()*(*_info.ray_directions)[i];
    octomap::point3d direction( dir_oct.x(), dir_oct.y(), dir_oct.z() );
    if( display_rays_ )
    {
      int display_every = _info.ray_directions->size()/20; // display 20 rays...
      if( i%display_every==0 )
      {
	displayRay( _info.origins->back(), direction );
	ros::Duration(0.005).sleep();
      }
    }
    retrieveInformationForRay( _info.octree, metrics, _info.origins->back(), direction, _info.request->call.min_ray_depth, _info.request->call.max_ray_depth, _info.request->call.occupied_passthrough_threshold, _info.request->call.ray_step_size );
  }
  
  //ros::Duration calc_time = ros::Time::now()-start;
  //ROS_INFO_STREAM("Average calculation time per ray was: "<<calc_time.toSec()/_info.ray_directions->size()<<" seconds.");
  
  _info.response->expected_information.metric_names = _info.request->call.metric_names;
  
  std::vector<double> information;
  BOOST_FOREACH( boost::shared_ptr<InformationMetric> metric, metrics )
  {
    information.push_back( metric->getInformation() );
  }
  _info.response->expected_information.values = information;
  
  return;
}

void OctomapDRServer::retrieveInformationForRay( octomap::OccupancyOcTreeBase<octomap::ColorOcTreeNode>* _octree, std::vector<boost::shared_ptr<InformationMetric> >& _metrics, octomap::point3d& _origin, octomap::point3d& _direction, double _min_ray_depth, double _max_ray_depth, double _occupied_passthrough_threshold, unsigned int _ray_step_size )
{
  octomap::point3d end_point; // calculate endpoint (if any)
  
  double max_range = (_max_ray_depth>0)?_max_ray_depth:10.0;
  double log_odd_passthrough_threshold = std::log(_occupied_passthrough_threshold);
  
  double min_ray_depth = (_min_ray_depth==0)?0.005:_min_ray_depth; //default for min ray depth [m]
  
  bool found_endpoint = _octree->castRay( _origin, _direction, end_point, true, max_range ); // ignore unknown cells
  
  
  /* // deactivated since it seemed it literally never happend
  if( found_endpoint ) // check that endpoint satisfies constraints and move startpoint once if on occupied voxel
  {
    octomap::point3d offset_origin = _origin;
    double max_range_for_offset = max_range;
    bool search = true;
    
    if( end_point==offset_origin ) // try an offset once
    {
      ROS_INFO("View origin is on occupied voxel, attempt to offset it.");
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
	ROS_INFO("Found endpoint doesn't fulfill occupancy threshold, raycasting on.");
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
  }*/
  
  // calculate metrics for all points on ray, calculate points on ray
  //ros::Time time1 = ros::Time::now();
  //if( !found_endpoint ) // *artificial end point* // actually don't care about rays that don't hit anything!
  //  end_point = _origin + _direction*max_range;
  
  if( found_endpoint )
  {
    //ros::Time time2 = ros::Time::now();
    octomap::KeyRay ray;
    _octree->computeRayKeys( _origin, end_point, ray );
    
    //ros::Time time3 = ros::Time::now();
    unsigned int count=0;
    int test;
    for( octomap::KeyRay::iterator it = ray.begin() ; it!=ray.end(); ++it, ++count )
    {
      if( count%_ray_step_size==0 )
      {
	for( unsigned int i=0; i<_metrics.size();++i )
	{
	  _metrics[i]->includeRayMeasurement( *it );
	}
      }
    }
    //ros::Time time4 = ros::Time::now();
    // calculate metric for end point of ray if it exists
  
    octomap::OcTreeKey end_key = _octree->coordToKey(end_point);
    for( unsigned int i=0; i<_metrics.size();++i )
    {
      _metrics[i]->includeEndPointMeasurement( end_key );
    }
  }
  //ros::Time time5 = ros::Time::now();
  
  /*ros::Duration endpoint_test = time2-time1;
  ros::Duration compute_ray = time3-time2;
  ros::Duration include_ray = time4-time3;
  ros::Duration include_end = time5-time3;
  ROS_INFO_STREAM("endpoint_test: "<<endpoint_test.toNSec()<<" ns");
  ROS_INFO_STREAM("compute_ray: "<<compute_ray.toNSec()<<" ns");
  ROS_INFO_STREAM("include_ray: "<<include_ray.toNSec()<<" ns");
  ROS_INFO_STREAM("include_end: "<<include_end.toNSec()<<" ns");*/
  
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
  if( nr_of_measurements_==0 )
    return 0; // no endpoints retrieved - no uncertainty
  
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
  already_counts_=false;
}

void ClassicFrontier::includeRayMeasurement( octomap::OcTreeKey& _to_measure )
{
  if( already_counts_ ) 
    return;
  
  octomap::ColorOcTreeNode* to_measure = octree_->search(_to_measure);
  if( to_measure==NULL )
  {
    if( previous_voxel_free_ )
    {
      ++frontier_voxel_count_;
      already_counts_=true;
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

double EndNodeOccupancySum::getInformation()
{
  return sum_;
}

void EndNodeOccupancySum::makeReadyForNewRay()
{
  // void
}

void EndNodeOccupancySum::includeRayMeasurement( octomap::OcTreeKey& _to_measure )
{
  // void
}

void EndNodeOccupancySum::includeEndPointMeasurement( octomap::OcTreeKey& _to_measure )
{
  octomap::ColorOcTreeNode* to_measure = octree_->search(_to_measure);
  sum_+=to_measure->getOccupancy(); // end points are always occupied
  /*if( to_measure==NULL )
  {
    // unknown
  }
  else
  {
    if( !octree_->isNodeOccupied( to_measure ) )
    {
      sum_+=to_measure->getOccupancy();
    }
  }*/
}

void TotalOccupancyCertainty::calculateOnTree( octomap::OccupancyOcTreeBase<octomap::ColorOcTreeNode>* _octree )
{
  for( octomap::OccupancyOcTreeBase<octomap::ColorOcTreeNode>::leaf_iterator it = _octree->begin_leafs(); it!=_octree->end_leafs(); ++it )
  {
    if( octree_->isNodeOccupied( *it ) )
    {
      sum_ += it->getOccupancy();
    }
  }
}

double TotalOccupancyCertainty::getInformation()
{
  return sum_;
}

void TotalNrOfOccupieds::calculateOnTree( octomap::OccupancyOcTreeBase<octomap::ColorOcTreeNode>* _octree )
{
  for( octomap::OccupancyOcTreeBase<octomap::ColorOcTreeNode>::leaf_iterator it = _octree->begin_leafs(); it!=_octree->end_leafs(); ++it )
  {
    if( octree_->isNodeOccupied( *it ) )
    {
      ++sum_;
    }
  }
}

double TotalNrOfOccupieds::getInformation()
{
  return sum_;
}

void TotalNrOfFree::calculateOnTree( octomap::OccupancyOcTreeBase<octomap::ColorOcTreeNode>* _octree )
{
  for( octomap::OccupancyOcTreeBase<octomap::ColorOcTreeNode>::leaf_iterator it = _octree->begin_leafs(); it!=_octree->end_leafs(); ++it )
  {
    if( !octree_->isNodeOccupied( *it ) )
    {
      ++sum_;
    }
  }
}

double TotalNrOfFree::getInformation()
{
  return sum_;
}

void TotalEntropy::calculateOnTree( octomap::OccupancyOcTreeBase<octomap::ColorOcTreeNode>* _octree )
{
  for( octomap::OccupancyOcTreeBase<octomap::ColorOcTreeNode>::leaf_iterator it = _octree->begin_leafs(); it!=_octree->end_leafs(); ++it )
  {
    double p_occ = it->getOccupancy();
    double p_free = 1-p_occ;
    double vox_ig = -p_occ*log(p_occ)-p_free*log(p_free);
    
    sum_+=vox_ig;
  }
}

double TotalEntropy::getInformation()
{
  return sum_;
}

void TotalNrOfNodes::calculateOnTree( octomap::OccupancyOcTreeBase<octomap::ColorOcTreeNode>* _octree )
{
  sum_ = _octree->size();
}

double TotalNrOfNodes::getInformation()
{
  return sum_;
}


double IgnorantTotalIG::unknown_p_prior_ = 0.2;
double IgnorantTotalIG::unknown_lower_bound_ = 0.15;
double IgnorantTotalIG::unknown_upper_bound_ = 0.8;

double IgnorantTotalIG::getOccupancy( octomap::OcTreeKey& _to_measure )
{
  double p_occ;
  octomap::ColorOcTreeNode* added = octree_->search(_to_measure);
  if( added==NULL )
  {
    p_occ=unknown_p_prior_; // default for unknown
  }
  else
  {
    p_occ = added->getOccupancy();
  }
  return p_occ;
}

double IgnorantTotalIG::calcIG( double _p_occ )
{
  double p_free = 1-_p_occ;
  double vox_ig = -_p_occ*log(_p_occ)-p_free*log(p_free);
  return vox_ig;
}

double IgnorantTotalIG::getInformation()
{
  return ig_;
}

void IgnorantTotalIG::makeReadyForNewRay()
{
  ig_=0;
}

void IgnorantTotalIG::includeRayMeasurement( octomap::OcTreeKey& _to_measure )
{
  includeMeasurement(_to_measure);
}

void IgnorantTotalIG::includeEndPointMeasurement( octomap::OcTreeKey& _to_measure )
{
  includeMeasurement(_to_measure);
}

void IgnorantTotalIG::includeMeasurement( octomap::OcTreeKey& _to_measure )
{
  double p_occ = getOccupancy(_to_measure);
  double vox_ig = calcIG(p_occ);
  ig_ += vox_ig;
}

double OccupancyAwareTotalIG::getInformation()
{
  return ig_;
}

void OccupancyAwareTotalIG::makeReadyForNewRay()
{
  ig_=0;
  p_vis_=1;
}

void OccupancyAwareTotalIG::includeRayMeasurement( octomap::OcTreeKey& _to_measure )
{
  includeMeasurement(_to_measure);
}

void OccupancyAwareTotalIG::includeEndPointMeasurement( octomap::OcTreeKey& _to_measure )
{
  includeMeasurement(_to_measure);
}

void OccupancyAwareTotalIG::includeMeasurement( octomap::OcTreeKey& _to_measure )
{
  double p_occ = getOccupancy(_to_measure);
  double vox_ig = calcIG(p_occ);
  ig_ += p_vis_*vox_ig;
  p_vis_*=p_occ;
}

double TotalUnknownIG::getInformation()
{
  return ig_;
}

void TotalUnknownIG::makeReadyForNewRay()
{
  ig_=0;
  p_vis_=1;
}

void TotalUnknownIG::includeRayMeasurement( octomap::OcTreeKey& _to_measure )
{
  includeMeasurement(_to_measure);
}

void TotalUnknownIG::includeEndPointMeasurement( octomap::OcTreeKey& _to_measure )
{
  includeMeasurement(_to_measure);
}

bool TotalUnknownIG::isFrontierPassRay( double _p_occ )
{
  if( already_counts_ )
    return true;
  
  if( _p_occ<IgnorantTotalIG::unknown_upper_bound_ && _p_occ>IgnorantTotalIG::unknown_lower_bound_ )
  {
    if( previous_voxel_free_ )
    {
      already_counts_=true;
      return true;
    }
  }
  else if(_p_occ<=IgnorantTotalIG::unknown_lower_bound_ ) // if it is free
  {
    previous_voxel_free_ = true;
  }
  else
  {
    previous_voxel_free_ = false;
  }
  return false;
}

void TotalUnknownIG::includeMeasurement( octomap::OcTreeKey& _to_measure )
{
  double p_occ = getOccupancy(_to_measure);
  
  if( isFrontierPassRay(p_occ) )
  {
    double vox_ig = calcIG(p_occ);
    ig_ += p_vis_*vox_ig;
  }
  p_vis_*=p_occ;
}

bool UnknownObjectVolumeIG::hitsUnknownSide()
{
  return hits_unknown_side_;
}

bool UnknownObjectVolumeIG::isUnknownVoxel( double _p_occ )
{
  return _p_occ<IgnorantTotalIG::unknown_upper_bound_ && _p_occ>IgnorantTotalIG::unknown_lower_bound_;
}

void UnknownObjectVolumeIG::updateUnknownSideHit( double _p_occ )
{
  if( isUnknownVoxel( _p_occ ) )
  {
    previous_voxel_unknown_ = true;
  }
  else
  {
    previous_voxel_unknown_ = false;
  }
}

double UnknownObjectVolumeIG::getInformation()
{
  if( !hitsUnknownSide() )
  {
    return 0;
  }
  return ig_;
}

void UnknownObjectVolumeIG::makeReadyForNewRay()
{
  ig_=0;
  p_vis_=1;
  previous_voxel_unknown_=false;
  hits_unknown_side_=false;
}

void UnknownObjectVolumeIG::includeRayMeasurement( octomap::OcTreeKey& _to_measure )
{
  includeMeasurement(_to_measure);
}

void UnknownObjectVolumeIG::includeEndPointMeasurement( octomap::OcTreeKey& _to_measure )
{
  if( previous_voxel_unknown_ )
  {
    hits_unknown_side_ = true;
  }
  includeMeasurement(_to_measure);
}

void UnknownObjectVolumeIG::includeMeasurement( octomap::OcTreeKey& _to_measure )
{
  double p_occ = getOccupancy(_to_measure);
  if( isUnknownVoxel(p_occ) )
  {
    previous_voxel_unknown_ = true;
    double vox_ig = calcIG(p_occ);
    ig_ += p_vis_*vox_ig;
  }
  else
  {
    previous_voxel_unknown_ = false;
    ig_ = 0;
  }
  p_vis_*=p_occ;
}



}