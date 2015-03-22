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


namespace octomap_server {
  
OctomapDRServer::OctomapDRServer(ros::NodeHandle _private_nh)
  :OctomapServer()
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
}

}

