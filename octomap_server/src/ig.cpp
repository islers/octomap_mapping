#include <octomap_server/ig.h>
#include "boost/foreach.hpp"

namespace octomap_server {

double NrOfUnknownVoxels::getInformation()
{
  return unknown_voxel_count;
}

void NrOfUnknownVoxels::makeReadyForNewRay()
{
  // void
}

void NrOfUnknownVoxels::includeRayMeasurement( octomap::DROcTreeNode* added )
{
  //octomap::DROcTreeNode* added = octree_->search(_to_measure);
  if( added==NULL )
    ++unknown_voxel_count;
  else if( !added->hasMeasurement() )
      ++unknown_voxel_count;
}

void NrOfUnknownVoxels::includeEndPointMeasurement( octomap::DROcTreeNode* to_measure )
{
  // void - an end point must be occupied and thus can't be unknown
}

double ExpectedNewSurfaceVoxels::getInformation()
{
  return surfaceVoxels_;
}

void ExpectedNewSurfaceVoxels::makeReadyForNewRay()
{
  rayAlreadyRegistered_ = false;
}

void ExpectedNewSurfaceVoxels::includeRayMeasurement( octomap::DROcTreeNode* added )
{
  //octomap::DROcTreeNode* added = octree_->search(_to_measure);
  if( added==NULL )
    ++surfaceVoxels_;
  else if( !added->hasMeasurement() )
      ++surfaceVoxels_;
}

void ExpectedNewSurfaceVoxels::includeEndPointMeasurement( octomap::DROcTreeNode* to_measure )
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

void AverageUncertainty::includeRayMeasurement( octomap::DROcTreeNode* _to_measure )
{
  includeMeasurement(_to_measure);
}

void AverageUncertainty::includeEndPointMeasurement( octomap::DROcTreeNode* _to_measure )
{
  includeMeasurement(_to_measure);
}

void AverageUncertainty::includeMeasurement( octomap::DROcTreeNode* added )
{
  double occupany_likelihood;
  //octomap::DROcTreeNode* added = octree_->search(_to_measure);
  if( added==NULL )
  {
    occupany_likelihood=0.5; // default for unknown
  }
  else if( !added->hasMeasurement() )
  {
      occupany_likelihood = 0.5; // default for unknown
  }
  else
  {
    occupany_likelihood = added->getOccupancy();
  }
  certainty_sum_+=std::fabs(0.5-occupany_likelihood);
  ++nr_of_measurements_;
}

void AverageEndPointUncertainty::includeRayMeasurement( octomap::DROcTreeNode* added )
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

void UnknownObjectSideFrontier::includeRayMeasurement( octomap::DROcTreeNode* added )
{
  includeMeasurement(added);
}

void UnknownObjectSideFrontier::includeEndPointMeasurement( octomap::DROcTreeNode* _to_measure )
{
  includeMeasurement(_to_measure);
}

bool UnknownObjectSideFrontier::isUnknown( double _p )
{
    double p_occ = _p;
    if( p_occ<IgnorantTotalIG::unknown_upper_bound_ && p_occ>IgnorantTotalIG::unknown_lower_bound_ )
    {
        return true;
    }
    return false;
}

void UnknownObjectSideFrontier::includeMeasurement( octomap::DROcTreeNode* added )
{
  //octomap::DROcTreeNode* added = octree_->search(_to_measure);
  if( added==NULL )
  {
    previous_voxel_unknown_=true;
    return;
  }
  else if( !added->hasMeasurement() )
  {
      previous_voxel_unknown_=true;
  }
  double p_occ = added->getOccupancy();
  if( isUnknown(p_occ) )
  {
      previous_voxel_unknown_=true;
  }
  else
  {
    if( p_occ>IgnorantTotalIG::unknown_upper_bound_ && previous_voxel_unknown_ ) // object side frontier!
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

void UnknownObjectVolumeFrontier::includeRayMeasurement( octomap::DROcTreeNode* added )
{
  includeMeasurement(added);
}

void UnknownObjectVolumeFrontier::includeEndPointMeasurement( octomap::DROcTreeNode* _to_measure )
{
  includeMeasurement(_to_measure);
}

void UnknownObjectVolumeFrontier::includeMeasurement( octomap::DROcTreeNode* to_measure )
{
  //octomap::DROcTreeNode* to_measure = octree_->search(_to_measure);
  if( to_measure==NULL )
  {
    ++running_count_;
  }
  else if( !to_measure->hasMeasurement() )
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

double ClassicFrontier::getOccupancy( octomap::DROcTreeNode* added )
{
  double p_occ;
  //octomap::DROcTreeNode* added = octree_->search(_to_measure);
  if( added==NULL )
  {
    p_occ=IgnorantTotalIG::unknown_p_prior_; // default for unknown
  }
  else if( !added->hasMeasurement() )
  {
    p_occ=IgnorantTotalIG::unknown_p_prior_; // default for unknown
  }
  else
  {
    p_occ = added->getOccupancy();
  }
  return p_occ;
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

void ClassicFrontier::includeRayMeasurement( octomap::DROcTreeNode* to_measure )
{
  if( already_counts_ ) 
    return;
  
  //octomap::DROcTreeNode* to_measure = octree_->search(_to_measure);
  double occ = getOccupancy(to_measure);
  
  if( to_measure==NULL || (occ>IgnorantTotalIG::unknown_lower_bound_ && occ<IgnorantTotalIG::unknown_upper_bound_) )
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
    if( occ<=IgnorantTotalIG::unknown_lower_bound_ ) // frontier
    {
      previous_voxel_free_ = true;
    }
    else
    {
      previous_voxel_free_ = false;
    }
  }
}

void ClassicFrontier::includeEndPointMeasurement( octomap::DROcTreeNode* to_measure )
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

void EndNodeOccupancySum::includeRayMeasurement( octomap::DROcTreeNode* to_measure )
{
  // void
}

void EndNodeOccupancySum::includeEndPointMeasurement( octomap::DROcTreeNode* to_measure )
{
  //octomap::DROcTreeNode* to_measure = octree_->search(_to_measure);
  if( to_measure!=NULL )
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

void TotalOccupancyCertainty::calculateOnTree( octomap::OccupancyOcTreeBase<octomap::DROcTreeNode>* _octree )
{
  for( octomap::OccupancyOcTreeBase<octomap::DROcTreeNode>::leaf_iterator it = _octree->begin_leafs(); it!=_octree->end_leafs(); ++it )
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

TotalNrOfOccupieds::TotalNrOfOccupieds()
: occupiedCount_(0)
, unknownCount_(0)
, emptyCount_(0)
, occludedCount_(0)
, knownVoxelCount_(0)
, statisticsCreated_(false)
, occupiedBorder_(0.6)
, emptyBorder_(0.4)
{
}

void TotalNrOfOccupieds::calculateOnTree( octomap::OccupancyOcTreeBase<octomap::DROcTreeNode>* _octree )
{
  int depth = 1;
  
  octomap::DROcTreeNode* root = octree_->getRoot();
  
  if( !root->hasChildren() )
  {
      if( isOccupied(root) )
          occupiedCount_ = numberOfChildren(depth);
      else if( isEmpty(root) )
          emptyCount_ = numberOfChildren(depth);
      else
          unknownCount_ = numberOfChildren(depth);
      return;
  }
  
  for(int i=0;i<8;++i)
  {
      if( root->childExists(i) )
      {
          octomap::DROcTreeNode* child = root->getChild(i);
          getNrOfOccupieds( child, depth+1 );
      }
  }
  statisticsCreated_ = true;
  
  /*for( octomap::OccupancyOcTreeBase<octomap::DROcTreeNode>::leaf_iterator it = _octree->begin_leafs(); it!=_octree->end_leafs(); ++it )
  {
    if( octree_->isNodeOccupied( *it ) )
    {
      ++sum_;
    }
  }*/
}

TotalNrOfOccupieds* TotalNrOfOccupieds::statisticsEngine()
{
    static TotalNrOfOccupieds instance;
    instance.reset();
    return &instance;
}

unsigned int TotalNrOfOccupieds::getNrOfOccupieds( octomap::DROcTreeNode* node, int depth )
{
    
  if( !node->hasChildren() )
  {
      unsigned int children = numberOfChildren(depth);
      
      knownVoxelCount_ += children;
      if( isOccupied(node) )
      {
          occupiedCount_ += children;
      }
      else if( isEmpty(node) )
      {
          emptyCount_ += children;
      }
      else
      {
          unknownCount_ += children;
      }
      if( isOccluded(node) )
      {
          occludedCount_ += children;
      }
      return 0;
  }
  
  for(int i=0;i<8;++i)
  {
      if( node->childExists(i) )
      {
          octomap::DROcTreeNode* child = node->getChild(i);
          getNrOfOccupieds( child, depth+1 );
      }
  }
  return 0;
}

unsigned int TotalNrOfOccupieds::numberOfChildren( int depth )
{
    int shifts = 16-depth;
    unsigned int children = 1;
    
    for( int i=0;i<shifts;++i )
    {
        children = children << 3; // multiply with 8...
    }
    
    return children;
}

bool TotalNrOfOccupieds::isOccupied( octomap::DROcTreeNode* node )
{
    double occ = node->getOccupancy();
    return (occ >= occupiedBorder_);
}

bool TotalNrOfOccupieds::isEmpty( octomap::DROcTreeNode* node )
{
    double occ = node->getOccupancy();
    return (occ < emptyBorder_);
}

bool TotalNrOfOccupieds::isOccluded( octomap::DROcTreeNode* node )
{
    return !node->hasMeasurement() && (node->occDist()!=-1);
}

double TotalNrOfOccupieds::getInformation()
{
  return occupiedCount_;
}

void TotalNrOfOccupieds::reset()
{
    occupiedCount_=0;
    unknownCount_=0;
    emptyCount_=0;
    occludedCount_=0;
    knownVoxelCount_=0;
    statisticsCreated_=false;
}

void TotalNrOfFree::calculateOnTree( octomap::OccupancyOcTreeBase<octomap::DROcTreeNode>* _octree )
{
  TotalNrOfOccupieds::calculateOnTree(_octree);
  /*TotalTreeMetricStatisticEngine* engine = TotalNrOfOccupieds::statisticsEngine();
  engine->calculateOnTree(_octree);
  sum_ = engine->emptyCount_;
  */
  /*for( octomap::OccupancyOcTreeBase<octomap::DROcTreeNode>::leaf_iterator it = _octree->begin_leafs(); it!=_octree->end_leafs(); ++it )
  {
    if( !octree_->isNodeOccupied( *it ) )
    {
      ++sum_;
    }
  }*/
}

double TotalNrOfFree::getInformation()
{
  return emptyCount_;
}

void TotalEntropy::calculateOnTree( octomap::OccupancyOcTreeBase<octomap::DROcTreeNode>* _octree )
{
  for( octomap::OccupancyOcTreeBase<octomap::DROcTreeNode>::leaf_iterator it = _octree->begin_leafs(); it!=_octree->end_leafs(); ++it )
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

void TotalNrOfNodes::calculateOnTree( octomap::OccupancyOcTreeBase<octomap::DROcTreeNode>* _octree )
{
    TotalNrOfOccupieds::calculateOnTree(_octree);
   /* TotalTreeMetricStatisticEngine* engine = TotalNrOfOccupieds::statisticsEngine();
    engine->calculateOnTree(_octree);
    sum_ = engine->knownVoxelCount_;*/
  //sum_ = _octree->size();
}

double TotalNrOfNodes::getInformation()
{
  return knownVoxelCount_;
}

// ATTENTION: It is extremely important, that these values are matched to the octomap update likelihoods!
double IgnorantTotalIG::unknown_p_prior_ = 0.5; //0.3
double IgnorantTotalIG::unknown_lower_bound_ = 0.2; //0.15
double IgnorantTotalIG::unknown_upper_bound_ = 0.8; //0.8
unsigned int IgnorantTotalIG::voxels_in_void_ray_ = 1.0/0.01; // approximated with max ray range/voxel size -> overestimated number of voxels on ray - therefore underestimates the average ig

double IgnorantTotalIG::getOccupancy( octomap::DROcTreeNode* added )
{
  double p_occ;
  //octomap::DROcTreeNode* added = octree_->search(_to_measure);
  if( added==NULL )
  {
    p_occ=unknown_p_prior_; // default for unknown
  }
  else if( !added->hasMeasurement() )
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
  if(_p_occ==0 || p_free==0)
  {
      return 0;
  }
  double vox_ig = -_p_occ*log(_p_occ)-p_free*log(p_free);
  return vox_ig;
}

double IgnorantTotalIG::getInformation()
{
  return ig_;
}

void IgnorantTotalIG::makeReadyForNewRay()
{
  //ig_=0;
}

void IgnorantTotalIG::includeRayMeasurement( octomap::DROcTreeNode* to_measure )
{
  ++traversedVoxelCount_;
  includeMeasurement(to_measure);
}

void IgnorantTotalIG::includeEndPointMeasurement( octomap::DROcTreeNode* to_measure )
{
  ++traversedVoxelCount_;
  includeMeasurement(to_measure);
}

void IgnorantTotalIG::includeMeasurement( octomap::DROcTreeNode* to_measure )
{
  double p_occ = getOccupancy(to_measure);
  double vox_ig = calcIG(p_occ);
  ig_ += vox_ig;
}

void IgnorantTotalIG::informAboutVoidRay()
{
  traversedVoxelCount_+=voxels_in_void_ray_;
  ig_ += voxels_in_void_ray_ * calcIG(unknown_p_prior_); // information in void ray...
}

double OccupancyAwareTotalIG::getInformation()
{
  return ig_;
}

void OccupancyAwareTotalIG::makeReadyForNewRay()
{
  //ig_=0;
  p_vis_=1;
}

void OccupancyAwareTotalIG::includeRayMeasurement( octomap::DROcTreeNode* to_measure )
{
  includeMeasurement(to_measure);
}

void OccupancyAwareTotalIG::includeEndPointMeasurement( octomap::DROcTreeNode* to_measure )
{
  includeMeasurement(to_measure);
}

void OccupancyAwareTotalIG::includeMeasurement( octomap::DROcTreeNode* to_measure )
{
  traversedVoxelCount_+=1;
  double p_occ = getOccupancy(to_measure);
  double vox_ig = calcIG(p_occ);
  ig_ += p_vis_*vox_ig;
  p_vis_*=p_occ;
}

void OccupancyAwareTotalIG::informAboutVoidRay()
{
  traversedVoxelCount_+=IgnorantTotalIG::voxels_in_void_ray_;
  // no approximation needed, can be exactly calculated using the geometric series formula
  ig_ += calcIG(unknown_p_prior_)/(1-unknown_p_prior_); // information in void ray...
}

double TotalUnknownIG::getInformation()
{
  return ig_;
}

void TotalUnknownIG::makeReadyForNewRay()
{
  //ig_=0;
  current_ray_ig_=0;
  p_vis_=1;
  previous_voxel_free_=false;
  already_counts_=false;
  voxels_on_current_ray_=0;
}

void TotalUnknownIG::includeRayMeasurement( octomap::DROcTreeNode* to_measure )
{
  includeMeasurement(to_measure);
}

void TotalUnknownIG::includeEndPointMeasurement( octomap::DROcTreeNode* to_measure )
{
  double p_occ = getOccupancy(to_measure);
  double vox_ig = calcIG(p_occ);
  
  if( isUnknown(p_occ) )
  {
    current_ray_ig_+=p_vis_*vox_ig;
    voxels_on_current_ray_+=1;
  }
  
  if( isFrontierPassRay(p_occ) )
  {
    ig_ += current_ray_ig_;
    traversedVoxelCount_+=voxels_on_current_ray_;
  }
}

bool TotalUnknownIG::isFrontierPassRay( double _p_occ )
{
  if( already_counts_ )
    return true;
  
  if( isUnknown(_p_occ) )
  {
    if( previous_voxel_free_ )
    {
      already_counts_=true;
      return true;
    }
  }
  return false;
}

bool TotalUnknownIG::isUnknown( double _p_occ )
{
  if( _p_occ<IgnorantTotalIG::unknown_upper_bound_ && _p_occ>IgnorantTotalIG::unknown_lower_bound_ )
  {
    return true;
  }
  else
    return false;
}

void TotalUnknownIG::includeMeasurement( octomap::DROcTreeNode* added )
{
  //octomap::DROcTreeNode* added = octree_->search(_to_measure);
  double p_occ;
  if( added==NULL )
  {
      p_occ = IgnorantTotalIG::unknown_p_prior_;
  }
  else if( !added->hasMeasurement() )
  {
      already_counts_ = true; // hits on occluded voxel!
      p_occ = IgnorantTotalIG::unknown_p_prior_;
  }
  else
  {
      p_occ = added->getOccupancy();
  }
  
  double vox_ig = calcIG(p_occ);
  
  if( isUnknown(p_occ) )
  {
    current_ray_ig_+=p_vis_*vox_ig;
    voxels_on_current_ray_+=1;
    
    previous_voxel_free_ = false;
    
  }
  p_vis_*=p_occ;
}


double AverageEntropy::getInformation()
{
  if( totalVoxelCount_==0 )
  {
    return 0;
  }
  return totalIG_/totalVoxelCount_;
}

void AverageEntropy::makeReadyForNewRay()
{
    totalEntropy_=0;
    voxelCount_=0;
}

void AverageEntropy::includeRayMeasurement( octomap::DROcTreeNode* to_measure )
{
    includeMeasurement(to_measure);
}

// only include rays that hit an occupied!
void AverageEntropy::includeEndPointMeasurement( octomap::DROcTreeNode* to_measure )
{
    traversedVoxelCount_+=1;
    double occ = getOccupancy(to_measure);
    double h = calcIG(occ);
    totalEntropy_ += h;
    ++voxelCount_;
    
    if( occ>=0.8 )
    {
        totalIG_+=totalEntropy_;
        totalVoxelCount_+=voxelCount_;
    }
}

double AverageEntropy::includeMeasurement( octomap::DROcTreeNode* to_measure )
{
    traversedVoxelCount_+=1;
    double occ = getOccupancy(to_measure);
    double h = calcIG(occ);
    totalEntropy_ += h;
    ++voxelCount_;
}

VasquezGomezAreaFactor::VasquezGomezAreaFactor()
: occupiedCount_(0)
, occplaneCount_(0)
, unobservedCount_(0)
, previousVoxelFree_(true)
, rayIsAlreadyRegistered_(false)
, desiredOccupiedPercentage_(0.2)
, desiredOccplanePercentage_(0.8)
{
    desiredOccupiedPercentage_ = 0.2;
    desiredOccplanePercentage_ = 0.8;
    
    setCoefficients(desiredOccupiedPercentage_,
                    a_f1_occu_,
                    b_f1_occu_,
                    a_f2_occu_,
                    b_f2_occu_,
                    c_f2_occu_,
                    d_f2_occu_ );
    setCoefficients(desiredOccplanePercentage_,
                    a_f1_occp_,
                    b_f1_occp_,
                    a_f2_occp_,
                    b_f2_occp_,
                    c_f2_occp_,
                    d_f2_occp_ );
    noKnownVoxelSoFar_ = true;
}
    
unsigned int VasquezGomezAreaFactor::occplanePercCount=0;

double VasquezGomezAreaFactor::getInformation()
{
    // print statistics...
    ROS_INFO_STREAM("VasquezGomez stats:");
    ROS_INFO_STREAM("Occupied voxels hit: "<<occupiedCount_);
    ROS_INFO_STREAM("Occplane voxels hit: "<<occplaneCount_);
    ROS_INFO_STREAM("Unobserved casts: "<<unobservedCount_);
    
    // using formulation by Vasquez-Gomez et al. to calculate information
    double voxelSum = occupiedCount_ + occplaneCount_ + unobservedCount_;
    
    if( voxelSum==0 )
        return 0;
    
    double occupiedVoxPerc = occupiedCount_ / voxelSum;
    double occplaneVoxPerc = occplaneCount_ / voxelSum;
    
    /*if( occplaneVoxPerc>0.2 )
    {
        occplanePercCount++;
        ROS_ERROR_STREAM("More than 20% occplane voxels! "<<occplaneVoxPerc<<"% occplane voxels are predicted.");
        ROS_ERROR_STREAM("This happened  "<<occplanePercCount<<" times so far.");
    }*/
        
    return areaFactor(occupiedVoxPerc,0) + areaFactor(occplaneVoxPerc,1);
}

void VasquezGomezAreaFactor::makeReadyForNewRay()
{
    previousVoxelFree_ = true;
    rayIsAlreadyRegistered_ = false;
    noKnownVoxelSoFar_ = true;
}

void VasquezGomezAreaFactor::includeRayMeasurement( octomap::DROcTreeNode* to_measure )
{
    includeMeasurement(to_measure);
}

void VasquezGomezAreaFactor::includeEndPointMeasurement( octomap::DROcTreeNode* to_measure )
{
    if( !includeMeasurement(to_measure) ) // if ray wasn't registered to the last voxel on it, it's an unmarked one
    {
        unobservedCount_+=1;
    }
}

void VasquezGomezAreaFactor::informAboutVoidRay()
{
    // void ray corresponds to an unmarked one...
    
    unobservedCount_+=1;
}

double VasquezGomezAreaFactor::areaFactor( double x, double alpha )
{
    double a_1, b_1, a_2, b_2, c_2, d_2;
    setCoefficients(alpha,a_1,b_1,a_2,b_2,c_2,d_2);
    
    double x2 = x*x;
    double x3 = x*x2;
    
    if( x<= alpha )
    {
        return a_1*x3 + b_1*x2;
    }
    else
    {
        return a_2*x3 + b_2*x2 + c_2*x + d_2;
    }
}

double VasquezGomezAreaFactor::areaFactor( double x, int setId )
{
    double x2 = x*x;
    double x3 = x*x2;
    
    switch(setId)
    {
        case 0: // occupied
        {
            if( x<=desiredOccupiedPercentage_ )
            {
                return a_f1_occu_*x3 + b_f1_occu_*x2;
            }
            else
            {
                return a_f2_occu_*x3 + b_f2_occu_*x2 + c_f2_occu_*x + d_f2_occu_;
            }
        }
        case 1: // occplane
        {
            if( x<=desiredOccplanePercentage_ )
            {
                return a_f1_occp_*x3 + b_f1_occp_*x2;
            }
            else
            {
                return a_f2_occp_*x3 + b_f2_occp_*x2 + c_f2_occp_*x + d_f2_occp_;
            }
        }
    }
}


bool VasquezGomezAreaFactor::includeMeasurement( octomap::DROcTreeNode* added )
{
    if(rayIsAlreadyRegistered_) // register each ray only once
    {
        return true;
    }
    
    //octomap::DROcTreeNode* added = octree_->search(_to_measure);
    double occ;
    if( added==NULL )
    {
        occ=unknown_p_prior_; // default for unknown
        if( noKnownVoxelSoFar_ )
        {
            previousVoxelFree_ = true; // treat unknown at beginning of ray, starting from sensor position as p_free
        }
        else
        {
            previousVoxelFree_ = false; // treat unknown at beginning of ray, starting from sensor position as p_free
        }

        return false;
    }
    else
    {
        occ = added->getOccupancy();
    }
    
    if( !added->hasMeasurement() && added->occDist()!=-1 )
    {
        ++occplaneCount_;
        rayIsAlreadyRegistered_ = true;
        previousVoxelFree_ = false;
        //ROS_ERROR_STREAM("Got an occplane voxel!");
        return true;
    }
    else
    {
        noKnownVoxelSoFar_=false;
    }
    
    if( occ >= unknown_upper_bound_ ) // voxel is occupied
    {
        if( previousVoxelFree_ ) // occupied voxels should only be registered as occupied if they are actually expected to be visible (and not hit from the backside...)
        {
            ++occupiedCount_;
        }
        previousVoxelFree_ = false;
        rayIsAlreadyRegistered_ = true;
        return true;
    }
    else if( occ > unknown_lower_bound_ ) // voxel is unknown
    {
        previousVoxelFree_ = false;
        
    }
    else
    {
        previousVoxelFree_ = true;
        // void, empty voxels are not of interest
    }
    return false;
}

void VasquezGomezAreaFactor::setCoefficients( double alpha, double& a_f1, double& b_f1, double& a_f2, double& b_f2, double& c_f2, double& d_f2 )
{
     double alpha2 = alpha*alpha;
     double alpha3 = alpha*alpha2;
     double a_m1 = alpha-1;
     double a_m1_3 = a_m1*a_m1*a_m1;
     
     a_f1 = -(2/alpha3);
     b_f1 = (3/alpha2);
     a_f2 = -(2/a_m1_3);
     b_f2 = ((3*alpha+3)/a_m1_3);
     c_f2 = -(6*alpha/a_m1_3);
     d_f2 = ((3*alpha-1)/a_m1_3);
}

double DepthHypothesis::getInformation()
{
  return ig_;
}

void DepthHypothesis::makeReadyForNewRay()
{
}

void DepthHypothesis::includeRayMeasurement( octomap::DROcTreeNode* to_measure )
{
  includeMeasurement(to_measure);
}

void DepthHypothesis::includeEndPointMeasurement( octomap::DROcTreeNode* to_measure )
{
  includeMeasurement(to_measure);
}

void DepthHypothesis::includeMeasurement( octomap::DROcTreeNode* added )
{
  traversedVoxelCount_+=1;
    //octomap::DROcTreeNode* added = octree_->search(_to_measure);
    
    if(added!=NULL)
    {
        double dist = added->occDist();
        if( !added->hasMeasurement() && dist>0 )
        {
            ig_+=dist;
        }
    }
    return;
}

void DepthHypothesis::informAboutVoidRay()
{
    //void
}

double DepthHypothesis::getObjectLikelihood( double voxelDist )
{
    // two phase linear model
    
    double maxDepth = 20; // [cm]
    double dropOffDist = 3; // [cm] dist from which likelihood starts to drop faster
    double closeDropRate = 0.01; // [%/cm]
    double voxelLength = 1; // [cm]
    
    double farDropRate = (1-closeDropRate*dropOffDist)/(maxDepth-dropOffDist);
    
    double metricDist = voxelLength*voxelDist;
    
    if( metricDist<=dropOffDist )
    {
        return (1-metricDist*closeDropRate);
    }
    else if( metricDist<=maxDepth )
    {
        return (1-dropOffDist*closeDropRate-farDropRate*(metricDist-dropOffDist));
    }
    else
        return 0;
}
double DepthHypothesis::getOccupancy( octomap::DROcTreeNode* added )
{
  double p_occ;
  //octomap::DROcTreeNode* added = octree_->search(_to_measure);
  
  if( added==NULL )
  {
      p_occ=unknown_p_prior_; // default for unobserved
  }
  else if( !added->hasMeasurement() )
  {
      p_occ=unknown_p_prior_; // default for unobserved
  }
  else
  {
    double dist = added->occDist();
    
    if( dist!=-1 )
    {
        p_occ=getObjectLikelihood(dist);
        passesOccluded_ = true;
    }
    else
    {
        p_occ = added->getOccupancy();
    }
  }
  return p_occ;
}

double OccupiedPercentage::getInformation()
{
    
    double voxelSum = occupiedCount_ + occplaneCount_;// + unobservedCount_;

    if( voxelSum<30 ) // 40 is arbitrary - just to ensure that the percentage is meaningful
        return 0;

    return occupiedCount_ / voxelSum;
}

bool UnknownObjectVolumeIG::isUnknownVoxel( double _p_occ )
{
  return _p_occ<IgnorantTotalIG::unknown_upper_bound_ && _p_occ>IgnorantTotalIG::unknown_lower_bound_;
}

double UnknownObjectVolumeIG::getInformation()
{
  return ig_;
}

void UnknownObjectVolumeIG::makeReadyForNewRay()
{
  //ig_=0;
  current_ray_ig_=0;
  p_vis_=1;
  previous_voxel_unknown_=false;
  current_ray_voxel_count_=0;
}

void UnknownObjectVolumeIG::includeRayMeasurement( octomap::DROcTreeNode* to_measure )
{
  includeMeasurement(to_measure);
}

void UnknownObjectVolumeIG::includeEndPointMeasurement( octomap::DROcTreeNode* added )
{  
  if( previous_voxel_unknown_ )
  {
    //octomap::DROcTreeNode* added = octree_->search(_to_measure);
    
    if( added==NULL )
    {
        return;
    }
    else if( !added->hasMeasurement() || added->getOccupancy()<0.8 )
    {
        return;
    }
    
    double p_occ = getOccupancy(added);
    double vox_ig = calcIG(p_occ);
    current_ray_ig_ += p_vis_*vox_ig;
    current_ray_voxel_count_+=1;
    
    traversedVoxelCount_+=current_ray_voxel_count_;
    ig_+=current_ray_ig_;
  }
}

void UnknownObjectVolumeIG::includeMeasurement( octomap::DROcTreeNode* _to_measure )
{
  double p_occ = getOccupancy(_to_measure);
  if( isUnknownVoxel(p_occ) )
  {
    previous_voxel_unknown_ = true;
    double vox_ig = calcIG(p_occ);
    current_ray_ig_ += p_vis_*vox_ig;
    current_ray_voxel_count_ += 1;
  }
  else
  {
    previous_voxel_unknown_ = false;
    current_ray_ig_ = 0;
    current_ray_voxel_count_ = 0;
  }
  p_vis_*=p_occ;
}



}