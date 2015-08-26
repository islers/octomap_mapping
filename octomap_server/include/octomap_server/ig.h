#pragma once


#include <octomap_server/OctomapServer.h>
#include <image_geometry/pinhole_camera_model.h>
#include <dense_reconstruction/ViewInformation.h>
#include <dense_reconstruction/ViewInformationRequest.h>
#include <dense_reconstruction/ViewInformationReturn.h>
#include <visualization_msgs/Marker.h>
#include <octomap_server/octomap_dr_server.h>

namespace octomap_server {

class IgnorantTotalIG: public OctomapDRServer::InformationMetric
{
public:
  IgnorantTotalIG():ig_(0.0){};
  inline std::string type()
  {
    return "IgnorantTotalIG";
  }
  
  /**
   * returns the occupancy likelihood for the voxel
   */
  virtual double getOccupancy( octomap::OcTreeKey& _to_measure );
  
  /**
   * calculates the information gain for the voxel (entropy)
   */
  virtual double calcIG( double _p_occ );
  
  virtual double getInformation();
  virtual void makeReadyForNewRay();
  virtual void includeRayMeasurement( octomap::OcTreeKey& _to_measure );
  virtual void includeEndPointMeasurement( octomap::OcTreeKey& _to_measure );
  virtual void informAboutVoidRay();
  
  static double unknown_p_prior_;
  static double unknown_lower_bound_;
  static double unknown_upper_bound_;
  static unsigned int voxels_in_void_ray_; // number of voxels counted in a void ray
  
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
  virtual void informAboutVoidRay();
  
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
  
  /**
   * returns the occupancy likelihodd for the voxel
   */
  virtual double getOccupancy( octomap::OcTreeKey& _to_measure );
  
  double getInformation();
  void makeReadyForNewRay();
  void includeRayMeasurement( octomap::OcTreeKey& _to_measure );
  void includeEndPointMeasurement( octomap::OcTreeKey& _to_measure );
  virtual unsigned int voxelCount(){return 1;} // since it's a simple voxel count, averaging destroy all information...
private:
  unsigned int frontier_voxel_count_;
  bool previous_voxel_free_;
  bool already_counts_;
};

class TotalUnknownIG: public IgnorantTotalIG
{
public:
  TotalUnknownIG():ig_(0.0),current_ray_ig_(0.0),p_vis_(1.0),previous_voxel_free_(false),already_counts_(false),voxels_on_current_ray_(0){};
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
  unsigned int voxels_on_current_ray_;
  
  void includeMeasurement( octomap::OcTreeKey& _to_measure );
};

/** information metric for rays:
 * implements the exploration term proposed in:
 * S. Kriegel, C. Rink, T. Bodenmèller, M. Suppa, "Efficient next-best scan planning for autonomous 3D surface reconstruction of unknown objects", Journal of Real-Time Image Processing 2013
 * - The average entropy in the traversed voxels
 */
class AverageEntropy: public IgnorantTotalIG
{
public:
    AverageEntropy(): totalEntropy_(0), voxelCount_(0){};
    inline std::string type()
    {
        return "AverageEntropy";
    }
    virtual double getInformation();
    virtual void makeReadyForNewRay();
    virtual void includeRayMeasurement( octomap::OcTreeKey& _to_measure );
    virtual void includeEndPointMeasurement( octomap::OcTreeKey& _to_measure );
    virtual unsigned int voxelCount(){ return 1; } // averaging already included!
    
protected:
    double includeMeasurement( octomap::OcTreeKey& _to_measure ); 
    double totalEntropy_;
    unsigned int voxelCount_;
};

/** information metric for rays:
 * implements the area factor from:
 * J.I. Vasquez-Gomez, L.E. Sucar, "Volumetric Next Best View Planning for 3D Object Reconstruction with Positioning Error", Journal of Advanced Robotic Systems 2014
 */
class VasquezGomezAreaFactor: public IgnorantTotalIG
{
public:
    VasquezGomezAreaFactor();
    inline std::string type()
    {
        return "VasquezGomezAreaFactor";
    }
    static unsigned int occplanePercCount;
    virtual double getInformation();
    virtual void makeReadyForNewRay();
    virtual void includeRayMeasurement( octomap::OcTreeKey& _to_measure );
    virtual void includeEndPointMeasurement( octomap::OcTreeKey& _to_measure );
    virtual void informAboutVoidRay();
    virtual unsigned int voxelCount(){ return 1; } //count based ig...
protected:
    /*! Area factor function as defined in the paper.
     * \param x voxel percentage
     * \param alpha desired voxel percentage
     */
    double areaFactor( double x, double alpha );
    /*! Area factor function as defiend in paper with precalculated coefficients
     * \param setId 0:occupied, 1:occplane
     */
    double areaFactor( double x, int setId );
    
    // returns true if the ray was registered
    bool includeMeasurement( octomap::OcTreeKey& _to_measure );
    
    // Computes coefficients for the area factor function with a given alpha
    void setCoefficients( double alpha, double& a_f1, double& b_f1, double& a_f2, double& b_f2, double& c_f2, double& d_f2 );
    
    unsigned int occupiedCount_;
    unsigned int occplaneCount_; // 
    unsigned int unobservedCount_; // we use p_occ = 0.5 for this type
    bool previousVoxelFree_;
    bool rayIsAlreadyRegistered_; // to ensure that every ray is only registered once
    
    
    // settings and precomputation for the are factor calculation
    double desiredOccupiedPercentage_;
    double desiredOccplanePercentage_;
    
    double occupiedVoxPercDes_;
    double occplaneVoxPercDes_;
    double a_f1_occu_, b_f1_occu_;
    double a_f2_occu_, b_f2_occu_, c_f2_occu_, d_f2_occu_;
    double a_f1_occp_, b_f1_occp_;
    double a_f2_occp_, b_f2_occp_, c_f2_occp_, d_f2_occp_;
};

/** information metric for rays:
 * returns occplane percentage as defined by Vasquez et al.
 */
class OccupiedPercentage: public VasquezGomezAreaFactor
{
public:
    inline std::string type()
    {
        return "OccupiedPercentage";
    }
    virtual double getInformation();
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
  bool isUnknown( double _p );
  virtual unsigned int voxelCount(){ return 1; } // since not voxel based...
private:
  unsigned int front_voxel_count_;
  bool previous_voxel_unknown_;
  void includeMeasurement( octomap::OcTreeKey& _to_measure );
};

class UnknownObjectVolumeIG: public IgnorantTotalIG
{
public:
  UnknownObjectVolumeIG():ig_(0.0),current_ray_ig_(0.0),p_vis_(1.0),previous_voxel_unknown_(false),current_ray_voxel_count_(0){};
  inline std::string type()
  {
    return "UnknownObjectVolumeIG";
  }
  
  bool isUnknownVoxel( double _p_occ );
  
  virtual double getInformation();
  virtual void makeReadyForNewRay();
  virtual void includeRayMeasurement( octomap::OcTreeKey& _to_measure );
  virtual void includeEndPointMeasurement( octomap::OcTreeKey& _to_measure );
    
private:
  double ig_;
  double current_ray_ig_;
  double p_vis_;
  bool previous_voxel_unknown_;
  unsigned int current_ray_voxel_count_;
  
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
 * counts how many new object voxels are expected to be in the scene
 */
class ExpectedNewSurfaceVoxels: public IgnorantTotalIG
{
public:
  ExpectedNewSurfaceVoxels():surfaceVoxels_(0), rayAlreadyRegistered_(false){};
  inline std::string type()
  {
    return "ExpectedNewSurfaceVoxels";
  }
  double getInformation();
  void makeReadyForNewRay();
  void includeRayMeasurement( octomap::OcTreeKey& _to_measure );
  void includeEndPointMeasurement( octomap::OcTreeKey& _to_measure );
private:
  unsigned int surfaceVoxels_;
  bool rayAlreadyRegistered_;
  double surfaceVoxelLikelihood_; // sum of occupancy likelihood along ray...
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
  virtual void calculateOnTree( octomap::OccupancyOcTreeBase<octomap::DROcTreeNode>* _octree )=0;
  bool isTotalTreeMetric(){ return true;}
  unsigned int voxelCount(){ return 1; } // we override it like this since we use TotalTreeMetrics to count
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
  void calculateOnTree( octomap::OccupancyOcTreeBase<octomap::DROcTreeNode>* _octree );
  double getInformation();
  void makeReadyForNewRay(){};
  void includeRayMeasurement( octomap::OcTreeKey& _to_measure ){};
  void includeEndPointMeasurement( octomap::OcTreeKey& _to_measure ){};
private:
  double sum_;
};

/** total number of occupieds in the whole tree (EDIT: now used as statistics vehicle)
 */
class TotalNrOfOccupieds: public TotalTreeMetric
{
public:
  TotalNrOfOccupieds();
  inline std::string type()
  {
    return "TotalNrOfOccupieds";
  }
  void calculateOnTree( octomap::OccupancyOcTreeBase<octomap::DROcTreeNode>* _octree );
  double getInformation();
  void makeReadyForNewRay(){};
  void includeRayMeasurement( octomap::OcTreeKey& _to_measure ){};
  void includeEndPointMeasurement( octomap::OcTreeKey& _to_measure ){};
  
  void reset();
  
  static TotalNrOfOccupieds* statisticsEngine(); // just for the sake of misusing...
public:
  unsigned int occupiedCount_;
  unsigned int unknownCount_;
  unsigned int emptyCount_;
  unsigned int occludedCount_;
  unsigned int knownVoxelCount_;
  
  bool statisticsCreated_; // whether the stats have been created or not
  double occupiedBorder_; // above and including this occupancy value a voxel is counted as occupied
  double emptyBorder_; // below this occupancy value a voxel is counted as empty
protected:
  // recursive function call for tree iteration including depth of the nodes
  unsigned int getNrOfOccupieds( octomap::DROcTreeNode* node, int depth );
  // returns how many children a node at depth "depth" has
  unsigned int numberOfChildren( int depth );
  // whether a node is occupied
  bool isOccupied( octomap::DROcTreeNode* node );
  // whether a node is free
  bool isEmpty( octomap::DROcTreeNode* node );
  // whether a voxel is occluded
  bool isOccluded( octomap::DROcTreeNode* node );
};
typedef TotalNrOfOccupieds TotalTreeMetricStatisticEngine;

/** total number of free nodes in the whole tree
 */
class TotalNrOfFree: public TotalNrOfOccupieds
{
public:
  TotalNrOfFree():sum_(0){};
  inline std::string type()
  {
    return "TotalNrOfFree";
  }
  void calculateOnTree( octomap::OccupancyOcTreeBase<octomap::DROcTreeNode>* _octree );
  double getInformation();
  void makeReadyForNewRay(){};
  void includeRayMeasurement( octomap::OcTreeKey& _to_measure ){};
  void includeEndPointMeasurement( octomap::OcTreeKey& _to_measure ){};
  
private:
  unsigned int sum_;
};

class TotalNrOfUnmarked: public TotalNrOfOccupieds
{
public:
    TotalNrOfUnmarked():sum_(0){};
  inline std::string type()
  {
    return "TotalNrOfUnmarked";
  }
  void calculateOnTree( octomap::OccupancyOcTreeBase<octomap::DROcTreeNode>* _octree )
  {
      TotalNrOfOccupieds::calculateOnTree(_octree);
    /*TotalTreeMetricStatisticEngine* engine = TotalNrOfOccupieds::statisticsEngine();
    engine->calculateOnTree(_octree);
    sum_ = engine->unknownCount_;*/
  };
  virtual double getInformation(){ return unknownCount_; };
  virtual void makeReadyForNewRay(){};
  virtual void includeRayMeasurement( octomap::OcTreeKey& _to_measure ){};
  virtual void includeEndPointMeasurement( octomap::OcTreeKey& _to_measure ){};
protected:
    unsigned int sum_;
};

class TotalNrOfOccluded: public TotalNrOfOccupieds
{
public:
    TotalNrOfOccluded():sum_(0){};
  inline std::string type()
  {
    return "TotalNrOfOccluded";
  }
  void calculateOnTree( octomap::OccupancyOcTreeBase<octomap::DROcTreeNode>* _octree )
  {
      TotalNrOfOccupieds::calculateOnTree(_octree);
    /*TotalTreeMetricStatisticEngine* engine = TotalNrOfOccupieds::statisticsEngine();
    engine->calculateOnTree(_octree);
    sum_ = engine->occludedCount_;*/
  };
  virtual double getInformation(){ return occludedCount_; };
  virtual void makeReadyForNewRay(){};
  virtual void includeRayMeasurement( octomap::OcTreeKey& _to_measure ){};
  virtual void includeEndPointMeasurement( octomap::OcTreeKey& _to_measure ){};
protected:
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
  void calculateOnTree( octomap::OccupancyOcTreeBase<octomap::DROcTreeNode>* _octree );
  double getInformation();
  void makeReadyForNewRay(){};
  void includeRayMeasurement( octomap::OcTreeKey& _to_measure ){};
  void includeEndPointMeasurement( octomap::OcTreeKey& _to_measure ){};
private:
  double sum_;
};

/** total number of occupieds in the whole tree
 */
class TotalNrOfNodes: public TotalNrOfOccupieds
{
public:
  TotalNrOfNodes():sum_(0){};
  inline std::string type()
  {
    return "TotalNrOfNodes";
  }
  void calculateOnTree( octomap::OccupancyOcTreeBase<octomap::DROcTreeNode>* _octree );
  double getInformation();
  void makeReadyForNewRay(){};
  void includeRayMeasurement( octomap::OcTreeKey& _to_measure ){};
  void includeEndPointMeasurement( octomap::OcTreeKey& _to_measure ){};
private:
  unsigned int sum_;
};

}

