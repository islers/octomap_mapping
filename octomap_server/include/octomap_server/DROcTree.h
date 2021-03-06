#pragma once

#include <octomap/OcTreeNode.h>
#include <octomap/OccupancyOcTreeStereo.h>

namespace octomap
{
    // node definition
    /*class DROcTreeNode: public OcTreeNode
    {
    public:
        DROcTreeNode();
        DROcTreeNode(const DROcTreeNode& rhs) : OcTreeNode(rhs), _occDist(rhs._occDist), _hasNoMeasurement(rhs._hasNoMeasurement) {}
        
        
        bool operator==(const DROcTreeNode& rhs) const{
            std::cout<<"\noperator== of DROcTreeNode called! yay!\n**************************************************\n";
            return false; // suppress pruning (TODO: weird pruning behaviour
        return (rhs.value == value && rhs._occDist == _occDist && rhs._hasNoMeasurement==_hasNoMeasurement);
        }
        
        // children
        inline DROcTreeNode* getChild(unsigned int i) {
            return static_cast<DROcTreeNode*> (OcTreeNode::getChild(i));
        }
        inline const DROcTreeNode* getChild(unsigned int i) const {
            return static_cast<const DROcTreeNode*> (OcTreeNode::getChild(i));
        }

        bool createChild(unsigned int i) {
            if (children == NULL) allocChildren();
            children[i] = new DROcTreeNode();
            return true;
        }

        // update occupancy and distances of inner nodes 
        inline void updateOccupancyChildren() {      
            this->setLogOdds(this->getMaxChildLogOdds());  // conservative
            _occDist = this->getMinChildOccDist();
            _hasNoMeasurement = !this->childHasMeasurement();
        }
        
        // returns the min child occ dist.
        double getMinChildOccDist();
        // whether or not a measurement within the voxel is available
        bool childHasMeasurement();
        
        double occDist();
        // sets occDist if it's smaller than the previous value
        void updateOccDist( double occDist );
        
        // whether this node has been measured or not
        bool hasMeasurement();
        void updateHasMeasurement( bool hasMeasurement );
        
    protected:
        double _occDist; // if node is occluded this sets the shortest distance from an occupied node for which the occlusion was registered, -1 if not registered so far
        bool _hasNoMeasurement; // True if this node was setup for additional data but was not actually part of a measurement (not free and not occupied)
    };*/
    typedef OcTreeNode DROcTreeNode;
    
    // tree definition
    class DROcTree: public OccupancyOcTreeStereo<OcTreeNode>
    {    
        public:
            /// Default constructor, sets resolution of leafs
            DROcTree(double resolution, double coeff, double max_range)
            : OccupancyOcTreeStereo<DROcTreeNode>(resolution,coeff,max_range) {};    
            
            /// virtual constructor: creates a new object of same type
            /// (Covariant return type requires an up-to-date compiler)
            DROcTree* create() const {return new DROcTree(resolution,coefficient,maximum_range); }

            std::string getTreeType() const {return "DROcTree";}
            

        protected:
            /**
            * Static member object which ensures that this OcTree's prototype
            * ends up in the classIDMapping only once
            */
            class StaticMemberInitializer{
            public:
            StaticMemberInitializer() {
                DROcTree* tree = new DROcTree(0.1,1,1);
                AbstractOcTree::registerTreeType(tree);
            }
            };
            /// to ensure static initialization (only once)
            static StaticMemberInitializer drOcTreeMemberInit;
    
  };
}