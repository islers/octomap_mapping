#include <octomap_server/DROcTree.h>
#include <iostream>

namespace octomap
{
    DROcTreeNode::DROcTreeNode()
    : OcTreeNode()
    , _occDist(-1.0)
    {}
    
    double DROcTreeNode::getMinChildOccDist()
    {
        double min = std::numeric_limits<double>::max();
        for (unsigned int i=0; i<8; i++) {
            if (childExists(i))
            {
                double occDist = getChild(i)->occDist();
                if( occDist!=-1 && occDist<min )
                {
                    min = occDist;
                }
            }
        }
    }
        
    double DROcTreeNode::occDist()
    {
        return _occDist;
    }
    
    void DROcTreeNode::updateOccDist( double occDist )
    {
        if( _occDist==-1 )
            _occDist = occDist;
        _occDist = std::min(occDist,_occDist);
    }
}