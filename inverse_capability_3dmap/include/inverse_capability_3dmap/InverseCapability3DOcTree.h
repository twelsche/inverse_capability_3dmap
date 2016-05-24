#ifndef INVERSECAPABILITY3DOCTREE_H
#define INVERSECAPABILITY3DOCTREE_H

#include "inverse_capability_3dmap/InverseCapability3DOcTreeNode.h"
#include <gtest/gtest.h>
#include <octomap/OcTreeBase.h>
#include <fstream>
#include <string>
#include <iostream>

using namespace octomap;

// tree definition
class InverseCapability3DOcTree : public OcTreeBase<InverseCapability3DOcTreeNode>
{
  public:

    // Default constructor, sets resolution of leafs
    FRIEND_TEST(InverseCapability3DOcTree, constructor);
    InverseCapability3DOcTree(double resolution) : OcTreeBase<InverseCapability3DOcTreeNode>(resolution), maximum_percent_(-1.0) {}

    // virtual constructor: creates a new object of same type
    // (Covariant return type requires an up-to-date compiler)
    InverseCapability3DOcTree* create() const { return new InverseCapability3DOcTree(resolution); }

    std::string getTreeType() const { return "InverseCapability3DOcTree"; }

    // writes the InverseCapability3DOcTree to file
    bool writeFile(const std::string &filename);

    // creates a new CapabilityOcTree from given file (you need to delete the created tree yourself)
    static InverseCapability3DOcTree* readFile(const std::string &filename);

    // returns the position at which the given coordinate ends up in the tree
    inline double getAlignment(double coordinate) { return keyToCoord(coordToKey(coordinate)); }

//    FRIEND_TEST(CapabilityOcTree, set_getNodeCapability);
    // set node inverse capability at given key or coordinate. Replaces previous inverse capability.
    InverseCapability3DOcTreeNode* setNodeInverseCapability(const OcTreeKey &key, const InverseCapability3D &inv_capa);

    InverseCapability3DOcTreeNode* setNodeInverseCapability(const double &x, const double &y, const double &z,
    										const std::map<std::vector<double>, double> &eulerMap);

    InverseCapability3DOcTreeNode* setNodeInverseCapability(const double &x, const double &y,
                                            const double &z, const InverseCapability3D &inv_capa);

    // get node inverse capability at given coordinate
    InverseCapability3D getNodeInverseCapability(const double &x, const double &y, const double &z) const;

    // convenience functions
//    FRIEND_TEST(CapabilityOcTree, isPosePossible);
    const std::map<std::vector<double>, double>* getEulerMap(const double &x, const double &y, const double &z) const;
    double getPoseQuality(const double &x, const double &y, const double &z, const std::vector<double> &eulerangles) const;
    const std::pair<std::vector<double>, double>* getMaxPointPercent(const double &x, const double &y, const double &z) const;
    std::map<std::vector<double>, double> getEulersWithMinPercent(const double &x, const double &y, const double &z, const double &percent) const;

    InverseCapability3DOcTree* TransformOctTree(InverseCapability3DOcTree &tree,const double &alpha, const octomath::Vector3 &trans);

    //std::vector<octomath::Vector3> CapabilityOcTree::getPositionsWithMinReachablePercent(double percent)



    void setGroupName(const std::string &name) { group_name_ = name; }
    std::string getGroupName() const { return group_name_; }

    void setBaseName(const std::string &name) { base_name_ = name; }
    std::string getBaseName() const { return base_name_; }

    void setTipName(const std::string &name) { tip_name_ = name; }
    std::string getTipName() const { return tip_name_; }

    void setAngleResolution(const unsigned int &angle_resolution) { angle_resolution_ = angle_resolution; }
    unsigned int getAngleResolution() const { return angle_resolution_; }

    void setMaximumPercent(const double& percent) { maximum_percent_ = percent; }
    double getMaximumPercent() const { return maximum_percent_; }



  protected:

    InverseCapability3DOcTreeNode* setNodeInverseCapabilityRecurs(InverseCapability3DOcTreeNode* node, bool node_just_created, const OcTreeKey& key,
                           unsigned int depth, const InverseCapability3D &inv_capa);

    /**
     * Static member object which ensures that this OcTree's prototype
     * ends up in the classIDMapping only once
     */
    class StaticMemberInitializer
    {
      public:
        StaticMemberInitializer()
        {
            InverseCapability3DOcTree* tree = new InverseCapability3DOcTree(0.1);
            AbstractOcTree::registerTreeType(tree);
         }
    };
    /// static member to ensure static initialization (only once)
    static StaticMemberInitializer InverseCapability3DOcTreeMemberInit;

  private:

    std::string group_name_;
    std::string base_name_;
    std::string tip_name_;
    unsigned int angle_resolution_;
    double maximum_percent_;
};


#endif // INVERSECAPABILITY3DOCTREE_H
