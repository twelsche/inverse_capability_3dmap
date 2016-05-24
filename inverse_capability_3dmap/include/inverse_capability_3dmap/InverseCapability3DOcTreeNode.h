#ifndef InverseCapability3D3DOCTREENODE_H
#define InverseCapability3D3DOCTREENODE_H

#include <gtest/gtest.h>
#include <octomap/OcTreeDataNode.h>
#include <map>

using namespace octomap;

class InverseCapability3D
{
  public:
    /// Class to find max value in map
    struct LessThanSecond
    {
        template <typename Lhs, typename Rhs>
        bool operator()(const Lhs& lhs, const Rhs& rhs) const
        {
            return lhs.second < rhs.second;
        }
    };

  public:

    FRIEND_TEST(InverseCapability3D, constructor);
	InverseCapability3D();
	InverseCapability3D(const std::map<std::vector<double> , double> &eulerMap);

    // setter and getter
    void setEulerMap(const std::map<std::vector<double>, double> &eulerMap) { eulerMap_ =  eulerMap; }
    const std::map<std::vector<double>, double> & getEulerMap() const { return eulerMap_; }

    void setEulerEntry(const std::pair<std::vector<double>, double> &p) { eulerMap_.insert(p); }
    double getEulerEntry(const std::vector<double> eulerangles);

//    FRIEND_TEST(Capability, equalityOperators);
    bool operator==(const InverseCapability3D &other) const;
    // bool operator!=(const InverseCapability3D &other) const;

    // add other to inverseCapabilities
    // InverseCapability3D operator+(const InverseCapability3D &other) const;

    // join InverseCapability3D, only take highest reachability of both
    // InverseCapability3D operator&(const InverseCapability3D &other) const;

    // void normalize(const double& value);

    // Return the <theta, percent> with the highest percentage
//    FRIEND_TEST(Capability, isDirectionPossible);
    // const std::pair<double, double> & getMaxThetaPercent();

    // Return all thetas (<theta, percent>) with minimum percent above minPercent
    // std::map<double, double> getThetasWithMinPercent(double minPercent) const;

  protected:

    // if object is reachable from given theta, then store [theta, percent] into map
    std::map<std::vector<double> , double> eulerMap_;
};


class InverseCapability3DOcTreeNode : public OcTreeDataNode<InverseCapability3D>
{
  public:

    // Constructors
	InverseCapability3DOcTreeNode();
	InverseCapability3DOcTreeNode(InverseCapability3D inv_capa);
	InverseCapability3DOcTreeNode(const InverseCapability3DOcTreeNode &rhs);

    ~InverseCapability3DOcTreeNode();

//    FRIEND_TEST(CapabilityOcTreeNode, equalityOperator);
    // bool operator==(const InverseCapability3DOcTreeNode &rhs) const
    // {
    //     return (rhs.value == value);
    // }

    // children

//    FRIEND_TEST(CapabilityOcTreeNode, children);
    bool createChild(unsigned int i);

    inline InverseCapability3DOcTreeNode* getChild(unsigned int i)
    {
        return static_cast<InverseCapability3DOcTreeNode*>(OcTreeDataNode<InverseCapability3D>::getChild(i));
    }

    inline const InverseCapability3DOcTreeNode* getChild(unsigned int i) const
    {
        return static_cast<const InverseCapability3DOcTreeNode*>(OcTreeDataNode<InverseCapability3D>::getChild(i));
    }

//    // TODO: should not be overwritten, node only gets pruned when value is equal for all children (uncomment if problems arise)
//    // bool collapsible() { return false; }
//    // bool pruneNode() { return false; }
//    // void expandNode() { }
//
//    FRIEND_TEST(CapabilityOcTreeNode, set_getCapability);
//    // setter/getter for Capability (value derived from OcTreeDataNode)
    inline void setInverseCapability3D(InverseCapability3D inv_capa) { value = inv_capa; }
    inline void setInverseCapability3D(std::map<std::vector<double> , double>  &eulerMap)
    {
        value = InverseCapability3D(eulerMap);
    }

    inline InverseCapability3D getInverseCapability3D() const { return value; }

    // inline void normalize(const double& val) { value.normalize(val); }

    // file I/O
    std::ostream& writeValue(std::ostream &s) const;
    std::istream& readValue(std::istream &s);

};

#endif // InverseCapability3D3DOCTREENODE_H
