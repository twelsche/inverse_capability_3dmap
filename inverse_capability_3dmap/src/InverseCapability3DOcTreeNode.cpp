#include "inverse_capability_3dmap/InverseCapability3DOcTreeNode.h"
#include <ros/ros.h>
#include <algorithm>
//#include <cmath>

/**************************************************************************************************
 *
 * 										InverseCapability3D
 *
 *************************************************************************************************/

InverseCapability3D::InverseCapability3D()
{
}

InverseCapability3D::InverseCapability3D(const std::map<std::vector<double>, double>  &eulerMap)
{
	eulerMap_ = eulerMap;
}
double InverseCapability3D::getEulerEntry(const std::vector<double> eulerangles)
{
	std::map<std::vector<double>, double>::iterator it;
	it = eulerMap_.find(eulerangles);
	if (it != eulerMap_.end())
		return it->second;
	else
		return 0.0;
}
// double InverseCapability3D::getThetaPercent(const double theta)
// {
// 	std::map<double, double>::iterator it;
// 	it = thetas_.find(theta);
// 	if (it != thetas_.end())
// 		return it->second;
// 	else
// 		return 0.0;
// }

bool InverseCapability3D::operator==(const InverseCapability3D &other) const
{
    return eulerMap_.size() == other.eulerMap_.size()
        && std::equal(eulerMap_.begin(), eulerMap_.end(),
                      other.eulerMap_.begin());
}

// bool InverseCapability3D::operator!=(const InverseCapability3D &other) const
// {
//     return !(this->thetas_ == other.thetas_);
// }

// InverseCapability3D InverseCapability3D::operator+(const InverseCapability3D &other) const
// {
// 	InverseCapability3D result, copy_other;
// 	copy_other = other;
// 	std::map<double, double>::const_iterator it;
// 	std::map<double, double>::iterator other_it;
// 	for (it = this->thetas_.begin(); it != this->thetas_.end(); it++)
// 	{
// 		other_it = copy_other.thetas_.find(it->first);
// 		// key not found in other, store in result inverse capability
// 		if (other_it == copy_other.thetas_.end())
// 			result.setThetaPercent(*it);
// 		else
// 		{
// 			// if match found in other, add percentages of both and add to result
// 			// further, remove match from other, since its match was treated
// 			ROS_ASSERT(it->first == other_it->first);
// 			double percent = it->second + other_it->second;
// 			result.setThetaPercent(std::make_pair(it->first, percent));
// 			copy_other.thetas_.erase(other_it);
// 		}
// 	}

// 	// add (theta, percent) pairs from other which don't have a match
// 	for (it = copy_other.thetas_.begin(); it != copy_other.thetas_.end(); it++)
// 	{
// 		result.setThetaPercent(*it);
// 	}

// 	ROS_ASSERT(result.thetas_.size() <= this->thetas_.size() + other.thetas_.size());

// 	return result;
// }

// InverseCapability3D InverseCapability3D::operator&(const InverseCapability3D &other) const
// {
// 	InverseCapability3D result, copy_other;
// 	copy_other = other;
// 	std::map<double, double>::const_iterator it;
// 	std::map<double, double>::iterator other_it;
// 	for (it = this->thetas_.begin(); it != this->thetas_.end(); it++)
// 	{
// 		other_it = copy_other.thetas_.find(it->first);
// 		// key not found in other, store in result inverse capability
// 		if (other_it == copy_other.thetas_.end())
// 			result.setThetaPercent(*it);
// 		else
// 		{
// 			// if match found in other, take highest percentage of both and store in result
// 			// further, remove match from other, since its match was treated
// 			ROS_ASSERT(it->first == other_it->first);
// 			double percent;
// 			if (it->second < other_it->second)
// 				percent = other_it->second;
// 			else
// 				percent = it->second;
// 			result.setThetaPercent(std::make_pair(it->first, percent));
// 			copy_other.thetas_.erase(other_it);
// 		}
// 	}

// 	// add (theta, percent) pairs from other which don't have a match
// 	for (it = copy_other.thetas_.begin(); it != copy_other.thetas_.end(); it++)
// 	{
// 		result.setThetaPercent(*it);
// 	}

// 	ROS_ASSERT(result.thetas_.size() <= this->thetas_.size() + other.thetas_.size());

// 	return result;
// }

// void InverseCapability3D::normalize(const double& value)
// {
// 	std::map<double, double>::iterator it;
// 	for (it = this->thetas_.begin(); it != this->thetas_.end(); it++)
// 		it->second = it->second / value;
// }

// const std::pair<double, double> & InverseCapability3D::getMaxThetaPercent()
// {
// 	std::map<double, double>::iterator it;
// 	it = std::max_element(thetas_.begin(), thetas_.end(), LessThanSecond());
// 	return *it;
// }

// std::map<double, double> InverseCapability3D::getThetasWithMinPercent(double minPercent) const
// {
// 	std::map<double, double> ret;
// 	std::map<double, double>::const_iterator it;
// 	for (it = thetas_.begin(); it != thetas_.end(); it++)
// 	{
// 		if (it->second > minPercent)
// 			ret.insert(*it);
// 	}
// 	return ret;
// }

/**************************************************************************************************
 *
 * 									InverseCapability3DOcTreeNode
 *
 *************************************************************************************************/

InverseCapability3DOcTreeNode::InverseCapability3DOcTreeNode()
{
}

InverseCapability3DOcTreeNode::InverseCapability3DOcTreeNode(InverseCapability3D inv_capa)
	: OcTreeDataNode<InverseCapability3D>(inv_capa)
{
}

InverseCapability3DOcTreeNode::InverseCapability3DOcTreeNode(const InverseCapability3DOcTreeNode &rhs)
	: OcTreeDataNode<InverseCapability3D>(rhs)
{
}

InverseCapability3DOcTreeNode::~InverseCapability3DOcTreeNode()
{
}

bool InverseCapability3DOcTreeNode::createChild(unsigned int i)
{
    if (children == NULL)
    {
        allocChildren();
    }
    assert (children[i] == NULL);
    children[i] = new InverseCapability3DOcTreeNode();
    return true;
}

std::ostream& InverseCapability3DOcTreeNode::writeValue(std::ostream &s) const
{
    // 1 bit for each children; 0: empty, 1: allocated
    std::bitset<8> children;
    for (unsigned int i = 0; i < 8; ++i)
    {
        if (childExists(i))
        {
          children[i] = 1;
        }
        else
        {
          children[i] = 0;
        }
    }
    char children_char = (char)children.to_ulong();

    // buffer inverse capability data
    std::map<std::vector<double>, double> eulerMap = value.getEulerMap();
    unsigned int size = eulerMap.size();

    // write node data
    s.write((const char*)&size, sizeof(unsigned int));
    std::map<std::vector<double>, double>::iterator it;
    // double theta, percent;
    double percent;
    
    for (it = eulerMap.begin(); it != eulerMap.end(); it++)
    {
    	// theta = it->first;
    	std::vector<double> eulerangles;
    	eulerangles = it->first;
    	percent = it->second;
    	// s.write((const char*)&theta  , sizeof(double));
    	s.write((const char*)&eulerangles[0]  , sizeof(double));
    	s.write((const char*)&eulerangles[1]  , sizeof(double));
    	s.write((const char*)&eulerangles[2]  , sizeof(double));
    	s.write((const char*)&percent, sizeof(double));
    }
    s.write((char*)&children_char, sizeof(char)); // child existence

    // write existing children
    for (unsigned int i = 0; i < 8; ++i)
    {
        if (children[i] == 1)
        {
            this->getChild(i)->writeValue(s);
        }
    }
    return s;
}

std::istream& InverseCapability3DOcTreeNode::readValue(std::istream &s)
{
    // buffer for capabilities' data
    unsigned int size;
    // std::map<double, double> thetas;
    std::map<std::vector<double>, double> eulerMap;
    char children_char;

    // read node data
    s.read((char*)&size, sizeof(unsigned int));

    // double theta, percent;
    double x,y,z,percent;
    
     // ROS_INFO("Start reading node:");
    for (unsigned int i = 0; i < size; i++)
    {
    	// s.read((char*)&theta  , sizeof(double));
    	s.read((char*)&x  , sizeof(double));
    	s.read((char*)&y  , sizeof(double));
    	s.read((char*)&z  , sizeof(double));
    	s.read((char*)&percent, sizeof(double));
    	// thetas.insert(std::make_pair(theta, percent));
    	std::vector<double> eulerangles;
    	eulerangles.push_back(x);
    	eulerangles.push_back(y);
    	eulerangles.push_back(z);
		// ROS_INFO("Read entry in map: (%g,%g,%g) with %g",x,y,z,percent);
    	eulerMap.insert(std::make_pair(eulerangles, percent));
    }
    s.read((char*)&children_char, sizeof(char)); // child existence

    // insert buffered data into node
    // value.setThetasPercent(thetas);
    value.setEulerMap(eulerMap);

    // read existing children
    std::bitset<8> children ((unsigned long long)children_char);
    for (unsigned int i = 0; i < 8; ++i)
    {
        if (children[i] == 1)
        {
            createChild(i);
            getChild(i)->readValue(s);
        }
    }
    return s;
}


