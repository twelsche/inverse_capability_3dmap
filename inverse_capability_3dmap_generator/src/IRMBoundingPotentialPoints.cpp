#include <capability_map/CapabilityOcTree.h>
#include <inverse_capability_3dmap/InverseCapability3DOcTree.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include <ros/ros.h>
#include <tclap/CmdLine.h>
#include <string>
#include <vector>
#include <algorithm>
#include <sstream>
#include <fstream>
#include <ostream>

int main(int argc, char** argv)
{


    std::string pathToIRM = "/home/twelsche/catkin_ws/src/inverse_capability_3dmap/inverse_capability_3dmap_generator/maps/inv_cap_pr2_right_arm_0.05m_16t.icpm";
    InverseCapability3DOcTree* inv_tree = InverseCapability3DOcTree::readFile(pathToIRM);
	ROS_INFO("IRM tree loaded");
    ROS_INFO("IRM tree max percent: %g", inv_tree->getMaximumPercent());
    const double& resolution = inv_tree->getResolution();
    double theta_resolution = inv_tree->getAngleResolution();

    // get and adjust the boundaries for iteration
    double startX = -2.0;
    double endX   = 2.0;
    double startY = -2.0;
    double endY   = 2.0;
    double startZ = -2.0;
    double endZ   = 2.0;

    std::map<std::vector<double> , double> min_xy;
    std::map<std::vector<double> , double> max_xy;
    std::map<std::vector<double> , double> min_xz;
    std::map<std::vector<double> , double> max_xz;
    std::map<std::vector<double> , double> min_yz;
    std::map<std::vector<double> , double> max_yz;
    // std::ofstream CSVFile2;
    // CSVFile2.open("IRM_BoundingPotentialPoints2.csv", std::ios::ate | std::ios::app);
    // insert title row 
    // CSVFile2 << "x" << "," <<"y" << "," <<"z" << "," <<"\n";

    for(double x = startX; x <= endX; x += resolution)
    {
        for(double y = 0; y <= endY; y += resolution)
        {
            for(double z = 0; z <= endZ; z += resolution)
            {
				//check if x/y/z is in tree
                const std::map<std::vector<double>, double>* inverse_capability = inv_tree->getEulerMap(x,y,z);
                InverseCapability3D inverse_capabil = inv_tree->getNodeInverseCapability(x,y,z);
                const std::map<std::vector<double>, double> inv_cap = inverse_capabil.getEulerMap();
                if (inv_cap.size() == 0)
                {
                    continue;
                }
                // ROS_INFO("test");
                // for(std::map<std::vector<double>, double>::const_iterator ii=inv_cap.begin(); ii!=inv_cap.end(); ++ii)
                // {
                //     ROS_INFO("inv_capi: (%g,%g,%g)",ii->first[0],ii->first[1],ii->first[2]);
                // }
                // Orientation towards gripper:
                // point3d Orientation(-x/sqrt(x*x+y*y+z*z),-y/sqrt(x*x+y*y+z*z),-z/sqrt(x*x+y*y+z*z));
                tf::Vector3 tfOrientation(-x/sqrt(x*x+y*y+z*z),-y/sqrt(x*x+y*y+z*z),-z/sqrt(x*x+y*y+z*z));
                // point3d Orientation(0.0,0.0,1.0);
                // double yaw_test = atan2(Orientation.y(), Orientation.x()) * 180.0 / M_PI;
                // double pitch_test = asin(Orientation.z()) * 180.0 / M_PI;


                // if (pitch_test < 0)
                //     pitch_test += 2*M_PI;
                // if (yaw_test < 0)
                //     yaw_test += 2*M_PI; 
                
                int rCounter = 0;
                for(int p = 0; p<theta_resolution;p++)//(int p = -4;p<5;p++)
                {
                    for(int yc = 0; yc<theta_resolution;yc++)//(int yc = -4;yc<5;yc++)
                    {
                        for(int r = 0; r<theta_resolution;r++)
                        { 
                            std::vector<double> eulers;
                            // int pitch_current_int = round(pitch_test/(2 * M_PI / theta_resolution));
                            // double pitch_counter =  pitch_current_int % (int) theta_resolution;
                            // int yaw_current_int = round(yaw_test/(2 * M_PI / theta_resolution));
                            // double yaw_counter =  yaw_current_int % (int) theta_resolution;
                            // pitch_counter = (int) (pitch_counter+p) % (int) theta_resolution;
                            // yaw_counter =  (int) (yaw_counter+yc) % (int) theta_resolution;
                            // eulers.push_back((2 * M_PI / theta_resolution) * r);
                            // eulers.push_back((2 * M_PI / theta_resolution) * pitch_counter);
                            // eulers.push_back((2 * M_PI / theta_resolution) * yaw_counter);   
                            eulers.push_back((2 * M_PI / theta_resolution) * r);
                            eulers.push_back((2 * M_PI / theta_resolution) * p);
                            eulers.push_back((2 * M_PI / theta_resolution) * yc); 
                            std::map<std::vector<double>, double>::const_iterator find_it_cap = inv_cap.find(eulers);
                            if (find_it_cap != inv_cap.end())
                            {
                                // ROS_INFO("eulers: (%g,%g,%g)",eulers[0],eulers[1],eulers[2]);
                                if(find_it_cap->second>10.0)
                                {
                                    tf::Matrix3x3 M_current;
                                    M_current.setRPY(eulers[0],eulers[1],eulers[2]);
                                    double test = M_current.tdotx(tfOrientation);
                                    if(test>0.4)
                                        rCounter++;
                                }
                            }
                            // else
                            //     ROS_INFO("not found eulers: (%g,%g,%g)",eulers[0],eulers[1],eulers[2]);
                        }
                    }
                }
                // ROS_INFO("rCounter for (%g,%g,%g):: %d",x,y,z,rCounter);
                if (rCounter < 170)
                    continue;
                // ROS_INFO("rCounter for (%g,%g,%g):: %d",x,y,z,rCounter);

                // ROS_INFO("size inverse_capability map: %lu",inverse_capability->size());
                OcTreeKey key = inv_tree->coordToKey(x,y,z);
                point3d  point = inv_tree->keyToCoord(key);
                std::vector<double> xy;
                xy.push_back(point.x());
                xy.push_back(point.y());
                std::vector<double> xz;
                xz.push_back(point.x());
                xz.push_back(point.z());
                std::vector<double> yz;
                yz.push_back(point.y());
                yz.push_back(point.z());
                // CSVFile2 << point.x() << "," << point.y() << "," << point.z() << "\n";

                std::map<std::vector<double> , double>::const_iterator find_it = min_xy.find(xy);
                if (find_it != min_xy.end())
                {
                    // already in map, check if outside
                    if (z<find_it->second)
                    {
                        min_xy[xy] = z;
                    }
                }
                else
                {
                    min_xy.insert(std::make_pair(xy, z));
                }

                find_it = max_xy.find(xy);
                if (find_it != max_xy.end())
                {
                    // already in map, check if outside
                    if (z>find_it->second)
                    {
                        max_xy[xy] = z;
                    }
                }
                else
                {
                    max_xy.insert(std::make_pair(xy, z));
                }

                find_it = min_xz.find(xy);
                if (find_it != min_xz.end())
                {
                    // already in map, check if outside
                    if (y<find_it->second)
                    {
                        min_xz[xz] = y;
                    }
                }
                else
                {
                    min_xz.insert(std::make_pair(xz, y));
                }

                find_it = max_xz.find(xz);
                if (find_it != max_xz.end())
                {
                    // already in map, check if outside
                    if (y>find_it->second)
                    {
                        max_xz[xz] = y;
                    }
                }
                else
                {
                    max_xz.insert(std::make_pair(xz, y));
                }


                find_it = min_yz.find(yz);
                if (find_it != min_yz.end())
                {
                    // already in map, check if outside
                    if (x<find_it->second)
                    {
                        min_yz[yz] = x;
                    }
                }
                else
                {
                    min_yz.insert(std::make_pair(yz, x));
                }

                find_it = max_yz.find(yz);
                if (find_it != max_yz.end())
                {
                    // already in map, check if outside
                    if (x>find_it->second)
                    {
                        max_yz[yz] = x;
                    }
                }
                else
                {
                    max_yz.insert(std::make_pair(yz, x));
                }
            }
        }
    }
    // CSVFile2.close();

    //  save all found points to a list as csv file
    // save the calculated values to a new  csv-file: 
    std::ofstream CSVFile;
    CSVFile.open("IRM_BoundingPotentialPoints.csv", std::ios::ate | std::ios::app);
    // insert title row 
    // CSVFile << "x" << "," <<"y" << "," <<"z" << "," <<"\n";


    ROS_INFO("points created");


    typedef std::map<std::vector<double> , double>::iterator it_type;
    for(it_type iterator = min_xy.begin(); iterator != min_xy.end(); ++iterator) 
    {
        if (iterator->first[1] != 0 && iterator->second != 0)
            CSVFile << iterator->first[0] << "," << iterator->first[1] << "," << iterator->second << "\n";
    }

    for(it_type iterator = max_xy.begin(); iterator != max_xy.end(); ++iterator) 
    {
        if (iterator->first[1] != 0 && iterator->second != 0)
            CSVFile << iterator->first[0] << "," << iterator->first[1] << "," << iterator->second << "\n";
    }

    // for(it_type iterator = min_xz.begin(); iterator != min_xz.end(); ++iterator) 
    // {
    //     if (iterator->first[1] != 0 && iterator->second != 0)
    //         CSVFile << iterator->first[0] << "," << iterator->second << "," << iterator->first[1] << "\n";
    // }

    for(it_type iterator = max_xz.begin(); iterator != max_xz.end(); ++iterator) 
    {
        if (iterator->first[1] != 0 && iterator->second != 0)
            CSVFile << iterator->first[0] << "," << iterator->second << "," << iterator->first[1] << "\n";
    }

    for(it_type iterator = min_yz.begin(); iterator != min_yz.end(); ++iterator) 
    {
        if (iterator->first[1] != 0 && iterator->first[0] != 0)
            CSVFile << iterator->second << "," << iterator->first[0] << "," << iterator->first[1] << "\n";
    }

    for(it_type iterator = max_yz.begin(); iterator != max_yz.end(); ++iterator) 
    {
        if (iterator->first[1] != 0 && iterator->first[0] != 0)
            CSVFile << iterator->second << "," << iterator->first[0] << "," << iterator->first[1] << "\n";
    }

    CSVFile.close();
    ROS_INFO("Saved csv file");
}