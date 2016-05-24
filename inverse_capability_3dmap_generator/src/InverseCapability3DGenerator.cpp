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

//using namespace inverse_capability_map_utils;

struct Input {
    double resolution;
    unsigned int theta_resolution;
    std::string path_capa;
    std::string path_name;
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> z;
    std::vector<double> gripper_orientation;
    bool loggingEnabled;
};

CapabilityOcTree* capa_tree = NULL;

Input verifyInput(int argc, const char * const * argv)
{
    std::string msg = "Generates the inverse capability map of the region specified by given bounding box.";
    TCLAP::CmdLine cmd(msg, ' ', "1.0");

    msg = "Distance between two voxels in meter, default is 0.1 m.";
    TCLAP::ValueArg<double> resolution_arg("r", "resolution", msg, false, 0.1, "floating point");

    msg = "Resolution indicating how many different angles should be computed for the base.";
    TCLAP::ValueArg<unsigned int> theta_resolution_arg("t", "theta-resolution", msg, false, 16, "integer");

    msg = "Desired Orientation (x) of the gripper x-axis for inverse_capability_map.";
    TCLAP::MultiArg<double> gripper_orientation_arg("g", "gripper-orientation", msg, true, "floating point");

    msg = "Filename and path to the capability map. \nExample: -c mydir/mysubdir/filename.cpm";
    TCLAP::ValueArg<std::string> path_capa_arg("c", "path-capability-map", msg, true, "./capability_map.cpm", "string");

    msg = "Filename and path where the inverse capability map should be stored. \nExample: -c mydir/mysubdir/filename.icpm";
    TCLAP::ValueArg<std::string> path_name_arg("p", "path-inverse-capability-map", msg, true, "./inverse_capability_map.icpm", "string");

    msg = "The start/end point of the bounding box in x-direction.\n\
           If only one x-value is given, a slice (or a point) at this position depending on y- and z-values gets computed.\n\
           If more than 2 values are given, the boundaries are from min(x1, x2, ...) to max(x1, x2, ...).\n\
           Example: -x -0.1 -x 2.3";
    TCLAP::MultiArg<double> x_arg("x", "x-pos", msg, true, "floating point");

    msg = "The start/end point of the bounding box in y-direction.\n\
           If only one y-value is given, a slice (or a point) at this position depending on x- and z-values gets computed.\n\
           If more than 2 values are given, the boundaries are from min(y1, y2, ...) to max(y1, y2, ...).\n\
           Example: -y -0.1 -y 2.3";
    TCLAP::MultiArg<double> y_arg("y", "y-pos", msg, true, "floating point");

    msg = "The start/end point of the bounding box in z-direction.\n\
           If only one z-value is given, a slice (or a point) at this position depending on x- and y-values gets computed.\n\
           If more than 2 values are given, the boundaries are from min(z1, z2, ...) to max(z1, z2, ...).\n\
           Example: -z -0.1 -z 2.3";
    TCLAP::MultiArg<double> z_arg("z", "z-pos", msg, true, "floating point");

//    msg = "If set, writes a log file containing time required and number of computed capabilities to map_name.cpm.build_log";
//    TCLAP::SwitchArg log_arg("l", "log", msg, false);

    cmd.add(z_arg);
    cmd.add(y_arg);
    cmd.add(x_arg);
    cmd.add(gripper_orientation_arg);
    cmd.add(resolution_arg);
    cmd.add(theta_resolution_arg);
    cmd.add(path_capa_arg);
    cmd.add(path_name_arg);
//    cmd.add(log_arg);

    // parse arguments with TCLAP
    try
    {
        cmd.parse(argc, argv);
    }
    catch (TCLAP::ArgException &e)  // catch any exceptions
    {
        ROS_ERROR("Error: %s for argument %s", e.error().c_str(), e.argId().c_str());
        ros::shutdown();
        exit(1);
    }

    // get values from arguments
    Input input;
    input.resolution = resolution_arg.getValue();
    input.theta_resolution = theta_resolution_arg.getValue();
    input.path_capa = path_capa_arg.getValue();
    input.path_name = path_name_arg.getValue();
    input.x = x_arg.getValue();
    input.y = y_arg.getValue();
    input.z = z_arg.getValue();
    input.gripper_orientation = gripper_orientation_arg.getValue();
//    input.loggingEnabled = log_arg.getValue();

    // load capability map
    capa_tree = CapabilityOcTree::readFile(input.path_capa);

    if (capa_tree == NULL)
    {
        ROS_ERROR("Could not load capability map file %s", input.path_capa.c_str());
        ros::shutdown();
        exit(1);
    }

    if (input.resolution <= 0.0)
    {
        ROS_ERROR("Error: resolution must be positive and greater than 0.0");
        ros::shutdown();
        exit(1);
    }

    if (input.theta_resolution <= 0)
    {
        ROS_ERROR("Error: theta resolution must be positive and greater than 0");
        ros::shutdown();
        exit(1);
    }

	return input;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "inverse_capability_generator");

    Input input = verifyInput(argc, argv);

    ros::NodeHandle nhPriv("~");

    // aliases
    std::vector<double>& x_args = input.x;
    std::vector<double>& y_args = input.y;
    std::vector<double>& z_args = input.z;
    std::vector<double>& gOrientation_args = input.gripper_orientation;
    const double& resolution       = input.resolution;
    const double& theta_resolution = input.theta_resolution;

    InverseCapability3DOcTree inv_tree(resolution);
    inv_tree.setGroupName(capa_tree->getGroupName());
    inv_tree.setBaseName(capa_tree->getBaseName());
    inv_tree.setTipName(capa_tree->getTipName());
    inv_tree.setAngleResolution(theta_resolution);
	ROS_INFO("Group name is: %s", inv_tree.getGroupName().c_str());
	ROS_INFO("Base frame is: %s", inv_tree.getBaseName().c_str());
	ROS_INFO("Tip frame is: %s", inv_tree.getTipName().c_str());
	ROS_INFO("Resolution is: %g", inv_tree.getResolution());
	ROS_INFO("Angle resolution is: %d", inv_tree.getAngleResolution());

    // sort x, y and z values
    std::sort(x_args.begin(), x_args.end());
    std::sort(y_args.begin(), y_args.end());
    std::sort(z_args.begin(), z_args.end());

    // get and adjust the boundaries for iteration
    double startX = inv_tree.getAlignment(x_args[0]);
    double endX   = inv_tree.getAlignment(x_args[x_args.size() - 1]);
    double startY = inv_tree.getAlignment(y_args[0]);
    double endY   = inv_tree.getAlignment(y_args[y_args.size() - 1]);
    double startZ = inv_tree.getAlignment(z_args[0]);
    double endZ   = inv_tree.getAlignment(z_args[z_args.size() - 1]);

    double numCapsToCompute = ((endX - startX) / resolution + 1.0) * ((endY - startY) / resolution + 1.0)
                              * ((endZ - startZ) / resolution + 1.0);
    double numCapsComputed = 0.0;

    ROS_INFO("Number of inverse capabilities to compute: %d", (unsigned int)numCapsToCompute);
    ROS_INFO("Angle resolution: %d", (unsigned int)theta_resolution);
    ROS_INFO("Start X: %g, End X %g", startX, endX);
    ROS_INFO("Start Y: %g, End Y %g", startY, endY);

    //  add a small value to end due to floating point precision
    endX += resolution/100.0;
    endY += resolution/100.0;
    endZ += resolution/100.0;

    // progress in percent
    double progress = 0.0;
    double progressLimiter = 0.0;

    // store highest percent
    double max_percent = 0.0;

    for(double x = startX; x <= endX; x += resolution)
    {
        for(double y = startY; y <= endY; y += resolution)
        {
            for(double z = startZ; z <= endZ; z += resolution)
            {
				std::map<std::vector<double>, double> inv_capa;                 
                for (unsigned int i = 0; i < theta_resolution; ++i)
                {
                    double roll = (2 * M_PI / theta_resolution) * i;
                    for (unsigned int j = 0; j < theta_resolution; ++j)
                    {
                        double pitch = (2 * M_PI / theta_resolution) * j;
                        for (unsigned int k = 0; k < theta_resolution; ++k)
                        {
                            double yaw = (2 * M_PI / theta_resolution) * k;
                            geometry_msgs::Pose robo;
                            robo.position.x = x;
                            robo.position.y = y;
                            robo.position.z = z;
                            tf::Quaternion q;
                            q.setRPY(roll, pitch, yaw);
                            tf::quaternionTFToMsg(q, robo.orientation);

                            tf::Pose roboPose;
                            tf::poseMsgToTF(robo, roboPose);
                            // compute inverse transformation, from object frame to robot frame
                            // object located at origin
                            tf::Pose result = roboPose.inverse();
                            // check if Pose with certain orientation is reachable 


                            // TODO: set orientation in base frame!!!!!
                            octomath::Quaternion quat(result.getRotation().w(),result.getRotation().x(),result.getRotation().y(),result.getRotation().z());
                            octomath::Vector3  pos(result.getOrigin().x(), result.getOrigin().y(), result.getOrigin().z());
                            octomath::Pose6D pose(pos,quat);
                            octomath::Vector3 rotatedVector = pose.rot().rotate(octomath::Vector3(1, 0, 0));

                            // octomath::Quaternion quat(roboPose.getRotation().w(),roboPose.getRotation().x(),roboPose.getRotation().y(),roboPose.getRotation().z());
                            // octomath::Vector3  pos(roboPose.getOrigin().x(), roboPose.getOrigin().y(), roboPose.getOrigin().z());
                            // octomath::Pose6D pose(pos,quat);
                            // octomath::Vector3 rotatedVector = pose.rot().rotate(octomath::Vector3(1, 0, 0));


                            double phi_rot = (atan2(rotatedVector.y(), rotatedVector.x()) * 180.0 / M_PI);
                            double theta_rot;
                            theta_rot = (acos(std::min(std::max((double) rotatedVector.z(),-1.0),1.0)) * 180.0 / M_PI);
                            
                            // ROS_INFO("Phi is: %g", phi_rot);
                            // ROS_INFO("Theta is: %g", theta_rot);
                            bool possible = capa_tree->isPosePossible(result.getOrigin().x(), result.getOrigin().y(), result.getOrigin().z(),  phi_rot, theta_rot);
                            if (!possible)
                            {
                                // inv_capa.insert(std::make_pair(theta, 0));
                            }
                            else 
                            {
                                // look up percent for current robot pose
                                double percent = capa_tree->getNodeCapability(result.getOrigin().x(), result.getOrigin().y(), result.getOrigin().z()).getPercentReachable();


                                tf::Matrix3x3 M_current(q);
                                double roll_test,pitch_test,yaw_test;
                                M_current.getRPY(roll_test,pitch_test,yaw_test);
                                if (roll_test < 0)
                                    roll_test += 2*M_PI;
                                if (pitch_test < 0)
                                    pitch_test += 2*M_PI;
                                if (yaw_test < 0)
                                    yaw_test += 2*M_PI;


                                int roll_current_int = round(roll_test/(2 * M_PI / theta_resolution));
                                double roll_counter =  roll_current_int % (int) theta_resolution;
                                int pitch_current_int = round(pitch_test/(2 * M_PI / theta_resolution));
                                double pitch_counter =  pitch_current_int % (int) theta_resolution;
                                int yaw_current_int = round(yaw_test/(2 * M_PI / theta_resolution));
                                double yaw_counter =  yaw_current_int % (int) theta_resolution;

                                // update the capability entry for the new node
                                std::vector<double> eulers;
                                eulers.push_back((2 * M_PI / theta_resolution) * roll_counter);
                                eulers.push_back((2 * M_PI / theta_resolution) * pitch_counter);
                                eulers.push_back((2 * M_PI / theta_resolution) * yaw_counter);

                                
                                // add (euler angles, percent) pair to map
                                // compute euler angles for current orientation
                                // std::vector<double> eulers;
                                // eulers.push_back(roll);
                                // eulers.push_back(pitch);
                                // eulers.push_back(yaw);

                                std::map<std::vector<double>, double>::const_iterator find_it = inv_capa.find(eulers);
                                if (find_it != inv_capa.end())
                                {
                                    // ROS_INFO("Desired orientation already in map");
                                }
                                else
                                {
                                    inv_capa.insert(std::make_pair(eulers, percent));
                                    // ROS_INFO("Inserting inv_cap wiht angles (%g,%g,%g)",eulers[0], eulers[1], eulers[2]);
                                    // ROS_INFO("looked up capability at (%g,%g,%g)",result.getOrigin().x(), result.getOrigin().y(), result.getOrigin().z());  
                                    // ROS_INFO("RoboPose position (%g,%g,%g)",roboPose.getOrigin().x(),result.getOrigin().y(),roboPose.getOrigin().z());
                                    // ROS_INFO("RoboPose orientation (%g,%g,%g,%g)",roboPose.getRotation().w(),roboPose.getRotation().x(),roboPose.getRotation().y(),roboPose.getRotation().z());
                                }


                                // ROS_INFO("Added pait with: %g,%g,%g", roll,pitch,yaw);
                                if (max_percent < percent)
                                    max_percent = percent;    
                            }
                        
                        }
                	}
                }
                if (inv_capa.size() != 0)
                {
                    // ROS_INFO("Build node at: (%g,%g,%g), with %u map entries \n", x,y,z,inv_capa.size());
                    inv_tree.setNodeInverseCapability(x, y, z, inv_capa);    
                }
                
				// ROS_INFO("MAP SIZE :%lu", inv_capa.size());

                numCapsComputed += 1.0;
                progress = 100.0 * numCapsComputed / numCapsToCompute;
                if (progress > progressLimiter)
                {
                    progressLimiter = progress + 0.1;
                    printf("progress: %3.2f%%\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b", progress);
                    fflush(stdout);
                }
            }
        }
    }
    printf("done              \n");

    // store maximum percent in tree
    inv_tree.setMaximumPercent(max_percent);
    if (!inv_tree.writeFile(input.path_name))
    {
        ROS_ERROR("Error: Could not write to file %s\n", input.path_name.c_str());
        ros::shutdown();
        exit(1);
    }
    else
    {
        ROS_INFO("Inverse Capability map written to file %s", input.path_name.c_str());
    }
}
