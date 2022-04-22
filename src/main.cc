#define PCL_NO_PRECOMPILE
#include "cloud_accumulator.hpp"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>

using point_t = pcl::PointXYZI;
using cloud_accumulator_t = smc::CloudAccumulator<point_t>;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cloud_accumulator");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    const std::string output_filename = pnh.param("output_filename", std::string("cloud.pcd"));
    const double downsampling_size_display = pnh.param("voxel_grid_size_display", 0.05); //0.0 or less: Off
    const double downsampling_size_save = pnh.param("voxel_grid_size_save", 0.01); //0.0 or less: Off
    const double min_distance = pnh.param("minimum_distance", 0.0);
    const double max_distance = pnh.param("maximum_distance", 5.0);
    const bool start_paused = pnh.param("start_paused", false);
    const bool keep_high_res = pnh.param("keep_high_resolution", true);

    // Get the topic names.
    const std::string pointcloud_topic = pnh.param("pointcloud_topic", std::string("pointcloud"));
    const std::string pose_topic = pnh.param("pose_topic", std::string("pose"));
    const std::string trajectory_topic = pnh.param("trajectory_topic", std::string("trajectory"));

    cloud_accumulator_t acc(
        downsampling_size_display, 
        downsampling_size_save,
        min_distance, 
        max_distance,
        output_filename, 
        keep_high_res, 
        start_paused);

    ros::Subscriber traj_sub;
    if(keep_high_res)
    {
        traj_sub = nh.subscribe<nav_msgs::Path>(trajectory_topic, 2, &cloud_accumulator_t::trajectoryCallback, &acc);
    }

    ros::Subscriber cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>(pointcloud_topic, 1, &cloud_accumulator_t::pointCloudCallback, &acc);

    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(pose_topic, 50, &cloud_accumulator_t::poseCallback, &acc);

    // ros::ServiceServer pause_srv = pnh.advertiseService("toggle_pause", &cloud_accumulator_t::togglePause, &acc);

    // ros::ServiceServer save_srv = pnh.advertiseService("save_cloud", &cloud_accumulator_t::saveCloud, &acc);

    ros::AsyncSpinner spinner(4); //Async spinner, since the save_cloud service may take long
    spinner.start();
    ros::waitForShutdown();
}
