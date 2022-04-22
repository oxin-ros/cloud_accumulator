// Standard includes.
#include <deque>
#include <limits>
#include <mutex>
#include <thread>

// Boost includes.
#include <boost/shared_ptr.hpp>

// ROS includes.
#include <tf2_ros/buffer.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_srvs/Empty.h>

// PCL includes.
#define PCL_NO_PRECOMPILE
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// Custom includes.
#include "cloud_visualizer.hpp"
#include "cloud_utilities.h"

namespace smc
{

/**
 * Demonstration for how to create a registered point cloud map.
 */
template<typename PointT>
class CloudAccumulator
{

public:
    CloudAccumulator(double voxelgrid_size_display,
                    double voxelgrid_size_save,
                    double min_distance,
                    double max_distance,
                    std::string output_filename,
                    bool keep_high_res,
                    bool start_paused);

    /**
     * For each cloud
     * - Filter by distance along the optical axis
     * - Store filtered cloud for later saving (if keep_high_res_)
     * - Transform to world coordinates according to live pose
     * - Merge with previous clouds
     * - Apply "display" voxel grid filter on merged cloud
     * - Update viewer with the result
     */
    void pointCloudCallback(const typename pcl::PointCloud<PointT>::ConstPtr& pointcloud);
    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& pointcloud);

    /** Replace all saved camera poses by those in \p path */
    void trajectoryCallback(const nav_msgs::PathConstPtr& path);

    /** Saves camera pose */
    void poseCallback(const geometry_msgs::PoseStampedConstPtr& current_pose);

    /**
     * For all saved clouds
     * - Merge
     * - Voxel-grid filter
     * - Display
     */
    typename pcl::PointCloud<PointT>::Ptr rebuildCloud();

    // /**
    //  * - rebuild cloud (if keep_high_res is set) and update viewer
    //  * - Save as .pcd to disk
    //  */
    // bool saveCloud(const std_srvs::Empty::Request& req, const std_srvs::Empty::Request& resp);

    // /** Toggle whether to ignore incoming data */
    // bool togglePause(const std_srvs::Empty::Request& req, const std_srvs::Empty::Request& resp)
    // {
    //     pause_ = !pause_;
    //     return true;
    // }

private:
    Eigen::Affine3f lookupTransform(const double timestamp);

    //Members
    std::deque<typename pcl::PointCloud<PointT>::Ptr> clouds_;///< Storage for later corraection and saving
    std::mutex tf_buffer_mutex_;
    tf2_ros::Buffer tf_buffer_;      ///< Stores the history of poses and allows to interpolate
    size_t num_poses_;               ///< Keep track of pose count (to show a good error message)
    std::mutex display_cloud_mutex_;
    typename pcl::PointCloud<PointT>::Ptr display_cloud_;///< Cloud that is displayed
    double voxelgrid_size_display_;  ///< Filter size for the display cloud
    double voxelgrid_size_save_;     ///< Filter size for the saved cloud
    double min_distance_;            ///< Discard points closer than this
    double max_distance_;            ///< Discard points farther than this
    std::string output_filename_;    ///< Where to save the cloud
    bool keep_high_res_;             ///< If false, do not store clouds. On save, save display cloud
    bool pause_;                     ///< Whether to ignore input data
    CloudVisualizer<PointT> viewer_;
    std::thread viewer_thread_;
    ros::Time last_pose_stamp_;
    std::string target_frame_;
    std::string source_frame_;
};

template<typename PointT>
CloudAccumulator<PointT>::CloudAccumulator(
    double voxelgrid_size_display,
    double voxelgrid_size_save,
    double min_distance,
    double max_distance,
    std::string output_filename,
    bool keep_high_res,
    bool start_paused) : 
tf_buffer_(ros::Duration(std::numeric_limits<int>::max(), 0)),
num_poses_(0),
display_cloud_(new pcl::PointCloud<PointT>()),
voxelgrid_size_display_(voxelgrid_size_display),
voxelgrid_size_save_(voxelgrid_size_save),
min_distance_(min_distance),
max_distance_(max_distance),
output_filename_(output_filename),
keep_high_res_(keep_high_res),
pause_(start_paused),
viewer_("Cloud Viewer"),
viewer_thread_(std::ref(viewer_)),
target_frame_("map"),
source_frame_("odom")
{
    viewer_.addCloudToViewer(display_cloud_); // so update can be called.
    ROS_INFO("Distance filter range: %.2fm - %.2fm", min_distance_, max_distance_);
    ROS_INFO("Voxel grid filter for display: %.3fm", voxelgrid_size_display_);
    ROS_INFO("Voxel grid filter for saving:  %.3fm", voxelgrid_size_save_);
}

template<typename PointT>
void CloudAccumulator<PointT>::pointCloudCallback(const typename pcl::PointCloud<PointT>::ConstPtr& pointcloud)
{
    if(pause_){ return; }
    static int counter = 0;
    ++counter;
    ROS_INFO_THROTTLE(10, "Received %d point cloud%s", counter, counter == 1 ? "" : "s");
    double timestamp = pointcloud->header.stamp * 1e-6; // PCL stamps are in microseconds.
    ROS_DEBUG("Storing cloud (%.3f)", timestamp);

    typename pcl::PointCloud<PointT>::Ptr cropped_cloud = distanceFiltered<PointT>(min_distance_, max_distance_, pointcloud);
    if(keep_high_res_) { clouds_.push_back(cropped_cloud); } // store for later corrections & saving

    const Eigen::Affine3f transformation = lookupTransform(timestamp);
    if(transformation.matrix()(0,0) == transformation.matrix()(0,0)) //not nan
    {
        typename pcl::PointCloud<PointT>::Ptr transformed_cloud(new pcl::PointCloud<PointT>());
        pcl::transformPointCloud(*cropped_cloud, *transformed_cloud, transformation);

        std::scoped_lock guard(display_cloud_mutex_);
        *transformed_cloud += *display_cloud_;
        // Voxelgridfilter to speedup display
        voxelGridFilter<PointT>(voxelgrid_size_display_, transformed_cloud, display_cloud_);
        viewer_.updateCloudInViewer(display_cloud_);
        ROS_INFO_ONCE("Change view to start displaying the clouds.");
    }
    else if(num_poses_ == 0)
    {
        ROS_INFO_THROTTLE(5,"Cannot transform current point cloud: No poses were received yet. "
                          "Is Stereo INS or SLAM running?");
    }
    else 
    {
        ROS_INFO_THROTTLE(5,"Cannot transform current point cloud: Pose not (yet) received\n"
                          "Current cloud stamp: %.3f\n"
                          "Latest pose stamp:   %.3f", timestamp, last_pose_stamp_.toSec());
    }
}

template<typename PointT>
void CloudAccumulator<PointT>::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& pointcloud)
{    
    // GET THE POINT CLOUD FROM THE ROS MESSAGE.
    auto input_cloud = pcl::PointCloud<PointT>().makeShared();
    pcl::fromROSMsg(*pointcloud, *input_cloud);
    return pointCloudCallback(input_cloud);
}

template<typename PointT>
void CloudAccumulator<PointT>::trajectoryCallback(const nav_msgs::PathConstPtr& path)
{
    ROS_INFO("Received Trajectory with %zu poses", path->poses.size());
    std::scoped_lock guard(tf_buffer_mutex_);
    tf_buffer_.clear();
    for(const geometry_msgs::PoseStamped& pose : path->poses)
    {
        tf_buffer_.setTransform(poseStampedMsgToTF(pose), "trajectory");
    }
}


template<typename PointT>
void CloudAccumulator<PointT>::poseCallback(const geometry_msgs::PoseStampedConstPtr& current_pose)
{
    if(pause_){ return; }
    ROS_INFO_ONCE("Received first live pose");
    std::scoped_lock guard(tf_buffer_mutex_);
    tf_buffer_.setTransform(poseStampedMsgToTF(*current_pose), "current_pose");
    last_pose_stamp_ = current_pose->header.stamp;
    ++num_poses_;
}

template<typename PointT>
Eigen::Affine3f CloudAccumulator<PointT>::lookupTransform(const double timestamp)
{
    ros::Time rosstamp;
    rosstamp.fromSec(timestamp); //pcl timestamp is in microseconds

    //cannot use waitForTransform because of the concurrent use of the tf_buffer
    try
    {
        ros::Time endtime = ros::Time::now() + ros::Duration(0.25);
        do
        {
            { //lock-scope
                std::scoped_lock guard(tf_buffer_mutex_);
                if(tf_buffer_.canTransform(target_frame_, source_frame_, rosstamp))
                {
                    return toAffine(tf_buffer_.lookupTransform(target_frame_, source_frame_, rosstamp)); //target, source
                }
            }
            usleep(10000);
        }
        while(endtime > ros::Time::now());
    }
    catch(const tf2::LookupException& ex) { ROS_WARN("%s", ex.what()); }
    catch(const tf2::ExtrapolationException& ex) {  ROS_WARN("%s", ex.what()); }

    Eigen::Affine3f invalid_result;
    invalid_result.matrix().setConstant(std::numeric_limits<double>::quiet_NaN());
    return invalid_result;
}

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr CloudAccumulator<PointT>::rebuildCloud()
{
    ROS_INFO("(Re-)merging all stored clouds with latest poses.");
    int counter = 0;
    typename pcl::PointCloud<PointT>::Ptr merged(new pcl::PointCloud<PointT>());
    for(const auto& cloud : clouds_)
    //for(std::map<double, pcl::PointCloud<PointT>::ConstPtr>::iterator it = cloud_map_.begin(); it != cloud_map_.end(); ++it)
    {
        double timestamp = cloud->header.stamp * 1e-6;//pcl stamps are in microseconds.
        const Eigen::Affine3f transformation = lookupTransform(timestamp);
        if(transformation.matrix()(0,0) == transformation.matrix()(0,0)) //not nan
        {
            ++counter;
            pcl::PointCloud<PointT> transformed_cloud;
            pcl::transformPointCloud(*cloud, transformed_cloud, transformation);
            *merged += transformed_cloud;
            if(counter % 10 == 0){ ROS_INFO("Merged %d of %zu clouds.", counter, clouds_.size()); }
        }
        else { ROS_DEBUG("Skipped cloud at %.3f in merge, cannot get its position", timestamp); }
    }
    ROS_INFO("Merged %d/%zu clouds.", counter, clouds_.size());

    typename pcl::PointCloud<PointT>::Ptr voxel_filtered(new pcl::PointCloud<PointT>());
    voxelGridFilter<PointT>(voxelgrid_size_save_, merged, voxel_filtered);
    return voxel_filtered;
}

// template<typename PointT>
// bool CloudAccumulator<PointT>::saveCloud(const std_srvs::Empty::Request& req, const std_srvs::Empty::Request& resp)
// {
//     if(keep_high_res_) // consider slam corrections
//     {
//         typename pcl::PointCloud<PointT>::Ptr merged = rebuildCloud();
//         pcl::io::savePCDFileASCII(output_filename_, *merged);

//         // Now show the saved cloud
//         // (it will quickly get downfiltered in the point cloud callback though if not paused)
//         std::scoped_lock guard(display_cloud_mutex_);
//         display_cloud_ = merged;
//         viewer_.addCloudToViewer(display_cloud_);
//     }
//     else 
//     {
//         // just save what we have. 
//         pcl::io::savePCDFileASCII(output_filename_, *display_cloud_); 
//     }

//     ROS_INFO("Saved point cloud to %s", output_filename_.c_str());
//     return true;
// }

}//namespace smc
