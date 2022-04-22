// Standard includes.
#include <mutex>
#include <thread>

// ROS includes.
#include <pcl/visualization/pcl_visualizer.h>
#include <ros/ros.h>

namespace smc
{

/// Displays one cloud using the PCLVisualizer.
/// This should be used as thread.
template<typename PointT>
class CloudVisualizer 
{
public:
    /// Creates the viewer object
    CloudVisualizer(const char* name);

    /// Viewer update loop, will be called when viewer thread is started
    void operator()();

    /// Thread-safe addition/replacement of the cloud
    void addCloudToViewer(const typename pcl::PointCloud<PointT>::Ptr& pointcloud);

    /// Thread-safe update to the cloud
    void updateCloudInViewer(const typename pcl::PointCloud<PointT>::Ptr& pointcloud);

    /// Signal thread to stop the visualization loop
    void stop() { stop_ = true; }

private:
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
    bool stop_;
    std::mutex viewer_mutex_;
};

// Constructor.
template<typename PointT>
CloudVisualizer<PointT>::CloudVisualizer(const char* name) : 
viewer_(new pcl::visualization::PCLVisualizer(name)),
stop_(false)
{
    viewer_->setBackgroundColor(0.2, 0.2, 0.2); //dark gray
    viewer_->addCoordinateSystem(0.1); //make a small coordinate system where the world pose is
    viewer_->initCameraParameters();
    viewer_->setCameraPosition(-1,0,0,  //1m behind the origin (x is the viewing direction).
                                0,0,0,  //look towards the origin.
                                0,0,1); //Z is up
}

template<typename PointT>
void CloudVisualizer<PointT>::operator()()
{
    while(!viewer_->wasStopped() && ros::ok())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if(!ros::ok()) break;
        std::scoped_lock guard(viewer_mutex_);
        viewer_->spinOnce();
    }
}

template<typename PointT>
void CloudVisualizer<PointT>::addCloudToViewer(const typename pcl::PointCloud<PointT>::Ptr& pointcloud)
{
    std::scoped_lock guard(viewer_mutex_);
    viewer_->removeAllPointClouds();

    if constexpr (std::is_same_v<PointT, pcl::PointXYZRGB>)
    {
        pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(pointcloud);
        viewer_->addPointCloud<PointT>(pointcloud, rgb);
        viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3);
    }
    else
    {
        viewer_->addPointCloud<PointT>(pointcloud);
    }
}

template<typename PointT>
void CloudVisualizer<PointT>::updateCloudInViewer(const typename pcl::PointCloud<PointT>::Ptr& pointcloud)
{
    std::scoped_lock guard(viewer_mutex_);
    if constexpr (std::is_same_v<PointT, pcl::PointXYZRGB>)
    {
        pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(pointcloud);
        viewer_->addPointCloud<PointT>(pointcloud, rgb);
    }
    else
    {
        viewer_->addPointCloud<PointT>(pointcloud);
    }
}

} //namespace smc
