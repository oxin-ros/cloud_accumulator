// ROS includes.
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
// Eigen includes.
#include <Eigen/Geometry>
// PCL includes.
#include <pcl/point_cloud.h>

/****************************** Utility functions ********************************************/

inline geometry_msgs::TransformStamped poseStampedMsgToTF(const geometry_msgs::PoseStamped& msg)
{
    geometry_msgs::TransformStamped bt;
    bt.transform.rotation = msg.pose.orientation;
    bt.transform.translation.x = msg.pose.position.x;
    bt.transform.translation.y = msg.pose.position.y;
    bt.transform.translation.z = msg.pose.position.z;
    bt.header.stamp = msg.header.stamp;
    bt.header.frame_id = msg.header.frame_id;
    bt.child_frame_id = "camera";
    return bt;
}

inline Eigen::Affine3f toAffine(const geometry_msgs::TransformStamped& tf_stamped)
{
    const geometry_msgs::Vector3& t = tf_stamped.transform.translation; //just abbreviate
    const geometry_msgs::Quaternion& q = tf_stamped.transform.rotation; //just abbreviate

    Eigen::Affine3f result = Eigen::Affine3f::Identity();
    result.translation() << t.x, t.y, t.z;
    Eigen::Quaternionf rot(q.w, q.x, q.y, q.z);
    result.rotate(rot);
    return result;
}


/// Filter if size >= 0, otherwise just copy the pointer
template<typename PointT>
void voxelGridFilter(
    double size, 
    typename pcl::PointCloud<PointT>::Ptr& input, 
    typename pcl::PointCloud<PointT>::Ptr& output)
{
    if(size <= 0.0){ output = input; }
    else
    {
        pcl::VoxelGrid<PointT> vgf;
        vgf.setInputCloud(input);
        vgf.setLeafSize(size, size, size);
        vgf.filter(*output);
    }
}

/// Filter cloud along the optical axis (Z)
/// Filter if min < max, otherwise copy (deep, not only the pointer)
template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr distanceFiltered(
    double min, 
    double max, 
    const typename pcl::PointCloud<PointT>::ConstPtr& input)
{
    typename pcl::PointCloud<PointT>::Ptr output(new pcl::PointCloud<PointT>());
    if(min >= max){ *output = *input; }
    else
    {
        pcl::PassThrough<PointT> pass;
        pass.setInputCloud(input);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(min, max);
        pass.filter(*output);
    }
    return output;
}