#pragma once

#include <nav_msgs/Odometry.h>
#include <object_cloud/ObjectCloudNode.h>

namespace nvinfer1
{
class IExecutionContext;
}

class YoloCloudNode : public ObjectCloudNode
{
protected:
  nvinfer1::IExecutionContext* context;

  void runDetector(cv_bridge::CvImageConstPtr rgb_image, std::vector<ImageBoundingBox>& bboxes);

public:
  explicit YoloCloudNode(ros::NodeHandle node, const Eigen::Matrix3f& camera_intrinsics);

  void dataCallback(const sensor_msgs::Image::ConstPtr& rgb_msg, const sensor_msgs::Image::ConstPtr& depth_msg,
                    const nav_msgs::Odometry::ConstPtr& odom);
};
