#include <uw_detection/YoloCloudNode.h>

#include <chrono>
#include <cv_bridge/cv_bridge.h>
#include <fstream>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <ros/package.h>
#include <ros/ros.h>
#include <uw_detection/logging.h>
#include <uw_detection/yololayer.h>
#include <uw_detection/yolov3.h>
#include <utility>
#include <object_cloud/PointCloudConstructor.h>
#define NMS_THRESH 0.4


using namespace nvinfer1;
using std::vector;
using std::pair;

// stuff we know about the network and the input/output blobs
static const int INPUT_H = Yolo::INPUT_H;
static const int INPUT_W = Yolo::INPUT_W;
static const int OUTPUT_SIZE = 1000 * 7 + 1;  // we assume the yololayer outputs
// no more than 1000 boxes that
// conf >= 0.1

static Logger gLogger;
// prepare input data ---------------------------
static float data[3 * INPUT_H * INPUT_W];
// for (int i = 0; i < 3 * INPUT_H * INPUT_W; i++)
//    data[i] = 1.0;
static float prob[OUTPUT_SIZE];

inline bool is_moving(const nav_msgs::Odometry::ConstPtr& odom)
{
  return Eigen::Vector3f(odom->twist.twist.linear.x, odom->twist.twist.linear.y, odom->twist.twist.linear.z).norm() >
             0.05 ||
         Eigen::Vector3f(odom->twist.twist.angular.x, odom->twist.twist.angular.y, odom->twist.twist.angular.z).norm() >
             0.05;
}

YoloCloudNode::YoloCloudNode(ros::NodeHandle node, const Eigen::Matrix3f& camera_intrinsics)
  : ObjectCloudNode(node, camera_intrinsics)
{
  cudaSetDevice(0);
  size_t size{ 0 };

  char* trtModelStream{ nullptr };

  std::string path = ros::package::getPath("uw_detection");
  std::ifstream file(path + "/share/yolov3.engine", std::ios::binary);
  if (file.good())
  {
    file.seekg(0, file.end);
    size = file.tellg();
    file.seekg(0, file.beg);
    trtModelStream = new char[size];
    assert(trtModelStream);
    file.read(trtModelStream, size);
    file.close();
  }

  IRuntime* runtime = createInferRuntime(gLogger);
  assert(runtime != nullptr);
  ICudaEngine* engine = runtime->deserializeCudaEngine(trtModelStream, size);
  assert(engine != nullptr);
  context = engine->createExecutionContext();
  assert(context != nullptr);
  delete[] trtModelStream;
}

void YoloCloudNode::runDetector(cv_bridge::CvImageConstPtr rgb_image, vector<ImageBoundingBox>& bboxes)
{
  cv::Mat img = rgb_image->image;
  cv::Mat pr_img = preprocess_img(img);

  for (int i = 0; i < INPUT_H * INPUT_W; i++)
  {
    data[i] = pr_img.at<cv::Vec3b>(i)[2] / 255.0;
    data[i + INPUT_H * INPUT_W] = pr_img.at<cv::Vec3b>(i)[1] / 255.0;
    data[i + 2 * INPUT_H * INPUT_W] = pr_img.at<cv::Vec3b>(i)[0] / 255.0;
  }

  auto start = std::chrono::system_clock::now();
  doInference(*this->context, data, prob, 1);
  auto end = std::chrono::system_clock::now();
  std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;
  std::vector<Yolo::Detection> res;
  nms(res, prob, NMS_THRESH);
  for (int i = 0; i < 20; i++)
  {
    std::cout << prob[i] << ",";
  }
  std::cout << res.size() << std::endl;
  for (size_t j = 0; j < res.size(); j++)
  {
    cv::Rect r = get_rect(rgb_image->image, res[j].bbox);
    ImageBoundingBox bbox;
    bbox.x = r.x;
    bbox.y = r.y;
    bbox.width = r.width;
    bbox.height = r.height;
    bbox.label = std::to_string(res[j].class_id);
    bboxes.push_back(bbox);
  }
}

void YoloCloudNode::dataCallback(const sensor_msgs::Image::ConstPtr& rgb_msg,
                                 const sensor_msgs::Image::ConstPtr& depth_msg,
                                 const nav_msgs::Odometry::ConstPtr& odom)
{
  std::lock_guard<std::mutex> global_lock(global_mutex);

  // Record start time
  auto start = std::chrono::high_resolution_clock::now();

  received_first_message = true;

  cv_bridge::CvImageConstPtr rgb = cv_bridge::toCvShare(rgb_msg, sensor_msgs::image_encodings::BGR8);
  vector<ImageBoundingBox> bboxes;
  runDetector(rgb, bboxes);

  geometry_msgs::TransformStamped cam_to_map_transform;
  geometry_msgs::TransformStamped base_to_map_transform;
  try
  {
    cam_to_map_transform =
        tf_buffer.lookupTransform("map", rgb_msg->header.frame_id, rgb_msg->header.stamp, ros::Duration(0.02));
    // base_to_map_transform = tf_buffer.lookupTransform("map", "base_link", rgb_image->header.stamp,
    //                                              ros::Duration(0.02));
    base_to_map_transform = cam_to_map_transform;
  }
  catch (tf2::TransformException& ex)
  {
    ROS_ERROR("%s", ex.what());
    return;
  }

  Eigen::Affine3f cam_to_map = tf2::transformToEigen(cam_to_map_transform).cast<float>();
  Eigen::Affine3f base_to_map = tf2::transformToEigen(base_to_map_transform).cast<float>();

  cv_bridge::CvImagePtr depth(new cv_bridge::CvImage());
  prepareDepthData(depth_msg, depth);

  processPointCloudRequests(depth);

  // Parts of the depth image that have objects
  // This will be useful for constructing a region-of-interest Point Cloud
  cv::Mat depth_masked = cv::Mat::zeros(depth_msg->height, depth_msg->width, CV_16UC1);

  vector<pair<ImageBoundingBox, Object>> detection_objects;

  for (const auto& bbox : bboxes)
  {
    // If the bounding box is at the edge of the image, ignore it
    if (bbox.x == 0 || bbox.x + bbox.width >= rgb->image.cols || bbox.y == 0 || bbox.y + bbox.height >= rgb->image.rows)
    {
      continue;
    }

    // 10 pixel buffer
    int min_x = std::max(0., bbox.x - 10.);
    int min_y = std::max(0., bbox.y - 10.);
    int max_x = std::min(rgb->image.cols - 1., bbox.x + bbox.width + 20.);
    int max_y = std::min(rgb->image.rows - 1., bbox.y + bbox.height + 20.);
    cv::Rect region(min_x, min_y, max_x - min_x, max_y - min_y);
    depth->image(region).copyTo(depth_masked(region));

    std::pair<bool, Object> ret = object_cloud.addObject(bbox, rgb->image, depth->image, cam_to_map);
    if (!ret.second.invalid())
    {
      detection_objects.emplace_back(bbox, ret.second);
    }

    // If object was added, add to knowledge base
    bool newObj = ret.first;
    if (newObj)
    {
      // std::cout << "New Object " << ret.second.position << std::endl;
      addToLtmc(ret.second);
    }
  }

  // OCTOMAP UPDATE--------------------------

  // If the robot is moving then don't update Octomap
  if (!is_moving(odom))
  {
    // Use depthMasked to construct a ROI Point Cloud for use with Octomap
    // Without this, Octomap takes forever
    float inf = std::numeric_limits<float>::infinity();
    Eigen::Vector2f nobounds(-inf, inf);
    octomap::Pointcloud cloud = PointCloudConstructor::construct(camera_intrinsics, depth_masked, cam_to_map, 3.,
                                                                 nobounds, nobounds, Eigen::Vector2f(0., inf));
    if (cloud.size() != 0)
    {
      // Insert ROI PointCloud into Octree
      Eigen::Vector3f origin = cam_to_map * Eigen::Vector3f::Zero();
      octree.insertPointCloud(cloud, octomap::point3d(origin(0), origin(1), origin(2)), 3,
                              false,  // We don't want lazy updates
                              true);  // Discretize speeds it up by approximating

      updateBoundingBoxes(detection_objects, cam_to_map);
    }
  }
#if (VISUALIZE)
  cv::Mat rgb_copy = rgb->image;
  if (viz_detections_pub.getNumSubscribers() > 0)
  {
    for (const auto& d : detection_objects)
    {
      cv::rectangle(rgb_copy, d.first, cv::Scalar(0, 0, 255), 3);
    }
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rgb->image).toImageMsg();
    viz_detections_pub.publish(msg);
  }

#endif
  // Record end time
  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;
  std::cout << "Elapsed time: " << elapsed.count() << " s\n";
}
