#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <uw_detection/yolov3-spp.h>
#include <uw_detection/logging.h>
#include <chrono>
#include <uw_detection/yololayer.h>
#include <ros/package.h>
#include <fstream>
#define NMS_THRESH 0.4

using namespace nvinfer1;

// stuff we know about the network and the input/output blobs
static const int INPUT_H = Yolo::INPUT_H;
static const int INPUT_W = Yolo::INPUT_W;
static const int OUTPUT_SIZE = 1000 * 7 + 1;  // we assume the yololayer outputs no more than 1000 boxes that conf >= 0.1

static Logger gLogger;
// prepare input data ---------------------------
static float data[3 * INPUT_H * INPUT_W];
//for (int i = 0; i < 3 * INPUT_H * INPUT_W; i++)
//    data[i] = 1.0;
static float prob[OUTPUT_SIZE];

static IExecutionContext* context;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    auto img = cv_bridge::toCvShare(msg, "bgr8")->image;

    cv::Mat pr_img = preprocess_img(img);

    for (int i = 0; i < INPUT_H * INPUT_W; i++) {
      data[i] = pr_img.at<cv::Vec3b>(i)[2] / 255.0;
      data[i + INPUT_H * INPUT_W] = pr_img.at<cv::Vec3b>(i)[1] / 255.0;
      data[i + 2 * INPUT_H * INPUT_W] = pr_img.at<cv::Vec3b>(i)[0] / 255.0;
    }

    auto start = std::chrono::system_clock::now();
    doInference(*context, data, prob, 1);
    auto end = std::chrono::system_clock::now();
    std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;
    std::vector<Yolo::Detection> res;
    nms(res, prob, NMS_THRESH);
    for (int i=0; i<20; i++) {
      std::cout << prob[i] << ",";
    }
    std::cout << res.size() << std::endl;
    for (size_t j = 0; j < res.size(); j++) {
      float *p = (float*)&res[j];
      for (size_t k = 0; k < 7; k++) {
        std::cout << p[k] << ", ";
      }
      std::cout << std::endl;
      cv::Rect r = get_rect(img, res[j].bbox);
      cv::rectangle(img, r, cv::Scalar(0x27, 0xC1, 0x36), 2);
      cv::putText(img, std::to_string((int)res[j].class_id), cv::Point(r.x, r.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
    }
    cv::imshow("view", img);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  cudaSetDevice(0);
  size_t size{0};

  char *trtModelStream{nullptr};

  ros::init(argc, argv, "image_listener");
    std::string path = ros::package::getPath("uw_detection");
  std::ifstream file(path + "/share/yolov3-spp.engine", std::ios::binary);
  if (file.good()) {
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
  ros::NodeHandle nh;
  cv::namedWindow("view");
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/head_camera/rgb/image_raw", 1, imageCallback);
  ros::spin();
  cv::destroyWindow("view");
}