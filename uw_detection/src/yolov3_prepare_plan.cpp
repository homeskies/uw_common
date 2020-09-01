#include "NvInfer.h"
#include "cuda_runtime_api.h"
#include <chrono>
#include <dirent.h>
#include <fstream>
#include <iostream>
#include <map>
#include <opencv2/opencv.hpp>
#include <ros/package.h>
#include <sstream>
#include <uw_detection/yololayer.h>
#include <uw_detection/yolov3.h>
#include <vector>

#define DEVICE 0  // GPU id

using namespace nvinfer1;

int main(int argc, char** argv)
{
  cudaSetDevice(DEVICE);
  IHostMemory* modelStream{ nullptr };
  std::string path = ros::package::getPath("uw_detection");
  APIToModel(1, &modelStream, path + "/share/yolov3.wts");
  assert(modelStream != nullptr);
  std::ofstream p(path + "/share/yolov3.engine", std::ios::binary);
  if (!p)
  {
    std::cerr << "could not open plan output file" << std::endl;
    return -1;
  }
  p.write(reinterpret_cast<const char*>(modelStream->data()), modelStream->size());
  modelStream->destroy();
  return 0;
}
