#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <vector>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <dirent.h>
#include "NvInfer.h"
#include "cuda_runtime_api.h"
#include "yolov3-spp/yololayer.h"
#include <uw_detection/yolov3-spp.h>
#include <ros/package.h>


#define DEVICE 0  // GPU id


using namespace nvinfer1;

int main(int argc, char** argv) {
  cudaSetDevice(DEVICE);
  IHostMemory *modelStream{nullptr};
  std::string path = ros::package::getPath("uw_detection");
  APIToModel(1, &modelStream, path + "/share/yolov3-spp_ultralytics68.wts");
  assert(modelStream != nullptr);
  std::ofstream p(path+ "/share/yolov3-spp.engine", std::ios::binary);
  if (!p) {
    std::cerr << "could not open plan output file" << std::endl;
    return -1;
  }
  p.write(reinterpret_cast<const char *>(modelStream->data()), modelStream->size());
  modelStream->destroy();
  return 0;
}
