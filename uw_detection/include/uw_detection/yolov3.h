#pragma once
#include "NvInfer.h"
#include <uw_detection/yololayer.h>
#include <opencv2/opencv.hpp>

void APIToModel(unsigned int maxBatchSize, nvinfer1::IHostMemory** modelStream, const std::string& weights_path);

void doInference(nvinfer1::IExecutionContext& context, float* input, float* output, int batchSize);

cv::Mat preprocess_img(cv::Mat& img);

cv::Rect get_rect(cv::Mat& img, float bbox[4]);

void nms(std::vector<Yolo::Detection>& res, float* output, float nms_thresh);