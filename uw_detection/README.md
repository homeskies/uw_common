# uw_detection

Wrappers for detection models. Tools for preparing data and training.

TensorRT YOLOv3 implementations taken from [tensorrtx](https://github.com/wang-xinyu/tensorrtx).

## Usage

### YOLO

All of the YOLO targets described here are only available when CMake can detect a working CUDA and TensorRT installation. Ensure these are set up correctly before beginning. They can't be run on the CPU

These implementations come with a helper for loading weights from the Ultralytics PyTorch trained models. Follow the steps in [their readme](https://github.com/wang-xinyu/tensorrtx/blob/master/yolov3/README.md) (the process involves using one of the `gen_wts.py` scripts to load the Torch graph and dump it into the necessary format). You may need to change the CUDA device set in the script.

TensorRT requires that you precompute an inference graph ("engine") before you can actually use the weights you've obtained. The `*_prepare_plan.cpp` targets will take care of this. They expect the weights file to be placed in the `share` folder at the root of the package. When they've run successfully (only required once), they'll output the serialized engine into the `share` folder. Note that if you're using a customized class list, you'll need to adjust the definition for the number of classes in "yololayer.h".

Now you can test the model on a ROS image topic using `yolo_ros_image_node.cpp`. It's configured to take images from Fetch's head camera topic and feed them through YOLOv3, outputting the results to the console and drawing them in a CV window. It'll work in simulation without changes assuming you've followed the previous steps for the Ultralytics provided yolov3 checkpoint. 