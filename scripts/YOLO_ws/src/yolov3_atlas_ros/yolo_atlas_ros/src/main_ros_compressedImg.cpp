/**
* Copyright 2020 Huawei Technologies Co., Ltd
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at

* http://www.apache.org/licenses/LICENSE-2.0

* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.

* File main.cpp
* Description: dvpp sample main func
*/

#include <ros/ros.h>
#include <ros/package.h>

#include <sensor_msgs/CompressedImage.h>

#include <cv_bridge/cv_bridge.h>

//接收检测器返回信息
#include <ros_rs_msgs/DetectionMessage.h>
#include <ros_rs_msgs/DetectionMessages.h>

#include <iostream>
#include <dirent.h>

#include "object_detect.h"
#include "utils.h"

using namespace std;

uint32_t kModelWidth = 416;
uint32_t kModelHeight = 416;
std::string pkg_path = ros::package::getPath("yolo_atlas_ros");
std::string model_path = pkg_path + "/model/yolov3.om";
const char *kPkgPath = pkg_path.c_str();
const char *kModelPath = model_path.c_str();

//Instantiate the target detection class with the parameters of the classification model path and the required width and height of the model input
ObjectDetect detect(kPkgPath, kModelPath, kModelWidth, kModelHeight);

ros::Publisher pub_results;

void img_callback(const sensor_msgs::CompressedImageConstPtr &img_msg) {
    	double start_t = ros::Time::now().toSec();
    cv_bridge::CvImagePtr cv_ptr;
    try {

//        cout << "starttime = " << to_string(ros::Time::now().toSec()) << endl;
        cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat input_img = cv_ptr->image;

        //图像预处理&&图像格式转换为davinci框架下模型输入数据
        Result ret = detect.Preprocess(input_img);
        if (ret != SUCCESS) {
            ERROR_LOG("Read file failed, continue to read next");
        }
        //模型推理，并通过指针获取推理结果
        aclmdlDataset *inferenceOutput = nullptr;
        ret = detect.Inference(inferenceOutput);
        if ((ret != SUCCESS) || (inferenceOutput == nullptr)) {
            ERROR_LOG("Inference model inference output data failed");
        }

        //Parses the inference output and sends the inference class, location, confidence, and image to the Presenter Server for display
        vector<BoundingBox> detectResults;
        ret = detect.Postprocess(input_img, inferenceOutput, detectResults);
        ros_rs_msgs::DetectionMessages detection_msgs;
        detection_msgs.header = img_msg->header;
        if (ret != SUCCESS) {
            ERROR_LOG("Process model inference output data failed");
        } else {
//            cout << "endtime = " << to_string(ros::Time::now().toSec()) << endl;
            for (auto &detectResult: detectResults) {
                //收集检测信息
                ros_rs_msgs::DetectionMessage temp;
                temp.x1 = detectResult.lt.x;
                temp.y1 = detectResult.lt.y;
                temp.x2 = detectResult.rb.x;
                temp.y2 = detectResult.rb.y;
                temp.class_pred = detectResult.attribute;
                temp.label = detectResult.result_text;
                temp.score = detectResult.score;
                detection_msgs.data.push_back(temp);
                detection_msgs.detection_num++;
            }
//            if (detection_msgs.detection_num != 0) {
//                pub_results.publish(detection_msgs);
//            }
        }
        pub_results.publish(detection_msgs);
        double end_t = ros::Time::now().toSec();
        cout << "YOLO cost: " << to_string(end_t - start_t) << " s" << endl;
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "yolo_atlas");
    ros::NodeHandle nh;

    ros::Subscriber sub_img = nh.subscribe("/realsense_2/color/image_raw/compressed", 1, img_callback);
    pub_results = nh.advertise<ros_rs_msgs::DetectionMessages>("/untracked_info", 100);

//    cout << "kModelPath:" << endl << kModelPath << endl;
    //Initializes the ACL resource for categorical reasoning, loads the model and requests the memory used for reasoning input
    Result ret = detect.Init();
    if (ret != SUCCESS) {
        ROS_ERROR("Classification Init resource failed");
        return FAILED;
    }

    ros::spin();
}
