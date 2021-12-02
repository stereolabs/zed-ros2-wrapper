/******************************************************************************** 
 * MIT License
 * 
 * Copyright (c) 2020 Stereolabs
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 ********************************************************************************/

#include "sl_tools.h"

#include <sstream>
#include <sys/stat.h>
#include <vector>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <float.h>

namespace sl_tools {

int checkCameraReady(unsigned int serial_number) {
    int id = -1;
    auto f = sl::Camera::getDeviceList();

    for (auto& it : f)
        if (it.serial_number == serial_number &&
                it.camera_state == sl::CAMERA_STATE::AVAILABLE) {
            id = it.id;
        }

    return id;
}

sl::DeviceProperties getZEDFromSN(unsigned int serial_number) {
    sl::DeviceProperties prop;
    auto f = sl::Camera::getDeviceList();

    for (auto& it : f) {
        if (it.serial_number == serial_number &&
                it.camera_state == sl::CAMERA_STATE::AVAILABLE) {
            prop = it;
        }
    }

    return prop;
}

std::vector<float> convertRodrigues(sl::float3 r) {
    float theta = sqrt(r.x * r.x + r.y * r.y + r.z * r.z);

    std::vector<float> R = {1.0f, 0.0f, 0.0f,
                            0.0f, 1.0f, 0.0f,
                            0.0f, 0.0f, 1.0f
                           };

    if (theta < DBL_EPSILON) {
        return R;
    } else {
        float c = cos(theta);
        float s = sin(theta);
        float c1 = 1.f - c;
        float itheta = theta ? 1.f / theta : 0.f;

        r *= itheta;

        std::vector<float> rrt = {1.0f, 0.0f, 0.0f,
                                  0.0f, 1.0f, 0.0f,
                                  0.0f, 0.0f, 1.0f
                                 };

        float* p = rrt.data();
        p[0] = r.x * r.x;
        p[1] = r.x * r.y;
        p[2] = r.x * r.z;
        p[3] = r.x * r.y;
        p[4] = r.y * r.y;
        p[5] = r.y * r.z;
        p[6] = r.x * r.z;
        p[7] = r.y * r.z;
        p[8] = r.z * r.z;

        std::vector<float> r_x = {1.0f, 0.0f, 0.0f,
                                  0.0f, 1.0f, 0.0f,
                                  0.0f, 0.0f, 1.0f
                                 };
        p = r_x.data();
        p[0] = 0;
        p[1] = -r.z;
        p[2] = r.y;
        p[3] = r.z;
        p[4] = 0;
        p[5] = -r.x;
        p[6] = -r.y;
        p[7] = r.x;
        p[8] = 0;

        // R = cos(theta)*I + (1 - cos(theta))*r*rT + sin(theta)*[r_x]

        sl::Matrix3f eye;
        eye.setIdentity();

        sl::Matrix3f sl_R(R.data());
        sl::Matrix3f sl_rrt(rrt.data());
        sl::Matrix3f sl_r_x(r_x.data());

        sl_R = eye * c + sl_rrt * c1 + sl_r_x * s;

        R[0] = sl_R.r00;
        R[1] = sl_R.r01;
        R[2] = sl_R.r02;
        R[3] = sl_R.r10;
        R[4] = sl_R.r11;
        R[5] = sl_R.r12;
        R[6] = sl_R.r20;
        R[7] = sl_R.r21;
        R[8] = sl_R.r22;
    }

    return R;
}

bool file_exist(const std::string& name) {
    struct stat buffer;
    return (stat(name.c_str(), &buffer) == 0);
}

std::string getSDKVersion(int& major, int& minor, int& sub_minor) {
    std::string ver = sl::Camera::getSDKVersion().c_str();
    std::vector<std::string> strings;
    std::istringstream f(ver);
    std::string s;

    while (getline(f, s, '.')) {
        strings.push_back(s);
    }

    major = 0;
    minor = 0;
    sub_minor = 0;

    switch (strings.size()) {
    case 3:
        sub_minor = std::stoi(strings[2]);

    case 2:
        minor = std::stoi(strings[1]);

    case 1:
        major = std::stoi(strings[0]);
    }

    return ver;
}

rclcpp::Time slTime2Ros(sl::Timestamp t, rcl_clock_type_t clock_type) {
    uint64_t ts_nsec = t.getNanoseconds();
    uint32_t sec = static_cast<uint32_t>(ts_nsec / 1000000000);
    uint32_t nsec = static_cast<uint32_t>(ts_nsec % 1000000000);
    return rclcpp::Time(sec, nsec, clock_type);
}

std::shared_ptr<sensor_msgs::msg::Image> imageToROSmsg(sl::Mat& img, std::string frameId, rclcpp::Time t) {

    std::shared_ptr<sensor_msgs::msg::Image> imgMessage = std::make_shared<sensor_msgs::msg::Image>();

    imgMessage->header.stamp = t;
    imgMessage->header.frame_id = frameId;
    imgMessage->height = img.getHeight();
    imgMessage->width = img.getWidth();

    int num = 1; // for endianness detection
    imgMessage->is_bigendian = !(*(char*)&num == 1);

    imgMessage->step = img.getStepBytes();

    size_t size = imgMessage->step * imgMessage->height;

    uint8_t* data_ptr=nullptr;

    sl::MAT_TYPE dataType = img.getDataType();

    switch (dataType) {
    case sl::MAT_TYPE::F32_C1: /**< float 1 channel.*/
        imgMessage->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
        data_ptr = (uint8_t*)img.getPtr<sl::float1>();
        imgMessage->data = std::vector<uint8_t>(data_ptr, data_ptr+size);
        break;

    case sl::MAT_TYPE::F32_C2: /**< float 2 channels.*/
        imgMessage->encoding = sensor_msgs::image_encodings::TYPE_32FC2;
        data_ptr = (uint8_t*)img.getPtr<sl::float2>();
        imgMessage->data = std::vector<uint8_t>(data_ptr, data_ptr+size);
        break;

    case sl::MAT_TYPE::F32_C3: /**< float 3 channels.*/
        imgMessage->encoding = sensor_msgs::image_encodings::TYPE_32FC3;
        data_ptr = (uint8_t*)img.getPtr<sl::float3>();
        imgMessage->data = std::vector<uint8_t>(data_ptr, data_ptr+size);
        break;

    case sl::MAT_TYPE::F32_C4: /**< float 4 channels.*/
        imgMessage->encoding = sensor_msgs::image_encodings::TYPE_32FC4;
        data_ptr = (uint8_t*)img.getPtr<sl::float4>();
        imgMessage->data = std::vector<uint8_t>(data_ptr, data_ptr+size);
        break;

    case sl::MAT_TYPE::U8_C1: /**< unsigned char 1 channel.*/
        imgMessage->encoding = sensor_msgs::image_encodings::MONO8;
        data_ptr = (uint8_t*)img.getPtr<sl::uchar1>();
        imgMessage->data = std::vector<uint8_t>(data_ptr, data_ptr+size);
        break;

    case sl::MAT_TYPE::U8_C2: /**< unsigned char 2 channels.*/
        imgMessage->encoding = sensor_msgs::image_encodings::TYPE_8UC2;
        data_ptr = (uint8_t*)img.getPtr<sl::uchar2>();
        imgMessage->data = std::vector<uint8_t>(data_ptr, data_ptr+size);
        break;

    case sl::MAT_TYPE::U8_C3: /**< unsigned char 3 channels.*/
        imgMessage->encoding = sensor_msgs::image_encodings::BGR8;        
        data_ptr = (uint8_t*)img.getPtr<sl::uchar3>();
        imgMessage->data = std::vector<uint8_t>(data_ptr, data_ptr+size);
        break;

    case sl::MAT_TYPE::U8_C4: /**< unsigned char 4 channels.*/
        imgMessage->encoding = sensor_msgs::image_encodings::BGRA8;
        data_ptr = (uint8_t*)img.getPtr<sl::uchar4>();
        imgMessage->data = std::vector<uint8_t>(data_ptr, data_ptr+size);
        break;
    }

    return imgMessage;
}

std::shared_ptr<sensor_msgs::msg::Image> imagesToROSmsg(sl::Mat& left, sl::Mat& right, std::string frameId, rclcpp::Time t)
{
    std::shared_ptr<sensor_msgs::msg::Image> imgMsgPtr = std::make_shared<sensor_msgs::msg::Image>();

    if (left.getWidth() != right.getWidth() ||
            left.getHeight() != right.getHeight() ||
            left.getChannels() != right.getChannels() ||
            left.getDataType() != right.getDataType()) {
        return imgMsgPtr;
    }

    imgMsgPtr->header.stamp = t;
    imgMsgPtr->header.frame_id = frameId;
    imgMsgPtr->height = left.getHeight();
    imgMsgPtr->width = 2 * left.getWidth();

    int num = 1; // for endianness detection
    imgMsgPtr->is_bigendian = !(*(char*)&num == 1);

    imgMsgPtr->step = 2 * left.getStepBytes();

    size_t size = imgMsgPtr->step * imgMsgPtr->height;
    imgMsgPtr->data.resize(size);

    sl::MAT_TYPE dataType = left.getDataType();

    int dataSize = 0;
    char* srcL;
    char* srcR;

    switch (dataType) {
    case sl::MAT_TYPE::F32_C1: /**< float 1 channel.*/
        imgMsgPtr->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
        dataSize = sizeof(float);
        srcL = (char*)left.getPtr<sl::float1>();
        srcR = (char*)right.getPtr<sl::float1>();
        break;

    case sl::MAT_TYPE::F32_C2: /**< float 2 channels.*/
        imgMsgPtr->encoding = sensor_msgs::image_encodings::TYPE_32FC2;
        dataSize = 2 * sizeof(float);
        srcL = (char*)left.getPtr<sl::float2>();
        srcR = (char*)right.getPtr<sl::float2>();
        break;

    case sl::MAT_TYPE::F32_C3: /**< float 3 channels.*/
        imgMsgPtr->encoding = sensor_msgs::image_encodings::TYPE_32FC3;
        dataSize = 3 * sizeof(float);
        srcL = (char*)left.getPtr<sl::float3>();
        srcR = (char*)right.getPtr<sl::float3>();
        break;

    case sl::MAT_TYPE::F32_C4: /**< float 4 channels.*/
        imgMsgPtr->encoding = sensor_msgs::image_encodings::TYPE_32FC4;
        dataSize = 4 * sizeof(float);
        srcL = (char*)left.getPtr<sl::float4>();
        srcR = (char*)right.getPtr<sl::float4>();
        break;

    case sl::MAT_TYPE::U8_C1: /**< unsigned char 1 channel.*/
        imgMsgPtr->encoding = sensor_msgs::image_encodings::MONO8;
        dataSize = sizeof(char);
        srcL = (char*)left.getPtr<sl::uchar1>();
        srcR = (char*)right.getPtr<sl::uchar1>();
        break;

    case sl::MAT_TYPE::U8_C2: /**< unsigned char 2 channels.*/
        imgMsgPtr->encoding = sensor_msgs::image_encodings::TYPE_8UC2;
        dataSize = 2 * sizeof(char);
        srcL = (char*)left.getPtr<sl::uchar2>();
        srcR = (char*)right.getPtr<sl::uchar2>();
        break;

    case sl::MAT_TYPE::U8_C3: /**< unsigned char 3 channels.*/
        imgMsgPtr->encoding = sensor_msgs::image_encodings::BGR8;
        dataSize = 3 * sizeof(char);
        srcL = (char*)left.getPtr<sl::uchar3>();
        srcR = (char*)right.getPtr<sl::uchar3>();
        break;

    case sl::MAT_TYPE::U8_C4: /**< unsigned char 4 channels.*/
        imgMsgPtr->encoding = sensor_msgs::image_encodings::BGRA8;
        dataSize = 4 * sizeof(char);
        srcL = (char*)left.getPtr<sl::uchar4>();
        srcR = (char*)right.getPtr<sl::uchar4>();
        break;
    }

    char* dest = (char*)(&imgMsgPtr->data[0]);

    for (int i = 0; i < left.getHeight(); i++) {
        memcpy(dest, srcL, left.getStepBytes());
        dest += left.getStepBytes();
        memcpy(dest, srcR, right.getStepBytes());
        dest += right.getStepBytes();

        srcL += left.getStepBytes();
        srcR += right.getStepBytes();
    }

    return imgMsgPtr;
}

std::string qos2str(rmw_qos_history_policy_t qos) {
    if (qos == RMW_QOS_POLICY_HISTORY_KEEP_LAST) {
        return "KEEP_LAST";
    }

    if (qos == RMW_QOS_POLICY_HISTORY_KEEP_ALL) {
        return "KEEP_ALL";
    }

    return "Unknown QoS value";
}

std::string qos2str(rmw_qos_reliability_policy_t qos) {
    if (qos == RMW_QOS_POLICY_RELIABILITY_RELIABLE) {
        return "RELIABLE";
    }

    if (qos == RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT) {
        return "BEST_EFFORT";
    }

    return "Unknown QoS value";
}

std::string qos2str(rmw_qos_durability_policy_t qos) {
    if (qos == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL) {
        return "TRANSIENT_LOCAL";
    }

    if (qos == RMW_QOS_POLICY_DURABILITY_VOLATILE) {
        return "VOLATILE";
    }

    return "Unknown QoS value";
}

SmartMean::SmartMean(int winSize) {
    mValCount = 0;

    mMeanCorr = 0.0;
    mMean = 0.0;
    mWinSize = winSize;

    mGamma = (static_cast<double>(mWinSize) - 1.) / static_cast<double>(mWinSize);
}

double SmartMean::addValue(double val) {
    mValCount++;

    mMeanCorr = mGamma * mMeanCorr + (1. - mGamma) * val;
    mMean = mMeanCorr / (1. - pow(mGamma, mValCount));

    return mMean;
}

bool isZED2OrZED2i(sl::MODEL camModel){
    if (camModel == sl::MODEL::ZED2){
        return true;
    }
#if ZED_SDK_MAJOR_VERSION==3 && ZED_SDK_MINOR_VERSION>=5
    if (camModel == sl::MODEL::ZED2i){
        return true;
    }
#endif
    return false;
}

bool isObjDetAvailable(sl::MODEL camModel)
{
    if (camModel == sl::MODEL::ZED2) {
        return true;
    }
#if ZED_SDK_MAJOR_VERSION == 3 && ZED_SDK_MINOR_VERSION >= 5
    if (camModel == sl::MODEL::ZED2i) {
        return true;
    }
#endif
#if ZED_SDK_MAJOR_VERSION == 3 && ZED_SDK_MINOR_VERSION >= 6
    if (camModel == sl::MODEL::ZED_M) {
        return true;
    }
#endif
    return false;
}

StopWatch::StopWatch()
{
    tic(); // Start the timer at creation
}

void StopWatch::tic()
{
    mStartTime = std::chrono::steady_clock::now(); // Set the start time point
}

double StopWatch::toc()
{
    auto now = std::chrono::steady_clock::now();
    double elapsed_usec = std::chrono::duration_cast<std::chrono::microseconds>(now - mStartTime).count();
    return elapsed_usec/1e6;
}

} // namespace
