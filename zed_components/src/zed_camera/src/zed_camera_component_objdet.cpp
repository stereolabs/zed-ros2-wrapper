// Copyright 2025 Stereolabs
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "zed_camera_component.hpp"
#include "sl_logging.hpp"
#include "sl_tools.hpp"

namespace stereolabs
{
bool ZedCamera::startObjDetect()
{
  DEBUG_OD("startObjDetect");

  if (!sl_tools::isObjDetAvailable(mCamRealModel)) {
    RCLCPP_ERROR(
      get_logger(),
      "Object detection not started. The camera model does not "
      "support it with the current version "
      "of the "
      "SDK");
    return false;
  }

  if (mDepthDisabled) {
    RCLCPP_WARN(
      get_logger(),
      "Cannot start Object Detection if "
      "`depth.depth_mode` is set to `0` [NONE]");
    return false;
  }

  if (!mObjDetEnabled) {
    return false;
  }

  RCLCPP_INFO(get_logger(), "*** Starting Object Detection ***");

  sl::ObjectDetectionParameters od_p;
  od_p.enable_segmentation = false;
  od_p.enable_tracking = mObjDetTracking;
  od_p.detection_model = mObjDetModel;
  od_p.filtering_mode = mObjFilterMode;
  od_p.prediction_timeout_s = mObjDetPredTimeout;
  od_p.allow_reduced_precision_inference = mObjDetReducedPrecision;
  od_p.max_range = mObjDetMaxRange;

  mObjDetInstID = ++mAiInstanceID;
  od_p.instance_module_id = mObjDetInstID;

  mObjDetFilter.clear();
  mObjDetClassConfMap.clear();
  if (mObjDetPeopleEnable) {
    mObjDetFilter.push_back(sl::OBJECT_CLASS::PERSON);
    mObjDetClassConfMap[sl::OBJECT_CLASS::PERSON] = mObjDetPeopleConf;
  }
  if (mObjDetVehiclesEnable) {
    mObjDetFilter.push_back(sl::OBJECT_CLASS::VEHICLE);
    mObjDetClassConfMap[sl::OBJECT_CLASS::VEHICLE] = mObjDetVehiclesConf;
  }
  if (mObjDetBagsEnable) {
    mObjDetFilter.push_back(sl::OBJECT_CLASS::BAG);
    mObjDetClassConfMap[sl::OBJECT_CLASS::BAG] = mObjDetBagsConf;
  }
  if (mObjDetAnimalsEnable) {
    mObjDetFilter.push_back(sl::OBJECT_CLASS::ANIMAL);
    mObjDetClassConfMap[sl::OBJECT_CLASS::ANIMAL] = mObjDetAnimalsConf;
  }
  if (mObjDetElectronicsEnable) {
    mObjDetFilter.push_back(sl::OBJECT_CLASS::ELECTRONICS);
    mObjDetClassConfMap[sl::OBJECT_CLASS::ELECTRONICS] = mObjDetElectronicsConf;
  }
  if (mObjDetFruitsEnable) {
    mObjDetFilter.push_back(sl::OBJECT_CLASS::FRUIT_VEGETABLE);
    mObjDetClassConfMap[sl::OBJECT_CLASS::FRUIT_VEGETABLE] =
      mObjDetFruitsConf;
  }
  if (mObjDetSportEnable) {
    mObjDetFilter.push_back(sl::OBJECT_CLASS::SPORT);
    mObjDetClassConfMap[sl::OBJECT_CLASS::SPORT] = mObjDetSportConf;
  }

  if (mUsingCustomOd) {
    od_p.enable_segmentation = false;
    od_p.custom_onnx_file = sl::String(mYoloOnnxPath.c_str());
    od_p.custom_onnx_dynamic_input_shape = sl::Resolution(mYoloOnnxSize, mYoloOnnxSize);

    if (!mCustomLabelsPath.empty()) {
      mCustomLabelsGood = sl_tools::ReadCocoYaml(mCustomLabelsPath, mCustomLabels);

      if (mCustomLabelsGood) {
        std::stringstream ss;
        ss << " * Custom labels: ";
        for (auto label:mCustomLabels) {
          ss << "'" << label.first << ":" << label.second << "' ";
        }
        RCLCPP_INFO(get_logger(), ss.str().c_str());
      } else {
        RCLCPP_WARN(get_logger(), "Custom open error. Using class ID instead of labels. ");
      }
    }
  }

  sl::ERROR_CODE objDetError = mZed->enableObjectDetection(od_p);

  if (objDetError != sl::ERROR_CODE::SUCCESS) {
    RCLCPP_ERROR_STREAM(
      get_logger(), "Object detection error: " << sl::toString(objDetError));

    mObjDetRunning = false;
    return false;
  }

  if (!mPubObjDet) {
    mPubObjDet = create_publisher<zed_msgs::msg::ObjectsStamped>(
      mObjectDetTopic, mQos, mPubOpt);
    RCLCPP_INFO_STREAM(
      get_logger(),
      " * Advertised on topic " << mPubObjDet->get_topic_name());
  }

  mObjDetRunning = true;
  return true;
}

void ZedCamera::stopObjDetect()
{
  if (mObjDetRunning) {
    RCLCPP_INFO(get_logger(), "*** Stopping Object Detection ***");
    mObjDetRunning = false;
    mObjDetEnabled = false;
    mZed->disableObjectDetection();

    // ----> Send an empty message to indicate that no more objects are tracked
    // (e.g clean RVIZ2)
    auto objMsg = std::make_unique<zed_msgs::msg::ObjectsStamped>();

    objMsg->header.stamp = mFrameTimestamp;
    objMsg->header.frame_id = mLeftCamFrameId;

    objMsg->objects.clear();

    DEBUG_STREAM_OD(
      "Publishing EMPTY OBJ message "
        << mPubObjDet->get_topic_name());
    try {
      mPubObjDet->publish(std::move(objMsg));
    } catch (std::system_error & e) {
      DEBUG_STREAM_COMM("Message publishing ecception: " << e.what() );
    } catch (...) {
      DEBUG_STREAM_COMM("Message publishing generic ecception: ");
    }
    // <---- Send an empty message to indicate that no more objects are tracked
    // (e.g clean RVIZ2)
  }
}

void ZedCamera::processDetectedObjects(rclcpp::Time t)
{
  size_t objdet_sub_count = 0;

  try {
    objdet_sub_count = count_subscribers(mPubObjDet->get_topic_name());
  } catch (...) {
    rcutils_reset_error();
    DEBUG_STREAM_OD(
      "processDetectedObjects: Exception while counting subscribers");
    return;
  }

  if (objdet_sub_count < 1) {
    mObjDetSubscribed = false;
    return;
  }

  sl_tools::StopWatch odElabTimer(get_clock());

  mObjDetSubscribed = true;

  sl::Objects objects;
  sl::ERROR_CODE objDetRes;

  if (!mUsingCustomOd || ZED_SDK_MAJOR_VERSION < 5) {
    // ----> Process realtime dynamic parameters
    sl::ObjectDetectionRuntimeParameters objectTracker_parameters_rt;

    objectTracker_parameters_rt.detection_confidence_threshold = 20.0f; // Default value, overwritten by single class parameters
    mObjDetFilter.clear();
    if (mObjDetPeopleEnable) {
      mObjDetFilter.push_back(sl::OBJECT_CLASS::PERSON);
    }
    if (mObjDetVehiclesEnable) {
      mObjDetFilter.push_back(sl::OBJECT_CLASS::VEHICLE);
    }
    if (mObjDetBagsEnable) {
      mObjDetFilter.push_back(sl::OBJECT_CLASS::BAG);
    }
    if (mObjDetAnimalsEnable) {
      mObjDetFilter.push_back(sl::OBJECT_CLASS::ANIMAL);
    }
    if (mObjDetElectronicsEnable) {
      mObjDetFilter.push_back(sl::OBJECT_CLASS::ELECTRONICS);
    }
    if (mObjDetFruitsEnable) {
      mObjDetFilter.push_back(sl::OBJECT_CLASS::FRUIT_VEGETABLE);
    }
    if (mObjDetSportEnable) {
      mObjDetFilter.push_back(sl::OBJECT_CLASS::SPORT);
    }
    objectTracker_parameters_rt.object_class_filter = mObjDetFilter;
    // <---- Process realtime dynamic parameters

    objDetRes = mZed->retrieveObjects(
      objects, objectTracker_parameters_rt, mObjDetInstID);
  }
#if (ZED_SDK_MAJOR_VERSION * 10 + ZED_SDK_MINOR_VERSION) >= 50
  else {
    // ----> Process realtime dynamic parameters
    sl::CustomObjectDetectionRuntimeParameters custom_objectTracker_parameters_rt;
    custom_objectTracker_parameters_rt.object_detection_properties.detection_confidence_threshold =
      20.0f;                                                                                               // Default value, overwritten by single class parameters
    // <---- Process realtime dynamic parameters

    objDetRes = mZed->retrieveCustomObjects(
      objects, custom_objectTracker_parameters_rt, mObjDetInstID);
  }
#endif

  if (objDetRes != sl::ERROR_CODE::SUCCESS) {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "Object Detection error: " << sl::toString(objDetRes));
    return;
  }

  if (!objects.is_new) {    // Async object detection. Update data only if new
    // detection is available
    DEBUG_OD("No new detected objects");
    return;
  }

  DEBUG_STREAM_OD("Detected " << objects.object_list.size() << " objects");

  size_t objCount = objects.object_list.size();

  auto objMsg = std::make_unique<zed_msgs::msg::ObjectsStamped>();

  objMsg->header.stamp = t;
  objMsg->header.frame_id = mLeftCamFrameId;

  objMsg->objects.resize(objCount);

  size_t idx = 0;
  for (auto data : objects.object_list) {
    if (mObjDetModel != sl::OBJECT_DETECTION_MODEL::CUSTOM_YOLOLIKE_BOX_OBJECTS) {
      objMsg->objects[idx].label = sl::toString(data.label).c_str();
      objMsg->objects[idx].sublabel = sl::toString(data.sublabel).c_str();
    } else {
      objMsg->objects[idx].sublabel = "";
      if (!mCustomLabelsGood) {
        objMsg->objects[idx].label = std::string("Class ID: ") + std::to_string(data.raw_label);
      } else {
        objMsg->objects[idx].label = mCustomLabels[std::to_string(data.raw_label)];
      }
    }

    objMsg->objects[idx].label_id = data.id;
    objMsg->objects[idx].confidence = data.confidence;

    memcpy(
      &(objMsg->objects[idx].position[0]), &(data.position[0]),
      3 * sizeof(float));
    memcpy(
      &(objMsg->objects[idx].position_covariance[0]),
      &(data.position_covariance[0]), 6 * sizeof(float));
    memcpy(
      &(objMsg->objects[idx].velocity[0]), &(data.velocity[0]),
      3 * sizeof(float));

    objMsg->objects[idx].tracking_available = mObjDetTracking;
    objMsg->objects[idx].tracking_state =
      static_cast<int8_t>(data.tracking_state);
    objMsg->objects[idx].action_state =
      static_cast<int8_t>(data.action_state);

    if (data.bounding_box_2d.size() == 4) {
      memcpy(
        &(objMsg->objects[idx].bounding_box_2d.corners[0]),
        &(data.bounding_box_2d[0]), 8 * sizeof(unsigned int));
    }
    if (data.bounding_box.size() == 8) {
      memcpy(
        &(objMsg->objects[idx].bounding_box_3d.corners[0]),
        &(data.bounding_box[0]), 24 * sizeof(float));
    }

    memcpy(
      &(objMsg->objects[idx].dimensions_3d[0]), &(data.dimensions[0]),
      3 * sizeof(float));

    if (data.head_bounding_box_2d.size() == 4) {
      memcpy(
        &(objMsg->objects[idx].head_bounding_box_2d.corners[0]),
        &(data.head_bounding_box_2d[0]), 8 * sizeof(unsigned int));
    }
    if (data.head_bounding_box.size() == 8) {
      memcpy(
        &(objMsg->objects[idx].head_bounding_box_3d.corners[0]),
        &(data.head_bounding_box[0]), 24 * sizeof(float));
    }

    memcpy(
      &(objMsg->objects[idx].head_position[0]), &(data.head_position[0]),
      3 * sizeof(float));

    objMsg->objects[idx].skeleton_available = false;
    // at the end of the loop
    idx++;
  }

  // DEBUG_STREAM_OD("Publishing OBJ DET message");
  try {
    mPubObjDet->publish(std::move(objMsg));
  } catch (std::system_error & e) {
    DEBUG_STREAM_COMM("Message publishing ecception: " << e.what());
  } catch (...) {
    DEBUG_STREAM_COMM("Message publishing generic ecception: ");
  }

  // ----> Diagnostic information update
  mObjDetElabMean_sec->addValue(odElabTimer.toc());
  mObjDetPeriodMean_sec->addValue(mOdFreqTimer.toc());
  mOdFreqTimer.tic();
  // <---- Diagnostic information update
}

} // namespace stereolabs
