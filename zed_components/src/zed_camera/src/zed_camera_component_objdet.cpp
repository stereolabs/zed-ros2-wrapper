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

void ZedCamera::getOdParams()
{
  rclcpp::Parameter paramVal;

  rcl_interfaces::msg::ParameterDescriptor read_only_descriptor;
  read_only_descriptor.read_only = true;

  RCLCPP_INFO(get_logger(), "=== OBJECT DETECTION parameters ===");
  if (sl_tools::isZED(mCamUserModel)) {
    RCLCPP_WARN(get_logger(), "!!! OD parameters are not used with ZED!!!");
    return;
  }

  sl_tools::getParam(
    shared_from_this(), "object_detection.od_enabled",
    mObjDetEnabled, mObjDetEnabled,
    " * Object Det. enabled: ");

  sl_tools::getParam(
    shared_from_this(),
    "object_detection.allow_reduced_precision_inference",
    mObjDetReducedPrecision, mObjDetReducedPrecision);
  RCLCPP_INFO_STREAM(
    get_logger(),
    " * Object Det. allow reduced precision: "
      << (mObjDetReducedPrecision ? "TRUE" : "FALSE"));
  sl_tools::getParam(
    shared_from_this(), "object_detection.max_range",
    mObjDetMaxRange, mObjDetMaxRange,
    " * Object Det. maximum range [m]: ", false, 0.1, 40.0);
  sl_tools::getParam(
    shared_from_this(), "object_detection.prediction_timeout",
    mObjDetPredTimeout, mObjDetPredTimeout,
    " * Object Det. prediction timeout [sec]: ", false, 0.0, 300.0);
  sl_tools::getParam(
    shared_from_this(), "object_detection.enable_tracking",
    mObjDetTracking, mObjDetTracking);
  RCLCPP_INFO_STREAM(
    get_logger(), " * Object Det. tracking: "
      << (mObjDetTracking ? "TRUE" : "FALSE"));

  bool matched = false;
  std::string filtering_mode_str = "NONE";
  sl_tools::getParam(
    shared_from_this(), "object_detection.filtering_mode",
    filtering_mode_str, filtering_mode_str);

  for (int idx = static_cast<int>(sl::OBJECT_FILTERING_MODE::NONE);
    idx < static_cast<int>(sl::OBJECT_FILTERING_MODE::LAST); idx++)
  {
    sl::OBJECT_FILTERING_MODE test_mode =
      static_cast<sl::OBJECT_FILTERING_MODE>(idx);
    std::string test_mode_str = sl::toString(test_mode).c_str();
    std::replace(
      test_mode_str.begin(), test_mode_str.end(), ' ', '_');   // Replace spaces with underscores to match the YAML setting
    DEBUG_OD(" Comparing '%s' to '%s'", filtering_mode_str.c_str(), test_mode_str.c_str());
    if (filtering_mode_str == test_mode_str) {
      mObjFilterMode = test_mode;
      matched = true;
      break;
    }
  }
  if (!matched) {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "The value of the parameter 'object_detection.filtering_mode' is not valid: '"
        << filtering_mode_str << "'. Using the default value.");
  }
  RCLCPP_INFO_STREAM(
    get_logger(), " * Object Filtering mode: "
      << sl::toString(mObjFilterMode).c_str());

  // ----> Object Detection model
  std::string model_str;
  sl_tools::getParam(
    shared_from_this(), "object_detection.detection_model",
    model_str, model_str);
  DEBUG_STREAM_OD(" 'object_detection.detection_model': " << model_str.c_str());

  matched = false;
  for (int idx =
    static_cast<int>(sl::OBJECT_DETECTION_MODEL::MULTI_CLASS_BOX_FAST);
    idx < static_cast<int>(sl::OBJECT_DETECTION_MODEL::LAST);
    idx++)
  {
    sl::OBJECT_DETECTION_MODEL test_model =
      static_cast<sl::OBJECT_DETECTION_MODEL>(idx);
    std::string test_model_str = sl::toString(test_model).c_str();
    std::replace(
      test_model_str.begin(), test_model_str.end(), ' ',
      '_');      // Replace spaces with underscores to match the YAML setting
    DEBUG_OD(" Comparing '%s' to '%s'", test_model_str.c_str(), model_str.c_str());
    if (model_str == test_model_str) {
      mObjDetModel = test_model;
      matched = true;
      break;
    }
  }
  if (!matched) {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "The value of the parameter 'object_detection.model' is not valid: '"
        << model_str << "'. Stopping the node");
    exit(EXIT_FAILURE);
  }
  if (mObjDetModel == sl::OBJECT_DETECTION_MODEL::CUSTOM_BOX_OBJECTS) {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "The value of the parameter 'object_detection.model' is not supported: '"
        << model_str << "'. Stopping the node");
    exit(EXIT_FAILURE);
  }
  RCLCPP_INFO_STREAM(
    get_logger(), " * Object Det. model: "
      << sl::toString(mObjDetModel).c_str());

  if (mObjDetModel == sl::OBJECT_DETECTION_MODEL::CUSTOM_YOLOLIKE_BOX_OBJECTS) {
    mUsingCustomOd = true;
  } else {
    mUsingCustomOd = false;
  }
  // <---- Object Detection model

  if (mUsingCustomOd) {
    getCustomOdParams();
  } else {
    // ----> MultiClassBox parameters
    sl_tools::getParam(
      shared_from_this(),
      "object_detection.class.people.enabled",
      mObjDetPeopleEnable, mObjDetPeopleEnable,
      " * MultiClassBox people: ", true);
    sl_tools::getParam(
      shared_from_this(),
      "object_detection.class.vehicle.enabled",
      mObjDetVehiclesEnable, mObjDetVehiclesEnable,
      " * MultiClassBox vehicles: ", true);
    sl_tools::getParam(
      shared_from_this(), "object_detection.class.bag.enabled",
      mObjDetBagsEnable, mObjDetBagsEnable,
      " * MultiClassBox bags: ", true);
    sl_tools::getParam(
      shared_from_this(),
      "object_detection.class.animal.enabled",
      mObjDetAnimalsEnable, mObjDetAnimalsEnable,
      " * MultiClassBox animals: ", true);
    sl_tools::getParam(
      shared_from_this(),
      "object_detection.class.electronics.enabled",
      mObjDetElectronicsEnable, mObjDetElectronicsEnable,
      " * MultiClassBox electronics: ", true);
    sl_tools::getParam(
      shared_from_this(),
      "object_detection.class.fruit_vegetable.enabled",
      mObjDetFruitsEnable, mObjDetFruitsEnable,
      " * MultiClassBox fruits and vegetables: ", true);
    sl_tools::getParam(
      shared_from_this(),
      "object_detection.class.sport.enabled",
      mObjDetSportEnable, mObjDetSportEnable,
      " * MultiClassBox sport-related objects: ", true);
    sl_tools::getParam(
      shared_from_this(),
      "object_detection.class.people.confidence_threshold",
      mObjDetPeopleConf, mObjDetPeopleConf,
      " * MultiClassBox people confidence: ", true, 0.0, 100.0);
    sl_tools::getParam(
      shared_from_this(),
      "object_detection.class.vehicle.confidence_threshold",
      mObjDetVehiclesConf, mObjDetVehiclesConf,
      " * MultiClassBox vehicles confidence: ", true, 0.0, 100.0);
    sl_tools::getParam(
      shared_from_this(),
      "object_detection.class.bag.confidence_threshold",
      mObjDetBagsConf, mObjDetBagsConf,
      " * MultiClassBox bags confidence: ", true, 0.0, 100.0);
    sl_tools::getParam(
      shared_from_this(),
      "object_detection.class.animal.confidence_threshold",
      mObjDetAnimalsConf, mObjDetAnimalsConf,
      " * MultiClassBox animals confidence: ", true, 0.0, 100.0);
    sl_tools::getParam(
      shared_from_this(),
      "object_detection.class.electronics.confidence_threshold",
      mObjDetElectronicsConf, mObjDetElectronicsConf,
      " * MultiClassBox electronics confidence: ", true, 0.0, 100.0);
    sl_tools::getParam(
      shared_from_this(),
      "object_detection.class.fruit_vegetable.confidence_threshold",
      mObjDetFruitsConf, mObjDetFruitsConf,
      " * MultiClassBox fruits and vegetables confidence: ", true, 0.0, 100.0);
    sl_tools::getParam(
      shared_from_this(),
      "object_detection.class.sport.confidence_threshold",
      mObjDetSportConf, mObjDetSportConf,
      " * MultiClassBox sport-related objects confidence: ", true, 0.0, 100.0);
    // <---- MultiClassBox parameters
  }
}

void ZedCamera::getCustomOdParams()
{
  sl_tools::getParam(
    shared_from_this(), "object_detection.custom_onnx_file",
    mYoloOnnxPath, mYoloOnnxPath, " * Custom ONNX file: ");
  if (mYoloOnnxPath.empty()) {
    RCLCPP_ERROR_STREAM(
      get_logger(),
      "The parameter 'object_detection.custom_onnx_file' is empty. "
      "Please check the value in the YAML file.");
    exit(EXIT_FAILURE);
  }
  sl_tools::getParam(
    shared_from_this(),
    "object_detection.custom_onnx_input_size", mYoloOnnxSize,
    mYoloOnnxSize, " * Custom ONNX input size: ", false, 128, 2048);
  sl_tools::getParam(
    shared_from_this(),
    "object_detection.custom_class_count", mCustomClassCount,
    mCustomClassCount, " * Custom ONNX class count: ", false, 1, 999);

  for (int i = 0; i < mCustomClassCount; i++) {
    std::string param_name;
    std::string label = "";
    int class_id = i;

    std::stringstream class_prefix;
    class_prefix << "class_";
    class_prefix.width(3);
    class_prefix.fill('0');
    class_prefix << i;

    std::string param_prefix = std::string("object_detection.") + class_prefix.str() + ".";

    param_name = param_prefix + "label";
    sl_tools::getParam(
      shared_from_this(), param_name, label, label, std::string(
        "  * ") + param_name + ": ", false);

    param_name = param_prefix + "model_class_id";
    sl_tools::getParam(
      shared_from_this(), param_name, class_id, class_id, std::string(
        "  * ") + param_name + ": ", false, 0, 999);

    mCustomClassIdMap[class_prefix.str()] = class_id; // Update the class prefix to class_id mapping
    DEBUG_STREAM_OD(" Mapped '" << class_prefix.str() << "' to ID '" << class_id << "'");
    mCustomLabels[class_id] = label; // Update the class id to label mapping
    DEBUG_STREAM_OD(" Mapped ID '" << class_id << "' to '" << label << "'");

    sl::CustomObjectDetectionProperties customOdProperties;

    param_name = param_prefix + "enabled";
    sl_tools::getParam(
      shared_from_this(), param_name,
      customOdProperties.enabled, customOdProperties.enabled, std::string(
        "  * ") + param_name + ": ", true);

    param_name = param_prefix + "confidence_threshold";
    sl_tools::getParam(
      shared_from_this(), param_name,
      customOdProperties.detection_confidence_threshold, customOdProperties.detection_confidence_threshold, std::string(
        "  * ") + param_name + ": ", true, 0.0f, 100.0f);
    param_name = param_prefix + "is_grounded";
    sl_tools::getParam(
      shared_from_this(), param_name,
      customOdProperties.is_grounded, customOdProperties.is_grounded, std::string(
        "  * ") + param_name + ": ", true);
    param_name = param_prefix + "is_static";
    sl_tools::getParam(
      shared_from_this(), param_name,
      customOdProperties.is_static, customOdProperties.is_static, std::string(
        "  * ") + param_name + ": ", true);
    param_name = param_prefix + "tracking_timeout";
    sl_tools::getParam(
      shared_from_this(), param_name,
      customOdProperties.tracking_timeout, customOdProperties.tracking_timeout, std::string(
        "  * ") + param_name + ": ", true, -1.0f,
      300.0f);
    param_name = param_prefix + "tracking_max_dist";
    sl_tools::getParam(
      shared_from_this(), param_name,
      customOdProperties.tracking_max_dist, customOdProperties.tracking_max_dist, std::string(
        "  * ") + param_name + ": ", true, -1.0f,
      100.0f);
    param_name = param_prefix + "max_box_width_normalized";
    sl_tools::getParam(
      shared_from_this(), param_name,
      customOdProperties.max_box_width_normalized, customOdProperties.max_box_width_normalized, std::string(
        "  * ") + param_name + ": ", true, -1.0f,
      1.0f);
    param_name = param_prefix + "min_box_width_normalized";
    sl_tools::getParam(
      shared_from_this(), param_name,
      customOdProperties.min_box_width_normalized, customOdProperties.min_box_width_normalized, std::string(
        "  * ") + param_name + ": ", true, -1.0f,
      1.0f);
    param_name = param_prefix + "max_box_height_normalized";
    sl_tools::getParam(
      shared_from_this(), param_name,
      customOdProperties.max_box_height_normalized, customOdProperties.max_box_height_normalized, std::string(
        "  * ") + param_name + ": ", true, -1.0f,
      1.0f);
    param_name = param_prefix + "min_box_height_normalized";
    sl_tools::getParam(
      shared_from_this(), param_name,
      customOdProperties.min_box_height_normalized, customOdProperties.min_box_height_normalized, std::string(
        "  * ") + param_name + ": ", true, -1.0f,
      1.0f);
    param_name = param_prefix + "max_box_width_meters";
    sl_tools::getParam(
      shared_from_this(), param_name,
      customOdProperties.max_box_width_meters, customOdProperties.max_box_width_meters,
      std::string("  * ") + param_name + ": ", true, -1.0f,
      10000.0f);
    param_name = param_prefix + "min_box_width_meters";
    sl_tools::getParam(
      shared_from_this(), param_name,
      customOdProperties.min_box_width_meters, customOdProperties.min_box_width_meters,
      std::string("  * ") + param_name + ": ", true, -1.0f,
      10000.0f);
    param_name = param_prefix + "max_box_height_meters";
    sl_tools::getParam(
      shared_from_this(), param_name,
      customOdProperties.max_box_height_meters, customOdProperties.max_box_height_meters, std::string(
        "  * ") + param_name + ": ", true, -1.0f,
      10000.0f);
    param_name = param_prefix + "max_allowed_acceleration";
    sl_tools::getParam(
      shared_from_this(), param_name,
      customOdProperties.max_allowed_acceleration, customOdProperties.max_allowed_acceleration, std::string(
        "  * ") + param_name + ": ", true, 0.0f,
      100000.0f);

    bool matched = false;
    std::string acc_preset_str = "DEFAULT";
    param_name = param_prefix + "object_acceleration_preset";
    sl_tools::getParam(shared_from_this(), param_name, acc_preset_str, acc_preset_str);

    for (int idx = static_cast<int>(sl::OBJECT_ACCELERATION_PRESET::DEFAULT);
      idx < static_cast<int>(sl::OBJECT_ACCELERATION_PRESET::LAST); idx++)
    {
      sl::OBJECT_ACCELERATION_PRESET test_mode =
        static_cast<sl::OBJECT_ACCELERATION_PRESET>(idx);
      std::string test_mode_str = sl::toString(test_mode).c_str();
      std::replace(
        test_mode_str.begin(), test_mode_str.end(), ' ', '_');   // Replace spaces with underscores to match the YAML setting
      DEBUG_OD(" Comparing '%s' to '%s'", acc_preset_str.c_str(), test_mode_str.c_str());
      if (acc_preset_str == test_mode_str) {
        customOdProperties.object_acceleration_preset = test_mode;
        matched = true;
        break;
      }
    }
    if (!matched) {
      RCLCPP_WARN_STREAM(
        get_logger(),
        "The value of the parameter 'object_detection.filtering_mode' is not valid: '"
          << acc_preset_str << "'. Using the default value.");
    }
    RCLCPP_INFO_STREAM(
      get_logger(), std::string("  * ") + param_name + ": "
        << sl::toString(customOdProperties.object_acceleration_preset).c_str());

    mCustomOdProperties[class_id] = customOdProperties; // Update the Custom OD Properties information
  }

}

bool ZedCamera::handleOdDynamicParams(
  const rclcpp::Parameter & param,
  rcl_interfaces::msg::SetParametersResult & result)
{
  DEBUG_OD("handleOdDynamicParams");

  if (param.get_name() == "object_detection.class.people.enabled") {
    rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_BOOL;
    if (param.get_type() != correctType) {
      result.successful = false;
      result.reason =
        param.get_name() + " must be a " + rclcpp::to_string(correctType);
      RCLCPP_WARN_STREAM(get_logger(), result.reason);
      return false;
    }

    mObjDetPeopleEnable = param.as_bool();

    RCLCPP_INFO_STREAM(
      get_logger(),
      "Parameter '"
        << param.get_name() << "' correctly set to "
        << (mObjDetPeopleEnable ? "TRUE" : "FALSE"));
  } else if (param.get_name() == "object_detection.class.vehicle.enabled") {
    rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_BOOL;
    if (param.get_type() != correctType) {
      result.successful = false;
      result.reason =
        param.get_name() + " must be a " + rclcpp::to_string(correctType);
      RCLCPP_WARN_STREAM(get_logger(), result.reason);
      return false;
    }

    mObjDetVehiclesEnable = param.as_bool();

    RCLCPP_INFO_STREAM(
      get_logger(),
      "Parameter '"
        << param.get_name() << "' correctly set to "
        << (mObjDetVehiclesEnable ? "TRUE" : "FALSE"));
  } else if (param.get_name() == "object_detection.class.bag.enabled") {
    rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_BOOL;
    if (param.get_type() != correctType) {
      result.successful = false;
      result.reason =
        param.get_name() + " must be a " + rclcpp::to_string(correctType);
      RCLCPP_WARN_STREAM(get_logger(), result.reason);
      return false;
    }

    mObjDetBagsEnable = param.as_bool();

    RCLCPP_INFO_STREAM(
      get_logger(),
      "Parameter '"
        << param.get_name() << "' correctly set to "
        << (mObjDetBagsEnable ? "TRUE" : "FALSE"));
  } else if (param.get_name() == "object_detection.class.animal.enabled") {
    rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_BOOL;
    if (param.get_type() != correctType) {
      result.successful = false;
      result.reason =
        param.get_name() + " must be a " + rclcpp::to_string(correctType);
      RCLCPP_WARN_STREAM(get_logger(), result.reason);
      return false;
    }

    mObjDetAnimalsEnable = param.as_bool();

    RCLCPP_INFO_STREAM(
      get_logger(),
      "Parameter '"
        << param.get_name() << "' correctly set to "
        << (mObjDetAnimalsEnable ? "TRUE" : "FALSE"));
  } else if (param.get_name() ==
    "object_detection.class.electronics.enabled")
  {
    rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_BOOL;
    if (param.get_type() != correctType) {
      result.successful = false;
      result.reason =
        param.get_name() + " must be a " + rclcpp::to_string(correctType);
      RCLCPP_WARN_STREAM(get_logger(), result.reason);
      return false;
    }

    mObjDetElectronicsEnable = param.as_bool();

    RCLCPP_INFO_STREAM(
      get_logger(),
      "Parameter '"
        << param.get_name() << "' correctly set to "
        << (mObjDetElectronicsEnable ? "TRUE" : "FALSE"));
  } else if (param.get_name() ==
    "object_detection.class.fruit_vegetable.enabled")
  {
    rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_BOOL;
    if (param.get_type() != correctType) {
      result.successful = false;
      result.reason =
        param.get_name() + " must be a " + rclcpp::to_string(correctType);
      RCLCPP_WARN_STREAM(get_logger(), result.reason);
      return false;
    }

    mObjDetFruitsEnable = param.as_bool();

    RCLCPP_INFO_STREAM(
      get_logger(),
      "Parameter '"
        << param.get_name() << "' correctly set to "
        << (mObjDetFruitsEnable ? "TRUE" : "FALSE"));
  } else if (param.get_name() == "object_detection.class.sport.enabled") {
    rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_BOOL;
    if (param.get_type() != correctType) {
      result.successful = false;
      result.reason =
        param.get_name() + " must be a " + rclcpp::to_string(correctType);
      RCLCPP_WARN_STREAM(get_logger(), result.reason);
      return false;
    }

    mObjDetSportEnable = param.as_bool();

    RCLCPP_INFO_STREAM(
      get_logger(),
      "Parameter '"
        << param.get_name() << "' correctly set to "
        << (mObjDetSportEnable ? "TRUE" : "FALSE"));
  } else if (param.get_name() == "object_detection.class.people.confidence_threshold") {
    rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_DOUBLE;
    if (param.get_type() != correctType) {
      result.successful = false;
      result.reason =
        param.get_name() + " must be a " + rclcpp::to_string(correctType);
      RCLCPP_WARN_STREAM(get_logger(), result.reason);
      return false;
    }

    mObjDetPeopleConf = param.as_double();

    RCLCPP_INFO_STREAM(
      get_logger(),
      "Parameter '"
        << param.get_name() << "' correctly set to "
        << mObjDetPeopleConf);
  } else if (param.get_name() == "object_detection.class.vehicle.confidence_threshold") {
    rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_DOUBLE;
    if (param.get_type() != correctType) {
      result.successful = false;
      result.reason =
        param.get_name() + " must be a " + rclcpp::to_string(correctType);
      RCLCPP_WARN_STREAM(get_logger(), result.reason);
      return false;
    }

    mObjDetVehiclesConf = param.as_double();

    RCLCPP_INFO_STREAM(
      get_logger(),
      "Parameter '"
        << param.get_name() << "' correctly set to "
        << mObjDetVehiclesConf);
  } else if (param.get_name() == "object_detection.class.bag.confidence_threshold") {
    rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_DOUBLE;
    if (param.get_type() != correctType) {
      result.successful = false;
      result.reason =
        param.get_name() + " must be a " + rclcpp::to_string(correctType);
      RCLCPP_WARN_STREAM(get_logger(), result.reason);
      return false;
    }

    mObjDetBagsConf = param.as_double();

    RCLCPP_INFO_STREAM(
      get_logger(),
      "Parameter '"
        << param.get_name() << "' correctly set to "
        << mObjDetBagsConf);
  } else if (param.get_name() == "object_detection.class.animal.confidence_threshold") {
    rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_DOUBLE;
    if (param.get_type() != correctType) {
      result.successful = false;
      result.reason =
        param.get_name() + " must be a " + rclcpp::to_string(correctType);
      RCLCPP_WARN_STREAM(get_logger(), result.reason);
      return false;
    }

    mObjDetAnimalsConf = param.as_double();

    RCLCPP_INFO_STREAM(
      get_logger(),
      "Parameter '"
        << param.get_name() << "' correctly set to "
        << mObjDetAnimalsConf);
  } else if (param.get_name() == "object_detection.class.electronics.confidence_threshold") {
    rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_DOUBLE;
    if (param.get_type() != correctType) {
      result.successful = false;
      result.reason =
        param.get_name() + " must be a " + rclcpp::to_string(correctType);
      RCLCPP_WARN_STREAM(get_logger(), result.reason);
      return false;
    }
    mObjDetElectronicsConf = param.as_double();
    RCLCPP_INFO_STREAM(
      get_logger(),
      "Parameter '"
        << param.get_name() << "' correctly set to "
        << mObjDetElectronicsConf);
  } else if (param.get_name() == "object_detection.class.fruit_vegetable.confidence_threshold") {
    rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_DOUBLE;
    if (param.get_type() != correctType) {
      result.successful = false;
      result.reason =
        param.get_name() + " must be a " + rclcpp::to_string(correctType);
      RCLCPP_WARN_STREAM(get_logger(), result.reason);
      return false;
    }

    mObjDetFruitsConf = param.as_double();

    RCLCPP_INFO_STREAM(
      get_logger(),
      "Parameter '"
        << param.get_name() << "' correctly set to "
        << mObjDetFruitsConf);
  } else if (param.get_name() == "object_detection.class.sport.confidence_threshold") {
    rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_DOUBLE;
    if (param.get_type() != correctType) {
      result.successful = false;
      result.reason =
        param.get_name() + " must be a " + rclcpp::to_string(correctType);
      RCLCPP_WARN_STREAM(get_logger(), result.reason);
      return false;
    }

    mObjDetSportConf = param.as_double();

    RCLCPP_INFO_STREAM(
      get_logger(),
      "Parameter '"
        << param.get_name() << "' correctly set to "
        << mObjDetSportConf);
  }

  return true;
}

bool ZedCamera::handleCustomOdDynamicParams(
  const rclcpp::Parameter & param,
  rcl_interfaces::msg::SetParametersResult & result)
{
  DEBUG_COMM("handleCustomOdDynamicParams");

  std::string param_full_name = param.get_name();
  DEBUG_STREAM_COMM("handleCustomOdDynamicParams: Parameter name: " << param_full_name);

  if (param_full_name.find("object_detection.class_") == std::string::npos) {
    DEBUG_STREAM_COMM(
      "handleCustomOdDynamicParams: Parameter '" << param_full_name <<
        "' is not a custom object detection parameter");
    return true;
  }

  // Get the class ID from the parameter name
  std::string class_id_str = param_full_name.substr(param_full_name.find("class_"), 9);
  DEBUG_STREAM_COMM("handleCustomOdDynamicParams: Class ID: " << class_id_str);

  int class_id = mCustomClassIdMap[class_id_str];
  if (mCustomClassIdMap.find(class_id_str) == mCustomClassIdMap.end()) {
    DEBUG_STREAM_COMM(
      "handleCustomOdDynamicParams: Class ID '" << class_id_str <<
        "' not found in the custom class ID map");
    return false;
  }
  DEBUG_STREAM_COMM("handleCustomOdDynamicParams: Class ID: " << class_id);

  std::string param_name = param_full_name.substr(param_full_name.find_last_of('.') + 1);
  DEBUG_STREAM_COMM("handleCustomOdDynamicParams: Custom OD Parameter name: " << param_name);

  if (param_name == "enabled") {
    rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_BOOL;
    if (param.get_type() != correctType) {
      result.successful = false;
      result.reason =
        param.get_name() + " must be a " + rclcpp::to_string(correctType);
      RCLCPP_WARN_STREAM(get_logger(), result.reason);
      return false;
    }

    mCustomOdProperties[class_id].enabled = param.as_bool();

    RCLCPP_INFO_STREAM(
      get_logger(),
      "Parameter '"
        << param.get_name() << "' correctly set to "
        << (mCustomOdProperties[class_id].enabled ? "TRUE" : "FALSE"));
  } else if (param_name == "confidence_threshold") {
    rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_DOUBLE;
    if (param.get_type() != correctType) {
      result.successful = false;
      result.reason =
        param.get_name() + " must be a " + rclcpp::to_string(correctType);
      RCLCPP_WARN_STREAM(get_logger(), result.reason);
      return false;
    }

    mCustomOdProperties[class_id].detection_confidence_threshold = param.as_double();

    RCLCPP_INFO_STREAM(
      get_logger(),
      "Parameter '"
        << param.get_name() << "' correctly set to "
        << mCustomOdProperties[class_id].detection_confidence_threshold);
  } else if (param_name == "is_grounded") {
    rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_BOOL;
    if (param.get_type() != correctType) {
      result.successful = false;
      result.reason =
        param.get_name() + " must be a " + rclcpp::to_string(correctType);
      RCLCPP_WARN_STREAM(get_logger(), result.reason);
      return false;
    }

    mCustomOdProperties[class_id].is_grounded = param.as_bool();

    RCLCPP_INFO_STREAM(
      get_logger(),
      "Parameter '"
        << param.get_name() << "' correctly set to "
        << (mCustomOdProperties[class_id].is_grounded ? "TRUE" : "FALSE"));
  } else if (param_name == "is_static") {
    rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_BOOL;
    if (param.get_type() != correctType) {
      result.successful = false;
      result.reason =
        param.get_name() + " must be a " + rclcpp::to_string(correctType);
      RCLCPP_WARN_STREAM(get_logger(), result.reason);
      return false;
    }

    mCustomOdProperties[class_id].is_static = param.as_bool();

    RCLCPP_INFO_STREAM(
      get_logger(),
      "Parameter '"
        << param.get_name() << "' correctly set to "
        << (mCustomOdProperties[class_id].is_static ? "TRUE" : "FALSE"));
  } else if (param_name == "tracking_timeout") {
    rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_DOUBLE;
    if (param.get_type() != correctType) {
      result.successful = false;
      result.reason =
        param.get_name() + " must be a " + rclcpp::to_string(correctType);
      RCLCPP_WARN_STREAM(get_logger(), result.reason);
      return false;
    }

    mCustomOdProperties[class_id].tracking_timeout = param.as_double();

    RCLCPP_INFO_STREAM(
      get_logger(),
      "Parameter '"
        << param.get_name() << "' correctly set to "
        << mCustomOdProperties[class_id].tracking_timeout);
  } else if (param_name == "tracking_max_dist") {
    rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_DOUBLE;
    if (param.get_type() != correctType) {
      result.successful = false;
      result.reason =
        param.get_name() + " must be a " + rclcpp::to_string(correctType);
      RCLCPP_WARN_STREAM(get_logger(), result.reason);
      return false;
    }

    mCustomOdProperties[class_id].tracking_max_dist = param.as_double();

    RCLCPP_INFO_STREAM(
      get_logger(),
      "Parameter '"
        << param.get_name() << "' correctly set to "
        << mCustomOdProperties[class_id].tracking_max_dist);
  } else if (param_name == "max_box_width_normalized") {
    rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_DOUBLE;
    if (param.get_type() != correctType) {
      result.successful = false;
      result.reason =
        param.get_name() + " must be a " + rclcpp::to_string(correctType);
      RCLCPP_WARN_STREAM(get_logger(), result.reason);
      return false;
    }

    mCustomOdProperties[class_id].max_box_width_normalized = param.as_double();

    RCLCPP_INFO_STREAM(
      get_logger(),
      "Parameter '"
        << param.get_name() << "' correctly set to "
        << mCustomOdProperties[class_id].max_box_width_normalized);
  } else if (param_name == "min_box_width_normalized") {
    rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_DOUBLE;
    if (param.get_type() != correctType) {
      result.successful = false;
      result.reason =
        param.get_name() + " must be a " + rclcpp::to_string(correctType);
      RCLCPP_WARN_STREAM(get_logger(), result.reason);
      return false;
    }

    mCustomOdProperties[class_id].min_box_width_normalized = param.as_double();

    RCLCPP_INFO_STREAM(
      get_logger(),
      "Parameter '"
        << param.get_name() << "' correctly set to "
        << mCustomOdProperties[class_id].min_box_width_normalized);
  } else if (param_name == "max_box_height_normalized") {
    rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_DOUBLE;
    if (param.get_type() != correctType) {
      result.successful = false;
      result.reason =
        param.get_name() + " must be a " + rclcpp::to_string(correctType);
      RCLCPP_WARN_STREAM(get_logger(), result.reason);
      return false;
    }
    mCustomOdProperties[class_id].max_box_height_normalized = param.as_double();
    RCLCPP_INFO_STREAM(
      get_logger(),
      "Parameter '"
        << param.get_name() << "' correctly set to "
        << mCustomOdProperties[class_id].max_box_height_normalized);
  } else if (param_name == "min_box_height_normalized") {
    rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_DOUBLE;
    if (param.get_type() != correctType) {
      result.successful = false;
      result.reason =
        param.get_name() + " must be a " + rclcpp::to_string(correctType);
      RCLCPP_WARN_STREAM(get_logger(), result.reason);
      return false;
    }
    mCustomOdProperties[class_id].min_box_height_normalized = param.as_double();
    RCLCPP_INFO_STREAM(
      get_logger(),
      "Parameter '"
        << param.get_name() << "' correctly set to "
        << mCustomOdProperties[class_id].min_box_height_normalized);
  } else if (param_name == "max_box_width_meters") {
    rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_DOUBLE;
    if (param.get_type() != correctType) {
      result.successful = false;
      result.reason =
        param.get_name() + " must be a " + rclcpp::to_string(correctType);
      RCLCPP_WARN_STREAM(get_logger(), result.reason);
      return false;
    }

    mCustomOdProperties[class_id].max_box_width_meters = param.as_double();

    RCLCPP_INFO_STREAM(
      get_logger(),
      "Parameter '"
        << param.get_name() << "' correctly set to "
        << mCustomOdProperties[class_id].max_box_width_meters);
  } else if (param_name == "min_box_width_meters") {
    rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_DOUBLE;
    if (param.get_type() != correctType) {
      result.successful = false;
      result.reason =
        param.get_name() + " must be a " + rclcpp::to_string(correctType);
      RCLCPP_WARN_STREAM(get_logger(), result.reason);
      return false;
    }

    mCustomOdProperties[class_id].min_box_width_meters = param.as_double();

    RCLCPP_INFO_STREAM(
      get_logger(),
      "Parameter '"
        << param.get_name() << "' correctly set to "
        << mCustomOdProperties[class_id].min_box_width_meters);
  } else if (param_name == "max_box_height_meters") {
    rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_DOUBLE;
    if (param.get_type() != correctType) {
      result.successful = false;
      result.reason =
        param.get_name() + " must be a " + rclcpp::to_string(correctType);
      RCLCPP_WARN_STREAM(get_logger(), result.reason);
      return false;
    }
    mCustomOdProperties[class_id].max_box_height_meters = param.as_double();
    RCLCPP_INFO_STREAM(
      get_logger(),
      "Parameter '"
        << param.get_name() << "' correctly set to "
        << mCustomOdProperties[class_id].max_box_height_meters);
  } else if (param_name == "max_allowed_acceleration") {
    rclcpp::ParameterType
      correctType = rclcpp::ParameterType::PARAMETER_DOUBLE;
    if (param.get_type() != correctType) {
      result.successful = false;
      result.reason =
        param.get_name() + " must be a " + rclcpp::to_string(correctType);
      RCLCPP_WARN_STREAM(get_logger(), result.reason);
      return false;
    }
    mCustomOdProperties[class_id].max_allowed_acceleration = param.as_double();
    RCLCPP_INFO_STREAM(
      get_logger(),
      "Parameter '"
        << param.get_name() << "' correctly set to "
        << mCustomOdProperties[class_id].max_allowed_acceleration);
  } else {
    RCLCPP_WARN_STREAM(get_logger(), "Unknown parameter: " << param.get_name());
  }

  return true;
}

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

  RCLCPP_INFO(get_logger(), "=== Starting Object Detection ===");

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

  if (mUsingCustomOd) {
    od_p.custom_onnx_dynamic_input_shape = mYoloOnnxSize;
    od_p.custom_onnx_file = mYoloOnnxPath;
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
    RCLCPP_INFO(get_logger(), "=== Stopping Object Detection ===");
    mObjDetRunning = false;
    mObjDetEnabled = false;
    mZed->disableObjectDetection();

    // ----> Send an empty message to indicate that no more objects are tracked
    // (e.g clean RVIZ2)
    auto objMsg = std::make_unique<zed_msgs::msg::ObjectsStamped>();

    objMsg->header.stamp = mUsePubTimestamps ? get_clock()->now() : mFrameTimestamp;
    objMsg->header.frame_id = mLeftCamFrameId;

    objMsg->objects.clear();

    DEBUG_STREAM_OD(
      "Publishing EMPTY OBJ message "
        << mPubObjDet->get_topic_name());
    try {
      mPubObjDet->publish(std::move(objMsg));
    } catch (std::system_error & e) {
      DEBUG_STREAM_COMM("Message publishing exception: " << e.what() );
    } catch (...) {
      DEBUG_STREAM_COMM("Message publishing generic exception: ");
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

    objectTracker_parameters_rt.detection_confidence_threshold = 50.0f; // Default value, overwritten by single class parameters
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
    objectTracker_parameters_rt.object_class_filter = mObjDetFilter;
    objectTracker_parameters_rt.object_class_detection_confidence_threshold = mObjDetClassConfMap;
    // <---- Process realtime dynamic parameters

    objDetRes = mZed->retrieveObjects(
      objects, objectTracker_parameters_rt, mObjDetInstID);
  }
#if (ZED_SDK_MAJOR_VERSION * 10 + ZED_SDK_MINOR_VERSION) >= 50
  else {
    // ----> Process realtime dynamic parameters
    sl::CustomObjectDetectionRuntimeParameters custom_objectTracker_parameters_rt;
    custom_objectTracker_parameters_rt.object_class_detection_properties = mCustomOdProperties; // Update realtime detection parameters
    // <---- Process realtime dynamic parameters

    objDetRes = mZed->retrieveCustomObjects(
      objects, custom_objectTracker_parameters_rt,
      mObjDetInstID);
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

  objMsg->header.stamp = mUsePubTimestamps ? get_clock()->now() : t;
  objMsg->header.frame_id = mLeftCamFrameId;

  objMsg->objects.resize(objCount);

  size_t idx = 0;
  for (auto data : objects.object_list) {
    if (!mUsingCustomOd) {
      objMsg->objects[idx].label = sl::toString(data.label).c_str();
      objMsg->objects[idx].sublabel = sl::toString(data.sublabel).c_str();
    } else {
      objMsg->objects[idx].label = mCustomLabels[data.raw_label];
      objMsg->objects[idx].sublabel = std::to_string(data.raw_label);
    }

    objMsg->objects[idx].label_id = data.id;
    objMsg->objects[idx].confidence = data.confidence;

    DEBUG_STREAM_OD(
      " * Object ID:" << data.id << " - " <<
        objMsg->objects[idx].label << " [" << objMsg->objects[idx].sublabel << "] - Confidence: " <<
        objMsg->objects[idx].confidence);

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
    DEBUG_STREAM_COMM("Message publishing exception: " << e.what());
  } catch (...) {
    DEBUG_STREAM_COMM("Message publishing generic exception: ");
  }

  // ----> Diagnostic information update
  mObjDetElabMean_sec->addValue(odElabTimer.toc());
  mObjDetPeriodMean_sec->addValue(mOdFreqTimer.toc());
  mOdFreqTimer.tic();
  // <---- Diagnostic information update
}

} // namespace stereolabs
