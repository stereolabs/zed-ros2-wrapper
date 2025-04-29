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

bool ZedCamera::startBodyTracking()
{
  DEBUG_BT("startBodyTracking");

  if (!sl_tools::isObjDetAvailable(mCamRealModel)) {
    RCLCPP_ERROR(
      get_logger(),
      "Body Tracking not started. The camera model does not support "
      "it with the current version "
      "of the SDK");
    return false;
  }

  DEBUG_BT("Body Tracking available");

  if (mDepthDisabled) {
    RCLCPP_WARN(
      get_logger(),
      "Cannot start Body Tracking if "
      "`depth.depth_mode` is set to `0` [NONE]");
    return false;
  }

  if (!mBodyTrkEnabled) {
    DEBUG_BT("Body Tracking not enabled -> NOT STARTING");
    return false;
  }

  if (!mCamera2BaseTransfValid || !mSensor2CameraTransfValid ||
    !mSensor2BaseTransfValid)
  {
    DEBUG_OD(
      "Tracking transforms not yet ready, Body Tracking starting postponed");
    return false;
  }

  RCLCPP_INFO(get_logger(), "*** Starting Body Tracking ***");

  sl::BodyTrackingParameters bt_p;
  bt_p.allow_reduced_precision_inference = mBodyTrkReducedPrecision;
  bt_p.body_format = mBodyTrkFmt;
  bt_p.body_selection = mBodyTrkKpSelection;
  bt_p.detection_model = mBodyTrkModel;
  bt_p.enable_body_fitting = mBodyTrkFitting;
  bt_p.enable_segmentation = false;
  bt_p.enable_tracking = mBodyTrkEnableTracking;
  bt_p.max_range = mBodyTrkMaxRange;
  bt_p.prediction_timeout_s = mBodyTrkPredTimeout;

  mBodyTrkInstID = ++mAiInstanceID;
  bt_p.instance_module_id = mBodyTrkInstID;

  sl::ERROR_CODE btError = mZed->enableBodyTracking(bt_p);

  if (btError != sl::ERROR_CODE::SUCCESS) {
    RCLCPP_ERROR_STREAM(
      get_logger(),
      "Body Tracking error: " << sl::toString(btError));

    mBodyTrkRunning = false;
    return false;
  }

  DEBUG_BT("Body Tracking enabled");

  if (!mPubBodyTrk) {
    mPubBodyTrk = create_publisher<zed_msgs::msg::ObjectsStamped>(
      mBodyTrkTopic, mQos, mPubOpt);
    RCLCPP_INFO_STREAM(
      get_logger(),
      "Advertised on topic " << mPubBodyTrk->get_topic_name());
  }

  DEBUG_BT("Body Tracking publisher created");

  mBodyTrkRunning = true;
  return true;
}

void ZedCamera::stopBodyTracking()
{
  if (mBodyTrkRunning) {
    RCLCPP_INFO(get_logger(), "*** Stopping Body Tracking ***");
    mBodyTrkRunning = false;
    mBodyTrkEnabled = false;
    mZed->disableBodyTracking();

    // ----> Send an empty message to indicate that no more objects are tracked
    // (e.g clean RVIZ2)
    auto objMsg = std::make_unique<zed_msgs::msg::ObjectsStamped>();

    objMsg->header.stamp = mFrameTimestamp;
    objMsg->header.frame_id = mLeftCamFrameId;

    objMsg->objects.clear();

    DEBUG_STREAM_OD(
      "Publishing EMPTY OBJ message "
        << mPubBodyTrk->get_topic_name());
    try {
      mPubBodyTrk->publish(std::move(objMsg));
    } catch (std::system_error & e) {
      DEBUG_STREAM_COMM("Message publishing ecception: " << e.what());
    } catch (...) {
      DEBUG_STREAM_COMM("Message publishing generic ecception: ");
    }
    // <---- Send an empty message to indicate that no more objects are tracked
    // (e.g clean RVIZ2)
  }
}

void ZedCamera::processBodies(rclcpp::Time t)
{
  // DEBUG_BT("processBodies");

  size_t bt_sub_count = 0;

  try {
    bt_sub_count = count_subscribers(mPubBodyTrk->get_topic_name());
  } catch (...) {
    rcutils_reset_error();
    DEBUG_STREAM_OD("processBodies: Exception while counting subscribers");
    return;
  }

  if (bt_sub_count < 1) {
    mBodyTrkSubscribed = false;
    return;
  }

  sl_tools::StopWatch btElabTimer(get_clock());

  mBodyTrkSubscribed = true;

  // ----> Process realtime dynamic parameters
  sl::BodyTrackingRuntimeParameters bt_params_rt;
  bt_params_rt.detection_confidence_threshold = mBodyTrkConfThresh;
  bt_params_rt.minimum_keypoints_threshold = mBodyTrkMinKp;
  // <---- Process realtime dynamic parameters

  sl::Bodies bodies;
  sl::ERROR_CODE btRes =
    mZed->retrieveBodies(bodies, bt_params_rt, mBodyTrkInstID);

  if (btRes != sl::ERROR_CODE::SUCCESS) {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "Body Tracking error: " << sl::toString(btRes));
    return;
  }

  if (!bodies.is_new) {    // Async body tracking. Update data only if new
    // detection is available
    DEBUG_BT("No new bodies detected");
    return;
  }

  size_t bodyCount = bodies.body_list.size();

  DEBUG_STREAM_BT("Detected " << bodyCount << " bodies");

  auto bodyMsg = std::make_unique<zed_msgs::msg::ObjectsStamped>();

  bodyMsg->header.stamp = t;
  bodyMsg->header.frame_id = mLeftCamFrameId;

  bodyMsg->objects.resize(bodyCount);

  size_t idx = 0;
  for (auto body : bodies.body_list) {
    std::string label = "Body_";
    label += std::to_string(body.id);
    DEBUG_STREAM_BT("Processing body: " << label);
    bodyMsg->objects[idx].label = sl::String(label.c_str());
    bodyMsg->objects[idx].sublabel = "";
    bodyMsg->objects[idx].label_id = body.id;
    bodyMsg->objects[idx].confidence = body.confidence;
    DEBUG_BT(" - Info OK");

    memcpy(
      &(bodyMsg->objects[idx].position[0]), &(body.position[0]),
      3 * sizeof(float));
    memcpy(
      &(bodyMsg->objects[idx].position_covariance[0]),
      &(body.position_covariance[0]), 6 * sizeof(float));
    memcpy(
      &(bodyMsg->objects[idx].velocity[0]), &(body.velocity[0]),
      3 * sizeof(float));
    DEBUG_BT(" - Pos/Cov/Speed OK");

    bodyMsg->objects[idx].tracking_available = mBodyTrkEnableTracking;
    bodyMsg->objects[idx].tracking_state =
      static_cast<int8_t>(body.tracking_state);
    bodyMsg->objects[idx].action_state =
      static_cast<int8_t>(body.action_state);
    DEBUG_BT(" - Status OK");

    if (body.bounding_box_2d.size() == 4) {
      memcpy(
        &(bodyMsg->objects[idx].bounding_box_2d.corners[0]),
        &(body.bounding_box_2d[0]), 8 * sizeof(unsigned int));
    }
    DEBUG_BT(" - BBox 2D OK");
    if (body.bounding_box.size() == 8) {
      memcpy(
        &(bodyMsg->objects[idx].bounding_box_3d.corners[0]),
        &(body.bounding_box[0]), 24 * sizeof(float));
    }
    DEBUG_BT(" - BBox 3D OK");

    memcpy(
      &(bodyMsg->objects[idx].dimensions_3d[0]), &(body.dimensions[0]),
      3 * sizeof(float));
    DEBUG_BT(" - Dims OK");

    bodyMsg->objects[idx].body_format = static_cast<uint8_t>(mBodyTrkFmt);

    if (body.head_bounding_box_2d.size() == 4) {
      memcpy(
        &(bodyMsg->objects[idx].head_bounding_box_2d.corners[0]),
        &(body.head_bounding_box_2d[0]), 8 * sizeof(unsigned int));
    }
    if (body.head_bounding_box.size() == 8) {
      memcpy(
        &(bodyMsg->objects[idx].head_bounding_box_3d.corners[0]),
        &(body.head_bounding_box[0]), 24 * sizeof(float));
    }

    memcpy(
      &(bodyMsg->objects[idx].head_position[0]),
      &(body.head_position[0]), 3 * sizeof(float));

    bodyMsg->objects[idx].skeleton_available = true;

    uint8_t kp_size = body.keypoint_2d.size();
    DEBUG_STREAM_BT(" * Skeleton KP: " << static_cast<int>(kp_size));
    if (kp_size <= 70) {
      memcpy(
        &(bodyMsg->objects[idx].skeleton_2d.keypoints[0]),
        &(body.keypoint_2d[0]), 2 * kp_size * sizeof(float));

      memcpy(
        &(bodyMsg->objects[idx].skeleton_3d.keypoints[0]),
        &(body.keypoint[0]), 3 * kp_size * sizeof(float));
    }

    // ----------------------------------
    // at the end of the loop

    // TODO(Walter) Add support for
    // body.global_root_orientation;
    // body.local_orientation_per_joint;
    // body.local_orientation_per_joint;

    idx++;
  }

  DEBUG_STREAM_OD("Publishing BODY TRK message");
  try {
    mPubBodyTrk->publish(std::move(bodyMsg));
  } catch (std::system_error & e) {
    DEBUG_STREAM_COMM("Message publishing ecception: " << e.what());
  } catch (...) {
    DEBUG_STREAM_COMM("Message publishing generic ecception: ");
  }

  // ----> Diagnostic information update
  mBodyTrkElabMean_sec->addValue(btElabTimer.toc());
  mBodyTrkPeriodMean_sec->addValue(mBtFreqTimer.toc());
  mBtFreqTimer.tic();
  // <---- Diagnostic information update
}

} // namespace stereolabs
