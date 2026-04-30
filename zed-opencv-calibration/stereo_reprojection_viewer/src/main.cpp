///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2025, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////
//
// © 2026, Cargo Robotics — YAML-driven virtual stereo config and container paths.
// Adopted from https://github.com/stereolabs/zed-opencv-calibration (upstream Stereolabs sources; Stereolabs license above; Cargo patches — see README).
//

// ZED includes
#include <sl/Camera.hpp>
#include <sl/CameraOne.hpp>

// Sample includes
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>

#include "GLViewer.hpp"

// Macros
#define SHOW_MASK 0

// Using std and sl namespaces
using namespace std;
using namespace sl;
namespace fs = std::filesystem;

struct ViewerConfig {
  int left_sn = -1;
  int right_sn = -1;
  std::string resolution = "HD1200";
  std::string calibration_output_dir = "/root/zed-calibration-out";
  bool verbose = true;
};

bool parseResolution(const std::string &resolution, sl::RESOLUTION &out_resolution) {
  if (resolution == "HD1200") {
    out_resolution = sl::RESOLUTION::HD1200;
    return true;
  }
  if (resolution == "HD1080") {
    out_resolution = sl::RESOLUTION::HD1080;
    return true;
  }
  if (resolution == "HD720") {
    out_resolution = sl::RESOLUTION::HD720;
    return true;
  }
  if (resolution == "SVGA") {
    out_resolution = sl::RESOLUTION::SVGA;
    return true;
  }
  return false;
}

bool loadViewerConfig(const std::string &path, ViewerConfig &cfg) {
  YAML::Node node;
  try {
    node = YAML::LoadFile(path);
  } catch (const std::exception &e) {
    std::cerr << "Error: failed to load config file '" << path << "': " << e.what() << std::endl;
    return false;
  }

  if (!node["left_sn"] || !node["right_sn"]) {
    std::cerr << "Error: config must define 'left_sn' and 'right_sn'." << std::endl;
    return false;
  }
  cfg.left_sn = node["left_sn"].as<int>();
  cfg.right_sn = node["right_sn"].as<int>();
  if (node["resolution"] && node["resolution"].IsScalar()) {
    cfg.resolution = node["resolution"].as<std::string>();
  }
  if (node["calibration_output_dir"] && node["calibration_output_dir"].IsScalar()) {
    cfg.calibration_output_dir = node["calibration_output_dir"].as<std::string>();
  }
  if (node["verbose"]) {
    cfg.verbose = node["verbose"].as<bool>();
  }
  return true;
}

cv::Mat cvtDisto(const sl::CameraParameters &camera_param, bool fisheye) {
  cv::Mat disto;
  if (!fisheye) {
    const int num_disto_coeffs =
        12;  // Number of distortion coefficients in ZED SDK
    disto = cv::Mat(1, num_disto_coeffs, CV_64F);
    for (int d = 0; d < num_disto_coeffs; ++d)
      disto.at<double>(0, d) = camera_param.disto[d];
  } else {
    const double *ptr = camera_param.disto;
    disto = cv::Mat(1, 4, CV_64FC1);
    disto.at<double>(0) = ptr[0];
    disto.at<double>(1) = ptr[1];
    disto.at<double>(2) = ptr[4];
    disto.at<double>(3) = ptr[5];
  }

  std::cout << "Distortion Coefficients:\n" << disto << std::endl;

  return disto;
}

cv::Mat cvtCameraParam(const sl::CameraParameters &camera_param) {
  // Convert the ZED camera parameters to OpenCV format
  cv::Mat camera_matrix = cv::Mat::zeros(3, 3, CV_64F);
  camera_matrix.at<double>(0, 0) = camera_param.fx;
  camera_matrix.at<double>(1, 1) = camera_param.fy;
  camera_matrix.at<double>(0, 2) = camera_param.cx;
  camera_matrix.at<double>(1, 2) = camera_param.cy;
  camera_matrix.at<double>(2, 2) = 1.0;
  return camera_matrix;
}

// Conversion function between sl::Mat and cv::Mat
cv::Mat slMat2cvMat(sl::Mat &input) {
  int cv_type = -1;
  switch (input.getDataType()) {
    case sl::MAT_TYPE::F32_C1:
      cv_type = CV_32FC1;
      break;
    case sl::MAT_TYPE::F32_C2:
      cv_type = CV_32FC2;
      break;
    case sl::MAT_TYPE::F32_C3:
      cv_type = CV_32FC3;
      break;
    case sl::MAT_TYPE::F32_C4:
      cv_type = CV_32FC4;
      break;
    case sl::MAT_TYPE::U8_C1:
      cv_type = CV_8UC1;
      break;
    case sl::MAT_TYPE::U8_C2:
      cv_type = CV_8UC2;
      break;
    case sl::MAT_TYPE::U8_C3:
      cv_type = CV_8UC3;
      break;
    case sl::MAT_TYPE::U8_C4:
      cv_type = CV_8UC4;
      break;
    default:
      break;
  }
  // Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer
  // from sl::Mat (getPtr<T>()) cv::Mat and sl::Mat will share a single memory
  // structure
  return cv::Mat(input.getHeight(), input.getWidth(), cv_type,
                 input.getPtr<sl::uchar1>(sl::MEM::CPU));
}

void reproDepth(cv::Mat &pc, cv::Mat &im, cv::Mat &disto,
                cv::Mat &camera_matrix, bool fisheye) {
  const int w = pc.cols;
  const int h = pc.rows;

  float d_min = 9999.f;  // Minimum depth threshold
  float d_max = 0.0f;    // Maximum depth threshold

  std::vector<cv::Point3f> world_points;
  world_points.reserve(w * h);
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const auto &p = pc.at<cv::Vec4f>(y, x);
      if (std::isfinite(p[2])) {  // Check if the point is valid
        cv::Point3f point(p[0], -p[1], -p[2]);

        // convert to image coordinates
        world_points.emplace_back(point);
        d_min = std::min(d_min, point.z);
        d_max = std::max(d_max, point.z);
      }
    }
  }

  world_points.shrink_to_fit();

  cv::Scalar blue(0, 0, 255, 250);
  cv::Scalar red(255, 0, 0, 250);

  std::vector<cv::Point2f> image_points;
  // Undistort the point cloud using OpenCV
  auto tvec = cv::Mat::zeros(3, 1, CV_64F);  // Translation vector
  auto rvec = cv::Mat::zeros(3, 1, CV_64F);  // Rotation vector
  if (fisheye) {
    // For fisheye cameras, use the fisheye projection
    cv::fisheye::projectPoints(world_points, image_points, rvec, tvec,
                               camera_matrix, disto);
  } else {
    // For non-fisheye cameras, use the standard projectPoints function
    cv::projectPoints(world_points, rvec, tvec, camera_matrix, disto,
                      image_points);
  }

  int count = 0;
  for (const auto &p : image_points) {
    if (p.x >= 0 && p.x < im.cols && p.y >= 0 && p.y < im.rows) {
      // Draw the projected points on the image
      // scale pt
      float scale = (world_points[count].z - d_min) / (d_max - d_min);
      scale = std::max(0.0f, std::min(1.0f, scale));  // Clamp to [0, 1]
      cv::Scalar color =
          blue * scale +
          red * (1.0f - scale);  // Interpolate between blue and red

      auto &color_im =
          im.at<cv::Vec4b>(static_cast<int>(p.y), static_cast<int>(p.x));

      // fade the color based on depth
      color_im = (color_im * 0.3 +
                  cv::Vec4b(color[0], color[1], color[2], color[3]) * 0.7);
    }
    count++;
  }
  cv::imshow("Projected Points", im);
}

#if SHOW_MASK
cv::Mat createMaskUsingUndistortPoints(int width, int height,
                                       cv::Mat &camera_matrix,
                                       cv::Mat &dist_coeffs,
                                       cv::Mat &new_camera_matrix,
                                       bool fisheye) {
  // Create all pixel coordinates
  std::vector<cv::Point2f> distorted_points;

  int rad = 0;
  int x, y;
  for (x = rad; x < (width - rad); ++x)
    distorted_points.emplace_back(static_cast<float>(x), static_cast<float>(y));

  for (y = rad; y < (height - rad); ++y)
    distorted_points.emplace_back(static_cast<float>(x), static_cast<float>(y));

  for (; x >= rad; --x)
    distorted_points.emplace_back(static_cast<float>(x), static_cast<float>(y));

  for (; y >= rad; --y)
    distorted_points.emplace_back(static_cast<float>(x), static_cast<float>(y));

  // Undistort points to check validity
  std::vector<cv::Point2f> undistorted_points;  // Initialize with the same size

  if (fisheye) {
    cv::fisheye::undistortPoints(distorted_points, undistorted_points,
                                 camera_matrix, dist_coeffs, cv::Mat(),
                                 new_camera_matrix);
  } else {
    cv::undistortPoints(distorted_points, undistorted_points, camera_matrix,
                        dist_coeffs, cv::Mat(), new_camera_matrix);
  }

  cv::Mat mask = cv::Mat::zeros(height, width, CV_8UC1);
  cv::fillPoly(mask,
               std::vector<std::vector<cv::Point>>{std::vector<cv::Point>(
                   undistorted_points.begin(), undistorted_points.end())},
               cv::Scalar(255));

  return mask;
}
#endif
int main(int argc, char **argv) {
  if (argc > 1) {
    std::cerr << "Error: zed_reprojection_viewer no longer accepts CLI arguments." << std::endl;
    std::cerr << "It now reads everything from YAML config: /opt/zed-opencv-calibration/config/fisheye_stereo.yaml" << std::endl;
    return EXIT_FAILURE;
  }

  const std::string config_path = "/opt/zed-opencv-calibration/config/fisheye_stereo.yaml";
  const std::string config_path_fallback = "/root/zed-opencv-calibration/config/fisheye_stereo.yaml";
  ViewerConfig cfg;
  if (!loadViewerConfig(config_path, cfg)) {
    if (!loadViewerConfig(config_path_fallback, cfg)) {
      return EXIT_FAILURE;
    }
  }

  Camera zed;
  // Set configuration parameters for the ZED
  InitParameters init_params;
  init_params.depth_mode = DEPTH_MODE::NEURAL;
  init_params.coordinate_system =
      COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP;  // OpenGL's coordinate system is
                                             // right_handed
  init_params.sdk_verbose = cfg.verbose ? 1 : 0;
  init_params.maximum_working_resolution = sl::Resolution(0, 0);
  init_params.camera_fps = 15;
  if (!parseResolution(cfg.resolution, init_params.camera_resolution)) {
    std::cerr << "Error: unsupported resolution '" << cfg.resolution
              << "'. Expected one of: HD1200, HD1080, HD720, SVGA." << std::endl;
    return EXIT_FAILURE;
  }

  int left_sn = cfg.left_sn;
  int right_sn = cfg.right_sn;
  int sn_stereo = sl::generateVirtualStereoSerialNumber(left_sn, right_sn);
  std::cout << " * Virtual stereo SN (SDK-generated): " << sn_stereo << std::endl;

  // Paths must live until zed.open() (sl::String stores the pointer).
  std::string optional_opencv_calib_abs;
  std::string optional_settings_dir_abs;

  if (!cfg.calibration_output_dir.empty()) {
    const fs::path calib_dir(cfg.calibration_output_dir);
    auto path_sn_conf = [&](int sn) { return calib_dir / ("SN" + std::to_string(sn) + ".conf"); };
    auto path_opencv_yml = [&](int sn) { return calib_dir / ("zed_calibration_" + std::to_string(sn) + ".yml"); };

    fs::path expected_conf = path_sn_conf(sn_stereo);
    if (!fs::exists(expected_conf)) {
      const int swapped_sn = sl::generateVirtualStereoSerialNumber(right_sn, left_sn);
      const fs::path swapped_conf = path_sn_conf(swapped_sn);
      if (fs::exists(swapped_conf)) {
        std::cout << " * Found calibration file for swapped serial order: " << swapped_conf << std::endl;
        std::cout << " * Switching virtual stereo order to match existing calibration file." << std::endl;
        std::swap(left_sn, right_sn);
        sn_stereo = swapped_sn;
        expected_conf = swapped_conf;
      }
    }

    const fs::path opencv_yml = path_opencv_yml(sn_stereo);
    const bool have_yml = fs::exists(opencv_yml);
    const bool have_conf = fs::exists(expected_conf);

    std::cout << " * Calibration dir:            " << calib_dir << std::endl;
    if (have_yml) {
      std::cout << " * Found OpenCV calibration:   " << opencv_yml << std::endl;
    }
    if (have_conf) {
      std::cout << " * Found SN*.conf:             " << expected_conf << std::endl;
    }

    // Prefer OpenCV YAML (same files produced by zed_stereo_calibration). The SDK parses this reliably;
    // "Fail to load valid camera calibration" often comes from rejecting SN*.conf under optional_settings_path.
    if (have_yml) {
      optional_opencv_calib_abs = fs::absolute(opencv_yml).generic_string();
      init_params.optional_opencv_calibration_file = sl::String(optional_opencv_calib_abs.c_str());
      std::cout << " * Using optional_opencv_calibration_file (preferred)." << std::endl;
    } else if (have_conf) {
      optional_settings_dir_abs = fs::absolute(calib_dir).generic_string();
      init_params.optional_settings_path = sl::String(optional_settings_dir_abs.c_str());
      std::cout << " * Using optional_settings_path for SN*.conf only (no zed_calibration_*.yml found)." << std::endl;
    } else {
      std::cerr << "Warning: no zed_calibration_" << sn_stereo << ".yml or SN" << sn_stereo << ".conf in "
                << calib_dir << std::endl;
      std::cerr << "         SDK will use factory/stream calibration; depth may not match your calibration run." << std::endl;
    }
  }

  init_params.input.setVirtualStereoFromSerialNumbers(left_sn, right_sn, sn_stereo);

  // Open the camera
  auto returned_state = zed.open(init_params);
  if (returned_state > ERROR_CODE::SUCCESS) {
    print("Camera Open", returned_state, "Exit program.");
    return EXIT_FAILURE;
  }

  // Automatically set to the optimal resolution
  sl::Resolution res(720, 404);

  auto camera_config = zed.getCameraInformation(res).camera_configuration;

  Mat point_cloud(res, sl::MAT_TYPE::F32_C4, MEM::BOTH);
  auto pc_ocv = slMat2cvMat(point_cloud);

  Mat image(res, sl::MAT_TYPE::U8_C4, MEM::CPU);
  auto im_ocv = slMat2cvMat(image);

  Mat image_rect(res, sl::MAT_TYPE::U8_C4, MEM::CPU);
  auto im_rect_ocv = slMat2cvMat(image_rect);

  const sl::CameraParameters &left_cam = camera_config.calibration_parameters_raw.left_cam;
  const bool fisheye = (left_cam.disto[2] == 0.0f && left_cam.disto[3] == 0.0f &&
                        left_cam.disto[4] != 0.0f && left_cam.disto[5] != 0.0f);

  auto stream = zed.getCUDAStream();

  auto K = cvtCameraParam(camera_config.calibration_parameters_raw.left_cam);
  auto disto =
      cvtDisto(camera_config.calibration_parameters_raw.left_cam, fisheye);

#if SHOW_MASK
  auto K_new = cvtCameraParam(camera_config.calibration_parameters.left_cam);


  auto mask_cv = createMaskUsingUndistortPoints(res.width, res.height, K, disto,
                                                K_new, fisheye);

  // Convert cv::Mat mask to sl::Mat
  cv::imshow("Mask", mask_cv);
  cv::imwrite("mask.png", mask_cv);

  sl::Mat mask_sl;
  mask_sl.read("mask.png");
  zed.setRegionOfInterest(mask_sl);
#endif

  // Point cloud viewer
  GLViewer viewer;
  // Initialize point cloud viewer
  GLenum errgl = viewer.init(
      argc, argv, camera_config.calibration_parameters.left_cam, stream, res);
  if (errgl != GLEW_OK) {
    print("Error OpenGL: " + std::string((char *)glewGetErrorString(errgl)));
    return EXIT_FAILURE;
  }

  RuntimeParameters runParameters;

  // Main Loop
  while (viewer.isAvailable()) {
    // Check that a new image is successfully acquired
    if (zed.grab(runParameters) <= ERROR_CODE::SUCCESS) {
      // retrieve the current 3D coloread point cloud in GPU
      zed.retrieveMeasure(point_cloud, MEASURE::XYZRGBA, MEM::GPU, res);
      viewer.updatePointCloud(point_cloud);

      point_cloud.updateCPUfromGPU();

      zed.retrieveImage(image, VIEW::LEFT_UNRECTIFIED, MEM::CPU, res);

      reproDepth(pc_ocv, im_ocv, disto, K, fisheye);
      zed.retrieveImage(image_rect, VIEW::LEFT, MEM::CPU, res);

#if SHOW_MASK
      // Apply mask to rectified image
      cv::Mat im_rect_masked = im_rect_ocv.clone();
      for (int y = 0; y < im_rect_masked.rows; ++y) {
        for (int x = 0; x < im_rect_masked.cols; ++x) {
          if (mask_cv.at<uchar>(y, x) == 0) {  // If mask is 0 (outside ROI)
            im_rect_masked.at<cv::Vec4b>(y, x) =
                cv::Vec4b(0, 0, 0, 0);  // Set to black/transparent
          }
        }
      }
      cv::imshow("Masked Rectified Image", im_rect_masked);
#endif

      cv::imshow("Rectified Image", im_rect_ocv);
      cv::waitKey(10);  // Wait for a short time to update the image display
    }
  }
  // free allocated memory before closing the ZED
  point_cloud.free();

  // close the ZED
  zed.close();

  return EXIT_SUCCESS;
}
