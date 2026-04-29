#pragma once

#include <cmath>
#include <numeric>
#include <opencv2/opencv.hpp>
#include <sl/Camera.hpp>
#include <sl/CameraOne.hpp>

// Default minimum stereo pairs (batch solve, live preview threshold, and fisheye drop floor).
// Override via config `sample_collection.min` in zed_stereo_calibration main.
constexpr int DEFAULT_MIN_STEREO_SAMPLES = 20;

struct CameraCalib {
  cv::Mat K;
  cv::Mat D;
  bool disto_model_RadTan = true;

  void print(const std::string &name) const {
    std::cout << name << " K:" << std::endl << K << std::endl;
    std::cout << " D:" << std::endl << D << std::endl;
  }

  void initDefault(bool radtan) {
    disto_model_RadTan = radtan;
    K = cv::Mat::eye(3, 3, CV_64FC1);
    if (disto_model_RadTan) {
      // Radial and tangential distortion
      const int nb_coeff = 8;  // 6 radial + 2 tangential; could be extended to
                               // 12 with prism distortion
      D = cv::Mat::zeros(1, nb_coeff, CV_64FC1);
    } else {
      // Fisheye model has 4 coefficients: k1, k2, k3, k4
      D = cv::Mat::zeros(1, 4, CV_64FC1);
    }
  }

  void setFrom(const sl::CameraParameters &cam) {
    K = cv::Mat::eye(3, 3, CV_64FC1);
    K.at<double>(0, 0) = static_cast<double>(cam.fx);
    K.at<double>(1, 1) = static_cast<double>(cam.fy);
    K.at<double>(0, 2) = static_cast<double>(cam.cx);
    K.at<double>(1, 2) = static_cast<double>(cam.cy);

    // tangential distortion coefficients are not used in the Fisheye model,
    // looking for p1 and p2 equal to 0
    if (cam.disto[2] == 0. && cam.disto[3] == 0. && cam.disto[4] != 0. &&
        cam.disto[5] != 0.) {
      disto_model_RadTan = false;  // -> Fisheye model
      // Fisheye model has 4 coefficients: k1, k2, k3, k4
      D = cv::Mat::zeros(1, 4, CV_64FC1);
      D.at<double>(0) = cam.disto[0];
      D.at<double>(1) = cam.disto[1];
      D.at<double>(2) = cam.disto[4];
      D.at<double>(3) = cam.disto[5];
    } else {
      disto_model_RadTan = true;  // Radial and tangential distortion
      const int nb_coeff = 8;     // 6 radial + 2 tangential; could be extended
                                  // to 12 with prism distortion
      D = cv::Mat::zeros(1, nb_coeff, CV_64FC1);
      for (int i = 0; i < nb_coeff; i++) D.at<double>(i) = cam.disto[i];
    }
  }

  std::vector<cv::Point2d> undistortPoints(
      const std::vector<cv::Point2d> &points) const {
    std::cout << "K:" << std::endl << K << std::endl;
    std::cout << "D:" << std::endl << D << std::endl;
    std::vector<cv::Point2d> undistorted_points;
    if (disto_model_RadTan) {
      cv::undistortPoints(points, undistorted_points, K, D);
    } else {
      cv::fisheye::undistortPoints(points, undistorted_points, K, D);
    }
    return undistorted_points;
  }

  // Note: object_points and image_points must be point3f. Point3d formatis not
  // supported by 'cv::calibrateCamera'
  double mono_calibrate(const std::vector<std::vector<cv::Point3f>> &object_points,
                  const std::vector<std::vector<cv::Point2f>> &image_points,
                  const cv::Size &image_size, int flags, bool verbose) {
    double rms = -1.0f;
    std::vector<cv::Mat> rvec, tvec;
    if (disto_model_RadTan) {
      if (D.cols >= 8) {
        flags += cv::CALIB_RATIONAL_MODEL;
        if (verbose) {
          std::cout << "[DEBUG][mono_calibrate] Using "
                       "Rational model (8 distortion coefficients) for calibration..."
                    << std::endl;
        }
      }
      if (verbose) {
        std::cout << "[DEBUG][mono_calibrate] Calibrating with "
                     "Radial-Tangential model..."
                  << std::endl;
      }
      rms = cv::calibrateCamera(object_points, image_points, image_size, K, D,
                                rvec, tvec, flags);
    } else {
      if (verbose) {
        std::cout << "[DEBUG][mono_calibrate] Calibrating with Fisheye model..."
                  << std::endl;
      }
      rms = cv::fisheye::calibrate(
          object_points, image_points, image_size, K, D, rvec, tvec,
          flags + cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC +
              cv::fisheye::CALIB_FIX_SKEW);
    }

    if (verbose) {
      std::cout << "[DEBUG][mono_calibrate] * Intrinsic matrix K:" << std::endl
                << K << std::endl;
      std::cout << "[DEBUG][mono_calibrate] * Distortion coefficients D:" << std::endl
                << D << std::endl;
      std::cout << "[DEBUG][mono_calibrate] * Re-projection error (RMS): " << rms << std::endl;
    }

    return rms;
  }
};

struct StereoCalib {
  CameraCalib left;
  CameraCalib right;

  cv::Mat R;   // Rotation matrix between left and right camera
  cv::Mat Rv;  // Rotation vector between left and right camera
  cv::Mat T;   // Translation vector between left and right camera
  
  cv::Size imageSize;

  void initDefault(bool radtan) {
    left.initDefault(radtan);
    right.initDefault(radtan);
    R = cv::Mat::eye(3, 3, CV_64FC1);
    Rv = cv::Mat::zeros(3, 1, CV_64FC1);
    T = cv::Mat::zeros(3, 1, CV_64FC1);
  }

  void setFrom(const sl::CalibrationParameters &calib_params) {
    left.setFrom(calib_params.left_cam);
    right.setFrom(calib_params.right_cam);

    auto translation = calib_params.stereo_transform.getTranslation();
    T.at<double>(0) = translation.x * -1;  // the zed configuration file store
                                          // the absolute value of the Tx part
    T.at<double>(1) = translation.y;
    T.at<double>(2) = translation.z;

    auto rot = calib_params.stereo_transform.getRotationVector();
    Rv.at<double>(0) = rot.x;
    Rv.at<double>(1) = rot.y;
    Rv.at<double>(2) = rot.z;
    cv::Rodrigues(Rv, R);
  }

  // Note: object_points and image_points must be point3f. Point3d format is not
  // supported by 'cv::stereoCalibrate'
  double stereo_calibrate(
      const std::vector<std::vector<cv::Point3f>> &object_points,
      const std::vector<std::vector<cv::Point2f>> &image_points_left,
      const std::vector<std::vector<cv::Point2f>> &image_points_right,
      const cv::Size &image_size, int flags, bool verbose,
      bool fisheye_retry_ill_conditioned = true,
      int *fisheye_ill_conditioned_index_out = nullptr,
      int min_pairs_after_drop = DEFAULT_MIN_STEREO_SAMPLES) {
    
    imageSize = image_size;
    if (fisheye_ill_conditioned_index_out) {
      *fisheye_ill_conditioned_index_out = -1;
    }

    if (image_size.width <= 0 || image_size.height <= 0) {
      std::cerr << "[ERROR][stereo_calibrate] Invalid image size: " << image_size
                << std::endl;
      return -1.0;
    }
    if (object_points.empty() || image_points_left.empty() || image_points_right.empty()) {
      std::cerr << "[ERROR][stereo_calibrate] Empty points input." << std::endl;
      return -1.0;
    }
    if (object_points.size() != image_points_left.size() ||
        object_points.size() != image_points_right.size()) {
      std::cerr << "[ERROR][stereo_calibrate] Mismatched sample counts: object="
                << object_points.size() << ", left=" << image_points_left.size()
                << ", right=" << image_points_right.size() << std::endl;
      return -1.0;
    }
    if (left.K.rows != 3 || left.K.cols != 3 || right.K.rows != 3 || right.K.cols != 3) {
      std::cerr << "[ERROR][stereo_calibrate] Invalid intrinsic matrix shape."
                << " left.K=" << left.K.rows << "x" << left.K.cols
                << ", right.K=" << right.K.rows << "x" << right.K.cols << std::endl;
      return -1.0;
    }
    
    double rms = 0.0;
    cv::Mat E, F;
    
    if (left.disto_model_RadTan && right.disto_model_RadTan) {
      if (verbose) {
        std::cout
            << "[DEBUG][stereo_calibrate] Calibrating with Radial-Tangential model..."
            << std::endl;
      }
      if (left.D.cols >= 8 && right.D.cols >= 8) {
        flags += cv::CALIB_RATIONAL_MODEL;
        if (verbose) {
          std::cout << "[DEBUG][stereo_calibrate] Using "
                       "Rational model (8 distortion coefficients) for stereo calibration..."
                    << std::endl;
        }
      }

      rms = cv::stereoCalibrate(object_points, image_points_left,
                                image_points_right, left.K, left.D, right.K,
                                right.D, image_size, R, T, E, F, flags);
    } else {
      if (verbose) {
        std::cout
            << "[DEBUG][stereo_calibrate] Calibrating with Fisheye model..."
            << std::endl;
      }

      // OpenCV fisheye::stereoCalibrate expects fisheye flag bits and 4-coeff distortion.
      int fisheye_flags = 0;
      if (flags & cv::CALIB_FIX_INTRINSIC) {
        fisheye_flags |= cv::fisheye::CALIB_FIX_INTRINSIC;
      }

      cv::Mat left_d = left.D.clone();
      cv::Mat right_d = right.D.clone();
      if (left_d.total() < 4 || right_d.total() < 4) {
        std::cerr << "[ERROR][stereo_calibrate] Invalid fisheye distortion size."
                  << " left.D total=" << left_d.total()
                  << ", right.D total=" << right_d.total() << std::endl;
        return -1.0;
      }
      // Keep only k1..k4 and force explicit 4x1 CV_64F layout to match fisheye API expectations.
      cv::Mat left_d_flat = left_d.reshape(1, static_cast<int>(left_d.total()));
      cv::Mat right_d_flat = right_d.reshape(1, static_cast<int>(right_d.total()));
      cv::Mat left_d4(4, 1, CV_64F);
      cv::Mat right_d4(4, 1, CV_64F);
      for (int i = 0; i < 4; ++i) {
        left_d4.at<double>(i, 0) = left_d_flat.at<double>(i, 0);
        right_d4.at<double>(i, 0) = right_d_flat.at<double>(i, 0);
      }

      cv::Mat left_k64, right_k64;
      left.K.convertTo(left_k64, CV_64F);
      right.K.convertTo(right_k64, CV_64F);

      // Build explicit double-precision point vectors to avoid wrapper ambiguity in some OpenCV builds.
      std::vector<std::vector<cv::Point3d>> object_points_d(object_points.size());
      std::vector<std::vector<cv::Point2d>> image_points_left_d(image_points_left.size());
      std::vector<std::vector<cv::Point2d>> image_points_right_d(image_points_right.size());
      for (size_t i = 0; i < object_points.size(); ++i) {
        object_points_d[i].reserve(object_points[i].size());
        for (const auto& p : object_points[i]) {
          object_points_d[i].emplace_back(static_cast<double>(p.x), static_cast<double>(p.y), static_cast<double>(p.z));
        }
      }
      for (size_t i = 0; i < image_points_left.size(); ++i) {
        image_points_left_d[i].reserve(image_points_left[i].size());
        image_points_right_d[i].reserve(image_points_right[i].size());
        for (const auto& p : image_points_left[i]) {
          image_points_left_d[i].emplace_back(static_cast<double>(p.x), static_cast<double>(p.y));
        }
        for (const auto& p : image_points_right[i]) {
          image_points_right_d[i].emplace_back(static_cast<double>(p.x), static_cast<double>(p.y));
        }
      }

      auto parse_input_array_index = [](const char *what) -> int {
        const std::string msg = what ? std::string(what) : std::string();
        const std::string needle = "input array ";
        auto pos = msg.find(needle);
        if (pos == std::string::npos) return -1;
        try {
          return std::stoi(msg.substr(pos + needle.size()));
        } catch (...) {
          return -1;
        }
      };

      if (fisheye_retry_ill_conditioned) {
        const int initial_count = static_cast<int>(object_points_d.size());
        const int max_drop_attempts = std::max(1, initial_count / 2);
        int dropped = 0;
        while (true) {
          try {
            rms = cv::fisheye::stereoCalibrate(object_points_d, image_points_left_d,
                                               image_points_right_d, left_k64, left_d4,
                                               right_k64, right_d4, image_size, R, T,
                                               fisheye_flags);
            break;
          } catch (const cv::Exception& e) {
            const int bad_idx = parse_input_array_index(e.what());

            const int remaining = static_cast<int>(object_points_d.size());
            const bool can_drop_indexed =
                bad_idx >= 0 &&
                bad_idx < remaining &&
                dropped < max_drop_attempts &&
                remaining - 1 >= min_pairs_after_drop;

            // Only apply robustness for explicit "input array N" errors.
            if (can_drop_indexed) {
              std::cerr << "[WARN][stereo_calibrate] Dropping ill-conditioned sample #"
                        << bad_idx << " and retrying (" << (remaining - 1)
                        << " remaining). Reason: " << e.what() << std::endl;
              object_points_d.erase(object_points_d.begin() + bad_idx);
              image_points_left_d.erase(image_points_left_d.begin() + bad_idx);
              image_points_right_d.erase(image_points_right_d.begin() + bad_idx);
              ++dropped;
              continue;
            }

            std::cerr << "[ERROR][stereo_calibrate] OpenCV exception: " << e.what() << std::endl;
            std::cerr << "[ERROR][stereo_calibrate] image_size=" << image_size
                      << ", left.K=" << left.K.rows << "x" << left.K.cols
                      << ", right.K=" << right.K.rows << "x" << right.K.cols
                      << ", left.D total=" << left_d4.total()
                      << ", right.D total=" << right_d4.total()
                      << ", fisheye_flags=" << fisheye_flags
                      << ", samples_remaining=" << object_points_d.size()
                      << ", dropped=" << dropped << std::endl;
            return -1.0;
          }
        }
        if (dropped > 0) {
          std::cout << "[INFO][stereo_calibrate] Stereo calibration succeeded after dropping "
                    << dropped << " explicit ill-conditioned sample(s). Final: "
                    << object_points_d.size() << "/" << initial_count << " pairs."
                    << std::endl;
        }
      } else {
        try {
          rms = cv::fisheye::stereoCalibrate(object_points_d, image_points_left_d,
                                             image_points_right_d, left_k64, left_d4,
                                             right_k64, right_d4, image_size, R, T,
                                             fisheye_flags);
        } catch (const cv::Exception& e) {
          const int bad_idx = parse_input_array_index(e.what());
          if (fisheye_ill_conditioned_index_out && bad_idx >= 0) {
            *fisheye_ill_conditioned_index_out = bad_idx;
          }
          std::cerr << "[ERROR][stereo_calibrate] OpenCV exception: " << e.what() << std::endl;
          return -1.0;
        }
      }

      left.K = left_k64;
      right.K = right_k64;
      left.D = left_d4;
      right.D = right_d4;
    }

    cv::Rodrigues(R, Rv);

    if (verbose) {
      std::cout << "[DEBUG][stereo_calibrate] * New Intrinsic matrix K left:"
                << std::endl
                << left.K << std::endl;
      std::cout << "[DEBUG][stereo_calibrate] * New Distortion coefficients D left:" << std::endl
                << left.D << std::endl;
      std::cout << "[DEBUG][stereo_calibrate] * New Intrinsic matrix K right:" << std::endl
                << right.K << std::endl;
      std::cout << "[DEBUG][stereo_calibrate] * New Distortion coefficients D right:" << std::endl
                << right.D << std::endl;
      std::cout << "[DEBUG][stereo_calibrate] * Re-projection error (RMS): " << rms << std::endl;
      std::cout << "[DEBUG][stereo_calibrate] * Rotation matrix R:" << std::endl
                << R << std::endl;
      std::cout << "[DEBUG][stereo_calibrate] * Rotation vector Rv:" << std::endl
                << Rv << std::endl;
      std::cout << "[DEBUG][stereo_calibrate] * Translation vector T:" << std::endl
                << T << std::endl;
      std::cout << "[DEBUG][stereo_calibrate] * Re-projection error (RMS): " << rms << std::endl;
    }

    return rms;
  }

  std::string saveCalibOpenCV(int serial, const std::string &calibration_output_dir = "");
  std::string saveCalibZED(int serial, int left_sn, int right_sn, bool is_4k,
                           const std::string &calibration_output_dir = "");
};

int calibrate(int img_count, const std::string &folder, StereoCalib &raw_data,
              int h_edges, int v_edges, double square_size, int serial,
              int left_sn, int right_sn,
              bool is_dual_mono, bool is_4k, bool save_calib_mono = false,
              bool use_intrinsic_prior = false,
              bool recalibrate_intrinsics = false,
              double max_repr_error = 1.0f, bool verbose = false,
              int min_stereo_samples = DEFAULT_MIN_STEREO_SAMPLES,
              const std::string &calibration_output_dir = "");