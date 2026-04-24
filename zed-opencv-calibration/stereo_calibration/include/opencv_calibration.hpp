#pragma once

#include <cmath>
#include <numeric>
#include <opencv2/opencv.hpp>
#include <sl/Camera.hpp>
#include <sl/CameraOne.hpp>

constexpr int MIN_IMAGE = 20;

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
      const cv::Size &image_size, int flags, bool verbose) {
    
    imageSize = image_size;
    
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
      rms = cv::fisheye::stereoCalibrate(object_points, image_points_left,
                                         image_points_right, left.K, left.D,
                                         right.K, right.D, image_size, R, T,
                                         flags + cv::fisheye::CALIB_CHECK_COND);
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

  std::string saveCalibOpenCV(int serial);
  std::string saveCalibZED(int serial, bool is_4k = false);
};

int calibrate(int img_count, const std::string &folder, StereoCalib &raw_data,
              int h_edges, int v_edges, double square_size, int serial,
              bool is_dual_mono, bool is_4k, bool save_calib_mono = false,
              bool use_intrinsic_prior = false, double max_repr_error = 0.5f,
              bool verbose = false);