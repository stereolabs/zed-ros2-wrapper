#include "opencv_calibration.hpp"

#include <filesystem>

int calibrate(int img_count, const std::string& folder, StereoCalib& calib_data,
              int h_edges, int v_edges, double square_size, int serial,
              bool is_dual_mono, bool is_4k, bool save_calib_mono,
              bool use_intrinsic_prior, double max_repr_error, bool verbose) {
  std::vector<cv::Mat> left_images, right_images;

  /// Read images
  cv::Size imageSize = cv::Size(0, 0);

  std::cout << std::endl
            << "Loading the stored images from folder: " << folder << std::endl;

  // Count images in the folder
  if (img_count==-1) {
    std::cout << " * Counting images in the folder..." << std::endl;
    int actual_img_count = 0;
    while(1) {
      std::string left_path = folder + "image_left_" + std::to_string(actual_img_count) + ".png";
      std::string right_path = folder + "image_right_" + std::to_string(actual_img_count) + ".png";
      if (std::filesystem::exists(left_path) && std::filesystem::exists(right_path)) {
        actual_img_count++;
      }else {
        break;
      }
    }
    img_count = actual_img_count;
    std::cout << "   - Found " << img_count << " images." << std::endl;
  }

  for (int i = 0; i < img_count; i++) {
    std::cout << "." << std::flush;
    cv::Mat grey_l =
        cv::imread(folder + "image_left_" + std::to_string(i) + ".png",
                   cv::IMREAD_GRAYSCALE);
    cv::Mat grey_r =
        cv::imread(folder + "image_right_" + std::to_string(i) + ".png",
                   cv::IMREAD_GRAYSCALE);

    if (!grey_l.empty() && !grey_r.empty()) {
      if (imageSize.width == 0)
        imageSize = grey_l.size();
      else if (imageSize != left_images.back().size()) {
        std::cerr << std::endl
                  << " !!! ERROR !!! " << std::endl
                  << "Frames number #" << i
                  << " do not have the same size as the previous ones: "
                  << imageSize << " vs " << left_images.back().size()
                  << std::endl;
        return EXIT_FAILURE;
      }

      left_images.push_back(grey_l);
      right_images.push_back(grey_r);
    }
  }

  std::cout << std::endl
            << " * " << left_images.size() << " samples collected" << std::endl;

  // Define object points of the target
  // Note: object points must be point3f. Point3d is not supported by
  // 'cv::calibrateCamera'
  std::vector<cv::Point3f> pattern_points;
  for (int i = 0; i < v_edges; i++) {
    for (int j = 0; j < h_edges; j++) {
      pattern_points.push_back(
          cv::Point3f(static_cast<float>(square_size * j), static_cast<float>(square_size * i), 0));
    }
  }

  std::vector<std::vector<cv::Point3f>> object_points;
  std::vector<std::vector<cv::Point2f>> pts_l, pts_r;

  cv::Size t_size(h_edges, v_edges);

  std::cout << "Detecting the target corners on the images" << std::endl;

  for (int i = 0; i < left_images.size(); i++) {
    std::cout << "." << std::flush;
    std::vector<cv::Point2f> pts_l_f, pts_r_f;
    bool found_l =
        cv::findChessboardCorners(left_images.at(i), t_size, pts_l_f, 3);
    bool found_r =
        cv::findChessboardCorners(right_images.at(i), t_size, pts_r_f, 3);

    if (found_l && found_r) {

      cv::cornerSubPix(
          left_images.at(i), pts_l_f, cv::Size(5, 5), cv::Size(-1, -1),
          cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER,
                           30, 0.001));

      cv::cornerSubPix(
          right_images.at(i), pts_r_f, cv::Size(5, 5), cv::Size(-1, -1),
          cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER,
                           30, 0.001));    

      pts_l.push_back(pts_l_f);
      pts_r.push_back(pts_r_f);
      object_points.push_back(pattern_points);
    } else {
      std::cout << std::endl
                << "- No valid targets detected on frames #" << i << " -"
                << std::endl;
    }
  }

  /// Compute calibration

  if (pts_l.size() < MIN_IMAGE) {
    std::cout << " !!! Not enough images with the target detected !!!"
              << std::endl;
    std::cout << " Please perform a new data acquisition." << std::endl
              << std::endl;
    return EXIT_FAILURE;
  }

  std::cout << std::endl
            << " * Valid samples: " << pts_l.size() << "/" << img_count
            << std::endl;

  auto flags = use_intrinsic_prior ? cv::CALIB_USE_INTRINSIC_GUESS : 0;
  if (use_intrinsic_prior && verbose) {
    std::cout
        << "[DEBUG][calibrate] Using intrinsic parameters as calibration prior."
        << std::endl;
  }

  std::cout << "Left camera calibration... " << std::flush;
  auto rms_l = calib_data.left.mono_calibrate(object_points, pts_l, imageSize,
                                              flags, verbose);
  std::cout << "Done." << std::endl;

  std::cout << "Right camera calibration... " << std::flush;
  auto rms_r = calib_data.right.mono_calibrate(object_points, pts_r, imageSize,
                                               flags, verbose);
  std::cout << "Done." << std::endl;

  std::cout << "Stereo calibration... " << std::flush;

  auto err = calib_data.stereo_calibrate(
      object_points, pts_l, pts_r, imageSize,
      cv::CALIB_FIX_INTRINSIC + cv::CALIB_ZERO_DISPARITY,
      // cv::CALIB_USE_INTRINSIC_GUESS + cv::CALIB_ZERO_DISPARITY,
      verbose);

  std::cout << "Done." << std::endl;

  std::cout << std::endl << "*** Calibration Report ***" << std::endl;

  std::cout << " * Reprojection errors: " << std::endl;
  std::cout << "   * Left:\t" << rms_l << " px"
            << (rms_l > max_repr_error ? "\t!!! TOO HIGH !!!" : "\t-> GOOD")
            << std::endl;
  std::cout << "   * Right:\t" << rms_r << " px"
            << (rms_r > max_repr_error ? "\t!!! TOO HIGH !!!" : "\t-> GOOD")
            << std::endl;
  std::cout << "   * Stereo:\t" << err << " px"
            << (err > max_repr_error ? "\t!!! TOO HIGH !!!" : "\t-> GOOD")
            << std::endl;
  if (rms_l > max_repr_error || rms_r > max_repr_error ||
      err > max_repr_error) {
    std::cerr
        << std::endl
        << "\t!!! ERROR !!!" << std::endl
        << "The max reprojection error looks too high (> " << max_repr_error
        << " px). Check that the lenses are clean (sharp images)"
           " and that the calibration pattern is printed/mounted on a RIGID "
           "and FLAT surface."
        << std::endl;

    return EXIT_FAILURE;
  }

  if (calib_data.left.K.type() == CV_64F) {
    std::cout << " * Data type: 'double'" << std::endl;
  } else if (calib_data.left.K.type() == CV_32F) {
    std::cout << " * Data type: 'float'" << std::endl;
  } else {
    std::cerr << " !!! Cannot save the calibration file: 'Invalid data type'"
              << std::endl;
    return EXIT_FAILURE;
  }

  if (calib_data.T.at<double>(0) > 0) {
    std::cerr << std::endl
              << "\t !! Warning !!" << std::endl
              << "The value of the baseline has opposite sign (T_x = "
              << calib_data.T.at<double>(0) << ")." << std::endl;
    std::cerr << "Swap left and right cameras and redo the calibration."
              << std::endl;

    return EXIT_FAILURE;
  }

  if (calib_data.T.at<double>(0) > 0) {
    std::cerr
        << std::endl
        << "\t !! Warning !!" << std::endl
        << "The value of the baseline has opposite sign than expected(T_x = "
        << calib_data.T.at<double>(0) << ")." << std::endl;
    std::cerr << "Swap left and right cameras and redo the calibration."
              << std::endl;

    return EXIT_FAILURE;
  }

  constexpr double MIN_BASELINE = 30.0f;  // Minimum possible baseline in mm

  if (fabs(calib_data.T.at<double>(0)) < MIN_BASELINE) {
    std::cerr << std::endl
              << "\t !! Warning !!" << std::endl
              << "The value of the baseline is too small (T_x = "
              << calib_data.T.at<double>(0) << ")." << std::endl;
    std::cerr << "Please redo the calibration to obtain a value that is "
                 "phisically coherent (at least "
              << MIN_BASELINE << " mm)." << std::endl;

    return EXIT_FAILURE;
  }

  std::cout << std::endl;

  std::cout << "** Camera parameters **" << std::endl;
  std::cout << "* Intrinsic mat left:" << std::endl
            << calib_data.left.K << std::endl;
  std::cout << "* Distortion mat left:" << std::endl
            << calib_data.left.D << std::endl;
  std::cout << "* Intrinsic mat right:" << std::endl
            << calib_data.right.K << std::endl;
  std::cout << "* Distortion mat right:" << std::endl
            << calib_data.right.D << std::endl;
  std::cout << std::endl;
  std::cout << "** Extrinsic parameters **" << std::endl;
  std::cout << "* Translation:" << std::endl << calib_data.T << std::endl;
  std::cout << "* Rotation:" << std::endl << calib_data.Rv << std::endl;
  std::cout << std::endl;

  std::cout << std::endl << "*** Save Calibration files ***" << std::endl;

  std::string opencv_file = calib_data.saveCalibOpenCV(serial);
  if (!opencv_file.empty()) {
    std::cout << " * OpenCV calibration file saved: " << opencv_file
              << std::endl;
  } else {
    std::cout << " !!! Failed to save OpenCV calibration file " << opencv_file
              << " !!!" << std::endl;
  }

  // SDK format is only supported for dual-mono setups
  if (is_dual_mono) {
    std::string zed_file = calib_data.saveCalibZED(serial, is_4k);
    if (!zed_file.empty()) {
      std::cout << " * ZED SDK calibration file saved: " << zed_file
                << std::endl;
    } else {
      std::cout << " !!! Failed to save ZED SDK calibration file " << zed_file
                << " !!!" << std::endl;
    }
  }

  return EXIT_SUCCESS;
}

std::string StereoCalib::saveCalibOpenCV(int serial) {
  std::string calib_filename =
      "zed_calibration_" + std::to_string(serial) + ".yml";

  cv::FileStorage fs(calib_filename, cv::FileStorage::WRITE);
  if (fs.isOpened()) {
    fs << "Size" << imageSize;
    fs << "K_LEFT" << left.K << "K_RIGHT" << right.K;

    if (left.disto_model_RadTan) {
      fs << "D_LEFT" << left.D << "D_RIGHT" << right.D;
    } else {
      fs << "D_LEFT_FE" << left.D << "D_RIGHT_FE" << right.D;
    }

    fs << "R" << Rv << "T" << T;
    fs.release();

    return calib_filename;
  }

  return std::string();
}

void printDisto(const CameraCalib& calib, std::ofstream& outfile) {
  if (calib.disto_model_RadTan) {
    size_t dist_size = calib.D.total();
    outfile << "k1 = " << calib.D.at<double>(0) << "\n";
    outfile << "k2 = " << calib.D.at<double>(1) << "\n";
    outfile << "p1 = " << calib.D.at<double>(2) << "\n";
    outfile << "p2 = " << calib.D.at<double>(3) << "\n";
    outfile << "k3 = " << calib.D.at<double>(4) << "\n";
    outfile << "k4 = " << (dist_size > 5 ? calib.D.at<double>(5) : 0.0) << "\n";
    outfile << "k5 = " << (dist_size > 6 ? calib.D.at<double>(6) : 0.0) << "\n";
    outfile << "k6 = " << (dist_size > 7 ? calib.D.at<double>(7) : 0.0) << "\n";
  } else {
    outfile << "k1 = " << calib.D.at<double>(0) << "\n";
    outfile << "k2 = " << calib.D.at<double>(1) << "\n";
    outfile << "k3 = " << calib.D.at<double>(2) << "\n";
    outfile << "k4 = " << calib.D.at<double>(3) << "\n";
  }
  outfile << "\n";
}

std::string StereoCalib::saveCalibZED(int serial, bool is_4k) {
  std::string calib_filename = "SN" + std::to_string(serial) + ".conf";

  // Write parameters to a text file
  std::ofstream outfile(calib_filename);
  if (!outfile.is_open()) {
    std::cerr
        << " !!! Cannot save the calibration file: 'Unable to open output file'"
        << std::endl;
    return std::string();
  }

  if (!is_4k) {  //  AR0234

    if (imageSize.height != 1200) {
      std::cout << "The resolution for the calibration is not valid\n\nUse "
                   "HD1200 (1920x1200) for ZED X One GS"
                << std::endl;
      return std::string();
    }

    outfile << "[LEFT_CAM_FHD1200]\n";
    outfile << "fx = " << left.K.at<double>(0, 0) << "\n";
    outfile << "fy = " << left.K.at<double>(1, 1) << "\n";
    outfile << "cx = " << left.K.at<double>(0, 2) << "\n";
    outfile << "cy = " << left.K.at<double>(1, 2) << "\n\n";

    outfile << "[RIGHT_CAM_FHD1200]\n";
    outfile << "fx = " << right.K.at<double>(0, 0) << "\n";
    outfile << "fy = " << right.K.at<double>(1, 1) << "\n";
    outfile << "cx = " << right.K.at<double>(0, 2) << "\n";
    outfile << "cy = " << right.K.at<double>(1, 2) << "\n\n";

    outfile << "[LEFT_CAM_FHD]\n";
    outfile << "fx = " << left.K.at<double>(0, 0) << "\n";
    outfile << "fy = " << left.K.at<double>(1, 1) << "\n";
    outfile << "cx = " << left.K.at<double>(0, 2) << "\n";
    outfile << "cy = " << left.K.at<double>(1, 2) - 60 << "\n\n";

    outfile << "[RIGHT_CAM_FHD]\n";
    outfile << "fx = " << right.K.at<double>(0, 0) << "\n";
    outfile << "fy = " << right.K.at<double>(1, 1) << "\n";
    outfile << "cx = " << right.K.at<double>(0, 2) << "\n";
    outfile << "cy = " << right.K.at<double>(1, 2) - 60 << "\n\n";

    outfile << "[LEFT_CAM_SVGA]\n";
    outfile << "fx = " << left.K.at<double>(0, 0) / 2 << "\n";
    outfile << "fy = " << left.K.at<double>(1, 1) / 2 << "\n";
    outfile << "cx = " << left.K.at<double>(0, 2) / 2 << "\n";
    outfile << "cy = " << left.K.at<double>(1, 2) / 2 << "\n\n";

    outfile << "[RIGHT_CAM_SVGA]\n";
    outfile << "fx = " << right.K.at<double>(0, 0) / 2 << "\n";
    outfile << "fy = " << right.K.at<double>(1, 1) / 2 << "\n";
    outfile << "cx = " << right.K.at<double>(0, 2) / 2 << "\n";
    outfile << "cy = " << right.K.at<double>(1, 2) / 2 << "\n\n";

    outfile << "[LEFT_DISTO]\n";
    printDisto(left, outfile);

    outfile << "[RIGHT_DISTO]\n";
    printDisto(right, outfile);

    outfile << "[STEREO]\n";
    outfile << "Baseline = " << -T.at<double>(0) << "\n";
    outfile << "TY = " << T.at<double>(1) << "\n";
    outfile << "TZ = " << T.at<double>(2) << "\n";
    outfile << "CV_FHD = " << Rv.at<double>(1) << "\n";
    outfile << "CV_SVGA = " << Rv.at<double>(1) << "\n";
    outfile << "CV_FHD1200 = " << Rv.at<double>(1) << "\n";
    outfile << "RX_FHD = " << Rv.at<double>(0) << "\n";
    outfile << "RX_SVGA = " << Rv.at<double>(0) << "\n";
    outfile << "RX_FHD1200 = " << Rv.at<double>(0) << "\n";
    outfile << "RZ_FHD = " << Rv.at<double>(2) << "\n";
    outfile << "RZ_SVGA = " << Rv.at<double>(2) << "\n";
    outfile << "RZ_FHD1200 = " << Rv.at<double>(2) << "\n\n";

    outfile.close();
    return calib_filename;
  } else {  //  IMX678

    if (imageSize.height != 2160) {
      std::cout << "The resolution for the calibration is not valid\n\nUse "
                   "4K (3840x2160) for ZED X One 4K"
                << std::endl;
      return std::string();
    }

    outfile << "[LEFT_CAM_4k]\n";
    outfile << "fx = " << left.K.at<double>(0, 0) << "\n";
    outfile << "fy = " << left.K.at<double>(1, 1) << "\n";
    outfile << "cx = " << left.K.at<double>(0, 2) << "\n";
    outfile << "cy = " << left.K.at<double>(1, 2) << "\n\n";

    outfile << "[RIGHT_CAM_4k]\n";
    outfile << "fx = " << right.K.at<double>(0, 0) << "\n";
    outfile << "fy = " << right.K.at<double>(1, 1) << "\n";
    outfile << "cx = " << right.K.at<double>(0, 2) << "\n";
    outfile << "cy = " << right.K.at<double>(1, 2) << "\n\n";

    outfile << "[LEFT_CAM_QHDPLUS]\n";
    outfile << "fx = " << left.K.at<double>(0, 0) << "\n";
    outfile << "fy = " << left.K.at<double>(1, 1) << "\n";
    outfile << "cx = " << left.K.at<double>(0, 2) - (3840 - 3200) / 2 << "\n";
    outfile << "cy = " << left.K.at<double>(1, 2) - (2160 - 1800) / 2 << "\n\n";

    outfile << "[RIGHT_CAM_QHDPLUS]\n";
    outfile << "fx = " << right.K.at<double>(0, 0) << "\n";
    outfile << "fy = " << right.K.at<double>(1, 1) << "\n";
    outfile << "cx = " << right.K.at<double>(0, 2) - (3840 - 3200) / 2 << "\n";
    outfile << "cy = " << right.K.at<double>(1, 2) - (2160 - 1800) / 2
            << "\n\n";

    outfile << "[LEFT_CAM_FHD]\n";
    outfile << "fx = " << left.K.at<double>(0, 0) / 2 << "\n";
    outfile << "fy = " << left.K.at<double>(1, 1) / 2 << "\n";
    outfile << "cx = " << left.K.at<double>(0, 2) / 2 << "\n";
    outfile << "cy = " << left.K.at<double>(1, 2) / 2 << "\n\n";

    outfile << "[RIGHT_CAM_FHD]\n";
    outfile << "fx = " << right.K.at<double>(0, 0) / 2 << "\n";
    outfile << "fy = " << right.K.at<double>(1, 1) / 2 << "\n";
    outfile << "cx = " << right.K.at<double>(0, 2) / 2 << "\n";
    outfile << "cy = " << right.K.at<double>(1, 2) / 2 << "\n\n";

    outfile << "[LEFT_CAM_FHD1200]\n";
    outfile << "fx = " << left.K.at<double>(0, 0) << "\n";
    outfile << "fy = " << left.K.at<double>(1, 1) << "\n";
    outfile << "cx = " << left.K.at<double>(0, 2) - (3840 - 1920) / 2 << "\n";
    outfile << "cy = " << left.K.at<double>(1, 2) - (2160 - 1200) / 2 << "\n\n";

    outfile << "[RIGHT_CAM_FHD1200]\n";
    outfile << "fx = " << right.K.at<double>(0, 0) << "\n";
    outfile << "fy = " << right.K.at<double>(1, 1) << "\n";
    outfile << "cx = " << right.K.at<double>(0, 2) - (3840 - 1920) / 2 << "\n";
    outfile << "cy = " << right.K.at<double>(1, 2) - (2160 - 1200) / 2
            << "\n\n";

    outfile << "[LEFT_DISTO]\n";
    printDisto(left, outfile);

    outfile << "[RIGHT_DISTO]\n";
    printDisto(right, outfile);

    outfile << "[STEREO]\n";
    outfile << "Baseline = " << -T.at<double>(0) << "\n";
    outfile << "TY = " << T.at<double>(1) << "\n";
    outfile << "TZ = " << T.at<double>(2) << "\n";
    outfile << "CV_FHD = " << Rv.at<double>(1) << "\n";
    outfile << "CV_FHD1200 = " << Rv.at<double>(1) << "\n";
    outfile << "CV_4k = " << Rv.at<double>(1) << "\n";
    outfile << "CV_QHDPLUS = " << Rv.at<double>(1) << "\n";
    outfile << "RX_FHD = " << Rv.at<double>(0) << "\n";
    outfile << "RX_FHD1200 = " << Rv.at<double>(0) << "\n";
    outfile << "RX_4k = " << Rv.at<double>(0) << "\n";
    outfile << "RX_QHDPLUS = " << Rv.at<double>(0) << "\n";
    outfile << "RZ_FHD = " << Rv.at<double>(2) << "\n";
    outfile << "RZ_FHD1200 = " << Rv.at<double>(2) << "\n";
    outfile << "RZ_4k = " << Rv.at<double>(2) << "\n\n";
    outfile << "RZ_QHDPLUS = " << Rv.at<double>(2) << "\n\n";

    outfile.close();
    return calib_filename;
  }
}