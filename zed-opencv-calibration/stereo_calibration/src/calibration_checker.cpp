#include "calibration_checker.hpp"

constexpr float PI = static_cast<float>(M_PI);

constexpr size_t up_left = 0;
constexpr size_t up_right = 1;
constexpr size_t down_right = 2;
constexpr size_t down_left = 3;

CalibrationChecker::CalibrationChecker(cv::Size board_size, float square_size,
                                       size_t min_samples, size_t max_samples,
                                       float min_target_area,
                                       DetectedBoardParams idealParams,
                                       bool verbose) {
  verbose_ = verbose;

  // Calibration parameters
  min_samples_ = min_samples;
  max_samples_ = max_samples;
  min_target_area_ = min_target_area;
  idealParams_ = idealParams;

  // Initialize the board parameters
  board_.board_size = board_size;
  board_.square_size = square_size;
  board_.objp.clear();
  board_.board_size_mm =
      cv::Size(board_size.width * square_size, board_size.height * square_size);

  // Prepare object points based on the known board size and square size
  for (int i = 0; i < board_size.height; i++) {
    for (int j = 0; j < board_size.width; j++) {
      board_.objp.push_back(cv::Point3f(j * square_size, i * square_size, 0));
    }
  }
}

bool CalibrationChecker::testSample(const std::vector<cv::Point2f>& corners,
                                    cv::Size image_size) {
  DetectedBoardParams params = getDetectedBoardParams(corners, image_size);

  if (params.size < 0 || params.skew < 0) {
    return false;  // Invalid parameters
  }

  if (verbose_) {
    std::cout << std::setprecision(3) << " * New Sample: Pos("
              << params.avg_pos.x << ", " << params.avg_pos.y
              << "), Size: " << params.size << ", Skew: " << params.skew
              << std::endl;
  }

  if (isGoodSample(params)) {
    // Store the valid parameters and associated corners
    paramDb_.push_back(params);
    validCorners_.push_back(corners);
    std::cout << " * Sample accepted. Total valid samples: "
              << validCorners_.size() << std::endl;
    return true;
  }

  std::cout << " * Sample rejected." << std::endl;
  return false;
}

float CalibrationChecker::compute_skew(
    const std::vector<cv::Point2f>& outside_corners) {
  /*  Get skew for given checkerboard detection.
    Scaled to [0,1], which 0 = no skew, 1 = high skew
    Skew is proportional to the divergence of three outside corners from 90
    degrees.
   */
  if (outside_corners.size() != 4) {
    return -1.0f;  // Invalid input
  }

  auto angle = [](const cv::Point2f& a, const cv::Point2f& b,
                  const cv::Point2f& c) -> float {
    cv::Point2f ab = a - b;
    cv::Point2f cb = c - b;
    float dot = ab.x * cb.x + ab.y * cb.y;
    float norm_ab = std::sqrt(ab.x * ab.x + ab.y * ab.y);
    float norm_cb = std::sqrt(cb.x * cb.x + cb.y * cb.y);
    float cos_angle = dot / (norm_ab * norm_cb);
    if (cos_angle < -1.0f)
      cos_angle = -1.0f;
    else if (cos_angle > 1.0f)
      cos_angle = 1.0f;
    return std::acos(cos_angle);
  };

  // Original code from here:
  // https://github.com/ros-perception/image_pipeline/blob/rolling/camera_calibration/src/camera_calibration/calibrator.py#L187-L207
  // float skew = std::min(
  //     1.0f, 2.0f * std::abs((PI / 2) - angle(outside_corners[up_left],
  //                                            outside_corners[up_right],
  //                                            outside_corners[down_right])));

  float maxDeviation = 0.0f;
  for (int i = 0; i < 4; i++) {
    float ang = angle(outside_corners[(i + 3) % 4], outside_corners[i],
                      outside_corners[(i + 1) % 4]);
    float deviation = std::abs((PI / 2) - ang);
    if (deviation > maxDeviation) {
      maxDeviation = deviation;
    }
  }

  return maxDeviation / (PI / 2);
}

float CalibrationChecker::compute_area(
    const std::vector<cv::Point2f>& outside_corners) {
  /* Get 2d image area of the detected checkerboard.
    The projected checkerboard is assumed to be a convex quadrilateral, and the
    area computed as |p X q|/2; see
    http://mathworld.wolfram.com/Quadrilateral.html.
   */

  if (outside_corners.size() != 4) {
    return -1.0f;  // Invalid input
  }

  // Using the shoelace formula to compute area of the quadrilateral
  cv::Point2f a = outside_corners[up_right] - outside_corners[up_left];
  cv::Point2f b = outside_corners[down_right] - outside_corners[up_right];
  cv::Point2f c = outside_corners[down_left] - outside_corners[down_right];

  cv::Point2f p = b + c;
  cv::Point2f q = a + b;

  float area = 0.5f * std::abs(p.x * q.y - p.y * q.x);

  return area;
}

std::vector<cv::Point2f> CalibrationChecker::get_outside_corners(
    const std::vector<cv::Point2f>& corners) {
  std::vector<cv::Point2f> outside_corners;

  if (corners.size() != board_.board_size.area()) {
    return outside_corners;
  }

  size_t x_dim = board_.board_size.width;
  size_t y_dim = board_.board_size.height;

  outside_corners.resize(4);

  outside_corners[up_left] = corners[0];           // Top-left
  outside_corners[up_right] = corners[x_dim - 1];  // Top-right
  outside_corners[down_right] =
      corners[(y_dim - 1) * x_dim + (x_dim - 1)];             // Bottom-right
  outside_corners[down_left] = corners[(y_dim - 1) * x_dim];  // Bottom-left

  return outside_corners;
}

DetectedBoardParams CalibrationChecker::getDetectedBoardParams(
    const std::vector<cv::Point2f>& corners, cv::Size image_size) {
  DetectedBoardParams params;

  auto outside_corners = get_outside_corners(corners);
  float area = compute_area(outside_corners);
  float skew = compute_skew(outside_corners);

  if (area < min_target_area_ || skew < 0) {
    // Return invalid params
    params.size = -1.0f;
    params.skew = -1.0f;
    return params;
  }

  // For X and Y, we "shrink" the image all around by approx.half the board
  // size. Otherwise large boards are penalized because you can't get much X/Y
  // variation.
  float avg_x = 0.0f;
  float avg_y = 0.0f;
  for (const auto& corner : corners) {
    avg_x += corner.x;
    avg_y += corner.y;
  }
  avg_x /= static_cast<float>(corners.size());
  avg_y /= static_cast<float>(corners.size());

  float p_x = std::min(1.0f, std::max(0.0f, avg_x / image_size.width));
  float p_y = std::min(1.0f, std::max(0.0f, avg_y / image_size.height));

  // Calculate the coordinates closer to the border
  float min_x = image_size.width, max_x = 0.0f, min_y = image_size.height,
        max_y = 0.0f;
  for (const auto& corner : outside_corners) {
    if (corner.x < min_x) {
      min_x = corner.x;
    }
    if (corner.x > max_x) {
      max_x = corner.x;
    }
    if (corner.y < min_y) {
      min_y = corner.y;
    }
    if (corner.y > max_y) {
      max_y = corner.y;
    }
  }

  float b_x, b_y;

  if (p_x < 0.5f) {
    b_x = min_x / image_size.width;
  } else {
    b_x = max_x / image_size.width;
  }
  if (p_y < 0.5f) {
    b_y = min_y / image_size.height;
  } else {
    b_y = max_y / image_size.height;
  }

  params.avg_pos = cv::Point2f(p_x, p_y);
  params.size = std::sqrt(area / (image_size.width * image_size.height));
  params.skew = skew;
  params.b_x = b_x;
  params.b_y = b_y;

  return params;
}

bool CalibrationChecker::isGoodSample(const DetectedBoardParams& params) {
  if (paramDb_.empty()) {
    return true;  // First sample is always good
  }

  // Original similarity check from:
  // https://github.com/ros-perception/image_pipeline/blob/rolling/camera_calibration/src/camera_calibration/calibrator.py#L485-L507
  auto param_distance = [](const DetectedBoardParams& p1,
                           const DetectedBoardParams& p2) -> float {
    return std::abs(p1.size - p2.size) + std::abs(p1.skew - p2.skew) +
           std::abs(p1.avg_pos.x - p2.avg_pos.x) +
           std::abs(p1.avg_pos.y - p2.avg_pos.y);
  };

  int idx = 0;
  for (auto& stored_params : paramDb_) {
    float dist = param_distance(params, stored_params);
    if (dist < 0.2f) {
      if (verbose_) {
        std::cout << "  Rejected: Too similar to sample #" << idx
                  << " (dist=" << dist << ")" << std::endl;
      }
      return false;
    }
    idx++;
  }

  return true;
}

bool CalibrationChecker::evaluateSampleCollectionStatus(
    float& size_score_, float& skew_score_, float& pos_score_x_,
    float& pos_score_y_, float& min_size_, float& max_size_, float& min_skew_,
    float& max_skew_, float& min_b_x_coverage_, float& max_b_x_coverage_,
    float& min_b_y_coverage_, float& max_b_y_coverage_) const {
  size_score_ = 0.0f;
  skew_score_ = 0.0f;
  pos_score_x_ = 0.0f;
  pos_score_y_ = 0.0f;
  if (paramDb_.empty()) {
    return false;
  }

  float min_bx = 1.0f, max_bx = 0.0f;
  float min_by = 1.0f, max_by = 0.0f;
  float min_size = 1.0f, max_size = 0.0f;
  float min_skew = 1.0f, max_skew = 0.0f;

  for (const auto& params : paramDb_) {
    if (params.b_x < min_bx) min_bx = params.b_x;
    if (params.b_x > max_bx) max_bx = params.b_x;
    if (params.b_y < min_by) min_by = params.b_y;
    if (params.b_y > max_by) max_by = params.b_y;
    if (params.size < min_size) min_size = params.size;
    if (params.size > max_size) max_size = params.size;
    if (params.skew < min_skew) min_skew = params.skew;
    if (params.skew > max_skew) max_skew = params.skew;
  }

  pos_score_x_ = std::min((max_bx - min_bx) / idealParams_.b_x, 1.0f);
  pos_score_y_ = std::min((max_by - min_by) / idealParams_.b_y, 1.0f);
  size_score_ = std::min((max_size - min_size) / idealParams_.size, 1.0f);
  skew_score_ = std::min((max_skew - min_skew) / idealParams_.skew, 1.0f);
  min_b_x_coverage_ = min_bx;
  max_b_x_coverage_ = max_bx;
  min_b_y_coverage_ = min_by;
  max_b_y_coverage_ = max_by;
  min_size_ = min_size;
  max_size_ = max_size;
  min_skew_ = min_skew;
  max_skew_ = max_skew;

  if (verbose_) {
    std::cout << "Sample Collection Status (normalized values):" << std::endl;
    std::cout << " - PosX status: [" << min_bx << " , " << max_bx << "] -> "
              << max_bx - min_bx << "/" << idealParams_.b_x << std::endl;
    std::cout << "  * PosX Score : " << std::setprecision(3) << pos_score_x_
              << std::endl;
    std::cout << " - PosY status: [" << min_by << " , " << max_by << "] -> "
              << max_by - min_by << "/" << idealParams_.b_y << std::endl;
    std::cout << "  * PosY Score : " << std::setprecision(3) << pos_score_y_
              << std::endl;
    std::cout << " - Size status: [" << min_size << " , " << max_size << "] -> "
              << max_size - min_size << "/" << idealParams_.size << std::endl;
    std::cout << "  * Size Score : " << std::setprecision(3) << size_score_
              << std::endl;
    std::cout << " - Skew status: [" << min_skew << " , " << max_skew << "] -> "
              << max_skew - min_skew << "/" << idealParams_.skew << std::endl;
    std::cout << "  * Skew Score : " << std::setprecision(3) << skew_score_
              << std::endl;
  }

  if (paramDb_.size() < min_samples_) {
    if (verbose_) {
      std::cout
          << "Sample collection incomplete: not reached the minimum sample "
             "count ("
          << paramDb_.size() << "/" << min_samples_ << ")" << std::endl;
    }
    return false;
  }

  if (paramDb_.size() >= max_samples_) {
    std::cout
        << "Sample collection complete: Reached the maximum sample count ("
        << paramDb_.size() << "/" << max_samples_ << ")" << std::endl;
    return true;
  }

  if (size_score_ >= 1.0f && skew_score_ >= 1.0f && pos_score_x_ >= 1.0f &&
      pos_score_y_ >= 1.0f) {
    std::cout << "Sample collection complete: All scores are above threshold"
              << std::endl;
    return true;
  }

  if (verbose_) {
    std::cout << "Sample collection incomplete." << std::endl;
  }
  return false;
}

const DetectedBoardParams& CalibrationChecker::getLastDetectedBoardParams()
    const {
  if (paramDb_.empty()) {
    static DetectedBoardParams empty_params = {cv::Point2f(-1.0f, -1.0f), -1.0f,
                                               -1.0f};
    return empty_params;
  }
  return paramDb_.back();
}