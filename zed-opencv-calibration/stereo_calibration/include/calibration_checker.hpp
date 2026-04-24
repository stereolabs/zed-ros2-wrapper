#ifndef CALIBRATION_CHECKER_HPP
#define CALIBRATION_CHECKER_HPP

#include <opencv2/opencv.hpp>

typedef struct _board {
  cv::Size board_size = {
      0, 0};  // Number of inner corners per a chessboard row and column
  float square_size =
      0.0f;  // Size of a square in your defined unit (point, millimeter,etc).
  std::vector<cv::Point3f> objp;            // 3D points in real world space
  cv::Size2f board_size_mm = {0.0f, 0.0f};  // Physical size of the board in mm
} Board;

typedef struct _detected_board_params {
  cv::Point2f avg_pos = {
      -1.0f, -1.0f};   // Normalized position of the checkerboard in the image
  float size = -1.0f;  // Normalized size of the checkerboard
  float skew = -1.0f;  // Normalized skew of the checkerboard
  float b_x = -1.0f;   // Normalized value of X closed to the border
  float b_y = -1.0f;   // Normalized value of Y closed to the border
} DetectedBoardParams;

// Constants
const size_t DEFAULT_MIN_SAMPLES = 20;
const size_t DEFAULT_MAX_SAMPLES = 50;
const float DEFAULT_MIN_TARGET_AREA = 0.1f;  // Ignore checkerboards smaller than 10% of the image area
const DetectedBoardParams DEFAULT_IDEAL_PARAMS = {
    cv::Point2f(
        0.65f,  // Checkerboard X position should cover 65% of the image width
        0.65f   // Checkerboard Y position should cover 65% of the image height
        ),
    0.4f,  // Checkerboard size variation should be at least 40%
    0.375f,  // Checkerboard skew variation should be at least 37.5%
    0.8f, // Checkerboard X position close to border should cover 80% of the image width
    0.8f  // Checkerboard Y position close to border should cover 80% of the image height
};         // Ideal parameters for a good sample database

class CalibrationChecker {
 public:
  CalibrationChecker(cv::Size board_size, float square_size,
                     size_t min_samples = DEFAULT_MIN_SAMPLES,
                     size_t max_samples = DEFAULT_MAX_SAMPLES,
                     float min_target_area = DEFAULT_MIN_TARGET_AREA,
                     DetectedBoardParams idealParams = DEFAULT_IDEAL_PARAMS,
                     bool verbose = false);
  ~CalibrationChecker() = default;

  // Test if the detected corners form a valid sample
  bool testSample(const std::vector<cv::Point2f>& corners, cv::Size image_size);

  // Retrieve valid corners
  const std::vector<std::vector<cv::Point2f>>& getValidCorners() const {
    return validCorners_;
  }

  // Retrieve valid sample count
  size_t getValidSampleCount() const { return validCorners_.size(); }

  // Calculate the sample collection status according to the stored samples
  bool evaluateSampleCollectionStatus(float& size_score, float& skew_score,
                                      float& pos_score_x,
                                      float& pos_score_y, 
                                      float& min_size, float& max_size,
                                      float& min_skew, float& max_skew, 
                                      float& min_b_x_coverage, float& max_b_x_coverage,
                                      float& min_b_y_coverage, float& max_b_y_coverage
                                    ) const;

  // Retrieve the last detected board parameters
  const DetectedBoardParams& getLastDetectedBoardParams() const;

 private:
  // Calculate the parameter of a detected checkerboard
  DetectedBoardParams getDetectedBoardParams(
      const std::vector<cv::Point2f>& corners, cv::Size image_size);

  // Check if the detected corners are valid
  bool isGoodSample(const DetectedBoardParams& params);

  // Helper functions
  std::vector<cv::Point2f> get_outside_corners(
      const std::vector<cv::Point2f>&
          corners);  // Get the 4 outside corners of a checkerboard
  float compute_skew(
      const std::vector<cv::Point2f>&
          outside_corners);  // Compute skew based on the 4 outside corners
  float compute_area(
      const std::vector<cv::Point2f>&
          outside_corners);  // Compute area based on the 4 outside corners

 private:
  Board board_;

  std::vector<DetectedBoardParams>
      paramDb_;  // Database of previously detected board parameters
  std::vector<std::vector<cv::Point2f>>
      validCorners_;  // All the corners associated to the single parameters in
                      // paramDb_

  DetectedBoardParams idealParams_ = DEFAULT_IDEAL_PARAMS;
  size_t min_samples_ = DEFAULT_MIN_SAMPLES;  // Minimum number of samples to
                                              // consider the database complete
  size_t max_samples_ = DEFAULT_MAX_SAMPLES;  // Maximum number of samples to
                                              // consider the database complete

  float min_target_area_ = DEFAULT_MIN_TARGET_AREA; // Ignore checkerboards smaller than this area (percentage of image area)

  bool verbose_ = false;
};

#endif  // CALIBRATION_CHECKER_HPP
