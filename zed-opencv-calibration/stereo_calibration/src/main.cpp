// © 2026, Cargo Robotics
// Adopted from https://github.com/stereolabs/zed-opencv-calibration (upstream Stereolabs sources; Cargo maintains local patches — see README).
//
// Fisheye stereo extrinsics calibration for a ZED XOne GS pair.
//
// This tool:
//   * loads serial numbers and capture resolution from a YAML config,
//   * pulls the FACTORY fisheye intrinsics from each ZED XOne GS via the SDK
//     and treats them as fixed,
//   * either runs the interactive image-pair acquisition loop (with quality
//     checks, coverage indicators, sample tracker, etc.) or skips straight to
//     extrinsics if a directory of pre-collected pairs is provided,
//   * solves only the stereo extrinsics (R, T) with cv::CALIB_FIX_INTRINSIC.

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "calibration_checker.hpp"
#include "opencv_calibration.hpp"

namespace fs = std::filesystem;

// Globals shared with the rendering / coverage helpers below.
// They are intentionally NOT pre-populated: values are required from YAML config.
int h_edges;
int v_edges;
float square_size;

// Folder used when capturing live (see capture_images_dir in fisheye_stereo.yaml; default matches zed-end-effector bind-mount).
std::string image_folder = "/var/cargo/zed-calibration/images/";

// Coverage indicator fill helpers (defined at the bottom of this file).
void addNewCheckerboardPosition(cv::Mat& coverage_indicator, cv::Mat& pos_indicator, cv::Mat& limits_indicator, float norm_x, float norm_y,
                                float norm_size, float min_x, float max_x, float min_y, float max_y, bool draw_rect);
void addNewCheckerboardPoly(cv::Mat& coverage_indicator, const std::vector<cv::Point2f>& pts_l);
void applyCoverageIndicatorOverlay(cv::Mat& image, const cv::Mat& coverage_indicator, const cv::Mat& limits_indicator);
void applyPosIndicatorOverlay(cv::Mat& image, const cv::Mat& pos_indicator);

// Rendering layout
constexpr int text_area_height = 430;
const cv::Size display_size(720, 404);

// Calibration / sample collection thresholds
const float max_repr_error = 1.0f;
// Set from YAML `sample_collection` (defaults applied before loadStereoConfig).
int min_samples = DEFAULT_MIN_STEREO_SAMPLES;
int max_samples = 35;
const float min_avg_x_coverage = 0.65f;
const float min_avg_y_coverage = 0.65f;
const float min_area_range = 0.4f;
const float min_skew_range = 0.375f;
const float min_b_x_coverage = 0.8f;
const float min_b_y_coverage = 0.8f;
const float min_target_area = 0.1f;

bool verbose = false;
int sdk_verbose = 0;

const bool image_stack_horizontal = true;

// Quadrant counters used by addNewCheckerboardPosition (reset when rebuilding coverage after dropping samples).
static int top_left_count = 0;
static int top_right_count = 0;
static int bottom_left_count = 0;
static int bottom_right_count = 0;

static void resetCoverageQuadrantCounters() {
    top_left_count = top_right_count = bottom_left_count = bottom_right_count = 0;
}

std::vector<cv::Point3f> makeChessboardObjectPointsMm(int h_edges_i, int v_edges_i, float square_mm) {
    std::vector<cv::Point3f> obj;
    for (int i = 0; i < v_edges_i; ++i) {
        for (int j = 0; j < h_edges_i; ++j) {
            obj.emplace_back(static_cast<float>(j) * square_mm, static_cast<float>(i) * square_mm, 0.f);
        }
    }
    return obj;
}

static void rotationMatrixToEulerZYXDeg(const cv::Mat& R, double& rx_deg, double& ry_deg, double& rz_deg) {
    CV_Assert(R.rows == 3 && R.cols == 3);
    cv::Mat R64;
    R.convertTo(R64, CV_64F);
    const double r00 = R64.at<double>(0, 0), r10 = R64.at<double>(1, 0), r20 = R64.at<double>(2, 0);
    const double r21 = R64.at<double>(2, 1), r22 = R64.at<double>(2, 2);
    double sy = std::sqrt(r00 * r00 + r10 * r10);
    if (sy < 1e-9) {
        rx_deg = std::atan2(-R64.at<double>(1, 2), R64.at<double>(1, 1)) * 180.0 / CV_PI;
        ry_deg = std::atan2(-r20, sy) * 180.0 / CV_PI;
        rz_deg = 0.0;
    } else {
        rx_deg = std::atan2(r21, r22) * 180.0 / CV_PI;
        ry_deg = std::atan2(-r20, sy) * 180.0 / CV_PI;
        rz_deg = std::atan2(r10, r00) * 180.0 / CV_PI;
    }
}

static void compactStereoPairFiles(const std::string& folder, int remove_index, int file_count) {
    std::error_code ec;
    const std::string left_rm = folder + "image_left_" + std::to_string(remove_index) + ".png";
    const std::string right_rm = folder + "image_right_" + std::to_string(remove_index) + ".png";
    fs::remove(left_rm, ec);
    fs::remove(right_rm, ec);
    for (int j = remove_index + 1; j < file_count; ++j) {
        const std::string from_l = folder + "image_left_" + std::to_string(j) + ".png";
        const std::string to_l = folder + "image_left_" + std::to_string(j - 1) + ".png";
        const std::string from_r = folder + "image_right_" + std::to_string(j) + ".png";
        const std::string to_r = folder + "image_right_" + std::to_string(j - 1) + ".png";
        fs::rename(from_l, to_l, ec);
        fs::rename(from_r, to_r, ec);
    }
}

void rebuildCoverageVisuals(const CalibrationChecker& checker, const std::vector<std::vector<cv::Point2f>>& scaled_pts_display,
                            cv::Mat& coverage_indicator, cv::Mat& pos_indicator, cv::Mat& limits_indicator) {
    resetCoverageQuadrantCounters();
    coverage_indicator.setTo(0);
    pos_indicator.setTo(0);
    limits_indicator.setTo(0);
    const size_t n = checker.getValidSampleCount();
    for (size_t i = 0; i < n; ++i) {
        float min_bx = 1.f, max_bx = 0.f, min_by = 1.f, max_by = 0.f;
        for (size_t j = 0; j <= i; ++j) {
            const DetectedBoardParams& pj = checker.getDetectedBoardParamsAt(j);
            if (pj.size < 0.f) {
                continue;
            }
            min_bx = std::min(min_bx, pj.b_x);
            max_bx = std::max(max_bx, pj.b_x);
            min_by = std::min(min_by, pj.b_y);
            max_by = std::max(max_by, pj.b_y);
        }
        const DetectedBoardParams& p = checker.getDetectedBoardParamsAt(i);
        addNewCheckerboardPosition(coverage_indicator, pos_indicator, limits_indicator, p.avg_pos.x, p.avg_pos.y, p.size, min_bx, max_bx,
                                   min_by, max_by, i >= 1);
        if (i < scaled_pts_display.size()) {
            addNewCheckerboardPoly(coverage_indicator, scaled_pts_display[i]);
        }
    }
}

static void discardLastAcceptedSample(CalibrationChecker& checker, std::vector<std::vector<cv::Point2f>>& cached_pts_l,
                                      std::vector<std::vector<cv::Point2f>>& cached_pts_r,
                                      std::vector<std::vector<cv::Point2f>>& cached_scaled_display_pts, int& image_count,
                                      const std::string& folder, cv::Mat& coverage_indicator, cv::Mat& pos_indicator,
                                      cv::Mat& limits_indicator) {
    if (checker.getValidSampleCount() == 0 || image_count <= 0) {
        return;
    }
    const size_t last_i = checker.getValidSampleCount() - 1;
    checker.removeSampleAt(last_i);
    cached_pts_l.pop_back();
    cached_pts_r.pop_back();
    cached_scaled_display_pts.pop_back();
    const int file_idx = image_count - 1;
    std::error_code ec;
    fs::remove(folder + "image_left_" + std::to_string(file_idx) + ".png", ec);
    fs::remove(folder + "image_right_" + std::to_string(file_idx) + ".png", ec);
    image_count--;
    rebuildCoverageVisuals(checker, cached_scaled_display_pts, coverage_indicator, pos_indicator, limits_indicator);
}

static cv::Scalar stereoRmsColor(double rms) {
    if (!(rms >= 0.0)) {
        return cv::Scalar(180, 180, 180);
    }
    if (rms > 1.0) {
        return cv::Scalar(0, 0, 255);
    }
    if (rms > 0.8) {
        return cv::Scalar(0, 255, 255);
    }
    return cv::Scalar(0, 255, 0);
}

// Scale keypoints from one image size to another.
void scaleKP(std::vector<cv::Point2f>& pts, cv::Size in, cv::Size out) {
    float rx = out.width / static_cast<float>(in.width);
    float ry = out.height / static_cast<float>(in.height);
    for (auto& it : pts) {
        it.x *= rx;
        it.y *= ry;
    }
}

// ----------------------------- Config -----------------------------

struct StereoConfig {
    int left_sn = -1;
    int right_sn = -1;
    std::string resolution;
    int h_edges = 0;
    int v_edges = 0;
    float square_size_mm = 0.0f;
    std::string images_dir;
    /// Live-capture output directory (only when images_dir is empty). Overrides default /var/cargo/zed-calibration/images/.
    std::string capture_images_dir;
    /// Where to write SN<virtual>.conf and zed_calibration_<virtual>.yml (see calibration_output_dir in YAML).
    std::string calibration_output_dir;
    bool verbose = true;
    int min_samples = DEFAULT_MIN_STEREO_SAMPLES;
    int max_samples = 35;
};

bool loadStereoConfig(const std::string& path, StereoConfig& cfg) {
    YAML::Node node;
    try {
        node = YAML::LoadFile(path);
    } catch (const std::exception& e) {
        std::cerr << "Error: failed to load config file '" << path << "': " << e.what() << std::endl;
        return false;
    }

    if (!node["left_sn"] || !node["right_sn"]) {
        std::cerr << "Error: config must define 'left_sn' and 'right_sn'." << std::endl;
        return false;
    }
    cfg.left_sn = node["left_sn"].as<int>();
    cfg.right_sn = node["right_sn"].as<int>();

    if (!node["resolution"] || !node["resolution"].IsScalar()) {
        std::cerr << "Error: config must define scalar 'resolution' as a ZED SDK mode "
                  << "(e.g. HD1200, HD1080, HD720, SVGA)." << std::endl;
        return false;
    }
    cfg.resolution = node["resolution"].as<std::string>();

    if (!node["checkerboard"]) {
        std::cerr << "Error: config must define 'checkerboard' with h_edges, v_edges, square_size_mm." << std::endl;
        return false;
    }
    if (!node["checkerboard"]["h_edges"] || !node["checkerboard"]["v_edges"] || !node["checkerboard"]["square_size_mm"]) {
        std::cerr << "Error: checkerboard config requires 'h_edges', 'v_edges', and 'square_size_mm'." << std::endl;
        return false;
    }
    cfg.h_edges = node["checkerboard"]["h_edges"].as<int>();
    cfg.v_edges = node["checkerboard"]["v_edges"].as<int>();
    cfg.square_size_mm = node["checkerboard"]["square_size_mm"].as<float>();
    if (node["images_dir"]) {
        cfg.images_dir = node["images_dir"].as<std::string>("");
    }
    if (node["capture_images_dir"]) {
        cfg.capture_images_dir = node["capture_images_dir"].as<std::string>("");
    }
    if (node["calibration_output_dir"]) {
        cfg.calibration_output_dir = node["calibration_output_dir"].as<std::string>("");
    }
    if (node["verbose"]) {
        cfg.verbose = node["verbose"].as<bool>();
    }
    if (node["sample_collection"]) {
        const YAML::Node sc = node["sample_collection"];
        if (sc["min"]) {
            cfg.min_samples = sc["min"].as<int>();
        }
        if (sc["max"]) {
            cfg.max_samples = sc["max"].as<int>();
        }
    }
    return true;
}

// Map config string to ZED SDK resolution enum.
bool parseResolution(const std::string& resolution, sl::RESOLUTION& out_resolution) {
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

// Open a single ZED XOne via sl::CameraOne, read factory intrinsics, then close.
bool fetchFactoryIntrinsics(int serial, sl::RESOLUTION resolution, sl::CameraParameters& out_intr, sl::MODEL& out_model) {
    sl::InitParametersOne init;
    init.camera_resolution = resolution;
    init.camera_fps = 15;
    init.input.setFromSerialNumber(serial);

    sl::CameraOne cam;
    auto status = cam.open(init);
    // INVALID_CALIBRATION_FILE is acceptable: factory intrinsics are still populated.
    if (status != sl::ERROR_CODE::SUCCESS && status != sl::ERROR_CODE::INVALID_CALIBRATION_FILE) {
        std::cerr << "  ! SN " << serial << " failed to open: " << sl::toString(status) << std::endl;
        return false;
    }
    auto info = cam.getCameraInformation();
    out_intr = info.camera_configuration.calibration_parameters_raw;
    out_model = info.camera_model;
    cam.close();
    return true;
}

bool validateIntrinsics(const sl::CameraParameters& intr, const std::string& side, bool verbose_log) {
    const bool finite_values =
        std::isfinite(intr.fx) && std::isfinite(intr.fy) &&
        std::isfinite(intr.cx) && std::isfinite(intr.cy);
    if (!finite_values) {
        std::cerr << "Error: " << side << " intrinsics contain non-finite values."
                  << " fx=" << intr.fx << " fy=" << intr.fy
                  << " cx=" << intr.cx << " cy=" << intr.cy << std::endl;
        return false;
    }
    if (intr.fx <= 0.0f || intr.fy <= 0.0f) {
        std::cerr << "Error: " << side << " intrinsics are invalid."
                  << " fx=" << intr.fx << " fy=" << intr.fy << std::endl;
        return false;
    }
    if (verbose_log) {
        std::cout << " * " << side << " intrinsics check OK"
                  << " (fx=" << intr.fx << ", fy=" << intr.fy
                  << ", cx=" << intr.cx << ", cy=" << intr.cy << ")" << std::endl;
    }
    return true;
}

// ----------------------------- Args -----------------------------

struct Args {
    std::string app_name;
    std::string config_path;
    std::string images_dir_override;
    bool verbose = false;

    void parse(int argc, char* argv[]) {
        app_name = argv[0];
        for (int i = 1; i < argc; i++) {
            std::string arg = argv[i];
            if (arg == "--config" && i + 1 < argc) {
                config_path = argv[++i];
            } else if (arg == "--images_dir" && i + 1 < argc) {
                images_dir_override = argv[++i];
            } else if (arg == "--verbose") {
                verbose = true;
            } else if (arg == "--help" || arg == "-h") {
                std::cout << "Usage: " << argv[0] << " --config <yaml> [--images_dir <dir>] [--verbose]" << std::endl
                          << std::endl
                          << "  --config <yaml>      Path to the fisheye stereo calibration config (required)." << std::endl
                          << "                       Defines left_sn, right_sn, resolution (e.g. HD1200)," << std::endl
                          << "                       checkerboard, optional sample_collection min/max, images_dir, verbose." << std::endl
                          << "  --images_dir <dir>   Override images_dir from the config. When set (here or in" << std::endl
                          << "                       the config), the live capture step is skipped and stereo" << std::endl
                          << "                       extrinsics are solved directly from the existing pairs." << std::endl
                          << "  --verbose            Force-enable verbose output (YAML can also set verbose)." << std::endl
                          << std::endl
                          << "Notes:" << std::endl
                          << "  * This tool only supports ZED XOne GS pairs (fisheye)." << std::endl
                          << "  * Factory fisheye intrinsics are read from the ZED SDK and held FIXED;" << std::endl
                          << "    only stereo extrinsics (R, T) are solved." << std::endl;
                exit(0);
            }
        }
    }
};

// ----------------------------- Live capture -----------------------------

// Runs the interactive image-pair acquisition loop on the open virtual stereo
// camera, populating image_folder with image_left_N.png / image_right_N.png.
// Returns the number of saved pairs (>= 0) or a negative value if the user aborted.
int runLiveCapture(sl::Camera& zed_camera, const sl::Resolution& camera_resolution, StereoCalib& calib) {
    const DetectedBoardParams idealParams = {cv::Point2f(min_avg_x_coverage, min_avg_y_coverage), min_area_range, min_skew_range, min_b_x_coverage,
                                             min_b_y_coverage};
    CalibrationChecker checker(cv::Size(h_edges, v_edges), square_size, min_samples, max_samples, min_target_area, idealParams, verbose);

    const std::vector<cv::Point3f> obj_template_mm = makeChessboardObjectPointsMm(h_edges, v_edges, square_size);
    std::vector<std::vector<cv::Point2f>> cached_pts_l;
    std::vector<std::vector<cv::Point2f>> cached_pts_r;
    std::vector<std::vector<cv::Point2f>> cached_scaled_display_pts;
    double live_rms = -1.0;
    double live_baseline_mm = -1.0;
    double live_rx_deg = 0.0, live_ry_deg = 0.0, live_rz_deg = 0.0;
    bool live_calib_ok = false;

    sl::Mat zed_imageL(camera_resolution, sl::MAT_TYPE::U8_C4, sl::MEM::CPU);
    auto rgb_l = cv::Mat(camera_resolution.height, camera_resolution.width, CV_8UC4, zed_imageL.getPtr<sl::uchar1>());
    sl::Mat zed_imageR(camera_resolution, sl::MAT_TYPE::U8_C4, sl::MEM::CPU);
    auto rgb_r = cv::Mat(camera_resolution.height, camera_resolution.width, CV_8UC4, zed_imageR.getPtr<sl::uchar1>());

    cv::Mat coverage_indicator = cv::Mat::zeros(display_size.height, display_size.width, CV_8UC1);
    cv::Mat pos_indicator = cv::Mat::zeros(display_size.height, display_size.width, CV_8UC1);
    cv::Mat limits_indicator = cv::Mat::zeros(display_size.height, display_size.width, CV_8UC1);
    cv::Mat rgb_d, rgb2_d, rgb_d_fill, display, rendering_image;

    int image_count = -1;
    bool acquisition_completed = false;

    char key = ' ';
    bool missing_target_on_last_pics = false;
    bool low_target_variability_on_last_pics = false;
    bool early_finish_not_ready = false;

    const std::string window_name = "ZED XOne GS Fisheye Stereo Calibration";
    cv::namedWindow(window_name, cv::WINDOW_KEEPRATIO);
    cv::resizeWindow(window_name, display_size.width * 2, display_size.height + text_area_height);

    // Coverage scores
    float size_score = 0.0f, skew_score = 0.0f, pos_score_x = 0.0f, pos_score_y = 0.0f;
    float min_bx = 0.0f, max_bx = 0.0f, min_by = 0.0f, max_by = 0.0f;
    float min_size = 0.0f, max_size = 0.0f, min_skew = 0.0f, max_skew = 0.0f;

    while (true) {
        if (key == 'q' || key == 'Q' || key == 27) {
            std::cout << "Calibration aborted by user." << std::endl;
            return -1;
        }

        const cv::Scalar info_color(50, 210, 50);
        const cv::Scalar warn_color(0, 50, 250);

        if (zed_camera.grab() == sl::ERROR_CODE::SUCCESS) {
            zed_camera.retrieveImage(zed_imageL, sl::VIEW::LEFT_UNRECTIFIED);
            zed_camera.retrieveImage(zed_imageR, sl::VIEW::RIGHT_UNRECTIFIED);

            cv::resize(rgb_l, rgb_d, display_size);
            cv::resize(rgb_r, rgb2_d, display_size);
            cv::resize(rgb_l, rgb_d_fill, display_size);

            applyCoverageIndicatorOverlay(rgb_d_fill, coverage_indicator, limits_indicator);
            applyPosIndicatorOverlay(rgb_d_fill, pos_indicator);

            std::vector<cv::Point2f> pts_l, pts_r;
            bool found_l = cv::findChessboardCorners(rgb_d, cv::Size(h_edges, v_edges), pts_l);
            cv::drawChessboardCorners(rgb_d_fill, cv::Size(h_edges, v_edges), cv::Mat(pts_l), found_l);
            bool found_r = false;
            if (found_l) {
                found_r = cv::findChessboardCorners(rgb2_d, cv::Size(h_edges, v_edges), pts_r);
                cv::drawChessboardCorners(rgb2_d, cv::Size(h_edges, v_edges), cv::Mat(pts_r), found_r);
            }

            if (image_stack_horizontal) {
                cv::hconcat(rgb_d_fill, rgb2_d, display);
            } else {
                cv::vconcat(rgb_d_fill, rgb2_d, display);
            }

            cv::Mat text_info = cv::Mat::ones(cv::Size(display.size[1], text_area_height), display.type());
            cv::vconcat(display, text_info, rendering_image);

            if (acquisition_completed) {
                cv::putText(rendering_image, "Acquisition completed! Wait for the calibration computation to complete...",
                            cv::Point(20, display.size[0] + 50), cv::FONT_HERSHEY_SIMPLEX, 0.7, info_color, 2);
                cv::putText(rendering_image, "Follow the console log for calibration progress details.", cv::Point(20, display.size[0] + 80),
                            cv::FONT_HERSHEY_SIMPLEX, 0.7, info_color, 2);
            } else {
                if (missing_target_on_last_pics || low_target_variability_on_last_pics) {
                    cv::putText(rendering_image, "Frames not saved for calibration.", cv::Point(display.size[1] / 2 - 20, display.size[0] + 285),
                                cv::FONT_HERSHEY_SIMPLEX, 0.75, warn_color, 2);
                }
                if (missing_target_on_last_pics) {
                    cv::putText(rendering_image, " * Missing target on one of the cameras.",
                                cv::Point(display.size[1] / 2 - 20, display.size[0] + 315), cv::FONT_HERSHEY_SIMPLEX, 0.75, warn_color, 2);
                }
                if (low_target_variability_on_last_pics) {
                    cv::putText(rendering_image, " * Target too similar to a previous acquisition or too small.",
                                cv::Point(display.size[1] / 2 - 20, display.size[0] + 345), cv::FONT_HERSHEY_SIMPLEX, 0.75, warn_color, 2);
                }

                cv::putText(rendering_image, "Press 's' or the spacebar to save the current frames when the target is visible in both images.",
                            cv::Point(10, display.size[0] + 25), cv::FONT_HERSHEY_SIMPLEX, 0.7, info_color, 1);
                {
                    std::ostringstream oss_fhelp;
                    oss_fhelp << "Press 'f' to finish capture early (min. " << min_samples << " good samples required).";
                    cv::putText(rendering_image, oss_fhelp.str(), cv::Point(10, display.size[0] + 65), cv::FONT_HERSHEY_SIMPLEX, 0.5, info_color, 1);
                }
                cv::putText(rendering_image,
                            "Move the target horizontally, vertically, forward and backward, and rotate it to "
                            "improve coverage and variability scores. Framerate can be low if no target is detected.",
                            cv::Point(10, display.size[0] + 85), cv::FONT_HERSHEY_SIMPLEX, 0.5, warn_color, 1);

                if (early_finish_not_ready) {
                    std::ostringstream ss_early_finish;
                    ss_early_finish << "Early finish needs at least " << min_samples << " good samples. Currently: "
                                    << checker.getValidSampleCount();
                    cv::putText(rendering_image, ss_early_finish.str(), cv::Point(10, display.size[0] + 110), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                                warn_color, 2);
                }

                std::stringstream ss_status;
                int v_pos = display.size[0] + 145;
                int v_space = 33;
                int h_pos = 10;
                int h_space = 180;
                double font_scale = 0.7;

                auto draw_text_row = [rendering_image, h_pos, h_space, font_scale, info_color, warn_color](
                                         const std::string& label, int v_pos, int min_val, int max_val, int req_i, float req_f, float score) {
                    cv::putText(rendering_image, label, cv::Point(h_pos, v_pos), cv::FONT_HERSHEY_SIMPLEX, font_scale,
                                (score >= 1.0f ? info_color : warn_color), 1);
                    cv::putText(rendering_image, std::to_string(min_val), cv::Point(h_pos + h_space, v_pos), cv::FONT_HERSHEY_SIMPLEX, font_scale,
                                (score >= 1.0f ? info_color : warn_color), 1);
                    cv::putText(rendering_image, std::to_string(max_val), cv::Point(h_pos + 2 * h_space, v_pos), cv::FONT_HERSHEY_SIMPLEX, font_scale,
                                (score >= 1.0f ? info_color : warn_color), 1);
                    cv::putText(rendering_image, std::to_string(max_val - min_val), cv::Point(h_pos + 3 * h_space, v_pos), cv::FONT_HERSHEY_SIMPLEX,
                                font_scale, (score >= 1.0f ? info_color : warn_color), 1);
                    cv::putText(rendering_image, std::to_string(req_i), cv::Point(h_pos + 4 * h_space, v_pos), cv::FONT_HERSHEY_SIMPLEX, font_scale,
                                (score >= 1.0f ? info_color : warn_color), 1);
                    std::stringstream ss;
                    ss << std::fixed << std::setprecision(2) << score * 100.0f << "%";
                    cv::putText(rendering_image, ss.str(), cv::Point(h_pos + 5 * h_space, v_pos), cv::FONT_HERSHEY_SIMPLEX, font_scale,
                                (score >= 1.0f ? info_color : warn_color), 1);
                };

                ss_status << "Sample Collection Status";
                cv::putText(rendering_image, ss_status.str(), cv::Point(10, v_pos), cv::FONT_HERSHEY_SIMPLEX, font_scale, info_color, 2);

                v_pos += v_space;
                cv::putText(rendering_image, "METRIC", cv::Point(h_pos, v_pos), cv::FONT_HERSHEY_SIMPLEX, font_scale, info_color, 2);
                cv::putText(rendering_image, "MIN_VAL", cv::Point(h_pos + h_space, v_pos), cv::FONT_HERSHEY_SIMPLEX, font_scale, info_color, 2);
                cv::putText(rendering_image, "MAX_VAL", cv::Point(h_pos + 2 * h_space, v_pos), cv::FONT_HERSHEY_SIMPLEX, font_scale, info_color, 2);
                cv::putText(rendering_image, "COVERAGE", cv::Point(h_pos + 3 * h_space, v_pos), cv::FONT_HERSHEY_SIMPLEX, font_scale, info_color, 2);
                cv::putText(rendering_image, "REQUIRED", cv::Point(h_pos + 4 * h_space, v_pos), cv::FONT_HERSHEY_SIMPLEX, font_scale, info_color, 2);
                cv::putText(rendering_image, "SCORE", cv::Point(h_pos + 5 * h_space, v_pos), cv::FONT_HERSHEY_SIMPLEX, font_scale, info_color, 2);

                v_pos += v_space;
                draw_text_row("X [px]", v_pos, static_cast<int>(min_bx * camera_resolution.width),
                              static_cast<int>(max_bx * camera_resolution.width), static_cast<int>(min_b_x_coverage * camera_resolution.width),
                              min_b_x_coverage, pos_score_x);
                v_pos += v_space;
                draw_text_row("Y [px]", v_pos, static_cast<int>(min_by * camera_resolution.height),
                              static_cast<int>(max_by * camera_resolution.height), static_cast<int>(min_b_y_coverage * camera_resolution.height),
                              min_b_y_coverage, pos_score_y);
                v_pos += v_space;
                draw_text_row("Size [sq. px]", v_pos, static_cast<int>(min_size * camera_resolution.height * camera_resolution.width),
                              static_cast<int>(max_size * camera_resolution.height * camera_resolution.width),
                              static_cast<int>(min_area_range * camera_resolution.height * camera_resolution.width), min_area_range, size_score);
                v_pos += v_space;
                draw_text_row("Skew [deg]", v_pos, static_cast<int>(min_skew * 90.0f), static_cast<int>(max_skew * 90.0f),
                              static_cast<int>(min_skew_range * 90.0f), min_skew_range * 90.0f, skew_score);

                std::stringstream ss_img_count;
                v_pos += v_space;
                ss_img_count << "* Sample saved: " << std::max(image_count, 0) << " [min. " << min_samples << ", max. " << max_samples << "]";
                cv::putText(rendering_image, ss_img_count.str(), cv::Point(10, v_pos), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                            (image_count > min_samples ? info_color : warn_color), 1);

                v_pos += v_space;
                const cv::Scalar base_euler_color(200, 220, 200);
                std::ostringstream oss_base;
                oss_base << std::fixed << std::setprecision(2);
                if (live_calib_ok) {
                    oss_base << "Extrinsic preview: baseline " << live_baseline_mm << " mm | Rx,Ry,Rz (deg) " << live_rx_deg << ", " << live_ry_deg << ", "
                             << live_rz_deg;
                } else if (static_cast<int>(checker.getValidSampleCount()) >= min_samples) {
                    oss_base << "Extrinsic preview: solving... (need valid stereo fit)";
                } else {
                    oss_base << "Extrinsic preview: available after " << min_samples << " good samples (have "
                             << checker.getValidSampleCount() << ")";
                }
                cv::putText(rendering_image, oss_base.str(), cv::Point(10, v_pos), cv::FONT_HERSHEY_SIMPLEX, 0.55, base_euler_color, 1);

                v_pos += static_cast<int>(v_space * 0.85);
                std::ostringstream oss_rms;
                oss_rms << std::fixed << std::setprecision(3);
                if (live_calib_ok && live_rms >= 0.0) {
                    oss_rms << "Stereo reprojection RMS: " << live_rms << " px";
                } else {
                    oss_rms << "Stereo reprojection RMS: —";
                }
                cv::putText(rendering_image, oss_rms.str(), cv::Point(10, v_pos), cv::FONT_HERSHEY_SIMPLEX, 0.65,
                            live_calib_ok ? stereoRmsColor(live_rms) : cv::Scalar(160, 160, 160), 2);
            }

            cv::imshow(window_name, rendering_image);
            key = cv::waitKey(10);
            if (key == 'f' || key == 'F') {
                if (static_cast<int>(checker.getValidSampleCount()) >= min_samples) {
                    std::cout << ">>> Early finish requested by user. Starting calibration with "
                              << checker.getValidSampleCount() << " good samples." << std::endl;
                    acquisition_completed = true;
                    early_finish_not_ready = false;
                } else {
                    std::cout << "  ! Early finish requested, but at least " << min_samples << " good samples are required. "
                              << "Current: " << checker.getValidSampleCount() << std::endl;
                    early_finish_not_ready = true;
                }
            } else {
                early_finish_not_ready = false;
            }

            if (acquisition_completed) {
                std::cout << " *** Starting the calibration process ***" << std::endl;
                break;
            }

            if ((key == 's' || key == 'S') || key == ' ') {
                std::cout << "*** New acquisition triggered ***" << std::endl;
                missing_target_on_last_pics = !found_r || !found_l;

                if (found_l && found_r) {
                    auto scaled_pts_l = pts_l;
                    scaleKP(pts_l, display_size, cv::Size(camera_resolution.width, camera_resolution.height));
                    scaleKP(pts_r, display_size, cv::Size(camera_resolution.width, camera_resolution.height));

                    if (checker.testSample(pts_l, cv::Size(camera_resolution.width, camera_resolution.height))) {
                        low_target_variability_on_last_pics = false;

                        if (image_count < 0) image_count = 0;
                        cv::imwrite(image_folder + "image_left_" + std::to_string(image_count) + ".png", rgb_l);
                        cv::imwrite(image_folder + "image_right_" + std::to_string(image_count) + ".png", rgb_r);
                        std::cout << " * Images saved: '" << image_folder << "image_left_" << image_count << ".png' and '"
                                  << image_folder << "image_right_" << image_count << ".png'" << std::endl;

                        cached_pts_l.push_back(pts_l);
                        cached_pts_r.push_back(pts_r);
                        cached_scaled_display_pts.push_back(scaled_pts_l);

                        image_count++;

                        if (checker.evaluateSampleCollectionStatus(size_score, skew_score, pos_score_x, pos_score_y, min_size, max_size, min_skew,
                                                                   max_skew, min_bx, max_bx, min_by, max_by)) {
                            std::cout << ">>> Sample collection status: COMPLETE <<<" << std::endl << std::endl;
                            acquisition_completed = true;
                        }

                        // Incremental extrinsic solve using cached corners (fixed intrinsics); drop explicit ill-conditioned pairs only.
                        bool coverage_rebuilt = false;
                        while (static_cast<int>(checker.getValidSampleCount()) >= min_samples) {
                            const size_t n = checker.getValidSampleCount();
                            std::vector<std::vector<cv::Point3f>> object_points_batch(n, obj_template_mm);
                            StereoCalib trial = calib;
                            int ill_idx = -1;
                            const double rms = trial.stereo_calibrate(
                                object_points_batch, cached_pts_l, cached_pts_r,
                                cv::Size(camera_resolution.width, camera_resolution.height),
                                cv::CALIB_FIX_INTRINSIC + cv::CALIB_ZERO_DISPARITY, false,
                                /*fisheye_retry_ill_conditioned=*/false, &ill_idx, min_samples);
                            if (rms >= 0.0 && ill_idx < 0) {
                                live_rms = rms;
                                live_baseline_mm = cv::norm(trial.T, cv::NORM_L2);
                                rotationMatrixToEulerZYXDeg(trial.R, live_rx_deg, live_ry_deg, live_rz_deg);
                                live_calib_ok = true;
                                std::cout << "[INFO][live] Online stereo RMS=" << rms << " px, baseline=" << live_baseline_mm << " mm" << std::endl;
                                break;
                            }
                            if (ill_idx >= 0 && ill_idx < static_cast<int>(n)) {
                                std::cout << "[WARN][live] OpenCV reported ill-conditioned sample " << ill_idx
                                          << "; removing pair and files." << std::endl;
                                checker.removeSampleAt(static_cast<size_t>(ill_idx));
                                cached_pts_l.erase(cached_pts_l.begin() + ill_idx);
                                cached_pts_r.erase(cached_pts_r.begin() + ill_idx);
                                cached_scaled_display_pts.erase(cached_scaled_display_pts.begin() + ill_idx);
                                compactStereoPairFiles(image_folder, ill_idx, image_count);
                                image_count--;
                                rebuildCoverageVisuals(checker, cached_scaled_display_pts, coverage_indicator, pos_indicator, limits_indicator);
                                coverage_rebuilt = true;
                                live_calib_ok = false;
                                continue;
                            }
                            live_calib_ok = false;
                            std::cerr << "[WARN][live] Online stereo calibration failed (RMS=" << rms
                                      << "); discarding last saved pair." << std::endl;
                            discardLastAcceptedSample(checker, cached_pts_l, cached_pts_r, cached_scaled_display_pts, image_count, image_folder,
                                                      coverage_indicator, pos_indicator, limits_indicator);
                            coverage_rebuilt = true;
                            break;
                        }

                        if (!coverage_rebuilt) {
                            float norm_x = checker.getLastDetectedBoardParams().avg_pos.x;
                            float norm_y = checker.getLastDetectedBoardParams().avg_pos.y;
                            float norm_size = checker.getLastDetectedBoardParams().size;
                            addNewCheckerboardPosition(coverage_indicator, pos_indicator, limits_indicator, norm_x, norm_y, norm_size, min_bx, max_bx,
                                                       min_by, max_by, (image_count >= 2));
                            addNewCheckerboardPoly(coverage_indicator, scaled_pts_l);
                        }
                    } else {
                        std::cout << "  ! Checkerboard detected, but sample not valid. Please try a new "
                                     "position/orientation, not similar to previous acquisitions."
                                  << std::endl;
                        low_target_variability_on_last_pics = true;
                    }
                } else {
                    if (!found_l) {
                        std::cerr << "  ! Checkerboard not detected in the LEFT image." << std::endl;
                    } else if (!found_r) {
                        std::cerr << "  ! Checkerboard not detected in the RIGHT image." << std::endl;
                    }
                }
            }
        }
    }

    return std::max(image_count, 0);
}

// ----------------------------- Main -----------------------------

int main(int argc, char* argv[]) {
    Args args;
    args.parse(argc, argv);

    if (args.config_path.empty()) {
        std::cerr << "Error: --config <yaml> is required. Use -h for help." << std::endl;
        return EXIT_FAILURE;
    }

    StereoConfig cfg;
    if (!loadStereoConfig(args.config_path, cfg)) {
        return EXIT_FAILURE;
    }
    if (!args.images_dir_override.empty()) {
        cfg.images_dir = args.images_dir_override;
    }

    if (cfg.h_edges <= 0 || cfg.v_edges <= 0 || cfg.square_size_mm <= 0.0f) {
        std::cerr << "Error: checkerboard values must be positive." << std::endl;
        return EXIT_FAILURE;
    }
    if (cfg.min_samples < 3) {
        std::cerr << "Error: sample_collection.min must be at least 3." << std::endl;
        return EXIT_FAILURE;
    }
    if (cfg.max_samples < cfg.min_samples) {
        std::cerr << "Error: sample_collection.max must be >= sample_collection.min." << std::endl;
        return EXIT_FAILURE;
    }
    sl::RESOLUTION zed_resolution = sl::RESOLUTION::AUTO;
    if (!parseResolution(cfg.resolution, zed_resolution)) {
        std::cerr << "Error: unsupported resolution '" << cfg.resolution
                  << "'. Expected one of: HD1200, HD1080, HD720, SVGA." << std::endl;
        return EXIT_FAILURE;
    }

    // Apply config params globally (used by capture loop + helpers).
    h_edges = cfg.h_edges;
    v_edges = cfg.v_edges;
    square_size = cfg.square_size_mm;
    verbose = cfg.verbose || args.verbose;
    min_samples = cfg.min_samples;
    max_samples = cfg.max_samples;

    std::cout << "*** ZED XOne GS Fisheye Stereo Calibration ***" << std::endl;
    std::cout << " * Config:                    " << args.config_path << std::endl;
    std::cout << " * Left SN:                   " << cfg.left_sn << std::endl;
    std::cout << " * Right SN:                  " << cfg.right_sn << std::endl;
    std::cout << " * Resolution:                " << cfg.resolution << std::endl;
    std::cout << " * Checkerboard inner edges:  " << cfg.h_edges << "x" << cfg.v_edges << std::endl;
    std::cout << " * Checkerboard square size:  " << cfg.square_size_mm << " mm" << std::endl;
    std::cout << " * Sample collection:         min " << min_samples << " / max " << max_samples
              << " good pairs (stereo minimum, live preview, coverage completion)" << std::endl;
    std::cout << " * Verbose logging:           " << (verbose ? "enabled" : "disabled") << std::endl;
    if (!cfg.calibration_output_dir.empty()) {
        std::cout << " * Calibration output dir:    " << cfg.calibration_output_dir << std::endl;
    } else {
        std::cout << " * Calibration output dir:    (current working directory — set calibration_output_dir in YAML)" << std::endl;
    }

    const bool from_dir = !cfg.images_dir.empty();
    if (from_dir) {
        std::cout << " * Mode:                      EXTRINSICS-ONLY (images_dir: " << cfg.images_dir << ")" << std::endl;
    } else {
        std::cout << " * Mode:                      LIVE CAPTURE + EXTRINSICS" << std::endl;
        if (!cfg.capture_images_dir.empty()) {
            image_folder = cfg.capture_images_dir;
            if (image_folder.back() != '/') {
                image_folder += "/";
            }
        } else {
            image_folder = "/var/cargo/zed-calibration/images/";
        }
        std::cout << " * Live capture output dir:   " << image_folder << std::endl;
    }

    // ---------- Pull factory fisheye intrinsics from each camera (kept FIXED) ----------
    sl::CameraParameters left_intr, right_intr;
    sl::MODEL left_model = sl::MODEL::ZED, right_model = sl::MODEL::ZED;

    std::cout << " * Reading factory intrinsics from cameras... " << std::flush;
    if (!fetchFactoryIntrinsics(cfg.left_sn, zed_resolution, left_intr, left_model) ||
        !fetchFactoryIntrinsics(cfg.right_sn, zed_resolution, right_intr, right_model)) {
        std::cerr << std::endl
                  << "Error: both ZED XOne GS cameras must be connected to read factory intrinsics from the SDK." << std::endl;
        return EXIT_FAILURE;
    }
    std::cout << "done." << std::endl;

    if (!validateIntrinsics(left_intr, "Left", verbose) || !validateIntrinsics(right_intr, "Right", verbose)) {
        return EXIT_FAILURE;
    }

    if (left_model != sl::MODEL::ZED_XONE_GS || right_model != sl::MODEL::ZED_XONE_GS) {
        std::cerr << "Error: this tool only supports ZED XOne GS pairs (got left=" << sl::toString(left_model)
                  << ", right=" << sl::toString(right_model) << ")." << std::endl;
        return EXIT_FAILURE;
    }

    StereoCalib calib;
    calib.initDefault(false /* fisheye */);
    calib.left.setFrom(left_intr);
    calib.right.setFrom(right_intr);

    if (calib.left.disto_model_RadTan || calib.right.disto_model_RadTan) {
        std::cerr << "Error: factory intrinsics for one or both cameras are not in fisheye format" << std::endl
                  << "       (p1/p2 are non-zero). This tool only supports fisheye factory calibration." << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << " * Factory intrinsics (fisheye, FIXED during stereo calibration):" << std::endl;
    calib.left.print(" Left ");
    calib.right.print(" Right ");

    const int virtual_serial = sl::generateVirtualStereoSerialNumber(cfg.left_sn, cfg.right_sn);

    // ---------- Acquire (or load) image pairs ----------
    int image_count = -1;  // -1 -> calibrate() auto-counts pairs in image_folder

    if (from_dir) {
        image_folder = cfg.images_dir;
        if (image_folder.back() != '/') image_folder += "/";
        if (!fs::exists(image_folder)) {
            std::cerr << "Error: images_dir does not exist: " << image_folder << std::endl;
            return EXIT_FAILURE;
        }
    } else {
        // Live virtual stereo capture with the existing UI / quality checks.
        sl::Camera zed_camera;
        sl::InitParameters init_params;
        init_params.depth_mode = sl::DEPTH_MODE::NONE;
        init_params.camera_resolution = zed_resolution;
        init_params.camera_fps = 15;
        init_params.enable_image_validity_check = false;
        init_params.camera_disable_self_calib = true;
        init_params.sdk_verbose = sdk_verbose;
        init_params.input.setVirtualStereoFromSerialNumbers(cfg.left_sn, cfg.right_sn, virtual_serial);

        std::cout << " * Opening ZED XOne GS virtual stereo (virtual SN " << virtual_serial << ")..." << std::endl;
        auto status = zed_camera.open(init_params);
        if (status > sl::ERROR_CODE::SUCCESS && status != sl::ERROR_CODE::INVALID_CALIBRATION_FILE) {
            std::cerr << "Error opening virtual stereo: " << sl::toString(status) << std::endl;
            return EXIT_FAILURE;
        }
        auto zed_info = zed_camera.getCameraInformation();
        std::cout << " * Virtual stereo opened (" << zed_info.camera_configuration.resolution.width << "x"
                  << zed_info.camera_configuration.resolution.height << ")." << std::endl;

        // Ensure capture folder exists.
        if (!fs::exists(image_folder) && !fs::create_directories(image_folder)) {
            std::cerr << "Error creating capture folder: " << image_folder << std::endl;
            zed_camera.close();
            return EXIT_FAILURE;
        }
        // Do not remove the mount point itself (can fail with "device busy" on bind mounts).
        // Only clear files from previous runs.
        if (fs::exists(image_folder)) {
            std::uintmax_t removed = 0;
            for (const auto& entry : fs::directory_iterator(image_folder)) {
                std::error_code ec;
                if (entry.is_regular_file()) {
                    const auto filename = entry.path().filename().string();
                    if (filename.rfind("image_left_", 0) == 0 || filename.rfind("image_right_", 0) == 0) {
                        if (fs::remove(entry.path(), ec) && !ec) {
                            removed++;
                        }
                    }
                }
            }
            if (verbose) {
                std::cout << "[DEBUG][main] Removed " << removed << " previous capture images from " << image_folder << std::endl;
            }
        }

        image_count = runLiveCapture(zed_camera, zed_info.camera_configuration.resolution, calib);
        zed_camera.close();

        if (image_count < 0) {
            return EXIT_SUCCESS;  // user-aborted
        }
    }

    if (!cfg.calibration_output_dir.empty()) {
        std::error_code ec;
        fs::create_directories(cfg.calibration_output_dir, ec);
        if (ec) {
            std::cerr << "Error: cannot create calibration_output_dir: " << cfg.calibration_output_dir << " (" << ec.message() << ")" << std::endl;
            return EXIT_FAILURE;
        }
        const fs::path out_dir(cfg.calibration_output_dir);
        fs::remove(out_dir / ("SN" + std::to_string(virtual_serial) + ".conf"), ec);
        fs::remove(out_dir / ("zed_calibration_" + std::to_string(virtual_serial) + ".yml"), ec);
    }

    // ---------- Solve stereo extrinsics with FIXED fisheye intrinsics ----------
    int err = calibrate(image_count, image_folder, calib, h_edges, v_edges, square_size,
                        virtual_serial, cfg.left_sn, cfg.right_sn,
                        /*is_dual_mono=*/true,
                        /*is_4k=*/false,
                        /*save_calib_mono=*/false,
                        /*use_intrinsic_prior=*/true,
                        /*recalibrate_intrinsics=*/false,
                        max_repr_error, verbose, min_samples, cfg.calibration_output_dir);

    if (err == EXIT_SUCCESS) {
        std::cout << std::endl << " +++++ Calibration successful +++++" << std::endl;
    } else {
        std::cout << std::endl << " ----- Calibration failed -----" << std::endl;
    }
    return EXIT_SUCCESS;
}

// ----------------------------- Coverage helpers -----------------------------

void addNewCheckerboardPosition(cv::Mat& coverage_indicator, cv::Mat& pos_indicator, cv::Mat& limits_indicator, float norm_x, float norm_y,
                                float norm_size, float min_x, float max_x, float min_y, float max_y, bool draw_rect) {
    // Checkerbaord position
    int x = static_cast<int>(norm_x * pos_indicator.cols);
    int y = static_cast<int>(norm_y * pos_indicator.rows);
    int size = static_cast<int>(norm_size * 30.0f);
    cv::circle(pos_indicator, cv::Point(x, y), size, cv::Scalar(255, 255, 255), -1);

    int min_x_px = static_cast<int>(min_x * pos_indicator.cols);
    int max_x_px = static_cast<int>(max_x * pos_indicator.cols);
    int min_y_px = static_cast<int>(min_y * pos_indicator.rows);
    int max_y_px = static_cast<int>(max_y * pos_indicator.rows);

    limits_indicator.setTo(cv::Scalar(0, 0, 0));

    if (draw_rect) {
        int col_val = 50;
        cv::rectangle(limits_indicator, cv::Point(0, 0), cv::Point(min_x_px, limits_indicator.rows - 1), cv::Scalar(col_val, col_val, col_val), -1);
        cv::rectangle(limits_indicator, cv::Point(0, 0), cv::Point(limits_indicator.cols - 1, min_y_px), cv::Scalar(col_val, col_val, col_val), -1);
        cv::rectangle(limits_indicator, cv::Point(0, max_y_px), cv::Point(limits_indicator.cols - 1, limits_indicator.rows - 1),
                      cv::Scalar(col_val, col_val, col_val), -1);
        cv::rectangle(limits_indicator, cv::Point(max_x_px, 0), cv::Point(limits_indicator.cols - 1, limits_indicator.rows - 1),
                      cv::Scalar(col_val, col_val, col_val), -1);
    }
    cv::line(limits_indicator, cv::Point(min_x * limits_indicator.cols, 0), cv::Point(min_x * limits_indicator.cols, limits_indicator.rows - 1),
             cv::Scalar(255, 255, 255), 2);
    cv::line(limits_indicator, cv::Point(max_x * limits_indicator.cols, 0), cv::Point(max_x * limits_indicator.cols, limits_indicator.rows - 1),
             cv::Scalar(255, 255, 255), 2);
    cv::line(limits_indicator, cv::Point(0, min_y * limits_indicator.rows), cv::Point(limits_indicator.cols - 1, min_y * limits_indicator.rows),
             cv::Scalar(255, 255, 255), 2);
    cv::line(limits_indicator, cv::Point(0, max_y * limits_indicator.rows), cv::Point(limits_indicator.cols - 1, max_y * limits_indicator.rows),
             cv::Scalar(255, 255, 255), 2);

    if (norm_x < 0.5f && norm_y < 0.5f) {
        top_left_count++;
    } else if (norm_x >= 0.5f && norm_y < 0.5f) {
        top_right_count++;
    } else if (norm_x < 0.5f && norm_y >= 0.5f) {
        bottom_left_count++;
    } else {
        bottom_right_count++;
    }

    if (top_left_count >= min_samples / 4) {
        cv::rectangle(coverage_indicator, cv::Point(0, 0), cv::Point(coverage_indicator.cols / 2, coverage_indicator.rows / 2), cv::Scalar(255), -1);
    }
    if (top_right_count >= min_samples / 4) {
        cv::rectangle(coverage_indicator, cv::Point(coverage_indicator.cols / 2, 0), cv::Point(coverage_indicator.cols, coverage_indicator.rows / 2),
                      cv::Scalar(255), -1);
    }
    if (bottom_left_count >= min_samples / 4) {
        cv::rectangle(coverage_indicator, cv::Point(0, coverage_indicator.rows / 2), cv::Point(coverage_indicator.cols / 2, coverage_indicator.rows),
                      cv::Scalar(255), -1);
    }
    if (bottom_right_count >= min_samples / 4) {
        cv::rectangle(coverage_indicator, cv::Point(coverage_indicator.cols / 2, coverage_indicator.rows / 2),
                      cv::Point(coverage_indicator.cols, coverage_indicator.rows), cv::Scalar(255), -1);
    }
}

void addNewCheckerboardPoly(cv::Mat& coverage_indicator, const std::vector<cv::Point2f>& pts_l) {
    cv::Point tl = pts_l[0];
    cv::Point tr = pts_l[h_edges - 1];
    cv::Point br = pts_l[pts_l.size() - 1];
    cv::Point bl = pts_l[pts_l.size() - h_edges];

    std::vector<cv::Point> poly_pts;
    poly_pts.push_back(tl);
    poly_pts.push_back(tr);
    poly_pts.push_back(br);
    poly_pts.push_back(bl);

    cv::Mat mask = cv::Mat::zeros(coverage_indicator.size(), CV_8UC1);
    cv::fillPoly(mask, std::vector<std::vector<cv::Point>>{poly_pts}, cv::Scalar(255 / 5, 255 / 5, 255 / 5));

    coverage_indicator = coverage_indicator + mask;
}

void applyCoverageIndicatorOverlay(cv::Mat& image, const cv::Mat& coverage_indicator, const cv::Mat& limits_indicator) {
    std::vector<cv::Mat> channels;
    cv::split(image, channels);
    channels[0] = channels[0] - coverage_indicator;
    channels[2] = channels[2] - coverage_indicator;
    channels[0] = channels[0] - limits_indicator;
    channels[1] = channels[1] - limits_indicator;
    cv::merge(channels, image);
}

void applyPosIndicatorOverlay(cv::Mat& image, const cv::Mat& pos_indicator) {
    std::vector<cv::Mat> channels;
    cv::split(image, channels);
    channels[2] = channels[2] - pos_indicator;
    channels[1] = channels[1] - pos_indicator;
    cv::merge(channels, image);
}
