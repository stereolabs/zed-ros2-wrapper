// © 2026, Cargo Robotics
// Adopted from https://github.com/stereolabs/zed-opencv-calibration (upstream Stereolabs sources; Cargo maintains local patches — see README).

#include "opencv_calibration.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <locale>
#include <map>
#include <set>
#include <vector>

int calibrate(int img_count, const std::string& folder, StereoCalib& calib_data, int h_edges, int v_edges, double square_size, int serial,
              int left_sn, int right_sn,
              bool is_dual_mono, bool is_4k, bool save_calib_mono, bool use_intrinsic_prior, bool recalibrate_intrinsics, double max_repr_error,
              bool verbose, int min_stereo_samples, const std::string& calibration_output_dir) {
    std::vector<cv::Mat> left_images, right_images;

    /// Read images
    cv::Size imageSize = cv::Size(0, 0);

    std::cout << std::endl << "Loading the stored images from folder: " << folder << std::endl;

    // Count images in the folder
    if (img_count == -1) {
        std::cout << " * Counting images in the folder..." << std::endl;
        int actual_img_count = 0;
        while (1) {
            std::string left_path = folder + "image_left_" + std::to_string(actual_img_count) + ".png";
            std::string right_path = folder + "image_right_" + std::to_string(actual_img_count) + ".png";
            if (std::filesystem::exists(left_path) && std::filesystem::exists(right_path)) {
                actual_img_count++;
            } else {
                break;
            }
        }
        img_count = actual_img_count;
        std::cout << "   - Found " << img_count << " images." << std::endl;
    }

    for (int i = 0; i < img_count; i++) {
        std::cout << "." << std::flush;
        cv::Mat grey_l = cv::imread(folder + "image_left_" + std::to_string(i) + ".png", cv::IMREAD_GRAYSCALE);
        cv::Mat grey_r = cv::imread(folder + "image_right_" + std::to_string(i) + ".png", cv::IMREAD_GRAYSCALE);

        if (!grey_l.empty() && !grey_r.empty()) {
            if (imageSize.width == 0)
                imageSize = grey_l.size();
            else if (imageSize != left_images.back().size()) {
                std::cerr << std::endl
                          << " !!! ERROR !!! " << std::endl
                          << "Frames number #" << i << " do not have the same size as the previous ones: " << imageSize << " vs "
                          << left_images.back().size() << std::endl;
                return EXIT_FAILURE;
            }

            left_images.push_back(grey_l);
            right_images.push_back(grey_r);
        }
    }

    std::cout << std::endl << " * " << left_images.size() << " samples collected" << std::endl;

    // Define object points of the target
    // Note: object points must be point3f. Point3d is not supported by
    // 'cv::calibrateCamera'
    std::vector<cv::Point3f> pattern_points;
    for (int i = 0; i < v_edges; i++) {
        for (int j = 0; j < h_edges; j++) {
            pattern_points.push_back(cv::Point3f(static_cast<float>(square_size * j), static_cast<float>(square_size * i), 0));
        }
    }

    std::vector<std::vector<cv::Point3f>> object_points;
    std::vector<std::vector<cv::Point2f>> pts_l, pts_r;

    cv::Size t_size(h_edges, v_edges);

    std::cout << "Detecting the target corners on the images" << std::endl;

    for (int i = 0; i < left_images.size(); i++) {
        std::cout << "." << std::flush;
        std::vector<cv::Point2f> pts_l_f, pts_r_f;
        bool found_l = cv::findChessboardCorners(left_images.at(i), t_size, pts_l_f, 3);
        bool found_r = cv::findChessboardCorners(right_images.at(i), t_size, pts_r_f, 3);

        if (found_l && found_r) {
            cv::cornerSubPix(left_images.at(i), pts_l_f, cv::Size(5, 5), cv::Size(-1, -1),
                             cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001));

            cv::cornerSubPix(right_images.at(i), pts_r_f, cv::Size(5, 5), cv::Size(-1, -1),
                             cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001));

            pts_l.push_back(pts_l_f);
            pts_r.push_back(pts_r_f);
            object_points.push_back(pattern_points);
        } else {
            std::cout << std::endl << "- No valid targets detected on frames #" << i << " -" << std::endl;
        }
    }

    /// Compute calibration

    if (pts_l.size() < static_cast<size_t>(min_stereo_samples)) {
        std::cout << " !!! Not enough images with the target detected !!!" << std::endl;
        std::cout << " Need at least " << min_stereo_samples << " valid stereo pairs (found " << pts_l.size() << ")." << std::endl;
        std::cout << " Please perform a new data acquisition." << std::endl << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << std::endl << " * Valid samples: " << pts_l.size() << "/" << img_count << std::endl;

    double rms_l = 0.0;
    double rms_r = 0.0;
    auto flags = use_intrinsic_prior ? cv::CALIB_USE_INTRINSIC_GUESS : 0;
    if (recalibrate_intrinsics) {
        if (use_intrinsic_prior && verbose) {
            std::cout << "[DEBUG][calibrate] Using intrinsic parameters as calibration prior." << std::endl;
        }

        std::cout << "Left camera calibration... " << std::flush;
        rms_l = calib_data.left.mono_calibrate(object_points, pts_l, imageSize, flags, verbose);
        std::cout << "Done." << std::endl;

        std::cout << "Right camera calibration... " << std::flush;
        rms_r = calib_data.right.mono_calibrate(object_points, pts_r, imageSize, flags, verbose);
        std::cout << "Done." << std::endl;
    } else {
        if (!use_intrinsic_prior) {
            std::cerr << std::endl
                      << " !!! ERROR !!! " << std::endl
                      << "Intrinsics recalibration is disabled, but no intrinsic prior "
                         "is available."
                      << std::endl
                      << "Provide priors (camera or file) or run with intrinsic "
                         "recalibration enabled."
                      << std::endl;
            return EXIT_FAILURE;
        }
        std::cout << "Skipping mono intrinsics calibration (using existing priors)." << std::endl;
    }

    std::cout << "Stereo calibration... " << std::flush;

    auto err = calib_data.stereo_calibrate(object_points, pts_l, pts_r, imageSize, cv::CALIB_FIX_INTRINSIC + cv::CALIB_ZERO_DISPARITY,
                                           // cv::CALIB_USE_INTRINSIC_GUESS + cv::CALIB_ZERO_DISPARITY,
                                           verbose, true, nullptr, min_stereo_samples);

    std::cout << "Done." << std::endl;

    if (err < 0.0) {
        std::cerr << std::endl
                  << " !!! ERROR !!! " << std::endl
                  << "Stereo calibration failed numerically (RMS=" << err << ")." << std::endl
                  << "This usually means the optimizer could not produce a valid extrinsic solution." << std::endl
                  << "No calibration files will be generated from this run." << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << std::endl << "*** Calibration Report ***" << std::endl;

    std::cout << " * Reprojection errors: " << std::endl;
    if (recalibrate_intrinsics) {
        std::cout << "   * Left:\t" << rms_l << " px" << (rms_l > max_repr_error ? "\t!!! TOO HIGH !!!" : "\t-> GOOD") << std::endl;
        std::cout << "   * Right:\t" << rms_r << " px" << (rms_r > max_repr_error ? "\t!!! TOO HIGH !!!" : "\t-> GOOD") << std::endl;
    } else {
        std::cout << "   * Left:\tSKIPPED (kept prior intrinsics)" << std::endl;
        std::cout << "   * Right:\tSKIPPED (kept prior intrinsics)" << std::endl;
    }
    std::cout << "   * Stereo:\t" << err << " px" << (err > max_repr_error ? "\t!!! TOO HIGH !!!" : "\t-> GOOD") << std::endl;
    if ((recalibrate_intrinsics && (rms_l > max_repr_error || rms_r > max_repr_error)) || err > max_repr_error) {
        std::cerr << std::endl
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
        std::cerr << " !!! Cannot save the calibration file: 'Invalid data type'" << std::endl;
        return EXIT_FAILURE;
    }

    if (calib_data.T.at<double>(0) > 0) {
        std::cerr << std::endl
                  << "\t !! Warning !!" << std::endl
                  << "The value of the baseline has opposite sign (T_x = " << calib_data.T.at<double>(0) << ")." << std::endl;
        std::cerr << "Swap left and right cameras and redo the calibration." << std::endl;

        return EXIT_FAILURE;
    }

    if (calib_data.T.at<double>(0) > 0) {
        std::cerr << std::endl
                  << "\t !! Warning !!" << std::endl
                  << "The value of the baseline has opposite sign than expected(T_x = " << calib_data.T.at<double>(0) << ")." << std::endl;
        std::cerr << "Swap left and right cameras and redo the calibration." << std::endl;

        return EXIT_FAILURE;
    }

    constexpr double MIN_BASELINE = 30.0f;  // Minimum plausible baseline in mm

    if (fabs(calib_data.T.at<double>(0)) < MIN_BASELINE) {
        std::cerr << std::endl
                  << "\t !! Warning !!" << std::endl
                  << "The measured baseline is below the plausibility threshold (T_x = " << calib_data.T.at<double>(0)
                  << " mm, expected >= " << MIN_BASELINE << " mm)." << std::endl
                  << "Continuing, but verify the result and consider redoing the calibration." << std::endl;
    }

    std::cout << std::endl;

    std::cout << "** Camera parameters **" << std::endl;
    std::cout << "* Intrinsic mat left:" << std::endl << calib_data.left.K << std::endl;
    std::cout << "* Distortion mat left:" << std::endl << calib_data.left.D << std::endl;
    std::cout << "* Intrinsic mat right:" << std::endl << calib_data.right.K << std::endl;
    std::cout << "* Distortion mat right:" << std::endl << calib_data.right.D << std::endl;
    std::cout << std::endl;
    std::cout << "** Extrinsic parameters **" << std::endl;
    std::cout << "* Translation:" << std::endl << calib_data.T << std::endl;
    std::cout << "* Rotation:" << std::endl << calib_data.Rv << std::endl;
    std::cout << std::endl;

    std::cout << std::endl << "*** Save Calibration files ***" << std::endl;

    if (!calibration_output_dir.empty()) {
        std::error_code ec;
        std::filesystem::create_directories(calibration_output_dir, ec);
        if (ec) {
            std::cerr << " !!! Cannot create calibration_output_dir '" << calibration_output_dir << "': " << ec.message() << std::endl;
            return EXIT_FAILURE;
        }
    }

    std::string opencv_file = calib_data.saveCalibOpenCV(serial, calibration_output_dir);
    if (!opencv_file.empty()) {
        std::cout << " * OpenCV calibration file saved: " << opencv_file << std::endl;
    } else {
        std::cout << " !!! Failed to save OpenCV calibration file " << opencv_file << " !!!" << std::endl;
    }

    // SDK format is only supported for dual-mono setups
    if (is_dual_mono) {
        std::string zed_file = calib_data.saveCalibZED(serial, left_sn, right_sn, is_4k, calibration_output_dir);
        if (!zed_file.empty()) {
            std::cout << " * ZED SDK calibration file saved: " << zed_file << std::endl;
        } else {
            std::cout << " !!! Failed to save ZED SDK calibration file " << zed_file << " !!!" << std::endl;
        }
    }

    return EXIT_SUCCESS;
}

std::string StereoCalib::saveCalibOpenCV(int serial, const std::string& calibration_output_dir) {
    const std::string basename = "zed_calibration_" + std::to_string(serial) + ".yml";
    const std::filesystem::path calib_path =
        calibration_output_dir.empty() ? std::filesystem::path(basename) : std::filesystem::path(calibration_output_dir) / basename;

    cv::FileStorage fs(calib_path.string(), cv::FileStorage::WRITE);
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

        return calib_path.string();
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
        outfile << "k5 = 0\n";
        outfile << "k6 = 0\n";
        outfile << "p1 = 0\n";
        outfile << "p2 = 0\n";
    }
    outfile << "\n";
}

namespace {

std::string trim(const std::string& s) {
    const auto begin = s.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) {
        return "";
    }
    const auto end = s.find_last_not_of(" \t\r\n");
    return s.substr(begin, end - begin + 1);
}

struct IniData {
    std::vector<std::string> section_order;
    std::map<std::string, std::vector<std::pair<std::string, std::string>>> sections;
};

bool parseIniFile(const std::filesystem::path& path, IniData& out_ini) {
    std::ifstream in(path);
    if (!in.is_open()) {
        return false;
    }

    std::string line;
    std::string section;
    while (std::getline(in, line)) {
        const std::string stripped = trim(line);
        if (stripped.empty() || stripped[0] == '#' || stripped[0] == ';') {
            continue;
        }
        if (stripped.front() == '[' && stripped.back() == ']') {
            section = stripped.substr(1, stripped.size() - 2);
            if (!out_ini.sections.count(section)) {
                out_ini.section_order.push_back(section);
                out_ini.sections[section] = {};
            }
            continue;
        }
        const auto eq = stripped.find('=');
        if (eq == std::string::npos || section.empty()) {
            continue;
        }
        out_ini.sections[section].push_back({trim(stripped.substr(0, eq)), trim(stripped.substr(eq + 1))});
    }
    return !out_ini.sections.empty();
}

void writeSection(std::ofstream& outfile, const std::string& section, const std::vector<std::pair<std::string, std::string>>& kvs) {
    outfile << "[" << section << "]\n";
    for (const auto& kv : kvs) {
        outfile << kv.first << " = " << kv.second << "\n";
    }
    outfile << "\n";
}

std::filesystem::path resolveTemplatePath(int serial) {
    const std::string file_name = "SN" + std::to_string(serial) + ".conf";
    const std::vector<std::filesystem::path> candidates = {
        std::filesystem::path(file_name),
        std::filesystem::path("/usr/local/zed/settings") / file_name,
    };
    for (const auto& candidate : candidates) {
        if (std::filesystem::exists(candidate)) {
            return candidate;
        }
    }
    return {};
}

void writeStereoSection(std::ofstream& outfile, const cv::Mat& T, const cv::Mat& Rv, const std::vector<std::string>& stereo_suffix_order) {
    outfile << "[STEREO]\n";
    outfile << "Baseline = " << -T.at<double>(0) << "\n";
    outfile << "TY = " << T.at<double>(1) << "\n";
    outfile << "TZ = " << T.at<double>(2) << "\n";

    for (const auto& suffix : stereo_suffix_order) {
        outfile << "CV_" << suffix << " = " << Rv.at<double>(1) << "\n";
    }
    for (const auto& suffix : stereo_suffix_order) {
        outfile << "RX_" << suffix << " = " << Rv.at<double>(0) << "\n";
    }
    for (const auto& suffix : stereo_suffix_order) {
        outfile << "RZ_" << suffix << " = " << Rv.at<double>(2) << "\n";
    }
    outfile << "\n";
}

// Order CV_/RX_/RZ_ keys like factory stereo files (see SN*.conf): FHD, SVGA, FHD1200, then other suffixes.
std::vector<std::string> stereoRotationKeyOrder(const std::vector<std::string>& suffixes_union) {
    static const std::vector<std::string> kPreferred = {"FHD", "SVGA", "FHD1200", "4k", "QHDPLUS"};
    std::set<std::string> remaining(suffixes_union.begin(), suffixes_union.end());
    std::vector<std::string> ordered;
    for (const auto& p : kPreferred) {
        if (remaining.count(p)) {
            ordered.push_back(p);
            remaining.erase(p);
        }
    }
    std::vector<std::string> rest(remaining.begin(), remaining.end());
    std::sort(rest.begin(), rest.end());
    ordered.insert(ordered.end(), rest.begin(), rest.end());
    return ordered;
}

// ZED SDK expects all eight keys in [LEFT_DISTO]/[RIGHT_DISTO] (k1..k4, k5, k6, p1, p2). Fisheye uses k1..k4
// for the main model; pad missing keys with 0 to match factory mono files and silence SDK warnings.
std::vector<std::pair<std::string, std::string>> normalizedStereoDisto(
    const std::vector<std::pair<std::string, std::string>>& raw) {
    std::map<std::string, std::string> m;
    for (const auto& kv : raw) {
        m[kv.first] = kv.second;
    }
    if (!m.count("k1") || !m.count("k2") || !m.count("k3") || !m.count("k4")) {
        return raw;
    }
    static const char* kKeyOrder[8] = {"k1", "k2", "k3", "k4", "k5", "k6", "p1", "p2"};
    std::vector<std::pair<std::string, std::string>> out;
    out.reserve(8);
    for (const char* key : kKeyOrder) {
        out.push_back({key, m.count(key) ? m.at(key) : std::string("0")});
    }
    return out;
}

bool writeMergedTemplateConfig(std::ofstream& outfile, int left_sn, int right_sn, const cv::Mat& T, const cv::Mat& Rv) {
    const auto left_path = resolveTemplatePath(left_sn);
    const auto right_path = resolveTemplatePath(right_sn);
    if (left_path.empty() || right_path.empty()) {
        return false;
    }

    IniData left_ini;
    IniData right_ini;
    if (!parseIniFile(left_path, left_ini) || !parseIniFile(right_path, right_ini)) {
        return false;
    }

    std::vector<std::string> camera_suffixes;
    for (const auto& section : left_ini.section_order) {
        if (section.rfind("CAM_", 0) == 0) {
            const std::string suffix = section.substr(4);
            camera_suffixes.push_back(suffix);
        }
    }
    for (const auto& section : right_ini.section_order) {
        if (section.rfind("CAM_", 0) == 0) {
            const std::string suffix = section.substr(4);
            if (std::find(camera_suffixes.begin(), camera_suffixes.end(), suffix) == camera_suffixes.end()) {
                camera_suffixes.push_back(suffix);
            }
        }
    }

    const std::string cam_section = "CAM_";
    for (const auto& suffix : camera_suffixes) {
        const std::string sec = cam_section + suffix;
        if (left_ini.sections.count(sec)) {
            writeSection(outfile, "LEFT_CAM_" + suffix, left_ini.sections.at(sec));
        }
        if (right_ini.sections.count(sec)) {
            writeSection(outfile, "RIGHT_CAM_" + suffix, right_ini.sections.at(sec));
        }
    }

    bool wrote_disto = false;
    if (left_ini.sections.count("DISTO")) {
        writeSection(outfile, "LEFT_DISTO", normalizedStereoDisto(left_ini.sections.at("DISTO")));
        wrote_disto = true;
    }
    if (right_ini.sections.count("DISTO")) {
        writeSection(outfile, "RIGHT_DISTO", normalizedStereoDisto(right_ini.sections.at("DISTO")));
        wrote_disto = true;
    }
    if (!wrote_disto) {
        return false;
    }

    std::string sensor_id = "1";
    if (left_ini.sections.count("MISC")) {
        for (const auto& kv : left_ini.sections.at("MISC")) {
            if (kv.first == "Sensor_ID") {
                sensor_id = kv.second;
                break;
            }
        }
    } else if (right_ini.sections.count("MISC")) {
        for (const auto& kv : right_ini.sections.at("MISC")) {
            if (kv.first == "Sensor_ID") {
                sensor_id = kv.second;
                break;
            }
        }
    }

    const std::vector<std::string> stereo_key_order = stereoRotationKeyOrder(camera_suffixes);
    writeStereoSection(outfile, T, Rv, stereo_key_order);
    // Append [MISC] after stereo — mono templates place it last; keeps core stereo layout recognizable to the SDK.
    writeSection(outfile, "MISC", {{"Sensor_ID", sensor_id}});
    return true;
}

}  // namespace

std::string StereoCalib::saveCalibZED(int serial, int left_sn, int right_sn, bool is_4k, const std::string& calibration_output_dir) {
    const std::string basename = "SN" + std::to_string(serial) + ".conf";
    const std::filesystem::path calib_path =
        calibration_output_dir.empty() ? std::filesystem::path(basename) : std::filesystem::path(calibration_output_dir) / basename;

    // Write parameters to a text file
    std::ofstream outfile(calib_path.string());
    if (!outfile.is_open()) {
        std::cerr << " !!! Cannot save the calibration file: 'Unable to open output file'" << std::endl;
        return std::string();
    }
    outfile.imbue(std::locale::classic());

    if (writeMergedTemplateConfig(outfile, left_sn, right_sn, T, Rv)) {
        outfile.close();
        return calib_path.string();
    }

    std::cerr << " ! Warning: could not load template SN config files for SN" << left_sn << " and SN" << right_sn
              << ". Falling back to generated intrinsics/distortion." << std::endl;

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

        outfile << "[MISC]\n";
        outfile << "Sensor_ID = 1\n\n";

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
        return calib_path.string();
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
        outfile << "cy = " << right.K.at<double>(1, 2) - (2160 - 1800) / 2 << "\n\n";

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
        outfile << "cy = " << right.K.at<double>(1, 2) - (2160 - 1200) / 2 << "\n\n";

        outfile << "[LEFT_DISTO]\n";
        printDisto(left, outfile);

        outfile << "[RIGHT_DISTO]\n";
        printDisto(right, outfile);

        outfile << "[MISC]\n";
        outfile << "Sensor_ID = 1\n\n";

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
        return calib_path.string();
    }
}
