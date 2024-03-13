#include "gnss_replay.hpp"

using json = nlohmann::json;

namespace sl_tools {

inline bool is_microseconds(uint64_t timestamp) {
    // Check if the timestamp is in microseconds
    return (1'000'000'000'000'000 <= timestamp && timestamp < 10'000'000'000'000'000ULL);
}

inline bool is_nanoseconds(uint64_t timestamp) {
    // Check if the timestamp is in microseconds
    return (1'000'000'000'000'000'000 <= timestamp && timestamp < 10'000'000'000'000'000'000ULL);
}

GNSSReplay::GNSSReplay(const std::shared_ptr<sl::Camera> &zed){_zed = zed;}

GNSSReplay::~GNSSReplay() {}

bool GNSSReplay::initialize() {
  auto svo_custom_data_keys = _zed->getSVODataKeys();
  std::string gnss_key = "GNSS_json";
  bool found = false;
  for (auto &it : svo_custom_data_keys) {
    if (it.find(gnss_key) != std::string::npos) {
      found = true;
      break;
    }
  }

  if(!found) {
    return false;
  }

  std::map<sl::Timestamp, sl::SVOData> data;
  auto status = _zed->retrieveSVOData(gnss_key, data);  // Get ALL

  /*
   We handle 2 formats:
   *
   * {
          "coordinates": {
              "latitude": XXX,
              "longitude": XXX,
              "altitude": XXX
          },
          "ts": 1694263390000000,
          "latitude_std": 0.51,
          "longitude_std": 0.51,
          "altitude_std": 0.73,
          "position_covariance": [
              0.2601,
              0,
              0,
              0,
              0.2601,
              0,
              0,
              0,
              0.5328999999999999
          ]
      },
   *********
   * Or
   * this one will be converted to the format above
      {
          "Eph": 0.467,
          "EpochTimeStamp": 1694266998000000,
          "Epv": 0.776,
          "Geopoint": {
              "Altitude": XXX,
              "Latitude": XXX,
              "Longitude": XXX
          },
          "Position": [
              [
                  XXX,
                  XXX,
                  XXX
              ]
          ],
          "Velocity": [
              [
                  -0.63,
                  0.25,
                  0.53
              ]
          ]
      }
   */

  auto tmp_array = json::array();
  for (auto &it : data) {
    try {
      auto gnss_data_point =
          json::parse(it.second.content.begin(), it.second.content.end());
      auto gnss_data_point_formatted = json::object();

      if (!gnss_data_point["Geopoint"].is_null()) {
        gnss_data_point_formatted["coordinates"] = {
            {"latitude", gnss_data_point["Geopoint"]["Latitude"]},
            {"longitude", gnss_data_point["Geopoint"]["Longitude"]},
            {"altitude", gnss_data_point["Geopoint"]["Altitude"]},
        };
        gnss_data_point_formatted["ts"] = gnss_data_point["EpochTimeStamp"];

        float latitude_std = gnss_data_point["Eph"];
        float longitude_std = gnss_data_point["Eph"];
        float altitude_std = gnss_data_point["Epv"];

        gnss_data_point_formatted["latitude_std"] = latitude_std;
        gnss_data_point_formatted["longitude_std"] = longitude_std;
        gnss_data_point_formatted["altitude_std"] = altitude_std;

        gnss_data_point_formatted["position_covariance"] =
            json::array({longitude_std + longitude_std, 0, 0, 0,
                         latitude_std + latitude_std, 0, 0, 0,
                         altitude_std + altitude_std});

        gnss_data_point_formatted["original_gnss_data"] = gnss_data_point;

      } else if (!gnss_data_point["coordinates"].is_null() &&
                 !gnss_data_point["latitude_std"].is_null() &&
                 !gnss_data_point["longitude_std"].is_null()) {
        // no conversion
        gnss_data_point_formatted = gnss_data_point;
      }

      tmp_array.push_back(gnss_data_point_formatted);

    } catch (const std::runtime_error &e) {
      std::cerr << "Error while reading GNSS data: " << e.what() << std::endl;
    }
  }
  gnss_data["GNSS"] = tmp_array;

  current_gnss_idx = 0;
  previous_ts = 0;

  return true;
}

void GNSSReplay::close() {
    gnss_data.clear();
    current_gnss_idx = 0;
}


inline std::string gps_mode2str(int status) {
    std::string out;
    switch (status) {
        case 1:
            out = "STATUS_GPS";
            break;
        case 2:
            out = "STATUS_DGPS";
            break;
        case 3:
            out = "STATUS_RTK_FIX";
            break;
        case 4:
            out = "STATUS_RTK_FLT";
            break;
        case 5:
            out = "STATUS_DR";
            break;
        case 6:
            out = "STATUS_GNSSDR";
            break;
        case 7:
            out = "STATUS_TIME";
            break;
        case 8:
            out = "STATUS_SIM";
            break;
        case 9:
            out = "STATUS_PPS_FIX";
            break;
        default:
        case 0:
            out = "STATUS_UNK";
            break;
    };
    return out;
}

sl::GNSSData getGNSSData(json &gnss_data, int gnss_idx) {
    sl::GNSSData current_gnss_data;
    current_gnss_data.ts = 0;

    // If we are at the end of GNSS data, exit
    if (gnss_idx >= gnss_data["GNSS"].size()) {
        std::cout << "Reached the end of the GNSS playback data." << std::endl;
        return current_gnss_data;
    }

    json current_gnss_data_json = gnss_data["GNSS"][gnss_idx];
    // Check inputs:
    if (
            current_gnss_data_json["coordinates"].is_null()
            || current_gnss_data_json["coordinates"]["latitude"].is_null()
            || current_gnss_data_json["coordinates"]["longitude"].is_null()
            || current_gnss_data_json["coordinates"]["altitude"].is_null()
            || current_gnss_data_json["ts"].is_null()
            ) {
        std::cout << "Null GNSS playback data." << std::endl;
        return current_gnss_data;
    }

    if (!current_gnss_data_json["original_gnss_data"].is_null()) {
        if (!current_gnss_data_json["original_gnss_data"]["fix"].is_null()) {
            if (!current_gnss_data_json["original_gnss_data"]["fix"]["status"].is_null())
                std::cout << "GNSS info: " << gps_mode2str(current_gnss_data_json["original_gnss_data"]["fix"]["status"]) << " " << current_gnss_data_json["longitude_std"] << " " << current_gnss_data_json["altitude_std"] << std::endl;
        }
    }

    auto gnss_timestamp = current_gnss_data_json["ts"].get<uint64_t>();
    // Fill out timestamp:
    if (is_microseconds(gnss_timestamp))
        current_gnss_data.ts.setMicroseconds(gnss_timestamp);
    else if (is_nanoseconds(gnss_timestamp))
        current_gnss_data.ts.setNanoseconds(gnss_timestamp);
    else
        std::cerr << "Warning: Invalid timestamp format from GNSS file" << std::endl;

    // Fill out coordinates:
    current_gnss_data.setCoordinates(current_gnss_data_json["coordinates"]["latitude"].get<float>(),
            current_gnss_data_json["coordinates"]["longitude"].get<float>(),
            current_gnss_data_json["coordinates"]["altitude"].get<float>(),
            false);

    // Fill out default standard deviation:
    current_gnss_data.longitude_std = current_gnss_data_json["longitude_std"];
    current_gnss_data.latitude_std = current_gnss_data_json["latitude_std"];
    current_gnss_data.altitude_std = current_gnss_data_json["altitude_std"];
    // Fill out covariance [must be not null]
    std::array<double, 9> position_covariance;
    for (unsigned i = 0; i < 9; i++)
        position_covariance[i] = 0.0; // initialize empty covariance

    // set covariance diagonal
    position_covariance[0] = current_gnss_data.longitude_std * current_gnss_data.longitude_std;
    position_covariance[1 * 3 + 1] = current_gnss_data.latitude_std * current_gnss_data.latitude_std;
    position_covariance[2 * 3 + 2] = current_gnss_data.altitude_std * current_gnss_data.altitude_std;
    current_gnss_data.position_covariance = position_covariance;

    return current_gnss_data;
}

sl::GNSSData GNSSReplay::getNextGNSSValue(uint64_t current_timestamp) {
    sl::GNSSData current_gnss_data = getGNSSData(gnss_data, current_gnss_idx);

    if (current_gnss_data.ts.data_ns == 0)
        return current_gnss_data;

    if (current_gnss_data.ts.data_ns > current_timestamp) {
        current_gnss_data.ts.data_ns = 0;
        return current_gnss_data;
    }

    sl::GNSSData last_data;
    int step = 1;
    while (1) {
        last_data = current_gnss_data;
        int diff_last = current_timestamp - current_gnss_data.ts.data_ns;
        current_gnss_data = getGNSSData(gnss_data, current_gnss_idx + step++);
        if (current_gnss_data.ts.data_ns == 0) //error / end of file 
            break;

        if (current_gnss_data.ts.data_ns > current_timestamp) {
            if ((current_gnss_data.ts.data_ns - current_timestamp) > diff_last) // keep last
                current_gnss_data = last_data;
            break;
        }
        current_gnss_idx++;
    }

    return current_gnss_data;
}

sl::FUSION_ERROR_CODE GNSSReplay::grab(sl::GNSSData &current_data, uint64_t current_timestamp) {
    current_data.ts.data_ns = 0;

    if (current_timestamp > 0 && (current_timestamp > last_cam_ts))
        current_data = getNextGNSSValue(current_timestamp);

    if (current_data.ts.data_ns == previous_ts)
        current_data.ts.data_ns = 0;

    last_cam_ts = current_timestamp;

    if (current_data.ts.data_ns == 0) // Invalid data
        return sl::FUSION_ERROR_CODE::FAILURE;

    previous_ts = current_data.ts.data_ns;
    return sl::FUSION_ERROR_CODE::SUCCESS;
}

}  // namespace sl_tools