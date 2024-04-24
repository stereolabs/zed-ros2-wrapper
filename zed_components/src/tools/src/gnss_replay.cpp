#include "gnss_replay.hpp"

using json = nlohmann::json;

namespace sl_tools
{

inline bool is_microseconds(uint64_t timestamp)
{
  // Check if the timestamp is in microseconds
  return 1'000'000'000'000'000 <= timestamp && timestamp < 10'000'000'000'000'000ULL;
}

inline bool is_nanoseconds(uint64_t timestamp)
{
  // Check if the timestamp is in microseconds
  return 1'000'000'000'000'000'000 <= timestamp && timestamp < 10'000'000'000'000'000'000ULL;
}

GNSSReplay::GNSSReplay(const std::shared_ptr<sl::Camera> & zed) {_zed = zed;}

GNSSReplay::~GNSSReplay() {close();}

bool GNSSReplay::initialize()
{
  auto svo_custom_data_keys = _zed->getSVODataKeys();
  std::string gnss_key = "GNSS_json";
  bool found = false;
  for (auto & it : svo_custom_data_keys) {
    if (it.find(gnss_key) != std::string::npos) {
      found = true;
      break;
    }
  }

  if (!found) {
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
  for (auto & it : data) {
    try {
      auto _gnss_data_point =
        json::parse(it.second.content.begin(), it.second.content.end());
      auto _gnss_data_point_formatted = json::object();

      if (!_gnss_data_point["Geopoint"].is_null()) {
        _gnss_data_point_formatted["coordinates"] = {
          {"latitude", _gnss_data_point["Geopoint"]["Latitude"]},
          {"longitude", _gnss_data_point["Geopoint"]["Longitude"]},
          {"altitude", _gnss_data_point["Geopoint"]["Altitude"]},
        };
        _gnss_data_point_formatted["ts"] = _gnss_data_point["EpochTimeStamp"];

        float latitude_std = _gnss_data_point["Eph"];
        float longitude_std = _gnss_data_point["Eph"];
        float altitude_std = _gnss_data_point["Epv"];

        _gnss_data_point_formatted["latitude_std"] = latitude_std;
        _gnss_data_point_formatted["longitude_std"] = longitude_std;
        _gnss_data_point_formatted["altitude_std"] = altitude_std;

        _gnss_data_point_formatted["position_covariance"] =
          json::array(
          {longitude_std + longitude_std, 0, 0, 0,
            latitude_std + latitude_std, 0, 0, 0,
            altitude_std + altitude_std});

        _gnss_data_point_formatted["original__gnss_data"] = _gnss_data_point;

      } else if (!_gnss_data_point["coordinates"].is_null() &&
        !_gnss_data_point["latitude_std"].is_null() &&
        !_gnss_data_point["longitude_std"].is_null())
      {
        // no conversion
        _gnss_data_point_formatted = _gnss_data_point;
      }

      tmp_array.push_back(_gnss_data_point_formatted);

    } catch (const std::runtime_error & e) {
      std::cerr << "Error while reading GNSS data: " << e.what() << std::endl;
    }
  }
  _gnss_data["GNSS"] = tmp_array;

  _current_gnss_idx = 0;
  _previous_ts = 0;

  return true;
}

void GNSSReplay::close()
{
  _gnss_data.clear();
  _current_gnss_idx = 0;
}


inline std::string gps_mode2str(int status)
{
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
  }
  return out;
}

sl::GNSSData getGNSSData(json & _gnss_data, int gnss_idx)
{
  sl::GNSSData current__gnss_data;
  current__gnss_data.ts = 0;

  // If we are at the end of GNSS data, exit
  if (gnss_idx >= _gnss_data["GNSS"].size()) {
    std::cout << "Reached the end of the GNSS playback data." << std::endl;
    return current__gnss_data;
  }

  json current__gnss_data_json = _gnss_data["GNSS"][gnss_idx];
  // Check inputs:
  if (
    current__gnss_data_json["coordinates"].is_null() ||
    current__gnss_data_json["coordinates"]["latitude"].is_null() ||
    current__gnss_data_json["coordinates"]["longitude"].is_null() ||
    current__gnss_data_json["coordinates"]["altitude"].is_null() ||
    current__gnss_data_json["ts"].is_null())
  {
    std::cout << "Null GNSS playback data." << std::endl;
    return current__gnss_data;
  }


  // if (!current__gnss_data_json["original__gnss_data"].is_null()) {
  //     if (!current__gnss_data_json["original__gnss_data"]["fix"].is_null()) {
  //         if (!current__gnss_data_json["original__gnss_data"]["fix"]["status"].is_null())
  //             std::cout << "GNSS info: " << gps_mode2str(current__gnss_data_json["original__gnss_data"]["fix"]["status"]) << " " << current__gnss_data_json["longitude_std"] << " " << current__gnss_data_json["altitude_std"] << std::endl;
  //     }
  // }

  auto gnss_timestamp = current__gnss_data_json["ts"].get<uint64_t>();
  // Fill out timestamp:
  if (is_microseconds(gnss_timestamp)) {
    current__gnss_data.ts.setMicroseconds(gnss_timestamp);
  } else if (is_nanoseconds(gnss_timestamp)) {
    current__gnss_data.ts.setNanoseconds(gnss_timestamp);
  } else {
    std::cerr << "Warning: Invalid timestamp format from GNSS file" << std::endl;
  }

  // Fill out coordinates:
  current__gnss_data.setCoordinates(
    current__gnss_data_json["coordinates"]["latitude"].get<float>(),
    current__gnss_data_json["coordinates"]["longitude"].get<float>(),
    current__gnss_data_json["coordinates"]["altitude"].get<float>(),
    false);

  // Fill out default standard deviation:
  current__gnss_data.longitude_std = current__gnss_data_json["longitude_std"];
  current__gnss_data.latitude_std = current__gnss_data_json["latitude_std"];
  current__gnss_data.altitude_std = current__gnss_data_json["altitude_std"];
  // Fill out covariance [must be not null]
  std::array<double, 9> position_covariance;
  for (unsigned i = 0; i < 9; i++) {
    position_covariance[i] = 0.0;     // initialize empty covariance

  }
  // set covariance diagonal
  position_covariance[0] = current__gnss_data.longitude_std * current__gnss_data.longitude_std;
  position_covariance[1 * 3 + 1] = current__gnss_data.latitude_std *
    current__gnss_data.latitude_std;
  position_covariance[2 * 3 + 2] = current__gnss_data.altitude_std *
    current__gnss_data.altitude_std;
  current__gnss_data.position_covariance = position_covariance;

  return current__gnss_data;
}

sl::GNSSData GNSSReplay::getNextGNSSValue(uint64_t current_timestamp)
{
  sl::GNSSData current__gnss_data = getGNSSData(_gnss_data, _current_gnss_idx);

  if (current__gnss_data.ts.data_ns == 0) {
    return current__gnss_data;
  }

  if (current__gnss_data.ts.data_ns > current_timestamp) {
    current__gnss_data.ts.data_ns = 0;
    return current__gnss_data;
  }

  sl::GNSSData last_data;
  int step = 1;
  while (1) {
    last_data = current__gnss_data;
    int diff_last = current_timestamp - current__gnss_data.ts.data_ns;
    current__gnss_data = getGNSSData(_gnss_data, _current_gnss_idx + step++);
    if (current__gnss_data.ts.data_ns == 0) {   //error / end of file
      break;
    }

    if (current__gnss_data.ts.data_ns > current_timestamp) {
      if ((current__gnss_data.ts.data_ns - current_timestamp) > diff_last) {     // keep last
        current__gnss_data = last_data;
      }
      break;
    }
    _current_gnss_idx++;
  }

  return current__gnss_data;
}

sl::FUSION_ERROR_CODE GNSSReplay::grab(sl::GNSSData & current_data, uint64_t current_timestamp)
{
  current_data.ts.data_ns = 0;

  if (current_timestamp > 0 && (current_timestamp > _last_cam_ts)) {
    current_data = getNextGNSSValue(current_timestamp);
  }

  if (current_data.ts.data_ns == _previous_ts) {
    current_data.ts.data_ns = 0;
  }

  _last_cam_ts = current_timestamp;

  if (current_data.ts.data_ns == 0) { // Invalid data
    return sl::FUSION_ERROR_CODE::FAILURE;
  }

  _previous_ts = current_data.ts.data_ns;
  return sl::FUSION_ERROR_CODE::SUCCESS;
}

}  // namespace sl_tools
