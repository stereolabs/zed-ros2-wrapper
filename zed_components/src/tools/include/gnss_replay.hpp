#ifndef GPSD_Replay_H
#define GPSD_Replay_H

#include <iostream>
#include <sl/Fusion.hpp>

#include "json.hpp"

namespace sl_tools
{

/**
 * @brief GNSSReplay is a common interface that read GNSS saved data
 */
class GNSSReplay
{
public:
  explicit GNSSReplay(const std::shared_ptr<sl::Camera> & zed);
  ~GNSSReplay();
  bool initialize();
  void close();

  sl::FUSION_ERROR_CODE grab(sl::GNSSData & current_data, uint64_t current_timestamp);

protected:
  sl::GNSSData getNextGNSSValue(uint64_t current_timestamp);

private:
  unsigned _current_gnss_idx = 0;
  unsigned long long _previous_ts = 0;
  unsigned long long _last_cam_ts = 0;
  nlohmann::json _gnss_data;

  std::shared_ptr<sl::Camera> _zed;
};

}  // namespace sl_tools

#endif
