#include "cost_traversability.hpp"

namespace stereolabs
{
namespace cost_traversability
{
template<typename T>
T clamp(T const & v, T const & lo, T const & hi)
{
  return (v < lo) ? lo : ((v > hi) ? hi : v);
}

void initCostTraversibily(
  sl::Terrain & cost_terrain, float resolution, float range,
  float height_threshold)
{
  std::vector<sl::LayerName> layers;
  layers.push_back(TRAVERSABILITY_COST);
  layers.push_back(OCCUPANCY);
  layers.push_back(TRAVERSABILITY_COST_STEP);
  layers.push_back(TRAVERSABILITY_COST_SLOPE);
  layers.push_back(TRAVERSABILITY_COST_ROUGHNESS);

  auto range_cell = round(range / resolution);
  cost_terrain.init(resolution, range_cell, layers);
}

void computeCost(
  sl::Terrain & elevation_terrain, sl::Terrain & cost_terrain,
  const float grid_resolution, RobotParameters robot_parameters,
  TraversabilityParameters traversability_parameters)
{
  auto square_size_cost = robot_parameters.radius / grid_resolution;
  if (square_size_cost < 1) {
    square_size_cost = 1;
  }

  auto factor_step_ = traversability_parameters.step_weight / robot_parameters.step_max;
  auto factor_slope_ = traversability_parameters.slope_weight / robot_parameters.slope_max;
  auto factor_roughness_ = traversability_parameters.roughness_weight /
    robot_parameters.roughness_max;

  // Update only the recent one, and manage the border ?
  const float step_height_crit = robot_parameters.step_max;

  double reso_d = grid_resolution * 1.;

  sl::Timestamp ts_tmp_elevation, ts_tmp_cost;

  double a_rad = robot_parameters.radius * 1.;
  int nb_cells = (2. * a_rad) / reso_d;           // big agent with small grid size is heavier to compute

  const sl::float3 z_vector(0, 0, 1);

  auto chunks_idx = elevation_terrain.getAllValidChunk();

  // for each chunk
  for (auto chunk_id : chunks_idx) {

    auto & chunk_elevation = elevation_terrain.getChunk(chunk_id);
    auto & layer_height = chunk_elevation.getLayer(sl::LayerName::ELEVATION);

    auto & chunk_cost = cost_terrain.getChunk(chunk_id);

    chunk_cost.getLayer(TRAVERSABILITY_COST).clear();
    chunk_cost.getLayer(OCCUPANCY).clear();
    chunk_cost.getLayer(TRAVERSABILITY_COST_STEP).clear();
    chunk_cost.getLayer(TRAVERSABILITY_COST_SLOPE).clear();
    chunk_cost.getLayer(TRAVERSABILITY_COST_ROUGHNESS).clear();

    auto & cost_data = chunk_cost.getLayer(TRAVERSABILITY_COST).getData();
    auto & occupancy_data = chunk_cost.getLayer(OCCUPANCY).getData();
    auto & cost_step_data = chunk_cost.getLayer(TRAVERSABILITY_COST_STEP).getData();
    auto & cost_slope_data = chunk_cost.getLayer(TRAVERSABILITY_COST_SLOPE).getData();
    auto & cost_roughness_data = chunk_cost.getLayer(TRAVERSABILITY_COST_ROUGHNESS).getData();

    auto dim = chunk_elevation.getDimension();
    const int size_ = dim.getSize() * dim.getSize();

    auto & elevation_data = layer_height.getData();

    unsigned int idx_tmp;
    float x, y;
    for (unsigned int idx_current = 0; idx_current < size_; idx_current++) {
      const float ref_height = elevation_data[idx_current];
      if (std::isfinite(ref_height)) {
        dim.index2x_y(idx_current, x, y);
        // SLOPE
        std::vector<sl::float3> normals_tmp;
        normals_tmp.reserve(nb_cells * nb_cells);

        float max_diff_height = 0;

        double x_area_min = x - a_rad;
        double y_area_min = y - a_rad;

        for (int x_ = 0; x_ < nb_cells; x_++) {
          float x_v = x_area_min + (x_ * reso_d);
          for (int y_ = 0; y_ < nb_cells; y_++) {
            float y_v = y_area_min + (y_ * reso_d);

            float curr_height;
            if (dim.getIndex(x_v, y_v, idx_tmp) /*True = error*/) {
              // Probably chunk edges
              curr_height = elevation_terrain.readValue(sl::LayerName::ELEVATION, x_v, y_v);
            } else {
              curr_height = elevation_data[idx_tmp];
            }

            if (std::isfinite(curr_height)) {
              normals_tmp.emplace_back(x_v, y_v, curr_height);
              max_diff_height = std::max(max_diff_height, fabsf32(curr_height - ref_height));
            }
          }
        }

        sl::float3 normal, centroid, eigen_values;
        plane::compute_pca(normals_tmp, normal, centroid, eigen_values);

        float roughness = 0, slope = 0, step = 0, cost;

        if (max_diff_height > step_height_crit) {
          step = max_diff_height;
        }

        if (normals_tmp.size() >= 3) {                                                                                                                              // minimum points
          slope =
            acos(
            sl::float3::dot(
              normal,
              z_vector) / sqrt(normal.x * normal.x + normal.y * normal.y + normal.z * normal.z)) *
            57.295779513;                                                                                                                                           // the norm of z_vector is 1

        }
        if (slope > 90) {
          slope = 180.f - slope;
        }

        roughness = sqrt(eigen_values.z);                 // Standard deviation of fitted plane

        cost = clamp(
          (roughness * factor_roughness_ + slope * factor_slope_ + step * factor_step_) * 0.3f,
          0.f, 1.f);
        cost_data[idx_current] = cost;
        occupancy_data[idx_current] =
          (cost > traversability_parameters.occupancy_threshold) ? OCCUPIED_CELL : FREE_CELL;
        if (slope == 0) {
          slope = INVALID_CELL_DATA;
        }
        cost_slope_data[idx_current] = slope;
        cost_step_data[idx_current] = step;
        cost_roughness_data[idx_current] = roughness;
      } else {
        occupancy_data[idx_current] = UNKNOWN_CELL;
      }
    }
  }
}

static const sl::float3 clr_a(244, 242, 246);
static const sl::float3 clr_b(0, 0, 0);

// generate  a linear ColorMap to match ogl interpol
inline sl::uchar3 getColorMap(float value)
{
  auto new_clr = clr_a * value + clr_b * (1.f - value);
  return sl::uchar3(new_clr.b, new_clr.g, new_clr.r);
}

void normalization(sl::Terrain & cost_terrain, sl::LayerName layer, sl::Mat & view)
{
  sl::Mat cost;
  auto cost_mat = cost_terrain.retrieveView(cost, sl::MAT_TYPE::F32_C1, layer);
  auto cost_res = cost.getResolution();
  view.alloc(cost_res, sl::MAT_TYPE::U8_C3);

  for (int y = 0; y < cost_res.height; y++) {

    auto ptr_cost = cost.getPtr<float>() + y * cost.getStep();
    auto ptr_view = view.getPtr<sl::uchar3>() + y * view.getStep();

    for (int x = 0; x < cost_res.width; x++) {
      float cost = ptr_cost[x];
      if (std::isfinite(cost)) {
        ptr_view[x] = getColorMap(cost);
      } else {
        ptr_view[x] = sl::uchar3(22, 22, 22);
      }
    }
  }
}
}
}
