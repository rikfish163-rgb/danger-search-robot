#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <limits>
#include <mutex>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

namespace
{
constexpr double kPi = 3.14159265358979323846;

double clampValue(const double value, const double lower, const double upper)
{
  return std::max(lower, std::min(value, upper));
}

double probabilityToLogOdds(const double probability)
{
  const double p = clampValue(probability, 1e-6, 1.0 - 1e-6);
  return std::log(p / (1.0 - p));
}

double logOddsToProbability(const double log_odds)
{
  if (log_odds >= 0.0)
  {
    const double exponential = std::exp(-log_odds);
    return 1.0 / (1.0 + exponential);
  }
  const double exponential = std::exp(log_odds);
  return exponential / (1.0 + exponential);
}

std::string defaultHomePath(const std::string& suffix)
{
  const char* home = std::getenv("HOME");
  if (home == nullptr || std::string(home).empty())
  {
    return suffix;
  }
  return std::string(home) + suffix;
}

template<typename T>
bool writeBinaryValue(std::ofstream& output, const T& value)
{
  output.write(reinterpret_cast<const char*>(&value), sizeof(T));
  return output.good();
}

template<typename T>
bool readBinaryValue(std::ifstream& input, T& value)
{
  input.read(reinterpret_cast<char*>(&value), sizeof(T));
  return input.good();
}

template<typename T>
bool writeBinaryVector(std::ofstream& output, const std::vector<T>& values)
{
  if (!values.empty())
  {
    output.write(
      reinterpret_cast<const char*>(values.data()),
      static_cast<std::streamsize>(sizeof(T) * values.size()));
  }
  return output.good();
}

template<typename T>
bool readBinaryVector(std::ifstream& input, std::vector<T>& values)
{
  if (!values.empty())
  {
    input.read(
      reinterpret_cast<char*>(values.data()),
      static_cast<std::streamsize>(sizeof(T) * values.size()));
  }
  return input.good();
}
}  // namespace

class NearestAzimuthProjectionNode
{
public:
  NearestAzimuthProjectionNode()
    : nh_(), pnh_("~"), tf_buffer_(ros::Duration(30.0)), tf_listener_(tf_buffer_)
  {
    loadParameters();
    validateParameters();
    initializeGrid();
    initializeFloorPersistence();

    map_publisher_ = nh_.advertise<nav_msgs::OccupancyGrid>(output_topic_, 1, true);
    confirmed_map_publisher_ =
      nh_.advertise<nav_msgs::OccupancyGrid>(confirmed_output_topic_, 1, true);
    status_publisher_ = pnh_.advertise<std_msgs::String>("status", 1, true);
    current_floor_publisher_ = pnh_.advertise<std_msgs::Int32>("current_floor", 1, true);

    clear_service_ = pnh_.advertiseService(
      "clear_map", &NearestAzimuthProjectionNode::clearMapCallback, this);
    clear_candidates_service_ = pnh_.advertiseService(
      "clear_candidates", &NearestAzimuthProjectionNode::clearCandidatesCallback, this);
    updates_service_ = pnh_.advertiseService(
      "set_updates_enabled", &NearestAzimuthProjectionNode::setUpdatesEnabledCallback, this);
    save_floor_service_ = pnh_.advertiseService(
      "save_current_floor", &NearestAzimuthProjectionNode::saveCurrentFloorCallback, this);
    sync_floor_service_ = pnh_.advertiseService(
      "sync_floor_state", &NearestAzimuthProjectionNode::syncFloorStateCallback, this);

    cloud_subscriber_ = nh_.subscribe(
      input_cloud_topic_, 2, &NearestAzimuthProjectionNode::cloudCallback, this,
      ros::TransportHints().tcpNoDelay());
    publish_timer_ = nh_.createTimer(
      ros::Duration(1.0 / publish_rate_),
      &NearestAzimuthProjectionNode::publishTimerCallback, this);
    floor_state_timer_ = nh_.createTimer(
      ros::Duration(1.0 / floor_state_poll_rate_),
      &NearestAzimuthProjectionNode::floorStateTimerCallback, this);
    autosave_timer_ = nh_.createTimer(
      ros::Duration(floor_autosave_period_),
      &NearestAzimuthProjectionNode::autosaveTimerCallback, this);

    publishCurrentFloor();
    publishStatus("WAITING_FOR_CLOUD");
    ROS_INFO_STREAM(
      "nearest_azimuth_projection_node started\n"
      << "  input: " << input_cloud_topic_ << " -> " << output_topic_ << "\n"
      << "  confirmed map: " << confirmed_output_topic_ << "\n"
      << "  frame: " << global_frame_ << ", sensor: " << sensor_frame_ << "\n"
      << "  active floor: " << current_floor_ << "\n"
      << "  floor state: " << floor_state_file_ << "\n"
      << "  floor maps: " << floor_maps_root_ << "\n"
      << "  angular bins: " << angular_bin_count_ << " ("
      << azimuth_bin_degrees_ << " deg requested)\n"
      << "  filtered-return rays: "
      << (clip_filtered_rays_to_height_band_ ? "height-band clipped" : "full observed ray")
      << "\n"
      << "  map: " << map_width_m_ << " x " << map_height_m_
      << " m at " << resolution_ << " m/cell");
  }

  ~NearestAzimuthProjectionNode()
  {
    if (!floor_persistence_enabled_)
    {
      return;
    }
    std::lock_guard<std::mutex> lock(grid_mutex_);
    if (!saveFloorStateLocked(current_floor_))
    {
      ROS_ERROR("Failed to save floor %d map while shutting down", current_floor_);
    }
  }

private:
  struct AzimuthObservation
  {
    bool obstacle_valid{false};
    double obstacle_range_squared{std::numeric_limits<double>::infinity()};
    int obstacle_cell_x{0};
    int obstacle_cell_y{0};

    bool free_valid{false};
    double free_range_squared{0.0};
    int free_cell_x{0};
    int free_cell_y{0};
  };

  void loadParameters()
  {
    pnh_.param<std::string>("input_cloud_topic", input_cloud_topic_, "/cloud_registered_body");
    pnh_.param<std::string>("output_topic", output_topic_, "/map_raw");
    pnh_.param<std::string>("confirmed_output_topic", confirmed_output_topic_, "/map_confirmed");
    pnh_.param<std::string>("global_frame", global_frame_, "map_level");
    pnh_.param<std::string>("sensor_frame", sensor_frame_, "body");
    pnh_.param<std::string>("height_filter_frame", height_filter_frame_, "map_level");

    pnh_.param("resolution", resolution_, 0.10);
    pnh_.param("map_width", map_width_m_, 60.0);
    pnh_.param("map_height", map_height_m_, 60.0);
    pnh_.param("origin_x", origin_x_, -30.0);
    pnh_.param("origin_y", origin_y_, -30.0);
    pnh_.param("min_obstacle_height", min_obstacle_height_, -0.30);
    pnh_.param("max_obstacle_height", max_obstacle_height_, 1.50);
    pnh_.param("use_sensor_relative_height", use_sensor_relative_height_, true);
    pnh_.param("clip_filtered_rays_to_height_band",
               clip_filtered_rays_to_height_band_, false);
    pnh_.param("min_range", min_range_, 0.30);
    pnh_.param("max_range", max_range_, 12.0);

    pnh_.param("hit_probability", hit_probability_, 0.72);
    pnh_.param("miss_probability", miss_probability_, 0.49);
    pnh_.param("min_probability", min_probability_, 0.12);
    pnh_.param("max_probability", max_probability_, 0.97);
    pnh_.param("free_threshold", free_threshold_, 0.30);
    pnh_.param("occupied_threshold", occupied_threshold_, 0.65);
    pnh_.param("occupied_latch_probability", occupied_latch_probability_, 0.70);
    pnh_.param("occupied_clear_probability", occupied_clear_probability_, 0.55);
    pnh_.param("occupied_miss_scale", occupied_miss_scale_, 1.0);
    pnh_.param("confirmation_hit_scans", confirmation_hit_scans_, 3);
    pnh_.param("confirmation_max_scan_gap", confirmation_max_scan_gap_, 20);
    pnh_.param("confirmation_min_neighbors", confirmation_min_neighbors_, 1);
    pnh_.param("confirmation_spatial_tolerance_cells",
               confirmation_spatial_tolerance_cells_, 1);

    pnh_.param("azimuth_bin_degrees", azimuth_bin_degrees_, 0.25);
    pnh_.param("endpoint_guard_radius", endpoint_guard_radius_, 0.10);
    pnh_.param("free_ray_endpoint_margin_cells", endpoint_margin_cells_, 0);
    pnh_.param("point_stride", point_stride_, 1);
    pnh_.param("max_rays_per_scan", max_rays_per_scan_, 5000);
    pnh_.param("publish_rate", publish_rate_, 2.0);
    pnh_.param("tf_timeout", tf_timeout_, 0.20);
    pnh_.param("tf_latest_fallback_max_skew", tf_latest_fallback_max_skew_, 0.05);
    pnh_.param("reject_zero_stamp", reject_zero_stamp_, true);

    pnh_.param("floor_persistence_enabled", floor_persistence_enabled_, true);
    pnh_.param("clear_all_floor_maps", clear_all_floor_maps_, false);
    pnh_.param<std::string>(
      "floor_state_file", floor_state_file_,
      defaultHomePath("/catkin_ws/results/floor_state.json"));
    pnh_.param<std::string>(
      "floor_maps_root", floor_maps_root_,
      defaultHomePath("/catkin_ws/results/floors"));
    pnh_.param("initial_floor", initial_floor_, 0);
    pnh_.param("floor_count", floor_count_, 3);
    pnh_.param("floor_state_poll_rate", floor_state_poll_rate_, 2.0);
    pnh_.param("floor_autosave_period", floor_autosave_period_, 5.0);
  }

  void validateParameters()
  {
    if (resolution_ <= 0.0 || map_width_m_ <= 0.0 || map_height_m_ <= 0.0)
    {
      throw std::runtime_error("map dimensions and resolution must be positive");
    }
    if (min_obstacle_height_ >= max_obstacle_height_ || min_range_ < 0.0 ||
        max_range_ <= min_range_)
    {
      throw std::runtime_error("height or range limits are invalid");
    }
    const auto valid_probability = [](const double probability) {
      return probability > 0.0 && probability < 1.0;
    };
    if (!valid_probability(hit_probability_) || !valid_probability(miss_probability_) ||
        !valid_probability(min_probability_) || !valid_probability(max_probability_) ||
        !valid_probability(free_threshold_) || !valid_probability(occupied_threshold_) ||
        !valid_probability(occupied_latch_probability_) ||
        !valid_probability(occupied_clear_probability_))
    {
      throw std::runtime_error("probability parameters must be in (0, 1)");
    }
    if (min_probability_ >= max_probability_ || free_threshold_ >= occupied_threshold_ ||
        occupied_clear_probability_ >= occupied_latch_probability_ ||
        occupied_miss_scale_ <= 0.0 || occupied_miss_scale_ > 1.0 ||
        azimuth_bin_degrees_ <= 0.0 || azimuth_bin_degrees_ > 360.0)
    {
      throw std::runtime_error("probability hysteresis or angular-bin parameters are invalid");
    }

    point_stride_ = std::max(1, point_stride_);
    max_rays_per_scan_ = std::max(1, max_rays_per_scan_);
    endpoint_margin_cells_ = std::max(0, endpoint_margin_cells_);
    endpoint_guard_radius_ = std::max(0.0, endpoint_guard_radius_);
    confirmation_hit_scans_ = std::max(1, confirmation_hit_scans_);
    confirmation_max_scan_gap_ = std::max(1, confirmation_max_scan_gap_);
    confirmation_min_neighbors_ = std::max(0, std::min(8, confirmation_min_neighbors_));
    confirmation_spatial_tolerance_cells_ =
      std::max(0, std::min(5, confirmation_spatial_tolerance_cells_));
    publish_rate_ = publish_rate_ > 0.0 ? publish_rate_ : 1.0;
    tf_timeout_ = tf_timeout_ > 0.0 ? tf_timeout_ : 0.20;
    tf_latest_fallback_max_skew_ =
      tf_latest_fallback_max_skew_ > 0.0 ? tf_latest_fallback_max_skew_ : 0.05;
    if (floor_count_ <= 0 || initial_floor_ < 0 || initial_floor_ >= floor_count_)
    {
      throw std::runtime_error("floor_count or initial_floor is invalid");
    }
    floor_state_poll_rate_ =
      floor_state_poll_rate_ > 0.0 ? floor_state_poll_rate_ : 1.0;
    floor_autosave_period_ =
      floor_autosave_period_ > 0.0 ? floor_autosave_period_ : 5.0;

    angular_bin_count_ = std::max(
      1, std::min(max_rays_per_scan_,
                  static_cast<int>(std::ceil(360.0 / azimuth_bin_degrees_))));
    actual_bin_radians_ = 2.0 * kPi / static_cast<double>(angular_bin_count_);
    guard_radius_cells_ = static_cast<int>(std::ceil(endpoint_guard_radius_ / resolution_));

    log_odds_hit_ = probabilityToLogOdds(hit_probability_);
    log_odds_miss_ = probabilityToLogOdds(miss_probability_);
    log_odds_min_ = probabilityToLogOdds(min_probability_);
    log_odds_max_ = probabilityToLogOdds(max_probability_);
  }

  void initializeGrid()
  {
    width_cells_ = static_cast<int>(std::ceil(map_width_m_ / resolution_));
    height_cells_ = static_cast<int>(std::ceil(map_height_m_ / resolution_));
    const std::size_t count =
      static_cast<std::size_t>(width_cells_) * static_cast<std::size_t>(height_cells_);
    log_odds_.assign(count, 0.0F);
    observed_.assign(count, 0U);
    occupied_latched_.assign(count, 0U);
    candidate_hit_scans_.assign(count, 0U);
    candidate_last_scan_.assign(count, 0U);
    confirmed_occupied_.assign(count, 0U);
    scan_sequence_ = 0U;
    updates_enabled_ = true;
    latest_stamp_ = ros::Time(0);
  }

  void resetGridStateLocked()
  {
    std::fill(log_odds_.begin(), log_odds_.end(), 0.0F);
    std::fill(observed_.begin(), observed_.end(), 0U);
    std::fill(occupied_latched_.begin(), occupied_latched_.end(), 0U);
    std::fill(candidate_hit_scans_.begin(), candidate_hit_scans_.end(), 0U);
    std::fill(candidate_last_scan_.begin(), candidate_last_scan_.end(), 0U);
    std::fill(confirmed_occupied_.begin(), confirmed_occupied_.end(), 0U);
    scan_sequence_ = 0U;
    latest_stamp_ = ros::Time(0);
  }

  bool ensureDirectoryRecursive(const std::string& path) const
  {
    if (path.empty())
    {
      return false;
    }

    std::string current;
    if (path.front() == '/')
    {
      current = "/";
    }

    std::stringstream stream(path);
    std::string component;
    while (std::getline(stream, component, '/'))
    {
      if (component.empty())
      {
        continue;
      }
      if (!current.empty() && current.back() != '/')
      {
        current += "/";
      }
      current += component;
      if (::mkdir(current.c_str(), 0755) != 0 && errno != EEXIST)
      {
        ROS_ERROR("Cannot create directory %s: %s", current.c_str(), std::strerror(errno));
        return false;
      }
    }
    return true;
  }

  std::string floorDirectory(const int floor) const
  {
    return floor_maps_root_ + "/floor_" + std::to_string(floor);
  }

  std::string floorMapStatePath(const int floor) const
  {
    return floorDirectory(floor) + "/projection_state.bin";
  }

  bool readRequestedFloor(int& floor) const
  {
    std::ifstream input(floor_state_file_);
    if (!input)
    {
      return false;
    }

    std::ostringstream buffer;
    buffer << input.rdbuf();
    const std::string payload = buffer.str();
    const std::regex pattern("\\\"current_floor\\\"\\s*:\\s*(-?[0-9]+)");
    std::smatch match;
    if (!std::regex_search(payload, match, pattern))
    {
      ROS_ERROR_THROTTLE(
        2.0, "Cannot parse current_floor from %s", floor_state_file_.c_str());
      return false;
    }

    try
    {
      floor = std::stoi(match[1].str());
    }
    catch (const std::exception& error)
    {
      ROS_ERROR_THROTTLE(2.0, "Invalid current_floor in %s: %s",
                         floor_state_file_.c_str(), error.what());
      return false;
    }

    if (floor < 0 || floor >= floor_count_)
    {
      ROS_ERROR_THROTTLE(
        2.0, "Requested floor %d is outside [0, %d]", floor, floor_count_ - 1);
      return false;
    }
    return true;
  }


  void clearAllFloorMaps()
  {
    for (int floor = 0; floor < floor_count_; ++floor)
    {
      const std::string directory = floorDirectory(floor);

      std::string command = "rm -rf " + directory;

      if (std::system(command.c_str()) == 0)
      {
        ROS_WARN("Cleared saved map directory: %s",
                 directory.c_str());
      }
      else
      {
        ROS_ERROR("Failed to clear directory: %s",
                  directory.c_str());
      }
    }
  }


  void initializeFloorPersistence()
  {
    current_floor_ = initial_floor_;

    if (clear_all_floor_maps_)
    {
      ROS_WARN("clear_all_floor_maps=true, clearing all floor maps");

      clearAllFloorMaps();

      clear_all_floor_maps_ = false;
    }

    if (!floor_persistence_enabled_)
    {
      ROS_WARN("Per-floor map persistence is disabled");
      return;
    }

    if (!ensureDirectoryRecursive(floor_maps_root_))
    {
      throw std::runtime_error("cannot create floor_maps_root");
    }

    int requested_floor = initial_floor_;
    if (readRequestedFloor(requested_floor))
    {
      current_floor_ = requested_floor;
    }
    else
    {
      ROS_WARN(
        "Cannot read %s at startup; using initial_floor=%d",
        floor_state_file_.c_str(), initial_floor_);
    }

    if (!loadFloorStateLocked(current_floor_))
    {
      ROS_INFO("Starting floor %d with a new empty 2D map", current_floor_);
    }
  }

  bool saveFloorStateLocked(const int floor)
  {
    if (!floor_persistence_enabled_)
    {
      return true;
    }
    if (floor < 0 || floor >= floor_count_)
    {
      ROS_ERROR("Refusing to save invalid floor %d", floor);
      return false;
    }

    const std::string directory = floorDirectory(floor);
    if (!ensureDirectoryRecursive(directory))
    {
      return false;
    }

    const std::string destination = floorMapStatePath(floor);
    const std::string temporary =
      destination + ".tmp." + std::to_string(static_cast<long long>(::getpid()));
    std::ofstream output(temporary, std::ios::binary | std::ios::trunc);
    if (!output)
    {
      ROS_ERROR("Cannot open map state for writing: %s", temporary.c_str());
      return false;
    }

    const char magic[8] = {'D', 'S', 'R', '2', 'D', 'M', 'A', 'P'};
    output.write(magic, sizeof(magic));
    const std::uint32_t version = 1U;
    const std::int32_t width = width_cells_;
    const std::int32_t height = height_cells_;
    const std::uint64_t cell_count =
      static_cast<std::uint64_t>(log_odds_.size());
    const std::uint32_t stamp_sec = latest_stamp_.sec;
    const std::uint32_t stamp_nsec = latest_stamp_.nsec;

    const bool ok =
      writeBinaryValue(output, version) &&
      writeBinaryValue(output, width) &&
      writeBinaryValue(output, height) &&
      writeBinaryValue(output, resolution_) &&
      writeBinaryValue(output, map_width_m_) &&
      writeBinaryValue(output, map_height_m_) &&
      writeBinaryValue(output, origin_x_) &&
      writeBinaryValue(output, origin_y_) &&
      writeBinaryValue(output, cell_count) &&
      writeBinaryValue(output, scan_sequence_) &&
      writeBinaryValue(output, stamp_sec) &&
      writeBinaryValue(output, stamp_nsec) &&
      writeBinaryVector(output, log_odds_) &&
      writeBinaryVector(output, observed_) &&
      writeBinaryVector(output, occupied_latched_) &&
      writeBinaryVector(output, candidate_hit_scans_) &&
      writeBinaryVector(output, candidate_last_scan_) &&
      writeBinaryVector(output, confirmed_occupied_);

    output.flush();
    const bool completed = ok && output.good();
    output.close();
    if (!completed)
    {
      ROS_ERROR("Failed while writing map state: %s", temporary.c_str());
      std::remove(temporary.c_str());
      return false;
    }

    if (std::rename(temporary.c_str(), destination.c_str()) != 0)
    {
      ROS_ERROR("Cannot commit map state %s: %s",
                destination.c_str(), std::strerror(errno));
      std::remove(temporary.c_str());
      return false;
    }

    ROS_INFO_THROTTLE(
      5.0, "Saved floor %d map: candidates=%zu, confirmed=%zu, path=%s",
      floor, candidateCount(), confirmedCount(), destination.c_str());
    return true;
  }

  bool loadFloorStateLocked(const int floor)
  {
    const std::string source = floorMapStatePath(floor);
    std::ifstream input(source, std::ios::binary);
    if (!input)
    {
      return false;
    }

    char magic[8] = {};
    input.read(magic, sizeof(magic));
    const char expected_magic[8] = {'D', 'S', 'R', '2', 'D', 'M', 'A', 'P'};
    if (!input.good() || std::memcmp(magic, expected_magic, sizeof(magic)) != 0)
    {
      ROS_ERROR("Invalid floor map file header: %s", source.c_str());
      return false;
    }

    std::uint32_t version = 0U;
    std::int32_t width = 0;
    std::int32_t height = 0;
    double resolution = 0.0;
    double map_width = 0.0;
    double map_height = 0.0;
    double origin_x = 0.0;
    double origin_y = 0.0;
    std::uint64_t cell_count = 0U;
    std::uint64_t scan_sequence = 0U;
    std::uint32_t stamp_sec = 0U;
    std::uint32_t stamp_nsec = 0U;

    const bool metadata_ok =
      readBinaryValue(input, version) &&
      readBinaryValue(input, width) &&
      readBinaryValue(input, height) &&
      readBinaryValue(input, resolution) &&
      readBinaryValue(input, map_width) &&
      readBinaryValue(input, map_height) &&
      readBinaryValue(input, origin_x) &&
      readBinaryValue(input, origin_y) &&
      readBinaryValue(input, cell_count) &&
      readBinaryValue(input, scan_sequence) &&
      readBinaryValue(input, stamp_sec) &&
      readBinaryValue(input, stamp_nsec);

    const std::size_t expected_count = log_odds_.size();
    const bool geometry_matches =
      version == 1U &&
      width == width_cells_ &&
      height == height_cells_ &&
      cell_count == static_cast<std::uint64_t>(expected_count) &&
      std::abs(resolution - resolution_) < 1e-9 &&
      std::abs(map_width - map_width_m_) < 1e-9 &&
      std::abs(map_height - map_height_m_) < 1e-9 &&
      std::abs(origin_x - origin_x_) < 1e-9 &&
      std::abs(origin_y - origin_y_) < 1e-9;

    if (!metadata_ok || !geometry_matches)
    {
      ROS_ERROR(
        "Floor %d map geometry/version does not match current configuration: %s",
        floor, source.c_str());
      return false;
    }

    std::vector<float> log_odds(expected_count);
    std::vector<std::uint8_t> observed(expected_count);
    std::vector<std::uint8_t> occupied_latched(expected_count);
    std::vector<std::uint8_t> candidate_hit_scans(expected_count);
    std::vector<std::uint64_t> candidate_last_scan(expected_count);
    std::vector<std::uint8_t> confirmed_occupied(expected_count);

    const bool data_ok =
      readBinaryVector(input, log_odds) &&
      readBinaryVector(input, observed) &&
      readBinaryVector(input, occupied_latched) &&
      readBinaryVector(input, candidate_hit_scans) &&
      readBinaryVector(input, candidate_last_scan) &&
      readBinaryVector(input, confirmed_occupied);
    if (!data_ok)
    {
      ROS_ERROR("Floor %d map file is truncated: %s", floor, source.c_str());
      return false;
    }

    log_odds_.swap(log_odds);
    observed_.swap(observed);
    occupied_latched_.swap(occupied_latched);
    candidate_hit_scans_.swap(candidate_hit_scans);
    candidate_last_scan_.swap(candidate_last_scan);
    confirmed_occupied_.swap(confirmed_occupied);
    scan_sequence_ = scan_sequence;
    latest_stamp_ = ros::Time(stamp_sec, stamp_nsec);

    ROS_INFO(
      "Loaded floor %d map: candidates=%zu, confirmed=%zu, path=%s",
      floor, candidateCount(), confirmedCount(), source.c_str());
    return true;
  }

  bool switchFloorLocked(const int target_floor)
  {
    if (target_floor == current_floor_)
    {
      return true;
    }
    if (target_floor < 0 || target_floor >= floor_count_)
    {
      ROS_ERROR("Cannot switch to invalid floor %d", target_floor);
      return false;
    }

    const int previous_floor = current_floor_;
    if (!saveFloorStateLocked(previous_floor))
    {
      ROS_ERROR(
        "Floor switch %d -> %d cancelled because the current map could not be saved",
        previous_floor, target_floor);
      return false;
    }

    const bool updates_were_enabled = updates_enabled_;
    current_floor_ = target_floor;
    ++floor_generation_;
    resetGridStateLocked();
    const bool restored = loadFloorStateLocked(current_floor_);
    updates_enabled_ = updates_were_enabled;
    latest_stamp_ = ros::Time::now();

    ROS_INFO(
      "Switched projected map floor %d -> %d (%s)",
      previous_floor, current_floor_, restored ? "restored" : "new empty map");
    return true;
  }

  void publishCurrentFloor()
  {
    std_msgs::Int32 message;
    message.data = current_floor_;
    current_floor_publisher_.publish(message);
  }

  bool synchronizeFloorFromFile()
  {
    if (!floor_persistence_enabled_)
    {
      return true;
    }

    int requested_floor = current_floor_;
    if (!readRequestedFloor(requested_floor))
    {
      return false;
    }

    std::lock_guard<std::mutex> lock(grid_mutex_);
    if (!switchFloorLocked(requested_floor))
    {
      return false;
    }

    publishCurrentFloor();
    map_publisher_.publish(createMapMessage(false));
    confirmed_map_publisher_.publish(createMapMessage(true));
    return true;
  }

  void floorStateTimerCallback(const ros::TimerEvent&)
  {
    if (!floor_persistence_enabled_)
    {
      return;
    }

    int requested_floor = initial_floor_;
    if (!readRequestedFloor(requested_floor))
    {
      return;
    }

    std::lock_guard<std::mutex> lock(grid_mutex_);
    if (requested_floor == current_floor_)
    {
      return;
    }
    if (!switchFloorLocked(requested_floor))
    {
      publishStatus("FLOOR_SWITCH_FAILED");
      return;
    }

    publishCurrentFloor();
    map_publisher_.publish(createMapMessage(false));
    confirmed_map_publisher_.publish(createMapMessage(true));
    publishStatus("FLOOR_SWITCHED");
  }

  void autosaveTimerCallback(const ros::TimerEvent&)
  {
    if (!floor_persistence_enabled_)
    {
      return;
    }

    std::lock_guard<std::mutex> lock(grid_mutex_);
    saveFloorStateLocked(current_floor_);
  }

  bool saveCurrentFloorCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
  {
    std::lock_guard<std::mutex> lock(grid_mutex_);
    const bool saved = saveFloorStateLocked(current_floor_);
    publishStatus(saved ? "FLOOR_SAVED" : "FLOOR_SAVE_FAILED");
    return saved;
  }

  bool syncFloorStateCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
  {
    const bool synchronized = synchronizeFloorFromFile();
    publishStatus(synchronized ? "FLOOR_SYNCED" : "FLOOR_SYNC_FAILED");
    return synchronized;
  }

  bool lookupTransform(const std::string& target, const std::string& source,
                       const ros::Time& stamp, tf2::Transform& output)
  {
    if (target == source)
    {
      output.setIdentity();
      return true;
    }
    try
    {
      const geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(
        target, source, stamp, ros::Duration(tf_timeout_));
      tf2::fromMsg(transform.transform, output);
      return true;
    }
    catch (const tf2::TransformException& error)
    {
      // A simulated sensor and its TF broadcaster can be separated by a few
      // milliseconds at startup.  An exact-time lookup then fails with
      // "extrapolation into the past" even though a current transform is
      // already available.  Use the latest transform only when the skew is
      // explicitly bounded; stale clouds must still fail closed.
      if (!stamp.isZero())
      {
        try
        {
          const geometry_msgs::TransformStamped latest = tf_buffer_.lookupTransform(
            target, source, ros::Time(0), ros::Duration(tf_timeout_));
          const double skew = (latest.header.stamp - stamp).toSec();
          if (std::isfinite(skew) && std::abs(skew) <= tf_latest_fallback_max_skew_)
          {
            tf2::fromMsg(latest.transform, output);
            ROS_WARN_THROTTLE(
              5.0,
              "Using latest TF (%s <- %s) for cloud timestamp skew %.3fs",
              target.c_str(), source.c_str(), skew);
            return true;
          }
        }
        catch (const tf2::TransformException&)
        {
          // Preserve the original exact-time error below.
        }
      }
      ROS_WARN_THROTTLE(2.0, "TF lookup failed (%s <- %s): %s", target.c_str(),
                        source.c_str(), error.what());
      return false;
    }
  }

  bool worldToCell(const double x, const double y, int& cell_x, int& cell_y) const
  {
    cell_x = static_cast<int>(std::floor((x - origin_x_) / resolution_));
    cell_y = static_cast<int>(std::floor((y - origin_y_) / resolution_));
    return cell_x >= 0 && cell_x < width_cells_ && cell_y >= 0 && cell_y < height_cells_;
  }

  int flattenIndex(const int x, const int y) const
  {
    return y * width_cells_ + x;
  }

  void addRayFreeCells(const int start_x, const int start_y, const int end_x,
                       const int end_y, const bool include_endpoint,
                       std::unordered_set<int>& free_cells) const
  {
    int x = start_x;
    int y = start_y;
    const int delta_x = std::abs(end_x - start_x);
    const int step_x = start_x < end_x ? 1 : -1;
    const int delta_y = -std::abs(end_y - start_y);
    const int step_y = start_y < end_y ? 1 : -1;
    int error = delta_x + delta_y;
    std::vector<int> ray;

    while (!(x == end_x && y == end_y))
    {
      if (x >= 0 && x < width_cells_ && y >= 0 && y < height_cells_)
      {
        ray.push_back(flattenIndex(x, y));
      }
      const int doubled_error = 2 * error;
      if (doubled_error >= delta_y)
      {
        error += delta_y;
        x += step_x;
      }
      if (doubled_error <= delta_x)
      {
        error += delta_x;
        y += step_y;
      }
    }

    if (include_endpoint &&
        end_x >= 0 && end_x < width_cells_ && end_y >= 0 && end_y < height_cells_)
    {
      ray.push_back(flattenIndex(end_x, end_y));
    }

    const std::size_t margin = include_endpoint ? 0U : std::min(
      ray.size(), static_cast<std::size_t>(endpoint_margin_cells_));
    for (std::size_t index = 0; index + margin < ray.size(); ++index)
    {
      free_cells.insert(ray[index]);
    }
  }

  void addEndpointGuard(const int occupied_index, std::unordered_set<int>& guard) const
  {
    if (guard_radius_cells_ <= 0)
    {
      return;
    }
    const int center_y = occupied_index / width_cells_;
    const int center_x = occupied_index - center_y * width_cells_;
    for (int offset_y = -guard_radius_cells_; offset_y <= guard_radius_cells_; ++offset_y)
    {
      for (int offset_x = -guard_radius_cells_; offset_x <= guard_radius_cells_; ++offset_x)
      {
        if (offset_x * offset_x + offset_y * offset_y >
            guard_radius_cells_ * guard_radius_cells_)
        {
          continue;
        }
        const int x = center_x + offset_x;
        const int y = center_y + offset_y;
        if (x >= 0 && x < width_cells_ && y >= 0 && y < height_cells_)
        {
          guard.insert(flattenIndex(x, y));
        }
      }
    }
  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud)
  {
    std::uint64_t callback_floor_generation = 0U;
    {
      std::lock_guard<std::mutex> lock(grid_mutex_);
      if (!updates_enabled_)
      {
        publishStatus("FROZEN");
        return;
      }
      callback_floor_generation = floor_generation_;
    }
    if (cloud->header.frame_id.empty() || (reject_zero_stamp_ && cloud->header.stamp.isZero()))
    {
      ROS_WARN_THROTTLE(2.0, "Input cloud has an invalid frame or timestamp");
      publishStatus("INVALID_CLOUD_HEADER");
      return;
    }
    const ros::Time stamp = cloud->header.stamp.isZero() ? ros::Time(0) : cloud->header.stamp;
    tf2::Transform global_from_cloud;
    tf2::Transform height_from_cloud;
    tf2::Transform global_from_sensor;
    tf2::Transform height_from_sensor;
    const bool require_height_from_sensor =
      use_sensor_relative_height_ || clip_filtered_rays_to_height_band_;
    if (!lookupTransform(global_frame_, cloud->header.frame_id, stamp, global_from_cloud) ||
        !lookupTransform(height_filter_frame_, cloud->header.frame_id, stamp, height_from_cloud) ||
        !lookupTransform(global_frame_, sensor_frame_, stamp, global_from_sensor) ||
        (require_height_from_sensor &&
         !lookupTransform(height_filter_frame_, sensor_frame_, stamp, height_from_sensor)))
    {
      publishStatus("WAITING_FOR_TF");
      return;
    }

    const tf2::Vector3 sensor_origin = global_from_sensor.getOrigin();
    const double height_reference =
      use_sensor_relative_height_ ? height_from_sensor.getOrigin().z() : 0.0;
    int sensor_cell_x = 0;
    int sensor_cell_y = 0;
    if (!worldToCell(sensor_origin.x(), sensor_origin.y(), sensor_cell_x, sensor_cell_y))
    {
      ROS_WARN_THROTTLE(2.0, "Sensor (%.2f, %.2f) is outside the fixed map",
                        sensor_origin.x(), sensor_origin.y());
      publishStatus("SENSOR_OUTSIDE_MAP");
      return;
    }

    std::unordered_set<int> occupied_cells;
    std::vector<AzimuthObservation> azimuth_observations(
      static_cast<std::size_t>(angular_bin_count_));
    std::size_t usable_observation_count = 0U;
    try
    {
      sensor_msgs::PointCloud2ConstIterator<float> iterator_x(*cloud, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iterator_y(*cloud, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iterator_z(*cloud, "z");
      std::size_t point_index = 0;
      for (; iterator_x != iterator_x.end();
           ++iterator_x, ++iterator_y, ++iterator_z, ++point_index)
      {
        if (point_index % static_cast<std::size_t>(point_stride_) != 0U)
        {
          continue;
        }
        const double x = static_cast<double>(*iterator_x);
        const double y = static_cast<double>(*iterator_y);
        const double z = static_cast<double>(*iterator_z);
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
        {
          continue;
        }
        const tf2::Vector3 point_cloud(x, y, z);
        const tf2::Vector3 point_height = height_from_cloud * point_cloud;
        const double obstacle_height = point_height.z() - height_reference;
        const tf2::Vector3 point_global = global_from_cloud * point_cloud;
        const tf2::Vector3 difference = point_global - sensor_origin;
        const double range_squared = difference.length2();
        if (range_squared < min_range_ * min_range_)
        {
          continue;
        }

        const double planar_range_squared =
          difference.x() * difference.x() + difference.y() * difference.y();
        if (planar_range_squared <= std::numeric_limits<double>::epsilon())
        {
          continue;
        }

        double azimuth = std::atan2(difference.y(), difference.x());
        if (azimuth < 0.0)
        {
          azimuth += 2.0 * kPi;
        }
        int bin_index = static_cast<int>(std::floor(azimuth / actual_bin_radians_));
        bin_index = std::max(0, std::min(bin_index, angular_bin_count_ - 1));
        AzimuthObservation& observation =
          azimuth_observations[static_cast<std::size_t>(bin_index)];

        const bool inside_height_band =
          obstacle_height >= min_obstacle_height_ &&
          obstacle_height <= max_obstacle_height_;
        const bool inside_range = range_squared <= max_range_ * max_range_;

        // A point is an occupied endpoint only when it is both inside the
        // obstacle-height band and inside the configured mapping range.
        if (inside_height_band && inside_range)
        {
          int cell_x = 0;
          int cell_y = 0;
          if (!worldToCell(point_global.x(), point_global.y(), cell_x, cell_y))
          {
            continue;
          }

          occupied_cells.insert(flattenIndex(cell_x, cell_y));
          ++usable_observation_count;
          if (!observation.obstacle_valid ||
              planar_range_squared < observation.obstacle_range_squared)
          {
            observation.obstacle_valid = true;
            observation.obstacle_range_squared = planar_range_squared;
            observation.obstacle_cell_x = cell_x;
            observation.obstacle_cell_y = cell_y;
          }
          continue;
        }

        // Ground, ceiling and over-range returns are not occupied endpoints in
        // the 2-D layer, but they are real beam observations. By default their
        // full XY ray is free-space evidence; this prevents a pitched 3-D lidar
        // from leaving V-shaped unknown sectors after height filtering. A real
        // obstacle in the same azimuth still wins below. Over-range returns are
        // clipped so clearing never exceeds the configured sensor range.
        double ray_scale = 1.0;
        if (!inside_range)
        {
          ray_scale = max_range_ / std::sqrt(range_squared);
        }

        if (!inside_height_band && clip_filtered_rays_to_height_band_)
        {
          // Optional conservative mode: a ray above or below the 2-D obstacle
          // layer may pass over/under a real obstacle, so only retain the
          // segment from the sensor to the layer edge.
          const double sensor_obstacle_height =
            height_from_sensor.getOrigin().z() - height_reference;
          if (sensor_obstacle_height < min_obstacle_height_ ||
              sensor_obstacle_height > max_obstacle_height_)
          {
            continue;
          }
          const double height_delta = obstacle_height - sensor_obstacle_height;
          if (std::abs(height_delta) <= std::numeric_limits<double>::epsilon())
          {
            continue;
          }
          const double height_boundary = obstacle_height < min_obstacle_height_ ?
            min_obstacle_height_ : max_obstacle_height_;
          const double height_scale =
            (height_boundary - sensor_obstacle_height) / height_delta;
          if (height_scale <= 0.0)
          {
            continue;
          }
          ray_scale = std::min(ray_scale, clampValue(height_scale, 0.0, 1.0));
        }

        const tf2::Vector3 free_endpoint = sensor_origin + difference * ray_scale;
        int free_cell_x = 0;
        int free_cell_y = 0;
        if (!worldToCell(free_endpoint.x(), free_endpoint.y(),
                         free_cell_x, free_cell_y))
        {
          continue;
        }

        const double free_dx = free_endpoint.x() - sensor_origin.x();
        const double free_dy = free_endpoint.y() - sensor_origin.y();
        const double free_planar_range_squared = free_dx * free_dx + free_dy * free_dy;
        ++usable_observation_count;
        if (!observation.free_valid ||
            free_planar_range_squared > observation.free_range_squared)
        {
          observation.free_valid = true;
          observation.free_range_squared = free_planar_range_squared;
          observation.free_cell_x = free_cell_x;
          observation.free_cell_y = free_cell_y;
        }
      }
    }
    catch (const std::runtime_error& error)
    {
      ROS_ERROR_THROTTLE(2.0, "Cannot read PointCloud2 x/y/z fields: %s", error.what());
      publishStatus("INVALID_CLOUD_FIELDS");
      return;
    }

    if (usable_observation_count == 0U)
    {
      ROS_WARN_THROTTLE(2.0, "No cloud points provided usable obstacle/free-space observations");
      publishStatus("NO_VALID_POINTS");
      return;
    }

    std::unordered_set<int> free_cells;
    for (const AzimuthObservation& observation : azimuth_observations)
    {
      // A real obstacle is the conservative stopping point for its azimuth.
      // Otherwise use the farthest observed non-obstacle return. Completely
      // unobserved bins remain unknown; they are never cleared synthetically.
      if (observation.obstacle_valid)
      {
        addRayFreeCells(sensor_cell_x, sensor_cell_y,
                        observation.obstacle_cell_x, observation.obstacle_cell_y,
                        false, free_cells);
      }
      else if (observation.free_valid)
      {
        addRayFreeCells(sensor_cell_x, sensor_cell_y,
                        observation.free_cell_x, observation.free_cell_y,
                        true, free_cells);
      }
    }
    std::unordered_set<int> endpoint_guard;
    endpoint_guard.reserve(occupied_cells.size() * 5U);
    for (const int occupied_index : occupied_cells)
    {
      addEndpointGuard(occupied_index, endpoint_guard);
    }

    std::lock_guard<std::mutex> lock(grid_mutex_);
    if (callback_floor_generation != floor_generation_)
    {
      ROS_WARN("Discarding a cloud scan because the active floor changed during processing");
      publishStatus("FLOOR_CHANGED_DURING_SCAN");
      return;
    }
    ++scan_sequence_;
    for (const int free_index : free_cells)
    {
      if (occupied_cells.count(free_index) != 0U || endpoint_guard.count(free_index) != 0U)
      {
        continue;
      }
      const double miss_scale = occupied_latched_[free_index] != 0U ? occupied_miss_scale_ : 1.0;
      const double updated = clampValue(
        static_cast<double>(log_odds_[free_index]) + log_odds_miss_ * miss_scale,
        log_odds_min_, log_odds_max_);
      log_odds_[free_index] = static_cast<float>(updated);
      observed_[free_index] = 1U;
      if (occupied_latched_[free_index] != 0U &&
          logOddsToProbability(updated) <= occupied_clear_probability_)
      {
        occupied_latched_[free_index] = 0U;
      }
      if (confirmed_occupied_[free_index] == 0U && candidate_hit_scans_[free_index] > 0U)
      {
        --candidate_hit_scans_[free_index];
      }
    }
    for (const int occupied_index : occupied_cells)
    {
      const double updated = clampValue(
        static_cast<double>(log_odds_[occupied_index]) + log_odds_hit_,
        log_odds_min_, log_odds_max_);
      log_odds_[occupied_index] = static_cast<float>(updated);
      observed_[occupied_index] = 1U;
      if (logOddsToProbability(updated) >= occupied_latch_probability_)
      {
        occupied_latched_[occupied_index] = 1U;
      }

      if (confirmed_occupied_[occupied_index] == 0U)
      {
        const std::uint8_t previous_support = nearbyCandidateSupport(occupied_index);
        candidate_hit_scans_[occupied_index] = static_cast<std::uint8_t>(
          std::min(255, static_cast<int>(previous_support) + 1));
        candidate_last_scan_[occupied_index] = scan_sequence_;

        if (candidate_hit_scans_[occupied_index] >= confirmation_hit_scans_ &&
            currentNeighborCount(occupied_index, occupied_cells) >= confirmation_min_neighbors_)
        {
          confirmed_occupied_[occupied_index] = 1U;
        }
      }
    }
    latest_stamp_ = cloud->header.stamp.isZero() ? ros::Time::now() : cloud->header.stamp;
    publishStatus("RUNNING");
    ROS_INFO_THROTTLE(5.0, "Stable map: candidates=%zu, confirmed=%zu, updates=%s",
                      candidateCount(), confirmedCount(), updates_enabled_ ? "on" : "frozen");
  }

  int currentNeighborCount(const int index, const std::unordered_set<int>& occupied_cells) const
  {
    const int center_y = index / width_cells_;
    const int center_x = index - center_y * width_cells_;
    int count = 0;
    for (int offset_y = -1; offset_y <= 1; ++offset_y)
    {
      for (int offset_x = -1; offset_x <= 1; ++offset_x)
      {
        if (offset_x == 0 && offset_y == 0)
        {
          continue;
        }
        const int x = center_x + offset_x;
        const int y = center_y + offset_y;
        if (x >= 0 && x < width_cells_ && y >= 0 && y < height_cells_ &&
            occupied_cells.count(flattenIndex(x, y)) != 0U)
        {
          ++count;
        }
      }
    }
    return count;
  }

  std::uint8_t nearbyCandidateSupport(const int index) const
  {
    const int center_y = index / width_cells_;
    const int center_x = index - center_y * width_cells_;
    std::uint8_t support = 0U;
    for (int offset_y = -confirmation_spatial_tolerance_cells_;
         offset_y <= confirmation_spatial_tolerance_cells_; ++offset_y)
    {
      for (int offset_x = -confirmation_spatial_tolerance_cells_;
           offset_x <= confirmation_spatial_tolerance_cells_; ++offset_x)
      {
        const int x = center_x + offset_x;
        const int y = center_y + offset_y;
        if (x < 0 || x >= width_cells_ || y < 0 || y >= height_cells_)
        {
          continue;
        }
        const int candidate_index = flattenIndex(x, y);
        const std::uint64_t last_scan = candidate_last_scan_[candidate_index];
        // Ignore cells already updated in this scan, otherwise one dense frame
        // could propagate support across a whole wall and confirm it instantly.
        if (last_scan == 0U || last_scan == scan_sequence_ ||
            scan_sequence_ - last_scan >
              static_cast<std::uint64_t>(confirmation_max_scan_gap_))
        {
          continue;
        }
        support = std::max(support, candidate_hit_scans_[candidate_index]);
      }
    }
    return support;
  }

  std::size_t candidateCount() const
  {
    return static_cast<std::size_t>(std::count_if(
      candidate_hit_scans_.begin(), candidate_hit_scans_.end(),
      [](const std::uint8_t value) { return value > 0U; }));
  }

  std::size_t confirmedCount() const
  {
    return static_cast<std::size_t>(std::count(
      confirmed_occupied_.begin(), confirmed_occupied_.end(), static_cast<std::uint8_t>(1U)));
  }

  nav_msgs::OccupancyGrid createMapMessage(const bool confirmed_layer) const
  {
    nav_msgs::OccupancyGrid map;
    map.header.frame_id = global_frame_;
    map.header.stamp = latest_stamp_.isZero() ? ros::Time::now() : latest_stamp_;
    map.info.map_load_time = map.header.stamp;
    map.info.resolution = static_cast<float>(resolution_);
    map.info.width = static_cast<std::uint32_t>(width_cells_);
    map.info.height = static_cast<std::uint32_t>(height_cells_);
    map.info.origin.position.x = origin_x_;
    map.info.origin.position.y = origin_y_;
    map.info.origin.orientation.w = 1.0;
    map.data.resize(log_odds_.size(), -1);
    for (std::size_t index = 0; index < log_odds_.size(); ++index)
    {
      if (observed_[index] == 0U)
      {
        continue;
      }
      if (confirmed_layer)
      {
        if (confirmed_occupied_[index] != 0U)
        {
          map.data[index] = 100;
        }
        else if (logOddsToProbability(log_odds_[index]) <= free_threshold_)
        {
          map.data[index] = 0;
        }
        continue;
      }
      if (occupied_latched_[index] != 0U)
      {
        map.data[index] = 100;
        continue;
      }
      const double probability = logOddsToProbability(log_odds_[index]);
      if (probability <= free_threshold_)
      {
        map.data[index] = 0;
      }
      else if (probability >= occupied_threshold_)
      {
        map.data[index] = 100;
      }
      else
      {
        map.data[index] = static_cast<std::int8_t>(
          std::round(clampValue(probability * 100.0, 1.0, 99.0)));
      }
    }
    return map;
  }

  void publishTimerCallback(const ros::TimerEvent&)
  {
    std::lock_guard<std::mutex> lock(grid_mutex_);
    map_publisher_.publish(createMapMessage(false));
    confirmed_map_publisher_.publish(createMapMessage(true));
  }

  bool clearMapCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
  {
    std::lock_guard<std::mutex> lock(grid_mutex_);
    resetGridStateLocked();
    updates_enabled_ = true;
    latest_stamp_ = ros::Time::now();
    const bool saved = saveFloorStateLocked(current_floor_);
    map_publisher_.publish(createMapMessage(false));
    confirmed_map_publisher_.publish(createMapMessage(true));
    publishStatus(saved ? "MAP_CLEARED" : "MAP_CLEARED_SAVE_FAILED");
    ROS_INFO("Nearest-azimuth projected floor %d map cleared", current_floor_);
    return saved;
  }

  bool clearCandidatesCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
  {
    std::lock_guard<std::mutex> lock(grid_mutex_);
    for (std::size_t index = 0; index < candidate_hit_scans_.size(); ++index)
    {
      if (confirmed_occupied_[index] == 0U)
      {
        candidate_hit_scans_[index] = 0U;
        candidate_last_scan_[index] = 0U;
      }
    }
    const bool saved = saveFloorStateLocked(current_floor_);
    ROS_INFO("Unconfirmed obstacle candidates cleared on floor %d", current_floor_);
    return saved;
  }

  bool setUpdatesEnabledCallback(std_srvs::SetBool::Request& request,
                                 std_srvs::SetBool::Response& response)
  {
    std::lock_guard<std::mutex> lock(grid_mutex_);
    updates_enabled_ = request.data;
    response.success = true;
    response.message = updates_enabled_ ? "mapping updates enabled" : "map frozen";
    publishStatus(updates_enabled_ ? "RUNNING" : "FROZEN");
    map_publisher_.publish(createMapMessage(false));
    confirmed_map_publisher_.publish(createMapMessage(true));
    ROS_INFO("Map updates %s", updates_enabled_ ? "enabled" : "frozen");
    return true;
  }

  void publishStatus(const std::string& status)
  {
    std_msgs::String message;
    message.data = status;
    status_publisher_.publish(message);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber cloud_subscriber_;
  ros::Publisher map_publisher_;
  ros::Publisher confirmed_map_publisher_;
  ros::Publisher status_publisher_;
  ros::Publisher current_floor_publisher_;
  ros::ServiceServer clear_service_;
  ros::ServiceServer clear_candidates_service_;
  ros::ServiceServer updates_service_;
  ros::ServiceServer save_floor_service_;
  ros::ServiceServer sync_floor_service_;
  ros::Timer publish_timer_;
  ros::Timer floor_state_timer_;
  ros::Timer autosave_timer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::string input_cloud_topic_;
  std::string output_topic_;
  std::string confirmed_output_topic_;
  std::string global_frame_;
  std::string sensor_frame_;
  std::string height_filter_frame_;
  std::string floor_state_file_;
  std::string floor_maps_root_;
  double resolution_{0.10};
  double map_width_m_{60.0};
  double map_height_m_{60.0};
  double origin_x_{-30.0};
  double origin_y_{-30.0};
  double min_obstacle_height_{-0.30};
  double max_obstacle_height_{1.50};
  double min_range_{0.30};
  double max_range_{12.0};
  double hit_probability_{0.72};
  double miss_probability_{0.49};
  double min_probability_{0.12};
  double max_probability_{0.97};
  double free_threshold_{0.30};
  double occupied_threshold_{0.65};
  double occupied_latch_probability_{0.70};
  double occupied_clear_probability_{0.55};
  double occupied_miss_scale_{1.0};
  double azimuth_bin_degrees_{0.25};
  double actual_bin_radians_{0.0};
  double endpoint_guard_radius_{0.10};
  double publish_rate_{2.0};
  double tf_timeout_{0.20};
  double tf_latest_fallback_max_skew_{0.05};
  double log_odds_hit_{0.0};
  double log_odds_miss_{0.0};
  double log_odds_min_{0.0};
  double log_odds_max_{0.0};
  int endpoint_margin_cells_{0};
  int guard_radius_cells_{1};
  int point_stride_{1};
  int max_rays_per_scan_{5000};
  int confirmation_hit_scans_{3};
  int confirmation_max_scan_gap_{20};
  int confirmation_min_neighbors_{1};
  int confirmation_spatial_tolerance_cells_{1};
  int angular_bin_count_{1440};
  int width_cells_{0};
  int height_cells_{0};
  int initial_floor_{0};
  int floor_count_{3};
  int current_floor_{0};
  double floor_state_poll_rate_{2.0};
  double floor_autosave_period_{5.0};
  bool floor_persistence_enabled_{true};
  bool clear_all_floor_maps_{false};
  bool reject_zero_stamp_{true};
  bool use_sensor_relative_height_{true};
  bool clip_filtered_rays_to_height_band_{false};
  std::vector<float> log_odds_;
  std::vector<std::uint8_t> observed_;
  std::vector<std::uint8_t> occupied_latched_;
  std::vector<std::uint8_t> candidate_hit_scans_;
  std::vector<std::uint64_t> candidate_last_scan_;
  std::vector<std::uint8_t> confirmed_occupied_;
  std::uint64_t scan_sequence_{0U};
  std::uint64_t floor_generation_{0U};
  bool updates_enabled_{true};
  ros::Time latest_stamp_;
  mutable std::mutex grid_mutex_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "nearest_azimuth_projection_node");
  try
  {
    NearestAzimuthProjectionNode node;
    ros::spin();
  }
  catch (const std::exception& error)
  {
    ROS_FATAL("Failed to start nearest_azimuth_projection_node: %s", error.what());
    return 1;
  }
  return 0;
}
