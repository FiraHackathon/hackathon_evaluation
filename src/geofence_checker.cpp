#include <fstream>
#include <hackathon_evaluation/geofence_checker.hpp>

namespace hackathon
{

GeofenceChecker::GeofenceChecker(rclcpp::Node::SharedPtr &node) : 
node_{node},
tf_buffer_{node_->get_clock()},
tf_listener_{tf_buffer_}
{
  using std::placeholders::_1;
  using namespace std::chrono_literals;

  node_->declare_parameter<std::string>("geofence_data_file");
  auto param = node->get_parameter("geofence_data_file");
  load_csv(param.as_string());

  pub_ = node_->create_publisher<FloatMsg>("~/safe_zone_exits_count", 1);
  sub_ = node_->create_subscription<PoseStamped>(
      "robot/robot_pose", 1, std::bind(&GeofenceChecker::pose_callback_, this, _1));

  timer_ = node_->create_wall_timer(1s, std::bind(&GeofenceChecker::timer_callback_, this));
  markers_pub_ = node_->create_publisher<Markers>("~/geofence_markers", 1);

  last_exit_time = node_->now();
}

void GeofenceChecker::timer_callback_()
{
  if (geofence_.outer().empty() || !tf_buffer_._frameExists("map"))
  {
    return;
  }

  Markers msg;
  Marker  marker;

  marker.header.frame_id = "map";
  // marker.header.stamp = node_->now();

  marker.ns = "geofence_markers";
  marker.id = ++m_count_;
  marker.action = Marker::ADD;
  marker.type = Marker::LINE_STRIP;
  marker.frame_locked = true;

  marker.color.a = 1;
  marker.color.r = 1;

  marker.scale.x = 1;

  for (const auto point : geofence_.outer())
  {
    geometry_msgs::msg::Point position;
    position.x = point.get<0>();
    position.y = point.get<1>();
    position.z = 0;
    marker.points.push_back(position);
  }

  msg.markers.push_back(marker);
  markers_pub_->publish(msg);
}

void GeofenceChecker::pose_callback_(const PoseStamped &robot_pose)
{
  using namespace std::chrono_literals;

  Point robot_position {robot_pose.pose.position.x, robot_pose.pose.position.y};
  if (bg::within(robot_position, geofence_)) {
    return;
  }

  rclcpp::Time current_time = node_->now();
  if((current_time - last_exit_time) < 3s) {
    return;
  }

  ++nb_exits_;
  last_exit_time = current_time;

  FloatMsg out;
  out.data = nb_exits_;
  pub_->publish(out);

  RCLCPP_INFO_STREAM(rclcpp::get_logger("geofence_checker"), "nb exits: " << nb_exits_);
}

void GeofenceChecker::load_csv(const std::string & filename)
{
  constexpr auto separator = ',';
  std::ifstream file(filename);

  if (!file.is_open()) {
    throw std::runtime_error("failed to open CSV data file");
  }

  std::string line;

  // skip first line (header)
  if (!std::getline(file, line)) {
    throw std::runtime_error("failed to read CSV file " + filename + ": file is empty");
  }

  std::size_t i = 2;

  while (std::getline(file, line)) {
    std::istringstream ss(line);
    std::string x, y;

    std::getline(ss, x, separator);
    std::getline(ss, y, separator);

    try {
      Point point{std::stod(x), std::stod(y)};
      bg::append(geofence_, point);
    } catch (const std::exception &) {
      throw std::runtime_error("malformed number in " + filename + ", line " + std::to_string(i));
    }

    if (!ss) {
      throw std::runtime_error("malformed line in " + filename + ", line" + std::to_string(i));
    }

    ++i;
  }

  file.close();  // Close the file
}

}  // namespace hackathon