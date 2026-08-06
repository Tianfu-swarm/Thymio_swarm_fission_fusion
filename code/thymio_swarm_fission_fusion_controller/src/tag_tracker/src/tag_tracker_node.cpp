#include "tag_tracker/tag_tracker_node.hpp"

#include <cmath>
#include <sstream>
#include <string>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/header.hpp>

namespace tag_tracker
{

// ─────────────────────────────────────────────────────────────────────────────
TagTrackerNode::TagTrackerNode(const rclcpp::NodeOptions & options)
: Node("tag_tracker_node", options)
{
  declareParameters();

  // ── Read parameters ────────────────────────────────────────────────────
  const std::string image_topic  = get_parameter("image_topic").as_string();
  const std::string tag_family   = get_parameter("tag_family").as_string();
  const int         nthreads     = static_cast<int>(get_parameter("detector_threads").as_int());
  const float       quad_dec     = static_cast<float>(get_parameter("quad_decimate").as_double());
  output_prefix_                 = get_parameter("output_topic_prefix").as_string();
  enable_debug_                  = get_parameter("publish_debug_image").as_bool();
  publish_rate_                  = get_parameter("publish_rate").as_double();

  // Moving-tag whitelist (empty = all non-anchors)
  {
    const auto ids = get_parameter("moving_tag_ids").as_integer_array();
    for (const auto id : ids) {
      moving_ids_.insert(static_cast<int>(id));
    }
  }

  // ── Anchor tags ────────────────────────────────────────────────────────
  auto anchors = loadAnchorTags();
  for (const auto & a : anchors) {
    anchor_ids_.insert(a.id);
  }
  coord_frame_ = std::make_unique<CoordinateFrame>(anchors);

  if (anchors.empty()) {
    RCLCPP_WARN(get_logger(),
      "No anchor tags configured! The coordinate frame will not be "
      "available until anchors are added to config/tag_tracker.yaml.");
  } else {
    std::ostringstream ss;
    for (const auto & a : anchors) { ss << a.id << " "; }
    RCLCPP_INFO(get_logger(),
      "Loaded %zu anchor tag(s): [ %s]", anchors.size(), ss.str().c_str());
  }

  // ── Detector ──────────────────────────────────────────────────────────
  detector_ = std::make_unique<AprilTagDetector>(tag_family, nthreads, quad_dec);
  RCLCPP_INFO(get_logger(), "AprilTag detector ready (family=%s)", tag_family.c_str());

  // ── QoS ───────────────────────────────────────────────────────────────
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

  // ── Publishers ────────────────────────────────────────────────────────
  pub_all_ = create_publisher<geometry_msgs::msg::PoseArray>(
    output_prefix_ + "/all", qos);

  // Pre-create per-tag publishers so topics always exist
  for (const int tid : moving_ids_) {
    const std::string topic = output_prefix_ + "/tag_" + std::to_string(tid);
    per_tag_pubs_[tid] = create_publisher<geometry_msgs::msg::Pose2D>(topic, qos);
    RCLCPP_INFO(get_logger(), "Created publisher for tag %d -> %s", tid, topic.c_str());
  }

  if (enable_debug_) {
    const std::string debug_topic = get_parameter("debug_image_topic").as_string();
    pub_debug_ = create_publisher<sensor_msgs::msg::Image>(debug_topic, qos);
  }

  // ── Subscription ──────────────────────────────────────────────────────
  image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    image_topic, qos,
    std::bind(&TagTrackerNode::imageCallback, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "Subscribed to %s", image_topic.c_str());
  RCLCPP_INFO(get_logger(), "tag_tracker_node started.");
}

// ─────────────────────────────────────────────────────────────────────────────
void TagTrackerNode::declareParameters()
{
  declare_parameter<std::string>("image_topic",         "/camera/image_raw");
  declare_parameter<std::string>("camera_info_topic",   "/camera/camera_info");
  declare_parameter<std::string>("tag_family",          "tag36h11");
  declare_parameter<double>     ("tag_size",             0.10);
  declare_parameter<int>        ("detector_threads",     2);
  declare_parameter<double>     ("quad_decimate",        1.0);
  declare_parameter<double>     ("publish_rate",         0.0);
  declare_parameter<std::string>("output_topic_prefix",  "/tag_tracker/pose");
  declare_parameter<std::string>("debug_image_topic",    "/tag_tracker/debug_image");
  declare_parameter<bool>       ("publish_debug_image",  true);
  declare_parameter<std::vector<int64_t>>("moving_tag_ids", std::vector<int64_t>{});

  // Anchor tag parallel arrays (Format B)
  declare_parameter<std::vector<int64_t>>("anchor_tag_ids",  std::vector<int64_t>{});
  declare_parameter<std::vector<double>> ("anchor_tag_xs",   std::vector<double>{});
  declare_parameter<std::vector<double>> ("anchor_tag_ys",   std::vector<double>{});
  declare_parameter<std::vector<double>> ("anchor_tag_yaws", std::vector<double>{});
}

// ─────────────────────────────────────────────────────────────────────────────
std::vector<AnchorTag> TagTrackerNode::loadAnchorTags()
{
  std::vector<AnchorTag> anchors;

  // ── Format B: parallel arrays ─────────────────────────────────────────
  const auto ids  = get_parameter("anchor_tag_ids").as_integer_array();
  const auto xs   = get_parameter("anchor_tag_xs").as_double_array();
  const auto ys   = get_parameter("anchor_tag_ys").as_double_array();
  const auto yaws = get_parameter("anchor_tag_yaws").as_double_array();

  if (!ids.empty()) {
    for (std::size_t i = 0; i < ids.size(); ++i) {
      AnchorTag a;
      a.id        = static_cast<int>(ids[i]);
      a.world_x   = (i < xs.size())   ? xs[i]   : 0.0;
      a.world_y   = (i < ys.size())   ? ys[i]   : 0.0;
      a.world_yaw = (i < yaws.size()) ? yaws[i] : 0.0;
      anchors.push_back(a);
    }
    return anchors;
  }

  // ── Format A: anchor_tags.N.{id,x,y,yaw} ─────────────────────────────
  for (int idx = 0; ; ++idx) {
    const std::string prefix = "anchor_tags." + std::to_string(idx);
    try {
      declare_parameter<int>   (prefix + ".id",  -1);
      declare_parameter<double>(prefix + ".x",   0.0);
      declare_parameter<double>(prefix + ".y",   0.0);
      declare_parameter<double>(prefix + ".yaw", 0.0);

      const int tid = get_parameter(prefix + ".id").as_int();
      if (tid < 0) { break; }

      AnchorTag a;
      a.id        = tid;
      a.world_x   = get_parameter(prefix + ".x").as_double();
      a.world_y   = get_parameter(prefix + ".y").as_double();
      a.world_yaw = get_parameter(prefix + ".yaw").as_double();
      anchors.push_back(a);
    } catch (...) {
      break;
    }
  }

  return anchors;
}

// ─────────────────────────────────────────────────────────────────────────────
void TagTrackerNode::imageCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  // ── Rate limiting ─────────────────────────────────────────────────────
  if (publish_rate_ > 0.0) {
    const rclcpp::Time now = this->now();
    const double elapsed   = (now - last_publish_time_).seconds();
    if (elapsed < (1.0 / publish_rate_)) { return; }
    last_publish_time_ = now;
  }

  // ── Image conversion ──────────────────────────────────────────────────
  cv::Mat gray = rosToCvGray(msg);
  if (gray.empty()) { return; }

  // ── Detect ────────────────────────────────────────────────────────────
  const auto detections = detector_->detect(gray);

  // ── Update coordinate frame ───────────────────────────────────────────
  coord_frame_->update(detections);

  if (!coord_frame_->isReady()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "Coordinate frame not ready — no anchor tags visible.");
  }

  // ── Compute world poses for moving tags ───────────────────────────────
  std::vector<TagPose2D> moving_poses;
  for (const auto & det : detections) {
    if (anchor_ids_.count(det.id)) { continue; }
    if (!moving_ids_.empty() && !moving_ids_.count(det.id)) { continue; }

    moving_poses.push_back(coord_frame_->transform(det));
  }

  // ── Publish ───────────────────────────────────────────────────────────
  std_msgs::msg::Header header;
  header.stamp    = msg->header.stamp;
  header.frame_id = "world";

  // Build a lookup for detected poses
  std::unordered_map<int, TagPose2D> detected_map;
  for (const auto & pose : moving_poses) {
    detected_map[pose.id] = pose;
  }

  publishAll(moving_poses, header);

  // Publish per-tag: detected pose or NaN if not visible
  for (const int tid : moving_ids_) {
    auto it = detected_map.find(tid);
    if (it != detected_map.end()) {
      publishSingle(it->second, header);
    } else {
      publishNaN(tid);
    }
  }

  // ── Debug image ───────────────────────────────────────────────────────
  if (enable_debug_) {
    publishDebugImage(gray, detections, moving_poses, header);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
void TagTrackerNode::publishAll(
  const std::vector<TagPose2D> & poses,
  const std_msgs::msg::Header  & header)
{
  geometry_msgs::msg::PoseArray msg;
  msg.header = header;

  for (const auto & p : poses) {
    geometry_msgs::msg::Pose pose;
    pose.position.x    = p.x;
    pose.position.y    = p.y;
    pose.position.z    = static_cast<double>(p.id);  // tag id encoded in z
    pose.orientation.z = p.yaw;                      // yaw encoded in quat.z
    pose.orientation.w = 1.0;
    msg.poses.push_back(pose);
  }

  pub_all_->publish(msg);
}

// ─────────────────────────────────────────────────────────────────────────────
void TagTrackerNode::publishSingle(
  const TagPose2D             & pose,
  const std_msgs::msg::Header & /*header*/)
{
  const int tid = pose.id;

  // Create per-tag publisher on first encounter
  if (per_tag_pubs_.find(tid) == per_tag_pubs_.end()) {
    const std::string topic = output_prefix_ + "/tag_" + std::to_string(tid);
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    per_tag_pubs_[tid] =
      create_publisher<geometry_msgs::msg::Pose2D>(topic, qos);
    RCLCPP_INFO(get_logger(),
      "Created publisher for tag %d → %s", tid, topic.c_str());
  }

  geometry_msgs::msg::Pose2D msg;
  msg.x     = pose.x;
  msg.y     = pose.y;
  msg.theta = pose.yaw;
  per_tag_pubs_[tid]->publish(msg);
}

// ─────────────────────────────────────────────────────────────────────────────
void TagTrackerNode::publishNaN(int tid)
{
  if (per_tag_pubs_.find(tid) == per_tag_pubs_.end()) { return; }
  geometry_msgs::msg::Pose2D msg;
  msg.x     = std::numeric_limits<double>::quiet_NaN();
  msg.y     = std::numeric_limits<double>::quiet_NaN();
  msg.theta = std::numeric_limits<double>::quiet_NaN();
  per_tag_pubs_[tid]->publish(msg);
}

// ─────────────────────────────────────────────────────────────────────────────
void TagTrackerNode::publishDebugImage(
  const cv::Mat                  & gray,
  const std::vector<DetectedTag> & detections,
  const std::vector<TagPose2D>   & moving_poses,
  const std_msgs::msg::Header    & header)
{
  cv::Mat dbg = drawDebug(gray, detections, moving_poses);

  auto img_msg = cv_bridge::CvImage(header, "bgr8", dbg).toImageMsg();
  pub_debug_->publish(*img_msg);
}

// ─────────────────────────────────────────────────────────────────────────────
cv::Mat TagTrackerNode::rosToCvGray(
  const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  try {
    if (msg->encoding == "mono8" || msg->encoding == "8UC1") {
      return cv_bridge::toCvCopy(msg, "mono8")->image;
    }
    auto bgr = cv_bridge::toCvCopy(msg, "bgr8")->image;
    cv::Mat gray;
    cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
    return gray;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Image conversion failed: %s", e.what());
    return {};
  }
}

// ─────────────────────────────────────────────────────────────────────────────
cv::Mat TagTrackerNode::drawDebug(
  const cv::Mat                  & gray,
  const std::vector<DetectedTag> & detections,
  const std::vector<TagPose2D>   & moving_poses)
{
  cv::Mat bgr;
  cv::cvtColor(gray, bgr, cv::COLOR_GRAY2BGR);

  // Build pose lookup by id
  std::unordered_map<int, const TagPose2D *> pose_map;
  for (const auto & p : moving_poses) {
    pose_map[p.id] = &p;
  }

  for (const auto & det : detections) {
    const bool is_anchor = anchor_ids_.count(det.id) > 0;
    const cv::Scalar color = is_anchor
      ? cv::Scalar(0, 200, 0)      // green for anchors
      : cv::Scalar(0, 120, 255);   // orange for moving

    // Draw corners
    std::vector<cv::Point> pts;
    for (const auto & c : det.corners) {
      pts.emplace_back(static_cast<int>(c.x), static_cast<int>(c.y));
    }
    cv::polylines(bgr, pts, true, color, 2);

    const int cx = static_cast<int>(det.center_u);
    const int cy = static_cast<int>(det.center_v);
    cv::circle(bgr, {cx, cy}, 4, color, -1);

    // Label
    std::string label;
    auto it = pose_map.find(det.id);
    if (it != pose_map.end()) {
      const auto & p = *it->second;
      char buf[128];
      std::snprintf(buf, sizeof(buf),
        "ID%d (%.2f, %.2f) %.0fdeg",
        det.id, p.x, p.y, p.yaw * 180.0 / M_PI);
      label = buf;
    } else if (is_anchor) {
      label = "ID" + std::to_string(det.id) + " [anchor]";
    } else {
      label = "ID" + std::to_string(det.id);
    }
    cv::putText(bgr, label, {cx + 5, cy - 5},
      cv::FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv::LINE_AA);

    // Heading arrow
    constexpr int arrow_len = 30;
    const int ax = cx + static_cast<int>(arrow_len * std::cos(det.rotation_img));
    const int ay = cy - static_cast<int>(arrow_len * std::sin(det.rotation_img));
    cv::arrowedLine(bgr, {cx, cy}, {ax, ay}, cv::Scalar(255, 255, 0), 2);
  }

  // Status bar
  const int n_anchors_vis = static_cast<int>(std::count_if(
    detections.begin(), detections.end(),
    [&](const DetectedTag & d) { return anchor_ids_.count(d.id) > 0; }));

  char status[256];
  std::snprintf(status, sizeof(status),
    "Anchors visible: %d/%zu  Frame ready: %s  Moving tags: %zu",
    n_anchors_vis, anchor_ids_.size(),
    coord_frame_->isReady() ? "YES" : "NO",
    moving_poses.size());
  cv::putText(bgr, status, {8, 20},
    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

  return bgr;
}

}  // namespace tag_tracker

// ─────────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<tag_tracker::TagTrackerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
