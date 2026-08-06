#pragma once

#include <limits>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>

#include "tag_tracker/coordinate_frame.hpp"
#include "tag_tracker/apriltag_detector.hpp"

namespace tag_tracker
{

class TagTrackerNode : public rclcpp::Node
{
public:
  explicit TagTrackerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});
  ~TagTrackerNode() override = default;

private:
  // ── initialisation ───────────────────────────────────────────────────────
  void declareParameters();
  std::vector<AnchorTag> loadAnchorTags();

  // ── callbacks ────────────────────────────────────────────────────────────
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);

  // ── publishing ───────────────────────────────────────────────────────────
  void publishAll(
    const std::vector<TagPose2D> & poses,
    const std_msgs::msg::Header  & header);

  void publishSingle(
    const TagPose2D             & pose,
    const std_msgs::msg::Header & header);

  void publishNaN(int tid);
  void publishDebugImage(
    const cv::Mat                           & bgr,
    const std::vector<DetectedTag>          & detections,
    const std::vector<TagPose2D>            & moving_poses,
    const std_msgs::msg::Header             & header);

  // ── utilities ────────────────────────────────────────────────────────────
  cv::Mat rosToCvGray(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
  cv::Mat drawDebug(
    const cv::Mat                  & gray,
    const std::vector<DetectedTag> & detections,
    const std::vector<TagPose2D>   & moving_poses);

  // ── detector / frame ─────────────────────────────────────────────────────
  std::unique_ptr<AprilTagDetector> detector_;
  std::unique_ptr<CoordinateFrame>  coord_frame_;

  // ── config ───────────────────────────────────────────────────────────────
  std::unordered_set<int>  anchor_ids_;
  std::unordered_set<int>  moving_ids_;      ///< empty → track all non-anchors
  std::string              output_prefix_;
  bool                     enable_debug_;
  double                   publish_rate_;
  rclcpp::Time             last_publish_time_;

  // ── ROS I/O ──────────────────────────────────────────────────────────────
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_all_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr       pub_debug_;

  /// Per-tag publishers created on first detection of a tag id.
  std::unordered_map<
    int,
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr
  > per_tag_pubs_;
};

}  // namespace tag_tracker
