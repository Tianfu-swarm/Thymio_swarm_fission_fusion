#pragma once

#include <cmath>
#include <optional>
#include <unordered_map>
#include <vector>

#include <opencv2/core.hpp>

namespace tag_tracker
{

// ─────────────────────────────────────────────────────────────────────────────
/// A fixed tag whose world-frame pose is known a-priori.
struct AnchorTag
{
  int    id        = 0;
  double world_x   = 0.0;   ///< metres (or any consistent unit)
  double world_y   = 0.0;
  double world_yaw = 0.0;   ///< orientation of the tag itself [rad]
};

// ─────────────────────────────────────────────────────────────────────────────
/// Raw detection from the camera image (image-space coordinates).
struct DetectedTag
{
  int    id            = 0;
  double center_u      = 0.0;   ///< pixel column
  double center_v      = 0.0;   ///< pixel row
  /// Corners in order: bottom-left, bottom-right, top-right, top-left
  std::array<cv::Point2f, 4> corners{};
  double rotation_img  = 0.0;   ///< tag heading in image plane [rad]
};

// ─────────────────────────────────────────────────────────────────────────────
/// Tag pose expressed in the world coordinate frame.
struct TagPose2D
{
  int    id      = 0;
  double x       = 0.0;
  double y       = 0.0;
  double yaw     = 0.0;
  bool   visible = true;
};

// ─────────────────────────────────────────────────────────────────────────────
/**
 * @brief Maintains a homography  H : image → world  derived from anchor-tag
 *        detections, and maps any detected tag into that frame.
 *
 * Estimation strategy:
 *   1 anchor  → translation only
 *   2–3       → similarity (translation + rotation + uniform scale)
 *   ≥ 4       → full homography via cv::findHomography (RANSAC)
 */
class CoordinateFrame
{
public:
  explicit CoordinateFrame(const std::vector<AnchorTag> & anchors);

  /// @return true once a valid transform has been estimated.
  bool isReady() const { return ready_; }

  /**
   * Re-estimate the homography from currently visible anchor tags.
   * Call this every frame before calling transform().
   */
  void update(const std::vector<DetectedTag> & detections);

  /**
   * Map a detected tag from image space to world space.
   * Returns visible=false if isReady() is false.
   */
  TagPose2D transform(const DetectedTag & det) const;

private:
  // ── helpers ──────────────────────────────────────────────────────────────
  void buildTranslationOnly(cv::Point2d img, cv::Point2d world);
  void buildSimilarity(
    const std::vector<cv::Point2d> & img_pts,
    const std::vector<cv::Point2d> & world_pts);
  void buildHomography(
    const std::vector<cv::Point2d> & img_pts,
    const std::vector<cv::Point2d> & world_pts);

  cv::Point2d imgToWorld(double u, double v) const;
  double      imgYawToWorldYaw(double img_yaw) const;

  // ── data ─────────────────────────────────────────────────────────────────
  std::unordered_map<int, AnchorTag> anchors_;

  cv::Mat H_;                  ///< 3×3 CV_64F homography (image → world)
  double  world_rotation_ = 0.0;
  bool    ready_          = false;
};

}  // namespace tag_tracker
