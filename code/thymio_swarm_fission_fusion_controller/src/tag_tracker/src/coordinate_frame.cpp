#include "tag_tracker/coordinate_frame.hpp"

#include <cmath>
#include <stdexcept>

#include <opencv2/calib3d.hpp>   // cv::findHomography
#include <opencv2/core.hpp>

namespace tag_tracker
{

// ─────────────────────────────────────────────────────────────────────────────
CoordinateFrame::CoordinateFrame(const std::vector<AnchorTag> & anchors)
{
  for (const auto & a : anchors) {
    anchors_[a.id] = a;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
void CoordinateFrame::update(const std::vector<DetectedTag> & detections)
{
  std::vector<cv::Point2d> img_pts, world_pts;

  for (const auto & det : detections) {
    auto it = anchors_.find(det.id);
    if (it == anchors_.end()) {
      continue;
    }
    img_pts.emplace_back(det.center_u, det.center_v);
    world_pts.emplace_back(it->second.world_x, it->second.world_y);
  }

  const std::size_t n = img_pts.size();

  if (n == 0) {
    ready_ = false;
    return;
  }

  if (n == 1) {
    buildTranslationOnly(img_pts[0], world_pts[0]);
  } else if (n < 4) {
    buildSimilarity(img_pts, world_pts);
  } else {
    buildHomography(img_pts, world_pts);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
TagPose2D CoordinateFrame::transform(const DetectedTag & det) const
{
  TagPose2D pose;
  pose.id = det.id;

  if (!ready_) {
    pose.visible = false;
    return pose;
  }

  auto [wx, wy] = [&]() -> std::pair<double, double> {
    auto p = imgToWorld(det.center_u, det.center_v);
    return {p.x, p.y};
  }();

  pose.x       = wx;
  pose.y       = wy;
  pose.yaw     = imgYawToWorldYaw(det.rotation_img);
  pose.visible = true;
  return pose;
}

// ─────────────────────────────────────────────────────────────────────────────
// Private helpers
// ─────────────────────────────────────────────────────────────────────────────

void CoordinateFrame::buildTranslationOnly(cv::Point2d img, cv::Point2d world)
{
  H_ = cv::Mat::eye(3, 3, CV_64F);
  H_.at<double>(0, 2) = world.x - img.x;
  H_.at<double>(1, 2) = world.y - img.y;
  world_rotation_ = 0.0;
  ready_ = true;
}

// ─────────────────────────────────────────────────────────────────────────────
void CoordinateFrame::buildSimilarity(
  const std::vector<cv::Point2d> & img_pts,
  const std::vector<cv::Point2d> & world_pts)
{
  const std::size_t n = img_pts.size();
  cv::Mat H = cv::Mat::eye(3, 3, CV_64F);

  if (n == 2) {
    // Closed-form 2-point similarity
    const double dxi  = img_pts[1].x   - img_pts[0].x;
    const double dyi  = img_pts[1].y   - img_pts[0].y;
    const double dxw  = world_pts[1].x - world_pts[0].x;
    const double dyw  = world_pts[1].y - world_pts[0].y;

    const double scale = std::hypot(dxw, dyw) / (std::hypot(dxi, dyi) + 1e-9);
    const double angle = std::atan2(dyw, dxw) - std::atan2(dyi, dxi);
    const double ca    = std::cos(angle);
    const double sa    = std::sin(angle);

    H.at<double>(0, 0) =  scale * ca;
    H.at<double>(0, 1) = -scale * sa;
    H.at<double>(1, 0) =  scale * sa;
    H.at<double>(1, 1) =  scale * ca;

    // Adjust translation so img_pts[0] maps to world_pts[0]
    const double mx = H.at<double>(0,0)*img_pts[0].x + H.at<double>(0,1)*img_pts[0].y;
    const double my = H.at<double>(1,0)*img_pts[0].x + H.at<double>(1,1)*img_pts[0].y;
    H.at<double>(0, 2) = world_pts[0].x - mx;
    H.at<double>(1, 2) = world_pts[0].y - my;

  } else {
    // 3-point least-squares affine
    // Build Ax = b  where x = [a,b,c, d,e,f]^T for
    //   wx = a*u + b*v + c
    //   wy = d*u + e*v + f
    cv::Mat A(static_cast<int>(2*n), 6, CV_64F, 0.0);
    cv::Mat b(static_cast<int>(2*n), 1, CV_64F, 0.0);

    for (int i = 0; i < static_cast<int>(n); ++i) {
      A.at<double>(2*i,   0) = img_pts[i].x;
      A.at<double>(2*i,   1) = img_pts[i].y;
      A.at<double>(2*i,   2) = 1.0;
      A.at<double>(2*i+1, 3) = img_pts[i].x;
      A.at<double>(2*i+1, 4) = img_pts[i].y;
      A.at<double>(2*i+1, 5) = 1.0;
      b.at<double>(2*i)      = world_pts[i].x;
      b.at<double>(2*i+1)    = world_pts[i].y;
    }

    cv::Mat params;
    cv::solve(A, b, params, cv::DECOMP_SVD);

    H.at<double>(0,0) = params.at<double>(0);
    H.at<double>(0,1) = params.at<double>(1);
    H.at<double>(0,2) = params.at<double>(2);
    H.at<double>(1,0) = params.at<double>(3);
    H.at<double>(1,1) = params.at<double>(4);
    H.at<double>(1,2) = params.at<double>(5);
  }

  H_ = H;
  world_rotation_ = std::atan2(H_.at<double>(1,0), H_.at<double>(0,0));
  ready_ = true;
}

// ─────────────────────────────────────────────────────────────────────────────
void CoordinateFrame::buildHomography(
  const std::vector<cv::Point2d> & img_pts,
  const std::vector<cv::Point2d> & world_pts)
{
  // cv::findHomography requires Point2f
  std::vector<cv::Point2f> src, dst;
  src.reserve(img_pts.size());
  dst.reserve(world_pts.size());
  for (const auto & p : img_pts)   src.emplace_back(static_cast<float>(p.x), static_cast<float>(p.y));
  for (const auto & p : world_pts) dst.emplace_back(static_cast<float>(p.x), static_cast<float>(p.y));

  cv::Mat H = cv::findHomography(src, dst, cv::RANSAC, 5.0);

  if (H.empty()) {
    // RANSAC failed — fall back to similarity
    buildSimilarity(img_pts, world_pts);
    return;
  }

  H.convertTo(H_, CV_64F);
  world_rotation_ = std::atan2(H_.at<double>(1,0), H_.at<double>(0,0));
  ready_ = true;
}

// ─────────────────────────────────────────────────────────────────────────────
cv::Point2d CoordinateFrame::imgToWorld(double u, double v) const
{
  cv::Mat p = (cv::Mat_<double>(3,1) << u, v, 1.0);
  cv::Mat r = H_ * p;
  const double w = r.at<double>(2);
  return {r.at<double>(0) / w, r.at<double>(1) / w};
}

// ─────────────────────────────────────────────────────────────────────────────
double CoordinateFrame::imgYawToWorldYaw(double img_yaw) const
{
  const double raw = img_yaw + world_rotation_;
  return std::atan2(std::sin(raw), std::cos(raw));
}

}  // namespace tag_tracker
