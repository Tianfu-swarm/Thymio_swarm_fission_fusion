#include "tag_tracker/apriltag_detector.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

// libapriltag C headers
#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
#include <apriltag/tag25h9.h>
#include <apriltag/tag16h5.h>
#include <apriltag/tagCircle21h7.h>
#include <apriltag/tagStandard41h12.h>
#include <apriltag/apriltag_pose.h>

#include <opencv2/imgproc.hpp>

namespace tag_tracker
{

// ─────────────────────────────────────────────────────────────────────────────
// Utility: derive tag heading from its 4 corner points.
//
// Corner ordering (libapriltag):  bottom-left (0), bottom-right (1),
//                                 top-right   (2), top-left    (3)
// We define heading as the direction from mid-bottom to mid-top.
// ─────────────────────────────────────────────────────────────────────────────
static double cornersToYaw(const double p[][2])
{
  // p[0]=bottom-left, p[1]=bottom-right, p[2]=top-right, p[3]=top-left
  
  const double mid_bot_x = (p[0][0] + p[1][0]) / 2.0;
  const double mid_bot_y = (p[0][1] + p[1][1]) / 2.0;
  const double mid_top_x = (p[3][0] + p[2][0]) / 2.0;
  const double mid_top_y = (p[3][1] + p[2][1]) / 2.0;
  const double dx =  mid_top_x - mid_bot_x;
  const double dy = -(mid_top_y - mid_bot_y);   // image y flipped
  return std::atan2(dy, dx);
}

// ─────────────────────────────────────────────────────────────────────────────
AprilTagDetector::AprilTagDetector(
  const std::string & family,
  int                 nthreads,
  float               quad_decimate,
  float               quad_sigma)
: family_name_(family)
{
  // Create tag family
  if      (family == "tag36h11")        tf_ = tag36h11_create();
  else if (family == "tag25h9")         tf_ = tag25h9_create();
  else if (family == "tag16h5")         tf_ = tag16h5_create();
  else if (family == "tagCircle21h7")   tf_ = tagCircle21h7_create();
  else if (family == "tagStandard41h12") tf_ = tagStandard41h12_create();
  else {
    throw std::invalid_argument(
      "Unknown AprilTag family: '" + family +
      "'.  Supported: tag36h11, tag25h9, tag16h5, tagCircle21h7, tagStandard41h12");
  }

  // Create detector
  td_ = apriltag_detector_create();
  apriltag_detector_add_family(td_, tf_);

  td_->nthreads      = nthreads;
  td_->quad_decimate = quad_decimate;
  td_->quad_sigma    = quad_sigma;
  td_->refine_edges  = 1;
  td_->decode_sharpening = 0.25;
  td_->debug         = 0;
}

// ─────────────────────────────────────────────────────────────────────────────
AprilTagDetector::~AprilTagDetector()
{
  if (td_) {
    apriltag_detector_destroy(td_);
    td_ = nullptr;
  }
  if (tf_) {
    if      (family_name_ == "tag36h11")         tag36h11_destroy(tf_);
    else if (family_name_ == "tag25h9")          tag25h9_destroy(tf_);
    else if (family_name_ == "tag16h5")          tag16h5_destroy(tf_);
    else if (family_name_ == "tagCircle21h7")    tagCircle21h7_destroy(tf_);
    else if (family_name_ == "tagStandard41h12") tagStandard41h12_destroy(tf_);
    tf_ = nullptr;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
std::vector<DetectedTag> AprilTagDetector::detect(const cv::Mat & gray) const
{
  CV_Assert(gray.type() == CV_8UC1);

  // Wrap OpenCV Mat as image_u8_t (zero-copy)
  image_u8_t img{
    gray.cols,
    gray.rows,
    gray.cols,
    gray.data
  };

  zarray_t * detections = apriltag_detector_detect(td_, &img);

  std::vector<DetectedTag> results;
  results.reserve(static_cast<std::size_t>(zarray_size(detections)));

  for (int i = 0; i < zarray_size(detections); ++i) {
    apriltag_detection_t * det = nullptr;
    zarray_get(detections, i, &det);

    DetectedTag tag;
    tag.id       = det->id;
    tag.center_u = det->c[0];
    tag.center_v = det->c[1];

    // corners: p[0]=bottom-left, p[1]=bottom-right, p[2]=top-right, p[3]=top-left
    for (int k = 0; k < 4; ++k) {
      tag.corners[static_cast<std::size_t>(k)] =
        cv::Point2f(static_cast<float>(det->p[k][0]),
                    static_cast<float>(det->p[k][1]));
    }

    tag.rotation_img = cornersToYaw(det->p);

    results.push_back(std::move(tag));
  }

  apriltag_detections_destroy(detections);
  return results;
}

}  // namespace tag_tracker
