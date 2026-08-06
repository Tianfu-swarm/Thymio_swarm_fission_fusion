#pragma once

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

// Forward-declare the C apriltag types so users of this header don't need to
// include the apriltag headers directly.
struct apriltag_family;
struct apriltag_detector;

#include "tag_tracker/coordinate_frame.hpp"   // DetectedTag

namespace tag_tracker
{

/**
 * @brief Thin RAII wrapper around the libapriltag C library.
 *
 * Supported families (pass as constructor string):
 *   "tag36h11"  "tag25h9"  "tag16h5"  "tagCircle21h7"  "tagStandard41h12"
 */
class AprilTagDetector
{
public:
  explicit AprilTagDetector(
    const std::string & family  = "tag36h11",
    int                 nthreads = 2,
    float               quad_decimate = 1.0f,
    float               quad_sigma    = 0.0f);

  ~AprilTagDetector();

  // Non-copyable, movable
  AprilTagDetector(const AprilTagDetector &)            = delete;
  AprilTagDetector & operator=(const AprilTagDetector &) = delete;
  AprilTagDetector(AprilTagDetector &&)                 = default;

  /**
   * Detect tags in a grayscale (CV_8UC1) image.
   * @param gray  8-bit single-channel image
   * @return      list of detected tags in image coordinates
   */
  std::vector<DetectedTag> detect(const cv::Mat & gray) const;

private:
  apriltag_family   * tf_  = nullptr;
  apriltag_detector * td_  = nullptr;
  std::string         family_name_;
};

}  // namespace tag_tracker
