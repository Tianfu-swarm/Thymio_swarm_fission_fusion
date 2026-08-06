# tag_tracker

ROS 2 node that tracks AprilTags using fixed anchor tags to define a 2D world coordinate frame.

## How it works

### Coordinate frame

**Anchor tags** are AprilTags placed at fixed, known positions in the scene. Their world coordinates `(x, y)` are specified in the config file. Each frame, the node collects all visible anchor tags and fits a 2D image-to-world transform using their pixel centers and known world positions:

| Visible anchors | Method |
|---|---|
| 1 | Translation only |
| 2–3 | Similarity transform (rotation + scale + translation) |
| ≥ 4 | Full planar homography (`cv::findHomography` + RANSAC) |

The transform is re-estimated every frame, so the camera may move freely as long as at least one anchor remains visible.

### Moving tag pose

For each moving tag, its image-space center is projected through the estimated transform to obtain world `(x, y)`. The heading `theta` is derived from the tag's four detected corner points: the vector from the midpoint of the bottom edge to the midpoint of the top edge gives the tag's facing direction in image space, which is then rotated into world space.

Tags listed in `moving_tag_ids` that are not detected in the current frame are still published, with all fields set to `NaN`.

## Build

```bash
source /opt/ros/jazzy/setup.bash
PKG_CONFIG_PATH=<apriltag_install>/lib/pkgconfig colcon build --symlink-install
source install/setup.bash
```

## Configuration

Edit `config/tag_tracker.yaml` to set anchor positions and moving tag IDs:

```yaml
anchor_tag_ids:  [3,   4,   5  ]
anchor_tag_xs:   [0.0, 1.0, 0.0]
anchor_tag_ys:   [0.0, 0.0, 1.0]
anchor_tag_yaws: [0.0, 0.0, 0.0]
moving_tag_ids:  [8, 9]
```

## Launch

```bash
ros2 launch tag_tracker tag_tracker.launch.py
```

## Topics

| Topic | Type | Description |
|---|---|---|
| `/tag_tracker/pose/all` | `PoseArray` | All moving tags; ID encoded in `position.z` |
| `/tag_tracker/pose/tag_<id>` | `Pose2D` | Per-tag pose; NaN when not visible |
| `/tag_tracker/debug_image` | `Image` | Annotated debug image |
