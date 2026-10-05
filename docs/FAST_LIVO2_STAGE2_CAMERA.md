# FAST-LIVO2 Stage 2 — Dabai DC1 Camera

## Status

Stage 2 validated on the physical AgileX LIMO Pro.

This stage validates the RGB camera and its intrinsic calibration.
LiDAR-camera extrinsic calibration and FAST-Calib are reserved for Stage 3.

## Camera

- Model: Orbbec Dabai DC1
- RGB device: `/dev/video0`
- Driver: `uvcvideo`
- ROS 2 driver: `v4l2_camera`
- ROS distribution: Humble
- Docker: Ubuntu 22.04 ARM64
- Resolution: 640 x 480
- Requested rate: 30 FPS
- Measured ROS rate: approximately 29.6 Hz
- Encoding: `rgb8`
- Image topic: `/camera/image_raw`
- CameraInfo topic: `/camera/camera_info`
- Image frame ID: `camera`

## Intrinsic calibration

Calibration target:

- Chessboard
- 8 x 6 internal corners
- 9 x 7 physical squares
- Square size: 25.0 mm
- Calibration resolution: 640 x 480
- Model: Pinhole
- ROS distortion model: `plumb_bob`

Intrinsic parameters:

    fx = 494.55179
    fy = 493.91134
    cx = 314.61947
    cy = 221.95045

Distortion:

    k1 =  0.095855
    k2 = -0.122538
    p1 =  0.003246
    p2 =  0.002270
    k3 =  0.000000

Canonical ROS calibration file:

    calibration/camera/dabai_dc1_640x480.yaml

FAST-LIVO2/Vikit camera configuration:

    ros2_ws/src/fast_livo2/config/camera_dabai_dc1.yaml

FAST-LIVO2 parameters:

    cam_model: Pinhole
    cam_width: 640
    cam_height: 480
    scale: 1.0
    cam_fx: 494.55179
    cam_fy: 493.91134
    cam_cx: 314.61947
    cam_cy: 221.95045
    cam_d0: 0.095855
    cam_d1: -0.122538
    cam_d2: 0.003246
    cam_d3: 0.002270

For the plumb_bob calibration:

- cam_d0 = k1
- cam_d1 = k2
- cam_d2 = p1
- cam_d3 = p2

The fifth coefficient k3 is zero and is not used by this four-coefficient
FAST-LIVO2 Pinhole configuration.

## CameraInfo validation

The calibration YAML was loaded by `v4l2_camera`.

`/camera/camera_info` published the calibrated non-zero K, D, R and P
matrices.

The V4L2 camera name differs from `dabai_dc1_rgb`, producing a warning,
but the calibration values were successfully loaded and published.

## Timestamp validation

One image was measured with:

    camera stamp = 1791172679.633936882
    system time  = 1791172679.649109840
    difference   = 15.173 ms

This is an end-to-end observation between the image timestamp and receipt by
a ROS subscriber. It is NOT used as the FAST-LIVO2 camera-LiDAR time offset.

Fine temporal synchronization remains a Stage 3 task.

## Preserved calibration data

The original calibration archive contained:

- 98 calibration images
- `ost.yaml`
- `ost.txt`

The raw images and archive are retained locally but excluded from Git.

## Stage 2 result

Validated:

- Dabai DC1 physical RGB camera
- ROS 2 Humble camera publication
- 640 x 480 RGB stream
- approximately 30 FPS
- intrinsic calibration
- distortion calibration
- CameraInfo loading
- FAST-LIVO2 Pinhole parameter file
- camera timestamps on the host time base

Not performed yet:

- FAST-Calib
- camera-LiDAR extrinsic calibration
- final camera-LiDAR temporal offset
- FAST-LIVO2 visual mode (`img_en = 1`)

Those items belong to Stage 3.
