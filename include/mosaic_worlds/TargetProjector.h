/**
 * @file TargetProjector.h
 * Header file for TargetProjector class.
 */
#ifndef MOSAIC_WORLDS_TARGET_PROJECTOR_H
#define MOSAIC_WORLDS_TARGET_PROJECTOR_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/image_encodings.h>

namespace mosaic_worlds {

/**
 * Provides ROS node implementation for projecting known target locations
 * into image pixel coordinates.
 */
class TargetProjector
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber sub_;
  image_transport::Publisher pub_;
  tf::TransformListener tf_listener_;
  image_geometry::PinholeCameraModel cam_model_;
  std::vector<std::string> frame_ids_;
  CvFont font_;

public:

  TargetProjector(const std::vector<std::string>& frame_ids);

  void imageCb(const sensor_msgs::ImageConstPtr& image_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg);
};

} // namespace mosaic_worlds

#endif  // MOSAIC_WORLDS_TARGET_PROJECTOR_H
