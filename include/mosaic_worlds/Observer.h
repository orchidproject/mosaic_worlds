/**
 * @file Observer.h
 * Header file for Observer class.
 */
#ifndef MOSAIC_WORLDS_OBSERVER_H
#define MOSAIC_WORLDS_OBSERVER_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <boost/foreach.hpp>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/image_encodings.h>
#include <mosaic_vision/Detections.h>

namespace mosaic_worlds {

/**
 * Provides ROS node implementation for extracting mosaic model observations 
 * from camera tf frames and visual detections.
 */
class Observer
{
    /**
     * ROS Node Handle for communication with ROS system.
     */
    ros::NodeHandle nh_;

    /**
     * Handle to Image transport system for receiving images
     */
    image_transport::ImageTransport it_;

    /**
     * Filter for receiving time sychronised depth images
     */
    image_transport::SubscriberFilter depth_image_filter_;

    /**
     * Filter for receiving time sychronised camera information
     */
    message_filters::Subscriber<CameraInfo> cam_info_filter_;

    /**
     * Filter for receiving time synchronised visual detections
     */
    message_filters::Subscriber<CameraInfo> 
            
    image_transport::CameraSubscriber sub_;
    image_transport::Publisher pub_;
    tf::TransformListener tf_listener_;
    image_geometry::PinholeCameraModel cam_model_;
    std::string world_frame_;
    std::string camera_frame_;

public:

  Observer(const std::string& world_frame, const std:string& camera_frame);

  void imageCb(const sensor_msgs::ImageConstPtr& image_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg);
};

} // namespace mosaic_worlds

#endif  // MOSAIC_WORLDS_OBSERVER_H
