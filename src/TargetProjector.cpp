/**
 * @file TargetProjector.cpp
 * Implementation of TargetProjector class
 */
#include<mosaic_worlds/TargetProjector.h>

namespace mosaic_worlds {


TargetProjector::TargetProjector(const std::vector<std::string>& frame_ids)
    : it_(nh_), frame_ids_(frame_ids)
{
    std::string image_topic = nh_.resolveName("image");
    sub_ = it_.subscribeCamera(image_topic, 1, &TargetProjector::imageCb, this);
    pub_ = it_.advertise("image_out", 1);
    cvInitFont(&font_, CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5);
}

void TargetProjector::imageCb(const sensor_msgs::ImageConstPtr& image_msg,
    const sensor_msgs::CameraInfoConstPtr& info_msg)
{
    ROS_DEBUG("Entered imageCb");

    //**************************************************************************
    //  Transform the image into OpenCV format
    //**************************************************************************
    cv::Mat image;
    cv_bridge::CvImagePtr input_bridge;
    try {
        input_bridge = cv_bridge::toCvCopy(image_msg,
                sensor_msgs::image_encodings::BGR8);
        image = input_bridge->image;
    }
    catch (cv_bridge::Exception& ex){
        ROS_ERROR("[draw_frames] Failed to convert image");
    return;
    }

    //**************************************************************************
    //  Get the camera model required for projection
    //**************************************************************************
    cam_model_.fromCameraInfo(info_msg);

    //**************************************************************************
    //  For each target frame we wish to project...
    //**************************************************************************
    BOOST_FOREACH(const std::string& frame_id, frame_ids_) {

        //**********************************************************************
        //  Get the transform between the current target position and the
        //  camera frame
        //**********************************************************************
        tf::StampedTransform transform;
        try {
                ros::Time acquisition_time = info_msg->header.stamp;
                ros::Duration timeout(1.0 / 30);
                tf_listener_.waitForTransform(cam_model_.tfFrame(), frame_id,
                    acquisition_time, timeout);
                tf_listener_.lookupTransform(cam_model_.tfFrame(), frame_id,
                    acquisition_time, transform);
        }
        catch (tf::TransformException& ex) {
            ROS_WARN("[draw_frames] TF exception:\n%s", ex.what());
            return;
        }

        //**********************************************************************
        //  Get the target's position within the camera frame
        //**********************************************************************
        tf::Point pt = transform.getOrigin();

        //**********************************************************************
        // Ensure point is not behind image frame before publishing
        // Note we should really be checking pt.z() -- see hack below
        //**********************************************************************
        if(0 > pt.x())
        {
            continue;
        }

        //**********************************************************************
        // horrible hack to get correct output
        // something work with transformation somewhere, but not sure where
        // negating and swaping the axis in this way seems to work though
        //**********************************************************************
        //cv::Point3d pt_cv(pt.x(), pt.y(), pt.z());
        cv::Point3d pt_cv(-pt.y(), -pt.z(), pt.x());
        ROS_DEBUG("coord %f,%f,%f",-pt.y(),-pt.z(),-pt.x());

        //**********************************************************************
        //  Project the centre position into pixel coordinates
        //**********************************************************************
        cv::Point2d uv;
        uv = cam_model_.project3dToPixel(pt_cv);

        static const int RADIUS = 3;
        cv::circle(image, uv, RADIUS, CV_RGB(255,0,0), -1);
        CvSize text_size;
        int baseline;
        cvGetTextSize(frame_id.c_str(), &font_, &text_size, &baseline);
        CvPoint origin = cvPoint(uv.x - text_size.width / 2,
            uv.y - RADIUS - baseline - 3);
        cv:putText(image, frame_id.c_str(), origin, cv::FONT_HERSHEY_SIMPLEX, 2, CV_RGB(255,0,0));
    }

    ROS_DEBUG("Publishing image");
    pub_.publish(input_bridge->toImageMsg());
    ROS_DEBUG("image published");

} // imageCb

} // namespace mosaic_worlds

