/**
 * @file target_projection_node
 * Implements target_projection_node.
 * ROS node for publishing 
 * ground truth image coordinates for known target positions within
 * field of view.
 */
#include<mosaic_worlds/TargetProjector.h>

/**
 * ROS node main function.
 * Takes names of target tf frames are command line arguments.
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "draw_frames");
  std::vector<std::string> frame_ids(argv + 1, argv + argc);
  mosaic_worlds::TargetProjector projector(frame_ids);
  ros::spin();
}
