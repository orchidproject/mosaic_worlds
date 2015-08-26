/**
 * @file observer_node
 * Implements observer_node.
 * ROS node for publishing visual observations in format
 * expected by mosaic_model package.
 */
#include<mosaic_worlds/Observer.h>

/**
 * ROS node main function.
 * Takes camera and world frames and command line arguments.
 */
int main(int argc, char** argv)
{
    //**************************************************************************
    //  Initialise ROS Node
    //**************************************************************************
    ros::init(argc, argv, "observer");

    //**************************************************************************
    //  Ensure we have the correct number of command line arguments
    //  (After ros::init has potentially removed remapping arguments)
    //  Should be 3 in total: command_name world_frame camera_frame
    //**************************************************************************
    if(3 != argc)
    {
        ROS_FATAL("%s: wrong number of arguments!",ros::this_node::getName());
        ROS_FATAL("%s: Usage: %s world_frame_name camera_frame_name",
                ros::this_node::getName(), argv[0]);
        ros::shutdown();
    }

    //**************************************************************************
    //  Get frame names from command line
    //**************************************************************************
    std::string world_frame(argv[1]);
    std::string camera_frame(argv[2]);

    //**************************************************************************
    //  Initialise Observer object to do the hard work
    //**************************************************************************
    mosaic_worlds::Observer observer(world_frame,camera_frame);
    ros::spin();
}
