#include <colour_registration/colour_registration_st.h>
#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc,argv,"colour_registration_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    colour_registration_st test(pnh);

    return 0;
}