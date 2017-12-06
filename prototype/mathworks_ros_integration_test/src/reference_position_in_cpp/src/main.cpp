#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sstream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "reference_position_in_cpp");
    ros::NodeHandle n;
    ros::Publisher ref_publisher = n.advertise<std_msgs::Float64>("reference", 1);
    ros::Rate loop_rate(50);
    int count = 0;
    while (ros::ok())
    {
        std_msgs::Float64 msg;
        msg.data = (count/20)%4+1;
        ref_publisher.publish(msg);
        loop_rate.sleep();
        count++;
    }

    return 0;
}