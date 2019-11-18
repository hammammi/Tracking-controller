#include <vehicle_control/tracking_control.hpp>

int main(int argc, char** argv)
{
    ros::init(argc,argv,"tracking_controller");

    tracking_controller track_control;

    track_control.cmd_vel_pub_setting();
    track_control.subscriber_declaration();

    ros::Rate loop_rate(200);

    while(ros::ok()){
        track_control.cmd_vel_pub();
        ros::spinOnce();
        loop_rate.sleep();
    }

}