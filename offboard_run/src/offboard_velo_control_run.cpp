#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");


    ros::NodeHandle nh;
    ros::Subscriber state_sub0 = nh.subscribe<mavros_msgs::State>
            ("uav0/mavros/state", 10, state_cb);
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>
            ("/uav0/mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("/uav0/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("/uav0/mavros/set_mode");

    // The setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // Wait for FCU connection
    while (ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    // Create a Twist message to command velocity
    geometry_msgs::Twist vel_cmd;
    vel_cmd.linear.x = 0.0;  // Set your desired linear velocity in the x direction
    vel_cmd.linear.y = 0.0;  // Set your desired linear velocity in the y direction
    vel_cmd.linear.z = 3.0;  // Set your desired linear velocity in the z direction
    vel_cmd.angular.x = 0.0; // Set your desired angular velocity in the roll axis
    vel_cmd.angular.y = 0.0; // Set your desired angular velocity in the pitch axis
    vel_cmd.angular.z = 0.0; // Set your desired angular velocity in the yaw axis

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while (ros::ok()){
        if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))){
            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if ((!current_state.armed) && (ros::Time::now() - last_request > ros::Duration(5.0))){
                if (arming_client.call(arm_cmd) && arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        // Publish velocity command
        vel_pub.publish(vel_cmd);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
