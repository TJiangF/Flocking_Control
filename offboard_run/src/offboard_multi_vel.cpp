/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <numeric>
#include <cmath>

// A function to calculate the relative position between two poses
geometry_msgs::Point calculate_Relative_Position(const geometry_msgs::PoseStamped& from, const geometry_msgs::PoseStamped& to) {
    geometry_msgs::Point relative_position;
    relative_position.x = to.pose.position.x - from.pose.position.x;
    relative_position.y = to.pose.position.y - from.pose.position.y;
    relative_position.z = to.pose.position.z - from.pose.position.z;
    return relative_position;
}

geometry_msgs::Point calculate_Relative_Vel(const geometry_msgs::TwistStamped& from, const geometry_msgs::TwistStamped& to) {
    geometry_msgs::Point relative_vel;
    relative_vel.x = to.twist.linear.x - from.twist.linear.x;
    relative_vel.y = to.twist.linear.y - from.twist.linear.y;
    relative_vel.z = to.twist.linear.z - from.twist.linear.z;
    return relative_vel;
}

std::vector<std::vector<geometry_msgs::Point>> calculateNormalizedPositionsMatrix(
    const std::vector<std::vector<geometry_msgs::Point>>& relative_positions) {
    
    std::vector<std::vector<geometry_msgs::Point>> normalized_positions_matrix(3, std::vector<geometry_msgs::Point>(3));

    for (size_t i = 0; i < relative_positions.size(); ++i) {
        for (size_t j = 0; j < relative_positions[i].size(); ++j) {
            if (i != j) { // Skip the diagonal
                const auto& point = relative_positions[i][j];
                double distance_square = point.x * point.x + point.y * point.y + point.z * point.z;
                
                if (distance_square != 0) {
                    geometry_msgs::Point normalized_point;
                    normalized_point.x = point.x / distance_square;
                    normalized_point.y = point.y / distance_square;
                    normalized_point.z = point.z / distance_square;
                    normalized_positions_matrix[i][j] = normalized_point;
                }
            }
            // If on the diagonal, you can choose to leave it as default (0,0,0) or handle it differently
        }
    }

    return normalized_positions_matrix;
}

std::vector<geometry_msgs::Point> calculate_mean_distance(const std::vector<std::vector<geometry_msgs::Point>>& dist_inv) {
    std::vector<geometry_msgs::Point> means(dist_inv.size());

    for (size_t i = 0; i < dist_inv.size(); ++i) {
        double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
        int count = 0;
        for (size_t j = 0; j < dist_inv[i].size(); ++j) {
            if (i != j) {
            sum_x += dist_inv[i][j].x;
            sum_y += dist_inv[i][j].y;
            sum_z += dist_inv[i][j].z;
            count++;
            }
        }
        // Calculate the mean for each component
        // means[i].x = sum_x / dist_inv[i].size();
        // means[i].y = sum_y / dist_inv[i].size();
        // means[i].z = sum_z / dist_inv[i].size();

        means[i].x = sum_x / count;
        means[i].y = sum_y / count;
        means[i].z = sum_z / count;
    }

    return means;
}

std::vector<geometry_msgs::Point> calculate_mean_rel_val(
    const std::vector<std::vector<geometry_msgs::Point>>& relative_positions) {
    
//     std::vector<geometry_msgs::Point> means;
    std::vector<geometry_msgs::Point> means(relative_positions.size());

    means.reserve(relative_positions.size());

    for (size_t i = 0; i < relative_positions.size(); ++i) {
        double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
        int count = 0;
        for (size_t j = 0; j < relative_positions[i].size(); ++j) {
            if (i != j) { // Exclude diagonal elements
                sum_x += relative_positions[i][j].x;
                sum_y += relative_positions[i][j].y;
                sum_z += relative_positions[i][j].z;
                ++count;
            }
        }

        // Only calculate the mean if there were any off-diagonal elements
        if (count > 0) {
            means[i].x = sum_x / count;
            means[i].y = sum_y / count;
            means[i].z = sum_z / count;
        }
    }

    return means;
}

std::vector<geometry_msgs::Point> Flocking_control(const std::vector<std::vector<geometry_msgs::Point>>& positions, 
                                     const std::vector<std::vector<geometry_msgs::Point>>& relative_vel,
                                     double separation_gain = 1.0, 
                                     double cohesion_gain = 1.0,
                                     double alignment_gain = 1.0) {

    /*
    dist_inv = positions / distances[:, np.newaxis] ** 2

    # Renolds command computations
    separation = -separation_gain * dist_inv.mean(axis=0)  # Sum may lead to instabilities
    cohesion = cohesion_gain * positions.mean(axis=0)
    alignment = alignment_gain * velocities.mean(axis=0)

    */
            
    std::vector<std::vector<geometry_msgs::Point>> dist_inv(3, std::vector<geometry_msgs::Point>(2));
    dist_inv = calculateNormalizedPositionsMatrix(positions);  // 3x3

    std::vector<geometry_msgs::Point> dist_inv_means(dist_inv.size());  //3x1
    dist_inv_means = calculate_mean_distance(dist_inv);

    std::vector<geometry_msgs::Point> positions_means(positions.size()); // 3x1
    positions_means = calculate_mean_rel_val(positions);

    std::vector<geometry_msgs::Point> vel_means(relative_vel.size()); // 3x1
    vel_means = calculate_mean_rel_val(relative_vel);

    std::vector<geometry_msgs::Point> final_vel(3);

    for (size_t i = 0; i < final_vel.size(); ++i) {
        final_vel[i].x = -separation_gain * dist_inv_means[i].x + cohesion_gain * positions_means[i].x + alignment_gain * vel_means[i].x;
        final_vel[i].y = -separation_gain * dist_inv_means[i].y + cohesion_gain * positions_means[i].y + alignment_gain * vel_means[i].y;
        final_vel[i].z = -separation_gain * dist_inv_means[i].z + cohesion_gain * positions_means[i].z + alignment_gain * vel_means[i].z;
    }

    return final_vel;
}


// state save
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

// pose save
geometry_msgs::PoseStamped uav0_pose;
void pose_uav0(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    uav0_pose = *msg;
}

geometry_msgs::PoseStamped uav1_pose;
void pose_uav1(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    uav1_pose = *msg;
}

geometry_msgs::PoseStamped uav2_pose;
void pose_uav2(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    uav2_pose = *msg;
}


// velocity save
geometry_msgs::TwistStamped uav0_twist;
void twist_uav0(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    uav0_twist = *msg;
}

geometry_msgs::TwistStamped uav1_twist;
void twist_uav1(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    uav1_twist = *msg;
}

geometry_msgs::TwistStamped uav2_twist;
void twist_uav2(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    uav2_twist = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");

    //uav0
    ros::NodeHandle nh0;
    ros::Subscriber state_sub0 = nh0.subscribe<mavros_msgs::State>
            ("uav0/mavros/state", 10, state_cb);
    ros::Subscriber pose_sub0 = nh0.subscribe<geometry_msgs::PoseStamped>
            ("/uav0/mavros/local_position/pose", 10, pose_uav0);
    ros::Subscriber twist_sub0 = nh0.subscribe<geometry_msgs::TwistStamped>
            ("/uav0/mavros/local_position/velocity_local", 10, twist_uav0);

    ros::Publisher local_pos_pub0 = nh0.advertise<geometry_msgs::PoseStamped>
            ("/uav0/mavros/setpoint_position/local", 10);
    // add velocity control
    ros::Publisher local_vel_pub0 = nh0.advertise<geometry_msgs::Twist>
            ("/uav0/mavros/setpoint_velocity/cmd_vel_unstamped", 10);

    ros::ServiceClient arming_client0 = nh0.serviceClient<mavros_msgs::CommandBool>
            ("/uav0/mavros/cmd/arming");
    ros::ServiceClient set_mode_client0 = nh0.serviceClient<mavros_msgs::SetMode>
            ("/uav0/mavros/set_mode");
    

    //uav1
    ros::NodeHandle nh1;
    ros::Subscriber state_sub1 = nh1.subscribe<mavros_msgs::State>
            ("uav1/mavros/state", 10, state_cb);
    ros::Subscriber pose_sub1 = nh1.subscribe<geometry_msgs::PoseStamped>
            ("/uav1/mavros/local_position/pose", 10, pose_uav1);
    ros::Subscriber twist_sub1 = nh1.subscribe<geometry_msgs::TwistStamped>
            ("/uav1/mavros/local_position/velocity_local", 10, twist_uav1);

    ros::Publisher local_pos_pub1 = nh1.advertise<geometry_msgs::PoseStamped>
            ("/uav1/mavros/setpoint_position/local", 10);
    // add velocity control
    ros::Publisher local_vel_pub1 = nh1.advertise<geometry_msgs::Twist>
            ("/uav1/mavros/setpoint_velocity/cmd_vel_unstamped", 10);

    ros::ServiceClient arming_client1 = nh1.serviceClient<mavros_msgs::CommandBool>
            ("/uav1/mavros/cmd/arming");
    ros::ServiceClient set_mode_client1 = nh1.serviceClient<mavros_msgs::SetMode>
            ("/uav1/mavros/set_mode");


    //uav2
    ros::NodeHandle nh2;
    ros::Subscriber state_sub2 = nh2.subscribe<mavros_msgs::State>
            ("uav2/mavros/state", 10, state_cb);
    ros::Subscriber pose_sub2 = nh2.subscribe<geometry_msgs::PoseStamped>
            ("/uav2/mavros/local_position/pose", 10, pose_uav2);
    ros::Subscriber twist_sub2 = nh2.subscribe<geometry_msgs::TwistStamped>
            ("/uav2/mavros/local_position/velocity_local", 10, twist_uav2);

    ros::Publisher local_pos_pub2 = nh2.advertise<geometry_msgs::PoseStamped>
            ("/uav2/mavros/setpoint_position/local", 10);
    ros::Publisher local_vel_pub2 = nh2.advertise<geometry_msgs::Twist>
            ("/uav2/mavros/setpoint_velocity/cmd_vel_unstamped", 10);

    ros::ServiceClient arming_client2 = nh2.serviceClient<mavros_msgs::CommandBool>
            ("/uav2/mavros/cmd/arming");
    ros::ServiceClient set_mode_client2 = nh2.serviceClient<mavros_msgs::SetMode>
            ("/uav2/mavros/set_mode");
                // add velocity control


    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    // x_num, y_num  refer to the desired uav input position
    //all the uavs have the same height : z

    float x0 = 0.0, y0 = 0.0;
    float x1 = 1.0, y1 = 0.0;
    float x2 = 0.0, y2 = 1.0;
    //float x3 = 0.0, y3 = 0.0;
    float z = 3.0;
    float w = 0.0;
    const float pi = 3.1415926;
    geometry_msgs::PoseStamped pose0;
    geometry_msgs::PoseStamped pose1;
    geometry_msgs::PoseStamped pose2;
    
    pose0.pose.position.x = x0;
    pose0.pose.position.y = y0;
    pose0.pose.position.z = z;
    pose1.pose.position.x = x1;
    pose1.pose.position.y = y1;
    pose1.pose.position.z = z;
    pose2.pose.position.x = x2;
    pose2.pose.position.y = y2;
    pose2.pose.position.z = z;


    //uavs have the initial position offset because of the .launch config file
    float x0_offset = 0;
    //float x1_offset = -1;
    //float x2_offset = 1;


    //define uavs' coordinates in the formation coordinate system
    // float x0_f = -0.5, y0_f = 0.5;
    // float x1_f = 0.5, y1_f = 0.5;
    // float x2_f = -0.5, y2_f = -0.5;

    float x0_f = 0.0, y0_f = 0.0;
    float x1_f = 1.0, y1_f = 0.0;
    float x2_f = 0.0, y2_f = 1.0;

    float xx, yy;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub0.publish(pose0);
        local_pos_pub1.publish(pose1);
        local_pos_pub2.publish(pose2);
        //ROS_INFO("should takeoff");
        //local_pos_pub3.publish(pose3);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    int takeoff_time_control = 0;
    int time_for_take_off = 5;
    bool flag = false;
    while(ros::ok() && !flag){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client0.call(offb_set_mode) &&
                set_mode_client1.call(offb_set_mode) &&
                set_mode_client2.call(offb_set_mode) &&
                //set_mode_client3.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled : 3");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client0.call(arm_cmd) &&
                    arming_client1.call(arm_cmd) &&
                    arming_client2.call(arm_cmd) &&
                    //arming_client3.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed : 3");
                }
                last_request = ros::Time::now();
            }
        }
        local_pos_pub0.publish(pose0);
        local_pos_pub1.publish(pose1);
        local_pos_pub2.publish(pose2);
        if(uav0_pose.pose.position.z > (z-0.3) && uav0_pose.pose.position.z < (z+0.3) && uav1_pose.pose.position.z > (z-0.3) && uav1_pose.pose.position.z < (z+0.3) && uav2_pose.pose.position.z > (z-0.3) && uav2_pose.pose.position.z < (z+0.3)){
            flag = true;
        }

        ros::spinOnce();
        rate.sleep();
        

    }
    while(ros::ok() && flag){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client0.call(offb_set_mode) &&
                set_mode_client1.call(offb_set_mode) &&
                set_mode_client2.call(offb_set_mode) &&
                //set_mode_client3.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled : 3");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client0.call(arm_cmd) &&
                    arming_client1.call(arm_cmd) &&
                    arming_client2.call(arm_cmd) &&
                    //arming_client3.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed : 3");
                }
                last_request = ros::Time::now();
            }
        }

        // local_pos_pub0.publish(pose0);
        // local_pos_pub1.publish(pose1);
        // local_pos_pub2.publish(pose2);

        // last_request = ros::Time::now();
        // if((ros::Time::now() - last_request <= ros::Duration(5.0))){
        //         pose0.pose.position.x = 0;
        //         pose0.pose.position.y = 0;
        //         pose0.pose.position.z = z;
        //         pose1.pose.position.x = 1;
        //         pose1.pose.position.y = 0;
        //         pose1.pose.position.z = z;
        //         pose2.pose.position.x = 0;
        //         pose2.pose.position.y = 1;
        //         pose2.pose.position.z = z;

        //         local_pos_pub0.publish(pose0);
        //         local_pos_pub1.publish(pose1);
        //         local_pos_pub2.publish(pose2);

        //         ros::spinOnce();
        //         rate.sleep();

        //         continue;
        // }

        w = w + 2*pi/(30/(1/20.0));
        if(w > 2*pi){
            w = w - 2*pi;
        }
        xx = 6*cos(w);
        yy = 6*sin(w);
        
        x0 = x0_f*cos(w) - y0_f*sin(w) + xx - x0_offset;// xx = 0 x0_offset = 0
        y0 = y0_f*cos(w) + x0_f*sin(w) + yy;
        pose0.pose.position.x = x0;
        pose0.pose.position.y = y0;

        // x1 = x1_f*cos(w) - y1_f*sin(w) + xx - x1_offset;
        // y1 = y1_f*cos(w) + x1_f*sin(w) + yy;
        // pose1.pose.position.x = x1;
        // pose1.pose.position.y = y1;

        // x2 = x2_f*cos(w) - y2_f*sin(w) + xx - x2_offset;
        // y2 = y2_f*cos(w) + x2_f*sin(w) + yy;
        // pose2.pose.position.x = x2;
        // pose2.pose.position.y = y2;


        // start calculate relative position
        std::vector<std::vector<geometry_msgs::Point>> relative_positions(3, std::vector<geometry_msgs::Point>(3));

        // Storing the relative positions in the matrix
        relative_positions[0][1] = calculate_Relative_Position(uav0_pose, uav1_pose);
        relative_positions[0][2] = calculate_Relative_Position(uav0_pose, uav2_pose);

        relative_positions[1][0] = calculate_Relative_Position(uav1_pose, uav0_pose);
        relative_positions[1][2] = calculate_Relative_Position(uav1_pose, uav2_pose);

        relative_positions[2][0] = calculate_Relative_Position(uav2_pose, uav0_pose);
        relative_positions[2][1] = calculate_Relative_Position(uav2_pose, uav1_pose);

        std::vector<std::vector<geometry_msgs::Point>> relative_vel(3, std::vector<geometry_msgs::Point>(3));
        relative_vel[0][1] = calculate_Relative_Vel(uav0_twist, uav1_twist);
        relative_vel[0][2] = calculate_Relative_Vel(uav0_twist, uav2_twist);

        relative_vel[1][0] = calculate_Relative_Vel(uav1_twist, uav0_twist);
        relative_vel[1][2] = calculate_Relative_Vel(uav1_twist, uav2_twist);

        relative_vel[2][0] = calculate_Relative_Vel(uav2_twist, uav0_twist);
        relative_vel[2][1] = calculate_Relative_Vel(uav2_twist, uav1_twist);

        std::vector<geometry_msgs::Point> combined_vel(3);
        combined_vel = Flocking_control(relative_positions, relative_vel);

        // ROS_INFO("v1 x: [%f], y: [%f], z: [%f]", combined_vel[1].x, combined_vel[1].y, combined_vel[1].z);

        geometry_msgs::Twist v1;
        geometry_msgs::Twist v2;

        v1.linear.x = combined_vel[1].x;
        v1.linear.y = combined_vel[1].y;
        v1.linear.z = combined_vel[1].z;

        v2.linear.x = combined_vel[2].x;
        v2.linear.y = combined_vel[2].y;
        v2.linear.z = combined_vel[2].z;
        v1.linear.x = combined_vel[1].x;
        v1.linear.y = combined_vel[1].y;
        v1.linear.z = combined_vel[1].z;

        v2.linear.x = combined_vel[2].x;
        v2.linear.y = combined_vel[2].y;
        v2.linear.z = combined_vel[2].z;
        
        local_pos_pub0.publish(pose0);
        local_vel_pub1.publish(v1);
        local_vel_pub2.publish(v2);


        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}


