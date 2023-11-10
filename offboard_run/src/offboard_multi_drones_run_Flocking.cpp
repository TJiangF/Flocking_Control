/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <numeric>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
Eigen::Vector3d Flocking_control(std::vector<Eigen::Vector3d> positions, std::vector<Eigen::Vector3d> velocities = {}, double separation_gain = 1.0, double cohesion_gain = 1.0, double alignment_gain = 1.0, double perception_radius = 0.0, int max_agents = 0) {
    Eigen::MatrixXd pos_matrix(positions.size(), 3);

    for (int i = 0; i < positions.size(); i++) {
        pos_matrix.row(i) = positions[i];
    }

    Eigen::MatrixXd vel_matrix;

    if (velocities.empty()) {
        vel_matrix = Eigen::MatrixXd::Zero(positions.size(), 3);
    } else {
        vel_matrix.resize(velocities.size(), 3);

        for (int i = 0; i < velocities.size(); i++) {
            vel_matrix.row(i) = velocities[i];
        }
    }

    int num_agents = pos_matrix.rows();
    int dims = pos_matrix.cols();

    Eigen::VectorXd distances = pos_matrix.rowwise().norm();
    std::vector<int> indices;

    if (perception_radius > 0.0) {
        for (int i = 0; i < num_agents; i++) {
            if (distances[i] < perception_radius) {
                indices.push_back(i);
            }
        }

        Eigen::VectorXd filtered_distances(indices.size());

        for (int i = 0; i < indices.size(); i++) {
        filtered_distances(i) = distances(indices[i]);
        }

        distances = filtered_distances;
    }

    if (max_agents > 0) {
        std::vector<int> sorted_indices(num_agents);
        std::iota(sorted_indices.begin(), sorted_indices.end(), 0);

        std::partial_sort(sorted_indices.begin(), sorted_indices.begin() + max_agents, sorted_indices.end(),
                          [&distances](int i, int j) { return distances[i] < distances[j]; });

        indices.clear();
        for (int i = 0; i < max_agents; i++) {
            indices.push_back(sorted_indices[i]);
        }
    }

    if (indices.empty()) {
        return Eigen::Vector3d::Zero();
    }

    pos_matrix = pos_matrix.block(0, 0, indices.size(), 3);  // Use block to extract specific rows
    vel_matrix = vel_matrix.block(0, 0, indices.size(), 3);  // Use block to extract specific rows
   

    Eigen::VectorXd distances_inv = (pos_matrix.array() / distances.array().pow(2)).colwise().sum();

    Eigen::Vector3d separation = -separation_gain * distances_inv.normalized();
    Eigen::Vector3d cohesion = cohesion_gain * pos_matrix.colwise().mean();
    Eigen::Vector3d alignment = alignment_gain * vel_matrix.colwise().mean();

    return separation + cohesion + alignment;
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");

    //uav0
    ros::NodeHandle nh0;
    ros::Subscriber state_sub0 = nh0.subscribe<mavros_msgs::State>
            ("uav0/mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub0 = nh0.advertise<geometry_msgs::PoseStamped>
            ("/uav0/mavros/setpoint_position/local", 10);
    ros::Publisher vel_pub0 = nh0.advertise<geometry_msgs::Twist>
            ("/uav0/mavros/velocity", 10);
    ros::ServiceClient arming_client0 = nh0.serviceClient<mavros_msgs::CommandBool>
            ("/uav0/mavros/cmd/arming");
    ros::ServiceClient set_mode_client0 = nh0.serviceClient<mavros_msgs::SetMode>
            ("/uav0/mavros/set_mode");

    //uav1
    ros::NodeHandle nh1;
    ros::Subscriber state_sub1 = nh1.subscribe<mavros_msgs::State>
            ("uav1/mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub1 = nh1.advertise<geometry_msgs::PoseStamped>
            ("/uav1/mavros/setpoint_position/local", 10);
    ros::Publisher vel_pub1 = nh1.advertise<geometry_msgs::Twist>
            ("/uav1/mavros/velocity", 10);
    ros::ServiceClient arming_client1 = nh1.serviceClient<mavros_msgs::CommandBool>
            ("/uav1/mavros/cmd/arming");
    ros::ServiceClient set_mode_client1 = nh1.serviceClient<mavros_msgs::SetMode>
            ("/uav1/mavros/set_mode");

    //uav2
    ros::NodeHandle nh2;
    ros::Subscriber state_sub2 = nh2.subscribe<mavros_msgs::State>
            ("uav2/mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub2 = nh2.advertise<geometry_msgs::PoseStamped>
            ("/uav2/mavros/setpoint_position/local", 10);
    ros::Publisher vel_pub2 = nh2.advertise<geometry_msgs::Twist>
            ("/uav2/mavros/velocity", 10);
    ros::ServiceClient arming_client2 = nh2.serviceClient<mavros_msgs::CommandBool>
            ("/uav2/mavros/cmd/arming");
    ros::ServiceClient set_mode_client2 = nh2.serviceClient<mavros_msgs::SetMode>
            ("/uav2/mavros/set_mode");
	
	/*
    //uav3
    ros::NodeHandle nh3;
    ros::Subscriber state_sub3 = nh3.subscribe<mavros_msgs::State>
            ("uav3/mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub3 = nh3.advertise<geometry_msgs::PoseStamped>
            ("/uav3/mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client3 = nh3.serviceClient<mavros_msgs::CommandBool>
            ("/uav3/mavros/cmd/arming");
    ros::ServiceClient set_mode_client3 = nh3.serviceClient<mavros_msgs::SetMode>
            ("/uav3/mavros/set_mode");
            */

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
    float x1 = 0.0, y1 = 0.0;
    float x2 = 0.0, y2 = 0.0;
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
    float x0_offset = -3;
    float x1_offset = -1;
    float x2_offset = 1;


    //define uavs' coordinates in the formation coordinate system
    float x0_f = -0.5, y0_f = 0.5;
    float x1_f = 0.5, y1_f = 0.5;
    float x2_f = -0.5, y2_f = -0.5;

    float xx, yy;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub0.publish(pose0);
        local_pos_pub1.publish(pose1);
        local_pos_pub2.publish(pose2);
        //local_pos_pub3.publish(pose3);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
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
        //local_pos_pub3.publish(pose3);

        w = w + 2*pi/(30/(1/20.0));
        if(w > 2*pi){
            w = w - 2*pi;
        }
        xx = 8*cos(w);
        yy = 8*sin(w);
        
        x0 = x0_f*cos(w) - y0_f*sin(w) + xx - x0_offset;
        y0 = y0_f*cos(w) + x0_f*sin(w) + yy;
        pose0.pose.position.x = x0;
        pose0.pose.position.y = y0;

        std::vector<Eigen::Vector3d> positions;
        positions.push_back(Eigen::Vector3d(pose0.pose.position.x, pose0.pose.position.y, pose0.pose.position.z));  // Agent 1 position (x=1, y=2, z=3)
        positions.push_back(Eigen::Vector3d(pose1.pose.position.x, pose1.pose.position.y, pose1.pose.position.z));  // Agent 2 position (x=4, y=5, z=6)
        positions.push_back(Eigen::Vector3d(pose2.pose.position.x, pose2.pose.position.y, pose2.pose.position.z));  // Agent 3 position (x=7, y=8, z=9)
        Eigen::Vector3d new_positions = Flocking_control(positions);
        //ROS_INFO(new_positions);
        
        x1 = x1_f*cos(w) - y1_f*sin(w) + xx - x1_offset;
        y1 = y1_f*cos(w) + x1_f*sin(w) + yy;
        pose1.pose.position.x = x1;
        pose1.pose.position.y = y1;

        x2 = x2_f*cos(w) - y2_f*sin(w) + xx - x2_offset;
        y2 = y2_f*cos(w) + x2_f*sin(w) + yy;
        pose2.pose.position.x = x2;
        pose2.pose.position.y = y2;
        /*
        x3 = x3_f*cos(w) - y3_f*sin(w) + xx - x3_offset;
        y3 = y3_f*cos(w) + x3_f*sin(w) + yy;
        pose3.pose.position.x = x3;
        pose3.pose.position.y = y3;
        */

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}


