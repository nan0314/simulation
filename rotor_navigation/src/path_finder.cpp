#include <ros/ros.h>
#include <cmath>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <waypoint_navigator/GoToWaypoints.h>
#include <nav_msgs/OccupancyGrid.h>
#include <vector>
#include "rotor_navigation/rotor_navigation.hpp"

static bool map_recieved = false;
static bool target_received = false;
static bool target_reached = false;
static rotor_navigation::Map map;
static geometry_msgs::PoseArray targets;
static geometry_msgs::Point land;

geometry_msgs::Point pos2point(std::vector<int> pos, int h){
    geometry_msgs::Point point;
    point.x = pos[0];
    point.y = pos[1];
    point.z = h;

    return point;
}

void map_callback(nav_msgs::OccupancyGrid msg){
    map_recieved = true;
    map = rotor_navigation::Map(msg);
}

void target_callback(geometry_msgs::PoseArray msg){
    target_received = true;
    targets = msg;
}

void pose_callback(geometry_msgs::Pose msg){
    double dist = sqrt(pow(msg.position.x - land.x,2) + pow(msg.position.y - land.y,2) + pow(msg.position.z - land.z,2));
    if (dist < 0.25){
        target_reached = true;
    }
}


int main(int argc, char** argv) {


    using std::vector;

    ros::init(argc, argv, "path_finder");
    ros::NodeHandle nh;
    ros::Subscriber map_sub = nh.subscribe("/risk_map", 10, map_callback);
    ros::Subscriber target_sub = nh.subscribe("/targets", 10, target_callback);
    ros::Subscriber pose_sub = nh.subscribe("/firefly/ground_truth/pose",10,pose_callback);

    ros::ServiceClient waypoints_client = nh.serviceClient<waypoint_navigator::GoToWaypoints>("/firefly/go_to_waypoints");

    int altitude = 20;
    std::vector<int> start = {-75,-20};
    std::vector<int> end = {-50,-20};

    vector<vector<int>> path;

    while (!map_recieved){
        // Check for new messages
        ros::spinOnce();
    }

    while(!target_received){
        ros::spinOnce();
    } 
    ROS_ERROR_STREAM("Targets Recieved"); 
    
    for (auto target : targets.poses){

        end = {int(target.position.x),int(target.position.y)};
        path = map.a_star(start,end);

        // Prepare client request
        waypoint_navigator::GoToWaypoints msg;

        // Flight path to target
        for (auto pos : path){
            geometry_msgs::Point point = pos2point(pos,altitude);
            msg.request.points.push_back(point);
        }

        // Landing command
        land = pos2point(path[path.size()-1],0);
        msg.request.points.push_back(land);

        waypoints_client.call(msg);
        target_reached = false;
        while(!target_reached){
            ros::spinOnce();
        }
        ros::Duration(2).sleep();
        msg.request.points = {};


        // Flight path to home
        reverse(path.begin(), path.end());
        for (auto pos : path){
            geometry_msgs::Point point = pos2point(pos,altitude);
            msg.request.points.push_back(point);
        }

        // Landing command
        land.x = -75;
        land.y = -20;
        msg.request.points.push_back(land);
        waypoints_client.call(msg);
        target_reached = false;
        while(!target_reached){
            ros::spinOnce();
        }
        ros::Duration(2).sleep();
    }

    
    ROS_ERROR_STREAM("Waypoints Published");

    ros::shutdown();
  

  return 0;
}