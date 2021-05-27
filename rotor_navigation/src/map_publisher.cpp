#include <ros/ros.h>
#include <cmath>
#include <nav_msgs/OccupancyGrid.h>
#include "rotor_navigation/rotor_navigation.hpp"


int main(int argc, char** argv) {

    using rotor_navigation::Map;

    ros::init(argc, argv, "map_publisher");
    ros::NodeHandle nh;
    ros::Publisher map_pub =
        nh.advertise<nav_msgs::OccupancyGrid>(
        "/risk_map", 10,true);

    // Intialize the map parameters
    int height = 200;
    int width = 100;
    int ox = 127;
    int oy = 50;

    // Create Map object
    Map map(height, width, ox, oy);

    // Build the map

    double tower_dist;
    for (int x = 32; x >= -6; x--){        // Cell tower
        for (int y = -4; y >= -42; y--){

            tower_dist = sqrt(pow(x - 13,2) + pow(y - -23,2));
            if (x >= 8 & x <= 19 & y >= -28 & y <= -17){
                map.setPoint(x,y,char(-1));
            } else if (tower_dist < 19 & tower_dist > 13){
                map.setPoint(x,y,char(1));
            } else if (tower_dist < 19 & tower_dist > 5){
                map.setPoint(x,y,char(2));
            } 
        }
    }

    for (int x = -19; x >= -41; x--){       // Bad neighborhood
        for (int y = 41; y >= 4; y--){
            if (y < 38 & y > 7 & x <-22 and x > -38){
                map.setPoint(x,y,char(2));
            } else {
                map.setPoint(x,y,char(1));
            }
        }
    }

    // map.print();

    // Convert Map object into nav_msgs::OccupancyGrid message
    nav_msgs::OccupancyGrid risk_map = map.getGrid();

    while (ros::ok()){
        map_pub.publish(risk_map);
        ros::spinOnce();
    }
    
    ros::shutdown();

  return 0;
}