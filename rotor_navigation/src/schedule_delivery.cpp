#include <ros/ros.h>
#include <cmath>
#include <vector>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include "rotor_navigation/rotor_navigation.hpp"

std::vector<std::vector<int>> getTargets(){
    using std::cout;
    using std::cin;
    using std::endl;
    using std::vector;

    vector<vector<int>> targets;
    
    bool done = false;
    while(!done){
        cout << "\nPick a location:\n" << std::endl;
        cout << "1. House 1 (-37,-18)\n" << "2. House 2 (-33,29)\n" << "3. House 3 (-5,-28)\n"
            << "4. House 4 (93,11)\n" << "5. House 5 (56,26)\n" << "6. Office 1 (9,6)"
            << "7. Restaurant 1 (24,-8)\n" << "8. Apartment 1 (87,10)\n" << "9. Done" << std::endl;

        int input;

        cin >> input;

        int num = int(input);

        switch (input)
        {
            case 1: // code to be executed if n = 1;
                targets.push_back({-37,-18});
                break;
            case 2: // code to be executed if n = 2;
                targets.push_back({-33,29});
                break;
            case 3:
                targets.push_back({-5,-28});
                break;
            case 4:
                targets.push_back({93,11});
                break;
            case 5:
                targets.push_back({56,26});
                break;
            case 6:
                targets.push_back({9,6});
                break;
            case 7:
                targets.push_back({24,-8});
                break;
            case 8:
                targets.push_back({87,10});
                break;
            case 9:
                done = true;
            default: // code to be executed if n doesn't match any cases
                break;
        }
        
    }
    
    return targets;
}


geometry_msgs::PoseArray vec2poses(std::vector<std::vector<int>> vec){

    geometry_msgs::PoseArray targets;

    for (auto pos : vec){
        geometry_msgs::Pose point;
        point.position.x = pos[0];
        point.position.y = pos[1];

        targets.poses.push_back(point);
    }

    return targets;
}


int main(int argc, char** argv) {

    using std::vector;
    

    ros::init(argc, argv, "schedule_delivery");
    ros::NodeHandle nh;
    ros::Publisher target_pub =
        nh.advertise<geometry_msgs::PoseArray>(
        "/targets", 10,true);


    vector<vector<int>> targets = getTargets();

    geometry_msgs::PoseArray msg = vec2poses(targets);

    while (ros::ok()){
        target_pub.publish(msg);
        ros::spinOnce();
    }
  

  return 0;
}