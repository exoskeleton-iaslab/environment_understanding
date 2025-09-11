#include "utility.h"

bool new_hip=false;
bool new_foot=false;
bool new_pivot=false;
bool new_obs=false;
bool moving=false;

void hip_height_cb(const std_msgs::Float32::ConstPtr& msg){
	new_hip=true;
}
void pivot_cb(const std_msgs::Float32::ConstPtr& msg){
	new_pivot=true;
}
void foothold_cb(const std_msgs::Float32::ConstPtr& msg){
	new_foot=true;
}
void obstacle_cb(const std_msgs::Float32MultiArray::ConstPtr& msg){
	new_obs=true;
}
void is_moving_cb(const std_msgs::Bool::ConstPtr& msg) {
    moving=msg->data;
}

int main (int argc, char** argv) {
 ros::init (argc, argv, "cfftg_starter");
 ros::NodeHandle nh;
 ros::Subscriber sub_hip_height = nh.subscribe ("/CoM_height", 1, hip_height_cb);
 ros::Subscriber sub_pivot = nh.subscribe ("/pivot", 1, pivot_cb);
 ros::Subscriber sub_foothold = nh.subscribe ("/foothold", 1, foothold_cb);
 ros::Subscriber sub_obstacle = nh.subscribe ("/obstacle_shape", 1, obstacle_cb);
 ros::Subscriber is_moving = nh.subscribe ("/exoskeleton/is_moving", 1, is_moving_cb);
 ros::Publisher cfftg_pub = nh.advertise<std_msgs::Bool> ("cfftg_start", 1);
 std_msgs::Bool bmsg;
 bmsg.data=false;
 std::string s;
 bool new_data=false;

 while(ros::ok()){
    try {
        new_data = new_hip && new_pivot && new_foot && new_obs;
        if (new_data && !moving) {
            std::cout << "All data received. Starting CFFTG" << std::endl;
            bmsg.data = true;
            cfftg_pub.publish(bmsg);
            ros::Duration(0.05).sleep();
            bmsg.data = false;
            new_hip = false;
            new_pivot = false;
            new_foot = false;
            new_obs = false;
        }
        ros::spinOnce();
    }
    catch (const ros::Exception &e) {
        std::cout << "ROS exception: " << e.what() << std::endl;
        moving = false;
        continue;
    }
    catch (const std::exception &e) {
        std::cout << "Standard exception: " << e.what() << std::endl;
        moving = false;
        continue;
    }
    catch (...) {
        std::cout << "Unknown exception occurred." << std::endl;
        moving = false;
        continue;
    }
 }
}
