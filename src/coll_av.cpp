#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include "laser_geometry/laser_geometry.h"
#include <tf/transform_listener.h>
#include <math.h>

#define ROTATION 0.5
#define SAFETY_DISTANCE 0.07

tf::TransformListener* listener;
ros::Publisher vel_pub;
geometry_msgs::Twist adj_msg;
geometry_msgs::Twist speed;

void cmd_callback(const geometry_msgs::Twist& msg){
    if(msg.linear.x!=0 || msg.angular.z!=0){ //mi interessa conoscere le velocita del robot in movimento
		ROS_INFO("\ninput  v: %f  input a : %f", msg.linear.x, msg.angular.z);
	}
    speed = msg; //mi salvo la velocita di input
}

void laser_scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_in){
	
	laser_geometry::LaserProjection projector;
    sensor_msgs::PointCloud cloud;
    
    try{
		listener->waitForTransform(scan_in->header.frame_id, "/base_link", ros::Time(0), ros::Duration(1.0));
		projector.transformLaserScanToPointCloud("/base_link", *scan_in, cloud, *listener); // passa i punti della scan al cloud
	}catch(tf::TransformException &e){
        return;
        }
    
    geometry_msgs::Twist dummy_v;
    dummy_v.linear.x = 0;
    dummy_v.angular.z = 0;
    speed = dummy_v;
    
    adj_msg = dummy_v; 

    for(int i = 0; i < cloud.points.size(); i++){
		ROS_INFO("CI SONO");

        float obs_dis = cloud.points[i].x*cloud.points[i].x + cloud.points[i].y*cloud.points[i].y; //distanza dall'ostacolo

        if(obs_dis < SAFETY_DISTANCE){
			ROS_WARN("Obstacle too near, distance : %f ! Possible collision detected",obs_dis);
		
            adj_msg.linear.x -= obs_dis; //va nella direzione opposta rispetto all'ostacolo
            adj_msg.angular.z -= ROTATION;//ruota a dx
                
            ROS_INFO("\n correction on v_x: %f correction of a_z: %f", adj_msg.linear.x, adj_msg.angular.z);
			vel_pub.publish(adj_msg);
            }
        else{
			vel_pub.publish(dummy_v);
			}
		}
}

int main(int argc, char* argv[]){
    ros::init(argc, argv, "col_av");
    ros::NodeHandle n; 
    
    ros::Subscriber scan_sub = n.subscribe("/base_scan", 1, scanCallback); 
    ros::Subscriber cmd_sub = n.subscribe("/cmd_vel", 1, cmd_callback);
    
    vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

    ros::spin();

    return 0;
}

