#include <ros/ros.h>
#include <math.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>

    double x = -1.8022716576; 
    double y = -269.8458315553; 
    double th = 0.0;

    double vx = 0.0;	///speed
    double vy = 0.0;	//steer
    double vth = 0.0; 

    double steering_factor = 43.0;
    double wheelbase = 1.765;
    double steering_angle = 0.0;
    double dt ;


    ros::Time current_time, last_time;

//speedsteer topic
    

void messageCallback(const geometry_msgs::PointStamped::ConstPtr &point_stamped1) 
{
    vy = point_stamped1->point.x;//bias?
    if(abs(vy) <= 9.5)
        vy = -1;
    vx = point_stamped1->point.y * 1000 / 3600; 

    //ROS_INFO("The speed is: x:%.f, the steer is y:%.f", vx, vy);

    current_time = ros::Time::now();

        
    double dt = (current_time - last_time).toSec(); 
    double delta_x = (vx * sin(th)) * dt;
    double delta_y = (vx * cos(th)) * dt;

        
    steering_angle = vy / steering_factor * M_PI / 180.0; //corrected, rad
    /*while (steering_angle > M_PI)  steering_angle -= 2.0 * M_PI;
    while (steering_angle < -M_PI) steering_angle += 2.0 * M_PI;*/
    
    double delta_th = vx * tan(steering_angle) / wheelbase * dt; // w*dt

    x -= delta_x;
    y += delta_y;
    th += delta_th;
    //ROS_INFO("The th is: x:%.20f", th);
    last_time = current_time;

   
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odometer");
    ros::NodeHandle n;

    ros::Subscriber SpeedSteer_sub = n.subscribe<geometry_msgs::PointStamped>("speedsteer", 1, messageCallback);
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 1);

    //show path
    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("trajectory",10);
    nav_msgs::Path path;
    geometry_msgs::PoseStamped this_pose_stamped;
    //show path

    tf::TransformBroadcaster odom_broadcaster;

    current_time = ros::Time::now();
    last_time = ros::Time::now();

    geometry_msgs::Quaternion odom_quat;


    ros::Rate r(10);

    while(n.ok())
    {
        ros::spinOnce();            
        current_time = ros::Time::now();
    
        odom_quat = tf::createQuaternionMsgFromYaw(th);
    
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "vehicle";

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        odom_broadcaster.sendTransform(odom_trans);

        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
    
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        odom.child_frame_id = "vehicle";
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.angular.z = vx * tan(steering_angle) / wheelbase;

    //path
        path.header.stamp=current_time;
        path.header.frame_id="odom";
    
        this_pose_stamped.pose.position.x = x;
        this_pose_stamped.pose.position.y = y;
        this_pose_stamped.pose.position.z = 0.0;
        this_pose_stamped.pose.orientation = odom_quat;

        this_pose_stamped.header.stamp = current_time;
        this_pose_stamped.header.frame_id = "odom";
        path.poses.push_back(this_pose_stamped);
        path_pub.publish(path);
    //path

    //publish the message
        odom_pub.publish(odom);
    
        r.sleep();
    }
 return 0;
}
