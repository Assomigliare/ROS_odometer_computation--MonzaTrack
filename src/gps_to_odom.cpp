#include <ros/ros.h>
#include <math.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <deque>
#include <numeric>
#include <vector>
#include <numeric>

//LLA
    double latitude = 0.0;
    double longitude = 0.0;
    double altitude = 0.0;  
    
    double lat_r = 0.0;
    double lon_r = 0.0;
    double alt_r = 0.0;

    double X = -1.8022716576;
    double Y = -269.8458315553;
    double Z = 0.0;

    double pre_X = 0.0;
    double pre_Y = 0.0;
    double pre_Z = 0.0;

    double th;

    double a = 6378137.0; //m
    double b = 6356752.0; //m

    int i = 0;


void comp_ECEF_ENU()
{
        //compuECEF
        double x, y, z, N, e, dx, dy, dz;
        e = 1 - (b * b) / (a * a);
        N = a / (sqrt(1 - pow(e, 2) * pow(sin(latitude * M_PI / 180), 2)));
        x = (N + altitude) * cos(latitude * M_PI / 180) * cos(longitude * M_PI / 180);
        y = (N + altitude) * cos(latitude * M_PI / 180) * sin(longitude * M_PI / 180);
        z = (N * (1 - pow(e, 2)) + altitude) * sin(latitude * M_PI / 180);
    
        //compuENU
        dx = x - lat_r;
        dy = y - lon_r;
        dz = z - alt_r;
        X = (-1) * sin(lon_r * M_PI / 180) * dx + cos(lon_r * M_PI / 180) * dy;
        Y = (-1) * sin(lat_r * M_PI / 180) * cos(lon_r * M_PI /180) * dx 
            + (-1) * sin(lat_r * M_PI / 180) * sin(lon_r * M_PI /180) * dy 
            + cos(lat_r * M_PI / 180) * dz;
        Z = cos(lat_r * M_PI / 180) * cos(lon_r * M_PI /180) * dx + cos(lat_r * M_PI / 180) * sin(lon_r * M_PI /180) * dy + sin(lat_r * M_PI / 180) * dz;
}


void gpsCallback() {
    std::deque<double> latitude_queue;
    std::deque<double> longitude_queue;
    const size_t window_size = 5; 
    
    
    latitude_queue.push_back(latitude);
    longitude_queue.push_back(longitude);

    

    
    if (latitude_queue.size() > window_size) {
        latitude_queue.pop_front();
        longitude_queue.pop_front();
    }

    
    double avg_latitude = std::accumulate(latitude_queue.begin(), latitude_queue.end(), 0.0) / latitude_queue.size();
    double avg_longitude = std::accumulate(longitude_queue.begin(), longitude_queue.end(), 0.0) / longitude_queue.size();

    latitude = avg_latitude;
    longitude = avg_longitude;
    //  avg_latitude and avg_longitude 
}

void gps_messageCallback(const sensor_msgs::NavSatFix::ConstPtr &NavSatFix1) 
{
    if(NavSatFix1->latitude != 0 || NavSatFix1->longitude != 0){
        latitude = NavSatFix1->latitude;
        longitude = NavSatFix1->longitude;
        altitude = NavSatFix1->altitude;
    }    

    gpsCallback();

    if(i == 0){
        lat_r = latitude;
        lon_r = longitude;
        alt_r = altitude;
        
        pre_X = latitude;
        pre_Y = longitude;
        pre_Z = altitude;

        i++;

    }


    //ROS_INFO("The latitude is: x:%.10f, the longitude is y:%.10f, the altitude is z:%.f", latitude, longitude, altitude);
    
    comp_ECEF_ENU();
    
    //ROS_INFO("The X is: x:%.10f, the Y is y:%.10f, the Z is z:%.f", X, Y, Z);
    th = atan2(Y - pre_Y, X - pre_X);
}

//compute the steering factor


int main(int argc, char **argv)
{
    ros::init(argc, argv, "gps_odometer");
    ros::NodeHandle n;

    ros::Subscriber gps_sub = n.subscribe<sensor_msgs::NavSatFix>("swiftnav/front/gps_pose", 1, gps_messageCallback);
    ros::Publisher gps_odom_pub = n.advertise<nav_msgs::Odometry>("gps_odom", 1);

    //show path
    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("trajectory",10);
    nav_msgs::Path path;
    geometry_msgs::PoseStamped this_pose_stamped;
    //show path

    tf::TransformBroadcaster gps_odom_broadcaster;

    ros::Rate r(10);

    while(n.ok())
    {
        ros::spinOnce();
        ros::Time current_time = ros::Time::now();

        
        //ROS_INFO("The theta is: %.10f", th);

        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
        
    
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "gps";

        odom_trans.transform.translation.x = X;
        odom_trans.transform.translation.y = Y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        gps_odom_broadcaster.sendTransform(odom_trans);

        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

    
        odom.pose.pose.position.x = X;
        odom.pose.pose.position.y = Y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

    //path
    
        path.header.stamp=current_time;
        path.header.frame_id="odom";
    
        this_pose_stamped.pose.position.x = X;
        this_pose_stamped.pose.position.y = Y;
        this_pose_stamped.pose.position.z = 0.0;
        this_pose_stamped.pose.orientation = odom_quat;

        this_pose_stamped.header.stamp = current_time;
        this_pose_stamped.header.frame_id = "odom";
        path.poses.push_back(this_pose_stamped);
        path_pub.publish(path);
    //path

    //publish the message
        gps_odom_pub.publish(odom);
    //last_time = current_time;
    
        pre_X = X;
        pre_Y = Y;
        pre_Z = Z;

        r.sleep();
    }
  return 0;
}
