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
#include <first_project/sector_times.h>
#include <vector>


struct SectorCheckpoint {
    double latitude;
    double longitude;
};

//sensor_msgs::NavSatFix current_pose;
double current_speed = 0.0; // m/s
int current_sector = 1;
std::vector<SectorCheckpoint> checkpoints;
std::vector<double> speed;

double latitude = 0.0;
double longitude = 0.0;

ros::Time sector_start_time;

bool reached_checkpoint = false;
bool flag = true;

//LLA
double haversine(double lat1, double lon1, double lat2, double lon2) {
    const double R = 6371000.0;
    double dLat = (lat2 - lat1) * M_PI / 180.0;
    double dLon = (lon2 - lon1) * M_PI / 180.0;
    double a = sin(dLat/2) * sin(dLat/2) +
               cos(lat1 * M_PI / 180.0) * cos(lat2 * M_PI / 180.0) *
               sin(dLon/2) * sin(dLon/2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    return R * c;
}

void gps_messageCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    //current_pose = msg;

    if((msg -> latitude != 0 ) && (msg -> longitude != 0)){
        latitude = msg -> latitude;
        longitude = msg -> longitude;
    }

}

void speed_messageCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
    if(msg->point.y != 0){
        current_speed = msg->point.y; 
        speed.push_back(current_speed);
        if(flag)
            {
                sector_start_time = ros::Time::now();
                flag = false;
            }
    }
    
}

double average_sp(){
    double N = speed.size(), sum = 0;
    double ave;
    for(int i=0; i < N; i++){
        sum += speed[i];
    }
    ave = sum / N;
    speed.clear();
    return ave;
}

void checkSector(ros::Publisher& pub) {
    if (current_sector > checkpoints.size()) 
        return;

    double dist = haversine(  //checkpoint dis
        latitude, 
        longitude, 
        checkpoints[current_sector - 1].latitude, 
        checkpoints[current_sector - 1].longitude
    );

    if ((dist < 5.0) && (!reached_checkpoint)) { // arrive condition
        ros::Time now = ros::Time::now();
        float sector_time = float((now - sector_start_time).toSec());
        
        float average_speed = float(average_sp());

        first_project::sector_times msg;
        msg.current_sector = current_sector;
        msg.current_sector_time = sector_time;
        msg.current_sector_mean_speed = average_speed;
        pub.publish(msg);

        ROS_INFO("Finished sector %d: Time = %.2f s, Avg Speed = %.2f km/h", 
                  current_sector, sector_time, average_speed);

        // next sector
        reached_checkpoint = true;
        current_sector++;
        sector_start_time = now;
    } 

    if(dist > 10.0){
        reached_checkpoint = false;
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "sector_times");
    ros::NodeHandle n;

    ros::Subscriber SpeedSteer_sub = n.subscribe<geometry_msgs::PointStamped>("speedsteer", 1, speed_messageCallback);
    ros::Subscriber gps_sub = n.subscribe<sensor_msgs::NavSatFix>("swiftnav/front/gps_pose", 1, gps_messageCallback);
    ros::Publisher sector_pub = n.advertise<first_project::sector_times>("sector_times", 1);

     // sector checkpoint
    
    checkpoints.push_back({45.630116, 9.289851}); // sector1 and sector2
    checkpoints.push_back({45.623546, 9.287241}); // sector2 and sector3
    checkpoints.push_back({45.6189323717, 9.2811788719}); //sector 3 and sector 1 
    sector_start_time = ros::Time::now();

    ros::Rate r(10);

    while(n.ok())
    {
        ros::spinOnce();
        checkSector(sector_pub);
        r.sleep();
    }
     return 0;
}
