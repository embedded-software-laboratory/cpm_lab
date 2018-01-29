#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <linux/input.h>       
#include <unistd.h>
#include <errno.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <geometry_msgs/Pose2D.h>
#include <ros/ros.h>

using namespace std;

ros::Time latest_pose_time;
geometry_msgs::Pose2D latest_pose;

void pose_callback(geometry_msgs::Pose2D pose) {
    cout << "pose rcv" << endl;
    latest_pose = pose;
    latest_pose_time = ros::Time::now();
}

void die(string s)
{
    perror(s.c_str());
    exit(1);
}

int main(int argc, char* argv[])
{

    ros::init(argc, argv, "path_controller");
    ros::NodeHandle nh;
    ros::Subscriber pose_subscriber = nh.subscribe("pose", 1, &pose_callback);

    latest_pose_time = ros::Time(0,0);

    struct sockaddr_in si_other;
    int s, i, slen=sizeof(si_other);
 
    if ( (s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
        die("socket");
    }
 
    memset((char *) &si_other, 0, sizeof(si_other));
    si_other.sin_family = AF_INET;
    si_other.sin_port = htons(4210);


    string hostname = "esp8266_44142.local";

    struct hostent * hst = gethostbyname(hostname.c_str());
    if(!hst) {
        die("gethostbyname");
    }

    if(hst->h_addr_list == NULL) {
        die("hst->h_addr_list");        
    }

    char addr_str[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, hst->h_addr_list[0], addr_str, INET_ADDRSTRLEN);
    cout << hostname << " == " << addr_str << endl;

    if (inet_aton(addr_str , &si_other.sin_addr) == 0)  {
        die("inet_aton");
    }
 

    while(1) {

        auto now = ros::Time::now();
        double dt = (now - latest_pose_time).toSec();

        char steering = 127;
        char throttle = 127;

        if(dt < 0.3) {
            steering = 0;

            double dx = (latest_pose.x - 1.5)*0.4;
            double dy = latest_pose.y - 0.95;
            double theta_ref = atan2(dy,dx) + (M_PI/2);
            double theta_error = theta_ref - latest_pose.theta;
            while(theta_error > M_PI) theta_error -= 2*M_PI;
            while(theta_error < -M_PI) theta_error += 2*M_PI;
            //double c = cos(latest_pose.theta);
            //double s = sin(latest_pose.theta);
            double e_rad = sqrt(dx*dx+dy*dy)-0.45;

            //steering = uint8_t(fmin(255,fmax(0,(127 + e_rad/0.2*127))));
            steering = uint8_t(fmin(255,fmax(0,
                (theta_error+e_rad)*127+127
                )));
            throttle = 255;
        }
        else {
            cout << "fail" << endl;
        }



        char message[2];
        message[0] = steering;
        message[1] = throttle;

        if (sendto(s, message, 2 , 0 , (struct sockaddr *) &si_other, slen)==-1) {
            die("sendto()");
        }
        ros::spinOnce();
        usleep(20000);

    }
}