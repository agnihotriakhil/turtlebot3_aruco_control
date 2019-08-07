

#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <aruco_mapping/ArucoMarker.h>
#include <std_srvs/Empty.h>
#include <sstream>
#include "geometry.h"
#include "pid.h"    
#include <math.h>
#include <iostream>
#include <fstream>


// global vars
tf::Point aruco_pos;   
double aruco_yaw;

// ROS Topic Publishers
ros::Publisher cmd_vel_pub;

// ROS Topic Subscribers
ros::Subscriber aruco_sub;

//Global variables for PID
double maxSpeed = 0.5;
double distanceConst = 0.5;
double dt = 0.1, maxT = M_PI, minT = -M_PI, Kp = 0.3, Ki = 0.05, Kd = 0.01;
double dtS = 0.1, maxS = maxSpeed, minS = 0.0, KpS = 0.08, KiS = 0.01, KdS = 0.005;

double getDistance(Point &p1, Point &p2);

void arucoCallback(const aruco_mapping::ArucoMarker &aruco_msg) {
 
    tf::pointMsgToTF(aruco_msg.global_camera_pose.position, aruco_pos);
    
    aruco_yaw = tf::getYaw(aruco_msg.global_camera_pose.orientation);

}

void make_traj(Geometry& geometry){
    
/*    using namespace std;

    ifstream file("trajectory.txt");
    while (file >> traj[i])

    if(file.is_open()){
        for(int i = 0; i < 5; ++i)
            file >> traj[i];
    } 
*/

    float traj[] = {.1,.1,.2,.2,.3,.3,.4,.4,.5,.5};
    int num_slice2 = 5;
    static int slice_index2 = 0;

    VECTOR2D *prev = NULL, *current = NULL;

    while (slice_index2 < num_slice2) {
        geometry_msgs::Point p;

        p.x = traj[2*slice_index2];       
        p.y = traj[2*slice_index2+1];
        p.z = 0;
        
        VECTOR2D *temp = new VECTOR2D(p.x, p.y);
        geometry.trajectory.push_back(*temp);
        current = temp;

        if (prev != NULL) {
            geometry.path.push_back(geometry.getLineSegment(*prev, *current));
        }
        
        prev = current;
        slice_index2++;
    }
}


int main(int argc, char **argv) {

    ros::init(argc, argv, "turtlebot3_control_node");
    ros::NodeHandle n("~");
    tf::TransformListener m_listener;
    tf::StampedTransform transform;

    cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    aruco_sub = n.subscribe("aruco_poses", 1, arucoCallback);

    ros::Rate loop_rate(10); // ros spins 10 frames per second

    geometry_msgs::Twist tw_msg;

    Geometry geometry;

    double angleError = 0.0;
    double speedError = 0.0;

    int frame_count = 0;
    while (ros::ok()) {
        
        make_traj(geometry);

        double omega = 0.0;
        double speed = 0.0;
        double prevDistError = 1000.0;
        double tb3_lenth = 0.125;

        PID pidTheta = PID(dt, maxT, minT, Kp, Kd, Ki);

        PID pidVelocity = PID(dtS, maxS, minS, KpS, KdS, KiS);

        /*Error calculation*/

        VECTOR2D current_pos, pos_error;
        current_pos.x = aruco_pos.x();
        current_pos.y = aruco_pos.y();

        Geometry::LineSegment *linesegment = geometry.getNearestLine(current_pos);

        Geometry::LineSegment linesegmentPerpen = geometry.getMinimumDistanceLine(*linesegment, current_pos);

        //Get Next LineSegment to do velocity PID

        Geometry::LineSegment *nextLinesegment = geometry.getNextLineSegment(linesegment);

        double targetDistanceEnd = geometry.getDistance(current_pos, linesegment->endP);
        double targetDistanceStart = geometry.getDistance(current_pos, linesegment->startP);

        //Distance Error
        double distError = 0.0;

        double targetAnglePerpen = geometry.getGradient(current_pos, linesegmentPerpen.endP);

        VECTOR2D target = linesegment->endP;
        double targetAngle = geometry.getGradient(current_pos, target);
        double distanceToClosestPath = abs(linesegment->disatanceToAObj);

        //Error calculation based on angles

        if (distanceToClosestPath < distanceConst) {

            //angleError = targetAngle - aruco_yaw;
            double directional = targetAngle;

            double discripancy = targetAnglePerpen - directional;
            discripancy = geometry.correctAngle(discripancy);

            discripancy = 0.5* discripancy / distanceConst * abs(distanceToClosestPath);

            double combined = targetAngle + discripancy;

            angleError = combined - aruco_yaw;


        } else {

            angleError = targetAnglePerpen - aruco_yaw;

        }

        speed = maxSpeed;

//If lines are long and it has sharp edges
        if (nextLinesegment->disatance > 3.0 && linesegment->disatance > 3.0) {
            //angleError correction for sharp turns
            if (targetDistanceEnd < 0.5) {
                double futureAngleChange = nextLinesegment->gradient - linesegment->gradient;
                futureAngleChange = geometry.correctAngle(futureAngleChange);

                futureAngleChange = futureAngleChange / distanceConst * abs(targetDistanceEnd);

                double combined = targetAngle + futureAngleChange;

                angleError = combined - aruco_yaw;
            }

            //Velocity Error Calculation for sharp turns
            if (targetDistanceStart < 0.7 || targetDistanceEnd < 0.7) {

                double targetDistance = targetDistanceEnd;

                if (targetDistanceStart < targetDistanceEnd)
                    targetDistance = targetDistanceStart;

                double speedError = 0.3 * maxSpeed * exp(-abs(targetDistance));

                speed = pidVelocity.calculate(maxSpeed, -speedError);
            }

        }

        //Error Angle correction for large angles
        angleError = geometry.correctAngle(angleError);
        //PID for the angle
        omega = pidTheta.calculate(0, -angleError);

        tw_msg.linear.x = speed;
        tw_msg.angular.z = omega;

        //publish this message to the robot
        cmd_vel_pub.publish(tw_msg);

        frame_count++;
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;

}

double getDistance(Point &p1, Point &p2) {
    return sqrt(pow((p1.x - p2.x), 2) + pow((p1.y - p2.y), 2));
}
