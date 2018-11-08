/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>

#include <opencv2/core/core.hpp>

#include"../../../include/System.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM), first(true){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    ORB_SLAM2::System* mpSLAM;
    ros::Publisher pub;
    bool first;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc != 5)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings topic use_map[0|1]" << endl;
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, true, (bool)atoi(argv[4]));

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nodeHandler;
    ros::Subscriber sub = nodeHandler.subscribe(argv[3], 1, &ImageGrabber::GrabImage,&igb);
    igb.pub = nodeHandler.advertise<geometry_msgs::PoseStamped>("/orb_slam/pose",100);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat pose = mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
    //mpSLAM->getTracker();

    if (pose.empty())
        return;
    cout << "Pose = "<< endl << " "  << pose << endl << endl;

    /* global left handed coordinate system */
    static cv::Mat pose_prev = cv::Mat::eye(4,4, CV_32F);
    static cv::Mat world_lh = cv::Mat::eye(4,4, CV_32F);
    // matrix to flip signs of sinus in rotation matrix, not sure why we need to do that
    static const cv::Mat flipSign = (cv::Mat_<float>(4,4) <<   1,-1,-1, 1,
                                                               -1, 1,-1, 1,
                                                               -1,-1, 1, 1,
                                                                1, 1, 1, 1);
    
    cout << "Prev = "<< endl << " "  << pose_prev << endl << endl;
    cout << "Prev_inv = "<< endl << " "  << pose_prev.inv() << endl << endl;
    cv::Mat translation =  (pose * pose_prev.inv()).mul(flipSign);
    cout << "Trans = "<< endl << " "  << translation << endl << endl;
    world_lh = world_lh * translation;
    cout << "World = "<< endl << " "  << world_lh << endl << endl;
    pose_prev = pose.clone();


    /* transform into global right handed coordinate system, publish in ROS*/
    tf::Matrix3x3 cameraRotation_rh(  - world_lh.at<float>(0,0),   world_lh.at<float>(0,1),   world_lh.at<float>(0,2),
				  - world_lh.at<float>(1,0),   world_lh.at<float>(1,1),   world_lh.at<float>(1,2),
				    world_lh.at<float>(2,0), - world_lh.at<float>(2,1), - world_lh.at<float>(2,2));

    tf::Vector3 cameraTranslation_rh( world_lh.at<float>(0,3),world_lh.at<float>(1,3), - world_lh.at<float>(2,3) );

    //rotate 270deg about x and 270deg about x to get ENU: x forward, y left, z up
    const tf::Matrix3x3 rotation270degXZ(   0, 1, 0,
                                            0, 0, 1,
                                            1, 0, 0);

    static tf::TransformBroadcaster br;

    double roll, pitch, yaw;
    tf::Matrix3x3 globalRotation_rh = cameraRotation_rh * rotation270degXZ;
    globalRotation_rh.getRPY(roll, pitch, yaw);
    cout << "RPY: " << roll <<", "<<pitch<<", "<<yaw<<endl<<endl;
    yaw -= M_PI/2;
    globalRotation_rh.setRPY(0.0, 0.0, yaw);
    tf::Vector3 globalTranslation_rh = cameraTranslation_rh * rotation270degXZ;
    tf::Transform transform = tf::Transform(globalRotation_rh, globalTranslation_rh);
    transform = tf::Transform(transform.getRotation().normalize(), globalTranslation_rh);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_link", "camera_pose"));

    geometry_msgs::PoseStamped p;
    p.header.stamp = ros::Time::now();
    p.header.frame_id = "camera_pose";
    p.pose.position.x = globalTranslation_rh[0];
    p.pose.position.y = globalTranslation_rh[1];
    p.pose.position.z = globalTranslation_rh[2];
    p.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    pub.publish(p);
}


