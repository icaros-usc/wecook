//
// Created by hejia on 12/27/19.
//

#include <iostream>
#include <ros/ros.h>
#include "apriltags/AprilTagDetections.h"
#include <Eigen/Dense>

Eigen::Isometry3d vectorToIsometry(std::vector<double> &poseVector) {
  double *ptr = &poseVector[0];
  Eigen::Map<Eigen::VectorXd> p(ptr, 7);

  Eigen::Isometry3d pose(Eigen::Isometry3d::Identity());
  pose.translation() = p.head(3);
  // w, x, y, z
  Eigen::Quaterniond q(p[6], p[3], p[4], p[5]);
  pose.linear() = Eigen::Matrix3d(q);
  return pose;
}

void processTagMsg(const apriltags::AprilTagDetections::ConstPtr &msg) {
  Eigen::Isometry3d T_basetag_cam;
  Eigen::Isometry3d T_testTag_cam;
  for (auto &detection: msg->detections) {
    std::vector<double> vector;
    vector.push_back(detection.pose.position.x);
    vector.push_back(detection.pose.position.y);
    vector.push_back(detection.pose.position.z);
    vector.push_back(detection.pose.orientation.x);
    vector.push_back(detection.pose.orientation.y);
    vector.push_back(detection.pose.orientation.z);
    vector.push_back(detection.pose.orientation.w);
    if (detection.id == 0) {
      T_basetag_cam = vectorToIsometry(vector);
    }
    if (detection.id == 1) {
      T_testTag_cam = vectorToIsometry(vector);
    }
  }

  // Replace the fixed values with the tag transformations in demo scenario (.py) file.
  Eigen::Isometry3d T_baseTag_robot = Eigen::Isometry3d::Identity();
  T_baseTag_robot.translation() = Eigen::Vector3d(0, -0.045, 0.13);
  Eigen::Matrix3d rot1;
  rot1 <<
      1, 0, 0,
      0, 0, -1,
      0, 1, 0;
  T_baseTag_robot.linear() = rot1;

  // Replace the fixed values with the tag transformations in demo scenario (.py) file.
  Eigen::Isometry3d T_testTag_sodacan = Eigen::Isometry3d::Identity();
  T_testTag_sodacan.translation() = Eigen::Vector3d(0, -0.04, 0.075);
  Eigen::Matrix3d rot2;
  rot2 <<
      1, 0, 0,
      0, 0, -1,
      0, 1, 0;
  T_testTag_sodacan.linear() = rot2;

  std::cout << "T_baseTag_robot Rotation: " << T_baseTag_robot.linear() << std::endl;
  std::cout << "T_baseTag_robot Translation: " << T_baseTag_robot.translation() << std::endl;
  std::cout << "T_basetag_cam Rotation: " << T_basetag_cam.linear() << std::endl;
  std::cout << "T_basetag_cam Translation: " << T_basetag_cam.translation() << std::endl;
  std::cout << "T_testTag_cam Rotation: " << T_testTag_cam.linear() << std::endl;
  std::cout << "T_testTag_cam Translation: " << T_testTag_cam.translation() << std::endl;
  std::cout << "T_testTag_sodacan Rotation: " << T_testTag_sodacan.linear() << std::endl;
  std::cout << "T_testTag_sodacan Translation: " << T_testTag_sodacan.translation() << std::endl;

  auto T_sodacan_robot = T_baseTag_robot * T_basetag_cam.inverse() * T_testTag_cam * T_testTag_sodacan.inverse();

  std::cout << "Rotation: " << T_sodacan_robot.linear() << std::endl;
  std::cout << "Translation: " << T_sodacan_robot.translation() << std::endl;
  ros::Duration(5).sleep();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_apriltag_transforms");
  ros::NodeHandle nh("~");
  ros::Subscriber s = nh.subscribe("/apriltags/detections", 10, processTagMsg);
  ros::spin();
  return 0;
}