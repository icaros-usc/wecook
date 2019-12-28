//
// Created by hejia on 12/27/19.
//

#include <iostream>
#include <ros/ros.h>
#include <Eigen/Dense>

int main(int argc, char **argv) {
  Eigen::Isometry3d Tbi = Eigen::Isometry3d::Identity();
  Tbi.translation() = Eigen::Vector3d(0.91, -0.187, 0.02);
  Eigen::Matrix3d rot1;
  rot1 <<
       1, 0, 0,
      0, 0, -1,
      0, 1, 0;
  Tbi.linear() = rot1;

  ros::init(argc, argv, "test_apriltag");
  ros::NodeHandle nh("~");

  Eigen::Isometry3d Tbc = Eigen::Isometry3d::Identity();
  Tbc.translation() = Eigen::Vector3d(0.621884348621,
                                      -0.206405778944,
                                      1.25697557244);
  Eigen::Quaterniond rot2(-0.074645740892,
                          0.995928692094,
                          -0.0296603146372,
                          0.0409184477553);
  Tbc.linear() = Eigen::Matrix3d(rot2);

  Eigen::Isometry3d Toc = Eigen::Isometry3d::Identity();
  Toc.translation() = Eigen::Vector3d(-0.249141358159,
                                      -0.10535720593,
                                      0.807244164426);
  Eigen::Quaterniond rot3(-0.0885178230092,
                          0.979865228865,
                          -0.044747986937,
                          0.173281118234);
  Toc.linear() = Eigen::Matrix3d(rot3);

  Eigen::Isometry3d Toi = Tbi * Tbc.inverse() * Toc;

  std::cout << "Rotation: " << Toi.linear() << std::endl;
  std::cout << "Translation: " << Toi.translation() << std::endl;
}