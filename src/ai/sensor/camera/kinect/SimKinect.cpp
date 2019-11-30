//
// Created by hejia on 11/29/19.
//

#include "ai/sensor/camera/kinect/SimKinect.h"

void updateObjectPoses(const Eigen::Affine3d &tf) {

}

void updateTree() {

}

cv::Point3d wecook::ai::sensor::camera::kinect::SimKinect::projectPixelTo3dRay(const cv::Point2f &uv_rect) const {
  cv::Point3d ray;
  ray.x = (uv_rect.x - m_params.m_cx) / m_params.m_fx;
  ray.y = (uv_rect.y - m_params.m_cy) / m_params.m_fy;
  ray.z = 1.0;
  return ray;
}

/// Function that intersects rays with the object model at current state
void wecook::ai::sensor::camera::kinect::SimKinect::intersect(const Eigen::Affine3d &tf) {
  cv::Mat disp(m_params.m_h, m_params.m_w, CV_32FC1);
  disp.setTo(m_invalidDisp);

  // go through the whole image and create a ray from a pixel -> dir
  std::vector<cv::Point3f> vec;
  vec.reserve(m_params.m_h * m_params.m_w);
  for (std::size_t c = 0; c < m_params.m_w; ++c) {
    for (std::size_t r = 0; r < m_params.m_h; ++r) {
      // compute ray from pixel and camera configuration
      cv::Point3f ray = projectPixelTo3dRay(cv::Point2f(c, r));
      // check if there is any intersection of the ray with the object by do_intersect
      uint32_t reach_mesh = m_search->m_tree.do_intersect(
          CGAL::Simple_cartesian<double>::Ray_3(
              CGAL::Simple_cartesian<double>::Point_3(0., 0., 0.),
              CGAL::Simple_cartesian<double>::Point_3(ray.x, ray.y, ray.z)));
      if (reach_mesh) {
        // TODO
      }
    }
  }
}

void ::wecook::ai::sensor::camera::kinect::SimKinect::measure(const Eigen::Affine3d &tf) {
  intersect(tf);
}

