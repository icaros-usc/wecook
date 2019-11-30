//
// Created by hejia on 11/29/19.
//

#ifndef WECOOK_SIMKINECT_H
#define WECOOK_SIMKINECT_H

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_triangle_primitive.h>
#include <CGAL/centroid.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/intersections.h>
#include <CGAL/Bbox_3.h>

#include "ai/sensor/camera/kinect/Kinect.h"

namespace wecook {
namespace ai {
namespace sensor {
namespace camera {
namespace kinect {

class SimKinect : public ::wecook::ai::sensor::camera::Kinect {
 public:
  struct TreeAndTri;

  SimKinect(const ::wecook::ai::sensor::Camera::Params &params,
            const std::string &objPath,
            const std::string &dotPath) : m_params(params) {
    m_depthMap = cv::Mat(params.m_h, params.m_w, CV_64FC1);
    m_depthMap.setTo(0.0);
    m_labels = cv::Mat(params.m_h, params.m_w, CV_8UC3);
    m_labels.setTo(cv::Scalar(60, 60, 60));
  }

  void measure(const Eigen::Affine3d &tf);

 private:
  void intersect(const Eigen::Affine3d &tf);

  cv::Point3d projectPixelTo3dRay(const cv::Point2f &uv_rect) const;

  Camera::Params m_params;
  cv::Mat m_pointCloud;
  cv::Mat m_depthMap;
  cv::Mat m_labels;

  TreeAndTri *m_search;

  float m_invalidDisp = 99999999.9;
};

struct SimKinect::TreeAndTri {
  std::vector<CGAL::Simple_cartesian<double>::Triangle_3> m_triangles;
  std::vector<CGAL::Simple_cartesian<double>::Point_3> m_points;
  std::vector<int> m_part_ids;
  CGAL::AABB_tree<CGAL::AABB_traits<CGAL::Simple_cartesian<double>,
                                    CGAL::AABB_triangle_primitive<CGAL::Simple_cartesian<double>,
                                                                  std::vector<CGAL::Simple_cartesian<double>::Triangle_3>::iterator>>>
      m_tree;
};

}
}
}
}
}

#endif //WECOOK_SIMKINECT_H
