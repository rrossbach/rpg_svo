// This file is part of SVO - Semi-direct Visual Odometry.
//
// Copyright (C) 2014 Christian Forster <forster at ifi dot uzh dot ch>
// (Robotics and Perception Group, University of Zurich, Switzerland).
//
// SVO is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or any later version.
//
// SVO is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include <svo/config.h>
#include <svo/frame_handler_mono.h>
#include <svo/map.h>
#include <svo/frame.h>
#include <vector>
#include <string>
#include <vikit/math_utils.h>
#include <vikit/vision.h>
#include <vikit/abstract_camera.h>
#include <vikit/atan_camera.h>
#include <vikit/pinhole_camera.h>
#include <opencv2/opencv.hpp>
#include <sophus/se3.h>
#include <iostream>
#include "test_utils.h"

namespace svo {

class BenchmarkNode
{
  vk::AbstractCamera* cam_;
  svo::FrameHandlerMono* vo_;
    cv::Mat* plot_;

public:
  BenchmarkNode();
  ~BenchmarkNode();
  void runFromFolder();
};

BenchmarkNode::BenchmarkNode()
{
  cam_ = new vk::PinholeCamera(752, 480, 315.5, 315.5, 376.0, 240.0);
  vo_ = new svo::FrameHandlerMono(cam_);
  vo_->start();
  plot_ = new cv::Mat(500,500,CV_8UC3, cv::Scalar(0));
}

BenchmarkNode::~BenchmarkNode()
{
  delete vo_;
  delete cam_;
  delete plot_;
}

void BenchmarkNode::runFromFolder()
{
  cv::line(*plot_, cv::Point(250,0), cv::Point(250,500), cv::Scalar(255,0,255));
  cv::line(*plot_, cv::Point(0,250), cv::Point(500,250), cv::Scalar(255, 0, 255));

  cv::Point origin = cv::Point(250,250);
  cv::Point prevPoint1 = origin, curPoint1 = origin;
  cv::Point prevPoint2 = origin, curPoint2 = origin;
  for(int img_id = 2; img_id < 188; ++img_id)
  {
    // load image
    std::stringstream ss;
    ss << svo::test_utils::getDatasetDir() << "/sin2_tex2_h1_v8_d/img/frame_"
       << std::setw( 6 ) << std::setfill( '0' ) << img_id << "_0.png";
    if(img_id == 2)
      std::cout << "reading image " << ss.str() << std::endl;
    cv::Mat img(cv::imread(ss.str().c_str(), 0));
    assert(!img.empty());

    // process frame
    vo_->addImage(img, 0.01*img_id);

    // display tracking quality
    if(vo_->lastFrame() != NULL)
    {
    	std::cout << "Frame-Id: " << vo_->lastFrame()->id_ << " \t"
                  << "#Features: " << vo_->lastNumObservations() << " \t"
                  << "Proc. Time: " << vo_->lastProcessingTime()*1000 << "ms \n";

    	// access the pose of the camera via vo_->lastFrame()->T_f_w_.
        Vector3d pos = vo_->lastFrame()->T_f_w_.translation();

        std::cout << "\tvo_->lastFrame()->pos() XPos: " << vo_->lastFrame()->pos().x() << " \t"
                << "YPos: " << vo_->lastFrame()->pos().y() << " \t"
                << "ZPos: " << vo_->lastFrame()->pos().z() << "\n";
        std::cout << "\tlastFrame()->T_f_w.transation() XPos: " << pos.x() << " \t"
                << "YPos: " << pos.y() << " \t"
                << "ZPos: " << pos.z() << "\n";
        curPoint2 = origin + cv::Point(pos.x() * 13, pos.y() * 13);
        curPoint1 = origin + cv::Point(vo_->lastFrame()->pos().x() * 13, vo_->lastFrame()->pos().y() * 13);
      cv::circle(*plot_, prevPoint1, 2, cv::Scalar(0,255,0));
      cv::circle(*plot_, curPoint1, 2, cv::Scalar(0,255,0));
      cv::circle(*plot_, prevPoint2, 2, cv::Scalar(0,0,255));
      cv::circle(*plot_, curPoint2, 2, cv::Scalar(0,0,255));
     // cv::line(*plot_, prevPoint1, curPoint1, cv::Scalar(0,255,0));
     // cv::line(*plot_, prevPoint2, curPoint2, cv::Scalar(0,0,255));
      prevPoint1 = curPoint1;
      prevPoint2 = curPoint2;
      cv::imshow("plot_", *plot_);
      cv::waitKey(0);
    }
  }
}

} // namespace svo

int main(int argc, char** argv)
{
  {
    svo::BenchmarkNode benchmark;
    benchmark.runFromFolder();
  }
  printf("BenchmarkNode finished.\n");
  return 0;
}

