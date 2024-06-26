#include <demo2d.hpp>
#include <opencv2/opencv.hpp>

#include <random>
#include <algorithm>

#include <iostream>

#define NB_SAMPLES_PER_M2 1000

int main(int argc, char* argv[]) {
  std::random_device rd;  
  std::mt19937 random_device(rd());

  // Parameters for shapes.  WARNING : this cannot be expiring, since
  // it provides references for building up densities. See warning in
  // example 001-001.
  
  auto variable_intensity          = demo::dyn::sin(0, 1, -90, 1);
  auto line_pos                    = demo::dyn::linear<demo::dyn::bound::bounce>(0, -1, 1, .1);
  auto theta                       = demo::dyn::linear<demo::dyn::bound::wrap>(0, 0, 360, 5);
  auto pos                         = demo2d::dyn::circle(1, 0, 1);
  
  double filled         = 1;
  
  double half_back_width           = 2;
  double half_back_height          = 4;
  demo2d::Point half_back_pos = {1., 0.};

#define UNIT .25
#define THICK .1
  double rec_width                 = 2 * UNIT;
  double rec_height                = 2 * UNIT;
  demo2d::Point rec_pos       = {-1.5 * UNIT, 0.};
  double cro_radius                = UNIT;
  double hole_radius               = UNIT - THICK;
  demo2d::Point cro_pos       = {1.5 * UNIT, 0.};
  double bar_width                 = UNIT+THICK;
  double bar_height                = THICK;
  

  // objects definitions
  
  auto half_back  = demo2d::sample::rectangle(half_back_width, half_back_height, variable_intensity()) + half_back_pos;

  auto rec = demo2d::sample::rectangle(rec_width, rec_height, filled) + rec_pos;
  auto cro = (demo2d::sample::disk(cro_radius, filled) - demo2d::sample::disk(hole_radius, filled)) + cro_pos;
  auto bar = demo2d::sample::rectangle(bar_width, bar_height, filled);
  auto obj = (rec || cro || bar) % theta() + pos();

  auto cus = demo2d::sample::custom({-2, -2, 2, 4},
					 [&line_pos] (const demo2d::Point& p) {
					   if(std::fabs(p.y - line_pos()) < .2)
					     return .5;
					   else
					     return .1;
					 });

  auto density = obj || half_back || cus;

  // drawing and video

  auto image = cv::Mat(480, 640, CV_8UC3, cv::Scalar(255,255,255));
  auto frame = demo2d::opencv::direct_orthonormal_frame(image.size(), image.size().width/4.0, true);
  auto dd    = demo2d::opencv::dot_drawer<demo2d::Point>(image, frame,
								   [](const demo2d::Point& pt) {return                    true;},
								   [](const demo2d::Point& pt) {return                      pt;},
								   [](const demo2d::Point& pt) {return                       1;},
								   [](const demo2d::Point& pt) {return cv::Scalar(250, 50, 50);},
								   [](const demo2d::Point& pt) {return                      -1;});

  auto gui = demo2d::opencv::gui("image", frame);
  gui.loop_ms = 10;
  cv::VideoWriter video("movie.avi", cv::VideoWriter::fourcc('D', 'I', 'V', 'X'), 25, image.size());

  std::cout << std::endl
	    << "The window will close automatically after a while (or ESC hit). Please wait." << std::endl
	    << std::endl;

  auto sampler = demo2d::sample::base_sampler::random(random_device, NB_SAMPLES_PER_M2);
  for(unsigned int frame = 0; frame < 500 && gui; ++frame) {
    image = cv::Scalar(255,255,255);
    
    auto S = demo2d::sample::sample_set(random_device, sampler, density);
    std::copy(S.begin(), S.end(), dd);
    
    gui   << image;
    video << image;

    // update variable parameters
    ++variable_intensity;
    ++line_pos;
    ++theta;
    ++pos;
  }

  std::cout << std::endl
	    << std::endl
	    << "\"movie.avi\" generated." << std::endl
	    << std::endl;
  
    
  return 0;
}
