#include <demo2d.hpp>
#include <opencv2/opencv.hpp>

#include <random>
#include <algorithm>

#include <iostream>

#define NB_SAMPLES_PER_M2 1000

int main(int argc, char* argv[]) {
  std::random_device rd;  
  std::mt19937 random_device(rd());


  auto theta = demo::dyn::linear<demo::dyn::bound::wrap>(0, 0, 360, .5);

#define SIZE 1
#define THICKNESS .2

  double fill_1   =  1;
  double fill_2   = .25;
  double radius_1 = SIZE * .7;
  double radius_2 = radius_1 - 1.5*THICKNESS;

  double rec_width  = SIZE*.5;
  double rec_height = SIZE;

  double bar_width  = SIZE+THICKNESS;
  double bar_height = THICKNESS;
  
  double data_radius = SIZE*1.2;

  demo2d::Point o_left  = {-1.25*SIZE,    0};
  demo2d::Point o_right = {-0.75*SIZE,    0};
  demo2d::Point h       = {.5 + radius_1, 0};

  
  auto left  = demo2d::sample::rectangle(rec_width, rec_height, fill_2) + o_left;
  auto right = demo2d::sample::rectangle(rec_width, rec_height, fill_1) + o_right;
  auto bar   = demo2d::sample::rectangle(bar_width, bar_height, fill_1);
  auto disk  = demo2d::sample::disk(radius_1, fill_1);
  auto hole  = demo2d::sample::disk(radius_2, fill_1);
  auto data  = demo2d::sample::disk(data_radius, fill_1);

  auto density = (left || right || bar || ((disk - hole) + h)) % theta();
  
  auto image = cv::Mat(1000, 1000, CV_8UC3, cv::Scalar(255,255,255));
  
  auto frame = demo2d::opencv::direct_orthonormal_frame(image.size(), image.size().width/4.0, true);
  auto dd    = demo2d::opencv::dot_drawer<demo2d::Point>(image, frame,
								   [](const demo2d::Point& pt) {return                    true;},
								   [](const demo2d::Point& pt) {return                      pt;},
								   [](const demo2d::Point& pt) {return                       1;},
								   [](const demo2d::Point& pt) {return cv::Scalar(250, 50, 50);},
								   [](const demo2d::Point& pt) {return                      -1;});

  std::string random    {"random"};
  std::string grid      {"grid"};
  std::string triangles {"triangles"};
  std::string from_data {"from data"};
  auto gui = demo2d::opencv::gui(random, frame);
 
  std::cout << std::endl
	    << std::endl
	    << "##################" << std::endl
	    << std::endl
	    << "press ESC to quit." << std::endl
	    << "press <space> to toggle rotation." << std::endl
	    << std::endl
	    << "##################" << std::endl
	    << std::endl;
  
  auto sampler_random    = demo2d::sample::base_sampler::random   (random_device, NB_SAMPLES_PER_M2);
  auto sampler_grid      = demo2d::sample::base_sampler::grid     (random_device, NB_SAMPLES_PER_M2);
  auto sampler_triangles = demo2d::sample::base_sampler::triangles(random_device, NB_SAMPLES_PER_M2);

  
  auto D = demo2d::sample::sample_set(random_device, sampler_random, data);
  std::vector<demo2d::Point> dataset;
  std::copy(D.begin(), D.end(), std::back_inserter(dataset));
  auto sampler_from_data = demo2d::sample::base_sampler::from_data(dataset.begin(), dataset.end());

  bool rotate = true;
  gui += {32, [&rotate](){rotate = !rotate;}};

  gui.loop_ms = 1;
  while(gui) {
    
    image = cv::Scalar(255,255,255);
    auto S1 = demo2d::sample::sample_set(random_device, sampler_random, density);
    std::copy(S1.begin(), S1.end(), dd);
    gui[random] << image;
    
    image = cv::Scalar(255,255,255);
    auto S2 = demo2d::sample::sample_set(random_device, sampler_grid, density);
    std::copy(S2.begin(), S2.end(), dd);
    gui[grid] << image;
    
    image = cv::Scalar(255,255,255);
    auto S3 = demo2d::sample::sample_set(random_device, sampler_triangles, density);
    std::copy(S3.begin(), S3.end(), dd);
    gui[triangles] << image;
    
    image = cv::Scalar(255,255,255);
    auto S4 = demo2d::sample::sample_set(random_device, sampler_from_data, density);
    std::copy(S4.begin(), S4.end(), dd);
    gui[from_data] << image;

    if(rotate) ++theta;
  }

  return 0;
}
