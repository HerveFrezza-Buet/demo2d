#include <demo2d.hpp>
#include <random>
#include <cstdlib>

int main(int argc, char* argv[]) {

  if(argc != 2) {
    std::cout << "Usage : " << argv[0] << " <device-id>" << std::endl
	      << "   e.g. " << argv[0] << " 0" << std::endl;
    return 0;
  }
  int video_id = std::atoi(argv[1]);
  
  std::random_device rd;  
  std::mt19937 random_device(rd());
  
  double N_slider =  5000;
  int T_slider =   127;

  auto video_data = demo2d::opencv::sample::video_data(video_id,
						       [&T_slider](const unsigned char* rgb_pixel) {if(rgb_pixel[1] < (unsigned char)(T_slider)) return 1.; else return 0.;},
						       true); // toggles mirror flipping.

  auto webcam_distribution = demo2d::opencv::sample::webcam(video_data);

  auto input_size  = video_data.image.size();
  // This frame converts video image pixel positions to mathematical coordinates.
  video_data.frame = demo2d::opencv::direct_orthonormal_frame(input_size, .5*input_size.width, true);

  std::string image {"image"};
  std::string webcam {"image source"};
  auto gui = demo2d::opencv::gui(webcam, video_data.frame);
  gui[image] += {"nb/m^2", [&N_slider](double slider_value){N_slider = 50000 * slider_value;}};
  gui[image] += {"threshold", [&T_slider](double slider_value){T_slider = 255 * slider_value;}};
  
  auto img       = cv::Mat(300, 400, CV_8UC3, cv::Scalar(255,255,255));
  auto frame       = demo2d::opencv::direct_orthonormal_frame(img.size(), .4*img.size().width, true);
  auto dd          = demo2d::opencv::dot_drawer<demo2d::Point>(img, frame,
									 [](const demo2d::Point& pt) {return                      true;},
									 [](const demo2d::Point& pt) {return                        pt;},
									 [](const demo2d::Point& pt) {return                         1;},
									 [](const demo2d::Point& pt) {return cv::Scalar(200, 200, 200);},
									 [](const demo2d::Point& pt) {return                        -1;});
  
  std::cout << std::endl
	    << std::endl
	    << std::endl
	    << "##################" << std::endl
	    << std::endl
	    << "click in the image to restart, press ESC to quit." << std::endl
	    << std::endl
	    << "##################" << std::endl
	    << std::endl;
  
  auto sampler = demo2d::sample::base_sampler::random(random_device, N_slider);
  
  gui.loop_ms = 1;
  while(gui) {
    sampler = N_slider;
    auto S  = demo2d::sample::sample_set(random_device, sampler, webcam_distribution);
    img = cv::Scalar(255, 255, 255);
    std::copy(S.begin(), S.end(), dd);
    gui[image] << img;
    gui[webcam] << video_data.image;
  }
  
  return 0;
}

