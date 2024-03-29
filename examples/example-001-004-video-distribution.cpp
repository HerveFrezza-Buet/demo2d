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
  
  int N_slider =  5000;
  int T_slider =   127;

  auto video_data = demo2d::opencv::sample::video_data(video_id,
						       [&T_slider](const unsigned char* rgb_pixel) {if(rgb_pixel[1] < (unsigned char)(T_slider)) return 1.; else return 0.;},
						       true); // toggles mirror flipping.

  auto webcam_distribution = demo2d::opencv::sample::webcam(video_data);

  auto input_size  = video_data.image.size();
  // This frame converts video image pixel positions to mathematical coordinates.
  video_data.frame = demo2d::opencv::direct_orthonormal_frame(input_size, .5*input_size.width, true);

  cv::namedWindow("webcam", cv::WINDOW_AUTOSIZE);
  
  cv::namedWindow("image", cv::WINDOW_AUTOSIZE);
  cv::createTrackbar("nb/m^2", "image", &N_slider, 50000, nullptr);
  cv::createTrackbar("threshold", "image", &T_slider, 255, nullptr);
  
  auto image       = cv::Mat(300, 400, CV_8UC3, cv::Scalar(255,255,255));
  auto frame       = demo2d::opencv::direct_orthonormal_frame(image.size(), .4*image.size().width, true);
  auto dd          = demo2d::opencv::dot_drawer<demo2d::Point>(image, frame,
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
  
  int keycode = 0;
  while(keycode != 27) {
    ++video_data; // get next frame.
    sampler = N_slider;
    auto S  = demo2d::sample::sample_set(random_device, sampler, webcam_distribution);
    image = cv::Scalar(255, 255, 255);
    std::copy(S.begin(), S.end(), dd);
    cv::imshow("image", image);
    cv::imshow("webcam", video_data.image);
    keycode = cv::waitKey(1) & 0xFF;
  }
  
  return 0;
}

