#include <demo2d.hpp>
#include <opencv2/opencv.hpp>

#include <random>
#include <algorithm>

#define NB_SAMPLES_PER_M2 2000

// These functions are used to parametrize the drawing
// iterations. Lambda function could be used, as for the color (see next).
bool               do_we_draw  (const demo2d::Point& pt) {return true;}
demo2d::Point      point_of    (const demo2d::Point& pt) {return   pt;}
int                radius_of   (const demo2d::Point& pt) {return    3;}
int                thickness_of(const demo2d::Point& pt) {return   -1;}

int main(int argc, char* argv[]) {
  std::random_device rd;  
  std::mt19937 random_device(rd());

  double rect_width              = 0.5;
  double rect_height             = 0.4;
  double rect_intensity          = 1.0;
  demo2d::Point rect_center = {.1, .2};
  double rect_theta              = 15;

  double hole_radius             = 0.1;
  double hole_intensity          = 1.0;
  demo2d::Point hole_center = {.1, 0};

  double lens_intensity              = 1.0;
  double lens_radius                 =  .4; 
  double lens_theta                  = -45;
  demo2d::Point lens_left_pos   = {-.2,  .0};
  demo2d::Point lens_right_pos  = { .2,  .0};
  demo2d::Point lens_pos        = { .0, -.3};

  double crown_density           = 1.0;
  double crown_radius            =  .3;
  double crown_hole_radius       =  .2;
  demo2d::Point crown_scale = {  1., 2.};
  demo2d::Point crown_pos   = { -.6,  0};

  demo2d::Point grad_offset = {-.5, -.5};
  demo2d::Point grad_scale  = { .6, 1.2};

  auto hole       = demo2d::sample::disk(hole_radius, hole_intensity) + hole_center;
  auto rectangle  = (demo2d::sample::rectangle(rect_width, rect_height, rect_intensity) - hole) + rect_center;
  auto obj1       = rectangle % rect_theta;

  auto lens_disk  = demo2d::sample::disk(lens_radius, lens_intensity);
  auto lens       = (lens_disk + lens_left_pos) && (lens_disk + lens_right_pos);
  auto obj2       = lens % lens_theta + lens_pos;

  auto crown      = (demo2d::sample::disk(crown_radius, crown_density) - demo2d::sample::disk(crown_hole_radius, crown_density)) * crown_scale;
  auto grad       = (demo2d::sample::custom(demo2d::sample::BBox(0,0,1,1),
						 [](const demo2d::Point& p) {return p.y;}) + grad_offset) * grad_scale;
  auto obj3       = (crown && grad) + crown_pos;

  auto density    =  obj1 || obj2 || obj3;
  
  
  auto image = cv::Mat(480, 640, CV_8UC3, cv::Scalar(255,255,255));
  auto frame = demo2d::opencv::direct_orthonormal_frame(image.size(), image.size().width*.5, true);
  auto dd    = demo2d::opencv::dot_drawer<demo2d::Point>(image, frame,
								   do_we_draw,
								   point_of,
								   radius_of,
								   [](const demo2d::Point& pt) {return cv::Scalar(200, 50, 50);},
								   thickness_of);
  // sampler provide random samples in the main area. Here, they are obtained from an uniform toss.
  auto sampler = demo2d::sample::base_sampler::random(random_device, NB_SAMPLES_PER_M2);
  auto S       = demo2d::sample::sample_set(random_device, sampler, density);
  std::copy(S.begin(), S.end(), dd);

  rect_theta =   5;
  lens_theta = -90;
  crown_pos  = {-.5,  0};

  // Let us change the base samples density.
  sampler    = NB_SAMPLES_PER_M2/2;
  auto SS    = demo2d::sample::sample_set(random_device,  sampler, density);
  dd         = demo2d::opencv::dot_drawer<demo2d::Point>(image, frame,
								   do_we_draw,
								   point_of,
								   radius_of,
								   [](const demo2d::Point& pt) {return cv::Scalar(50, 200, 200);},
								   thickness_of);
  std::copy(SS.begin(), SS.end(), dd);
  
  cv::namedWindow("image", CV_WINDOW_AUTOSIZE);
  cv::imshow     ("image", image);
  cv::waitKey(0);

  return 0;
}
