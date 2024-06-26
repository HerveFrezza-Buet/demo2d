#include <demo2d.hpp>
#include <opencv2/opencv.hpp>

#include <random>
#include <iterator>
#include <vector>
#include <algorithm>
#include <cstdlib>

#define NB_SAMPLES 100

// These functions are used to parametrize the drawing
// iterations. Lambda function could be used, as for the color (see next).
bool               do_we_draw  (const demo2d::Point& pt) {return true;}
demo2d::Point point_of    (const demo2d::Point& pt) {return   pt;}
int                radius_of   (const demo2d::Point& pt) {return    3;}
int                thickness_of(const demo2d::Point& pt) {return   -1;}


int main(int argc, char* argv[]) {

  std::mt19937 random_device(0);

  // (Amin,Amax) and (Bmin, Bmax) delimit two overlapping rectangles.
  demo2d::Point Amin(-.2,-.2);
  demo2d::Point Amax( .1, .1);
  demo2d::Point Bmin(-.1,-.1);
  demo2d::Point Bmax( .2, .2);

  // Let us toss NB_SAMPLES points in each rectangle, and collect them
  // in sets A and B.
  std::vector<demo2d::Point> A;
  std::vector<demo2d::Point> B;

  auto outa = std::back_inserter(A);
  auto outb = std::back_inserter(B);

  for(int i=0; i< NB_SAMPLES; ++i) {
    *(outa++) = demo2d::uniform(random_device, Amin, Amax);
    *(outb++) = demo2d::uniform(random_device, Bmin, Bmax);
  }

  // Let us set up opencv drawing. The frame enable to convert
  // coordinates into position on the opencv image. Here, we choose a
  // direct frame, with the origin at the center (true). A length of 1
  // corresponds to the image width.
  auto color_a = cv::Scalar(200,  50,  50);
  auto color_b = cv::Scalar( 50, 200, 200);
  
  auto image = cv::Mat(480, 640, CV_8UC3, cv::Scalar(255,255,255));
  auto frame = demo2d::opencv::direct_orthonormal_frame(image.size(), image.size().width, true);
 

  // Let us draw on the image. We can use direct opencv drawing
  // functions (do not forget to use the frame in order to convert
  // coordinates into the opencv world), as well as some provided
  // output iterators (dot_drawer here).
  auto dda   = demo2d::opencv::dot_drawer<demo2d::Point>(image, frame,
								   do_we_draw,
								   point_of,
								   radius_of,
								   [&color_a](const demo2d::Point& pt) {return color_a;},
								   thickness_of);
  auto ddb   = demo2d::opencv::dot_drawer<demo2d::Point>(image, frame,
								   do_we_draw,
								   point_of,
								   radius_of,
								   [&color_b](const demo2d::Point& pt) {return color_b;},
								   thickness_of);

  std::copy(A.begin(), A.end(), dda);
  std::copy(B.begin(), B.end(), ddb);
  cv::rectangle(image, frame(Amin), frame(Amax), color_a, 3);
  cv::rectangle(image, frame(Bmin), frame(Bmax), color_b, 3);
  
  // Here, let us save the image several times, with names like
  // "img-000054.png". This is what the videoframe_name class offers.
  auto filename = demo::videoframe_name("img", "png");

  // Let us display the result.
  cv::namedWindow("image", cv::WINDOW_AUTOSIZE);
  auto gui = demo2d::opencv::gui("image", frame);
  gui.loop_ms = 0;
  
  cv::imwrite(filename(37), image); // writes img-000037.png
  gui << image;
  if(gui);
  
  image = color_b;
  auto dd = demo2d::opencv::segment_drawer<double>(image, frame,
							[]        (double x) {return true;},
  							[]        (double x) {return std::make_pair(demo2d::Point(x, 0),
												    demo2d::Point(x, .3*std::sin(15*x)));},
  							[&color_a](double x) {return color_a;},
  							[]        (double x) {return 1;});
  auto abscissas = demo::range(-.5, .5, 100);
  std::copy(abscissas.begin(), abscissas.end(), dd);

  cv::imwrite(filename(), image); // writes img-000038.png
  gui << image;
  if(gui);

  return 0;
}
