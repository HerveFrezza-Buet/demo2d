#include <vector>
#include <string>
#include <iterator>
#include <cstdlib>

#include <demo2d.hpp>
#include <opencv2/opencv.hpp>

// This examples illustrates the use of frames in order to convert
// mathematical point into image pixel locations.


#define HOUSE_RADIUS .50
#define HOUSE_ROOF   .75
#define PIXEL_MARGIN 10

struct Drawing {
  cv::Mat image;
  std::vector<demo2d::Point> house;
  demo2d::Point click_pos;
  
  Drawing(unsigned int image_width,  unsigned int image_height)
    : image(image_height, image_width, CV_8UC3, cv::Scalar(255,255,255)),
      click_pos(0., 0) {
    auto out = std::back_inserter(house);
    *(out++) = {+HOUSE_RADIUS, -HOUSE_RADIUS};
    *(out++) = {-HOUSE_RADIUS, +HOUSE_RADIUS};
    *(out++) = {-HOUSE_RADIUS, -HOUSE_RADIUS};
    *(out++) = {+HOUSE_RADIUS, -HOUSE_RADIUS};
    *(out++) = {+HOUSE_RADIUS, +HOUSE_RADIUS};
    *(out++) = {            0,  HOUSE_ROOF  };
    *(out++) = {-HOUSE_RADIUS, +HOUSE_RADIUS};
    *(out++) = {+HOUSE_RADIUS, +HOUSE_RADIUS};
    *(out++) = {-HOUSE_RADIUS, -HOUSE_RADIUS};
  }

  auto draw(const demo2d::opencv::Frame& frame) {
    // The frame transforms mathematical points a pixel positions on
    // the image.
    
    image = cv::Scalar(255, 255, 255);
    auto prev = house[0];
    for(auto& pt : house) {
      cv::line(image, frame(prev), frame(pt), cv::Scalar(0, 0, 0), 3);
      prev = pt;
    }
    cv::circle(image, frame(demo2d::Point(+HOUSE_RADIUS, -HOUSE_RADIUS)), 10, cv::Scalar(0, 0, 0), -1);
    cv::circle(image, frame(demo2d::Point(+HOUSE_RADIUS, +HOUSE_RADIUS)), 10, cv::Scalar(0, 0, 0), -1);

    // Let us draw the reference frame.
    cv::line(image, frame(demo2d::Point(0,0)), frame(demo2d::Point(0,1)), cv::Scalar(0, 0, 255), 3);
    cv::line(image, frame(demo2d::Point(0,0)), frame(demo2d::Point(1,0)), cv::Scalar(255, 0, 0), 3);

    cv::circle(image, frame(click_pos), 5, cv::Scalar(0, 255, 0), -1);

    return image;
  }
};

    

int main(int argc, char* argv[]) {

  if(argc != 3) {
    std::cout << "Usage : " << argv[0] << " <img-width> <img-height>" << std::endl;
    return 0;
  }

  unsigned int img_width  = std::atoi(argv[1]);
  unsigned int img_height = std::atoi(argv[2]);
  
  Drawing drawing(img_width, img_height);

  // We draw a "house" made of a square with a triangle (roof) on the
  // top of it. The square side is 1, its center is (0,0). The drawing
  // are expressed in a "mathematical" frame, i.e. an orthonormal
  // frame where coordinates are floating point values.
  //
  // On an image, the frame coordinate of pixel positions is
  // indirect. The coordinates are integers, (0,0) is the top left
  // corner of the image. The x-axis points to the right and the y
  // axis points downwards. A point expressed in this "image" frame is
  // denoted by an "image coordinate".
  //
  // The mathematical frame (i.e. the x and w unit vectors at the
  // origin) is displayed on the image.
  


  // Here, the frame is such as mathematical (0,0) is at the center of
  // the image. The length 1 in the mathematical frame corresponds to
  // .5*img_height pixels in the image.
  
  std::vector<std::string> windows {
    "direct orthonormal centered",
    "direct orthonormal not centered"
    "direct orthonormal bbox",
    "direct orthogonal",
    "Frame"};
  auto win_it = windows.begin();
  auto click_cb = [&drawing](const demo2d::Point& click_location){drawing.click_pos = click_location;};
  
  // We create a gui with a first window...
  auto gui = demo2d::opencv::gui(*win_it++, demo2d::opencv::direct_orthonormal_frame(cv::Size(img_width, img_height), .5*img_height, true));
  // ...and then we add new widows with their specific frames.
  
  // This is same as previously, except that mathematical (0,0) is at
  // the bottom left pixel of the image.
  gui[*win_it++] = demo2d::opencv::direct_orthonormal_frame(cv::Size(img_width, img_height), .5*img_height, false);
  gui += {cv::EVENT_LBUTTONDOWN, click_cb};

  // This frame isœ such as the bounding box in the mathematical frame
  // fits the image (leaving a border margin), keeping the aspect
  // ration.
  gui[*win_it++] = demo2d::opencv::direct_orthonormal_frame(cv::Size(img_width, img_height),
									    demo2d::sample::BBox({-HOUSE_RADIUS, HOUSE_RADIUS}, {0, HOUSE_ROOF}),
									    PIXEL_MARGIN);
  gui += {cv::EVENT_LBUTTONDOWN, click_cb};
  
  // The origin of the mathematical frame, here, is at pixel (200,
  // 400). The length of the x unit vector is 200, the length of the y
  // unit vector is 50.
  gui[*win_it++] = demo2d::opencv::direct_orthogonal_frame(200, 50, {200, 400});
  gui += {cv::EVENT_LBUTTONDOWN, click_cb};
  
  // The origin of the mathematical frame is at pixel (200, 100). The
  // x unit vector of the mathematical frame is transformed into a
  // vector (200, -50) in the image frame. The y unit vector of the
  // mathematical frame is transformed into a vector (30, 120) in the
  // image frame.
  gui[*win_it++] = demo2d::opencv::Frame({200, 100}, {100, -50}, { 30, 120});
  gui += {cv::EVENT_LBUTTONDOWN, click_cb};

    
  gui.loop_ms = 100;
  while(gui)
    for(auto& window_name : windows)
      gui << drawing.draw(gui[window_name].frame());
  
  return 0;
}
