#include <demo2d.hpp>
#include <opencv2/opencv.hpp>



int main(int argc, char* argv[]) {

  // This is an image that opencv can handle.
  auto image = cv::Mat(480, 640, CV_8UC3, cv::Scalar(255,255,255));

  // This maps math coords to image coords. See next tutorials for
  // details.  Opencv need cv::Point (a pair of integers), demo2d
  // provides demo2d::Point (a smart value in the geometrical
  // plane). The frame links these two stuff.
  auto frame = demo2d::opencv::direct_orthonormal_frame(image.size(), image.size().width, true); 

  // We will have 2 windows. In opencv, they are refered to by their title.
  std::string main_window {"Click me ! Hit <space> ! <ESC> ends."};
  std::string auxiliary_window {"Chose color"};
  
  // We provide one window (it's name) at construction.
  auto gui = demo2d::opencv::gui(main_window, frame); 

  // These are the position and the speed of the dot displays on the main window.
  demo2d::Point dot_pos   {0., 0.};
  demo2d::Point dot_speed {0., 0.};

  // This is the position of the rectangle on the auxiliary window,
  // and the color of both that rectangle and the dot on the main
  // window.
  demo2d::Point rgb_pos {0., 0.};
  cv::Scalar    color (0., 0., 0);
  
  // Let us register click events
  gui += {cv::EVENT_LBUTTONDOWN, [&dot_pos](const demo2d::Point& click_location){dot_pos = click_location;}};
  /*  
      cv::EVENT_MOUSEMOVE
      cv::EVENT_LBUTTONDOWN
      cv::EVENT_RBUTTONDOWN
      cv::EVENT_MBUTTONDOWN
      cv::EVENT_LBUTTONUP
      cv::EVENT_RBUTTONUP
      cv::EVENT_MBUTTONUP
      cv::EVENT_LBUTTONDBLCLK
      cv::EVENT_RBUTTONDBLCLK
      cv::EVENT_MBUTTONDBLCLK
      cv::EVENT_MOUSEWHEEL
      cv::EVENT_MOUSEHWHEEL
  */

  // Let us add another mouse event to the auxiliary window.
  // A call of gui["name"] tells that we work until next call with the window "name" (if not already existing, the window is created).
  // The expression gui["name"] can be used as gui
  // Each window has its associated frame, and a newly created windo has the same frame as the current one.
  gui[auxiliary_window] += {cv::EVENT_LBUTTONDOWN, [&rgb_pos](const demo2d::Point& click_location){rgb_pos = click_location;}};

  // You can set/change the frame associated to a window like this...
  gui = frame; // for auxiliary_window due to the previous line
  // ... or like this explicitly.
  gui[auxiliary_window] = frame;

  // You can get the frame associated to the current_window (useless
  // here, since it is the variable frame for both).
  auto current_frame = gui.frame();
  
  // These are slider events. The slider_value is in [0, 1], according to the slider position.
  
  gui[main_window]      += {"x speed", [&dot_speed](double slider_value){dot_speed.x = (slider_value - .5) * .01;}};
  gui                   += {"y speed", [&dot_speed](double slider_value){dot_speed.y = (slider_value - .5) * .01;}};
  gui[auxiliary_window] += {"red",   [&color](double slider_value){color[2] = slider_value * 255;}};
  gui                   += {"green", [&color](double slider_value){color[1] = slider_value * 255;}};
  gui                   += {"blue",  [&color](double slider_value){color[0] = slider_value * 255;}};

  // Last let us add a keyboard event. Note that hitting ESC will
  // exit. Here, the space key (32) puts the dot at the origin.
  
  gui += {32, [&dot_pos](){dot_pos = {0., 0.};}};

  gui.loop_ms = 100; // We loop every 100ms (0 means wait the hit of a key for next iteration).
  while(gui) {

    image = cv::Scalar(255,255,255); // clear background

    // We use the frame to get opencv coordinates (cv::Point) from mathematical ones (demo2d::Point here).
    
    // Let us draw the reference frame.
    cv::line(image, frame(demo2d::Point( 0, -1)), frame(demo2d::Point(0, 1)), cv::Scalar(0, 0, 255), 1);
    cv::line(image, frame(demo2d::Point(-1,  0)), frame(demo2d::Point(1, 0)), cv::Scalar(255, 0, 0), 1);
    
    // This draws a green dot.
    cv::circle(image, frame(dot_pos), 10, color, -1);
    
    dot_pos += dot_speed; // We change the dot position for next display.

    gui[main_window] << image; // This draws the image on the main window.
    
    image = cv::Scalar(255,255,255); // clear background
    
    cv::rectangle(image, frame(rgb_pos), frame(rgb_pos + demo2d::Point(.2, .2)), color, -1);
    
    gui[auxiliary_window] << image; // This draws the image on the main window.
  }
  
  return 0;
}
