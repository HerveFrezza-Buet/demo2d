
/*
 *   Copyright (C) 2020,  CentraleSupelec
 *
 *   Author : Hervé Frezza-Buet
 *
 *   Contributor :
 *
 *   This library is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU General Public
 *   License (GPL) as published by the Free Software Foundation; either
 *   version 3 of the License, or any later version.
 *   
 *   This library is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *   General Public License for more details.
 *   
 *   You should have received a copy of the GNU General Public
 *   License along with this library; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 *   Contact : herve.frezza-buet@centralesupelec.fr
 *
 */



#pragma once

#include <opencv2/opencv.hpp>
#include <utility>
#include <functional>
#include <optional>
#include <algorithm>
#include <map>
#include <random>
#include <stdexcept>
#include <sstream>
#include <cmath>
#include <algorithm>
#include <iterator>
#include <array>
#include <tuple>
#include <string>

#include <demo2dPoint.hpp>
#include <demo2dSample.hpp>
#include <algorithm>


  
namespace demo2d {
  namespace opencv {
      
    class HueSelector {
    private:
      cv::Mat hsv;
      cv::Mat hsv_line;
      int hsv_thickness;
      int stride;
	
      void check_line(int cols) {
	if(cols == hsv_line.cols)
	  return;

	hsv_line.create(1, cols, CV_8UC3);

	unsigned char* it  = hsv_line.data;
	unsigned char* end = hsv_line.data + 3*cols;
	int col = 0;
	while(it != end) {
	  *(it++) = (unsigned char)(180.0*(col++)/(cols-1)+.5); // H
	  *(it++) = 255;                                        // S
	  *(it++) = 255;                                        // V
	}
	cv::cvtColor(hsv_line, hsv_line, cv::COLOR_HSV2BGR);
	stride = cols*3;
	hsv_thickness = std::min(cols, 20);
      }

      bool hue_test(unsigned int h) const {
	int href = (int)(H_slider*180e-3+.5);
	int dh   = href - (int)h;
	dh       = std::min(std::abs(dh), std::abs(dh+180));
	return dh < T_slider;
      }
	
      bool hsv_test(const unsigned char* hsv_pixel) const {
	unsigned char s = (unsigned char)(S_slider*255e-3+.5);
	unsigned char v = (unsigned char)(V_slider*255e-3+.5);

	return hsv_pixel[1] > s
	  &&   hsv_pixel[2] > v
	  &&   hue_test(hsv_pixel[0]);
      }
	
    public:
      cv::Mat image;
      int H_slider =   0;
      int S_slider = 500;
      int V_slider = 500;
      int T_slider =  20;
      double darken = .2;

      HueSelector() = default;


      auto build_pixel_test() {
	return [this](const unsigned char* rgb_pixel) {
	  cv::Mat src(1, 1, CV_8UC3, (void*)rgb_pixel);
	  cv::Mat dst;
	  cv::cvtColor(src, dst, cv::COLOR_BGR2HSV);
	  return this->hsv_test(dst.data);
	};
      }
	
      void build_sliders(const std::string& window_name) {
	cv::createTrackbar("Hue",            window_name, &H_slider, 1000, nullptr);
	cv::createTrackbar("Hue Tolerance",  window_name, &T_slider,  100, nullptr);
	cv::createTrackbar("Min Saturation", window_name, &S_slider, 1000, nullptr);
	cv::createTrackbar("Min Value",      window_name, &V_slider, 1000, nullptr);
      }

      void build_image(cv::Mat img) {
	check_line(img.cols);
	image.create(img.rows, img.cols, CV_8UC3);
	cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);
	  
	unsigned char* src = img.data;
	unsigned char* dst = image.data;
	unsigned char* end = src + img.rows*stride;

	for(int l = 0; l < hsv_thickness; ++l, dst += stride)
	  std::copy(hsv_line.data, hsv_line.data + stride, dst);
	auto offset = hsv_thickness*stride;
	src += offset;

	double h = H_slider*1e-3*img.cols;
	cv::line(image, cv::Point(h, 0), cv::Point(h, hsv_thickness), cv::Scalar(0,0,0), 5);
	  
	for(unsigned char* hsv_it = hsv.data + offset; src != end; hsv_it +=3) 
	  if(hsv_test(hsv_it)) {
	    *(dst++) = *(src++);
	    *(dst++) = *(src++);
	    *(dst++) = *(src++);
	  }
	  else {
	    *(dst++) = *(src++)*darken;
	    *(dst++) = *(src++)*darken;
	    *(dst++) = *(src++)*darken;
	  }
      }
    };

      
    namespace colormap {

      class jet {
      private :

	std::array<cv::Scalar, 11> colormap = {
	  cv::Scalar(130,   0,   0),  // 0.0
	  cv::Scalar(255,   0,   0),  // 0.1
	  cv::Scalar(255,  75,   0),  // 0.2
	  cv::Scalar(255, 175,   0),  // 0.3
	  cv::Scalar(200, 255,  40),  // 0.4
	  cv::Scalar(155, 255, 155),  // 0.5
	  cv::Scalar( 40, 255, 210),  // 0.6
	  cv::Scalar(  0, 200, 255),  // 0.7
	  cv::Scalar(  0,  90, 255),  // 0.8
	  cv::Scalar(  0,   0, 240),  // 0.9
	  cv::Scalar(  0,   0, 120)}; // 1.0
	  
	double min = 0;
	double max = 1;
	  
      public:
	jet()                      = default;
	jet(const jet&)            = default;
	jet& operator=(const jet&) = default;

	/**
	 * Set (min, max)
	 */
	void operator=(const std::pair<double, double>& minmax) {
	  std::tie(min,max) = minmax;
	}

	/**
	 * Get a color from a value.
	 */
	cv::Scalar operator()(double value) const {
	  auto c = std::min(std::max((value - min)/(max - min), 0.0), 1-1e-3);
	  auto nb = colormap.size()-1;

	  auto cnb  = c*nb;
	  unsigned int idx_min = (unsigned int)(cnb);
	  auto dc   = cnb - idx_min;
	  auto cmin = colormap[idx_min];
	  auto cmax = colormap[idx_min+1];
	  return cv::Scalar((int)(dc*cmax[0] + (1-dc)*cmin[0] + .5),
			    (int)(dc*cmax[1] + (1-dc)*cmin[1] + .5),
			    (int)(dc*cmax[2] + (1-dc)*cmin[2] + .5));
	}
      };
	
      template<typename RANDOM_DEVICE>
      class random {
      private:
	RANDOM_DEVICE& rd;
	double min_dist;
	unsigned int nb_retry;
	std::map<unsigned int, cv::Scalar> colors;
	  

	cv::Scalar random_color() {
	  std::array<unsigned int, 3> idx = { 0,  1, 2};
	  std::array<double,       3> rgb = {0., 0., 0.};

	  std::shuffle(idx.begin(), idx.end(), rd);
	  rgb[idx[0]] = 0;
	  rgb[idx[1]] = 1;
	  rgb[idx[2]] = std::uniform_real_distribution<double>(0,1)(rd);

	  switch(std::uniform_int_distribution<int>(1,3)(rd)) {
	  case 1:
	    rgb[0] *= .5;
	    rgb[1] *= .5;
	    rgb[2] *= .5;
	    break;
	  case 2:
	    rgb[0] = .5 * (1 + rgb[0]);
	    rgb[1] = .5 * (1 + rgb[1]);
	    rgb[2] = .5 * (1 + rgb[2]);
	    break;
	  default:
	    break;
	  }

	  return cv::Scalar((int)(255*rgb[0]+.5),
			    (int)(255*rgb[1]+.5),
			    (int)(255*rgb[2]+.5));
	}

	double distance(const cv::Scalar& c1, const cv::Scalar& c2) {
	  double diff = c1[0] - c2[0];
	  double d2   = diff*diff;
	  diff        = c1[1] - c2[1];
	  d2         += diff*diff;
	  diff        = c1[2] - c2[2];
	  d2         += diff*diff;
	  return std::sqrt(d2);
	}
	  
	cv::Scalar random_distinct_color() {
	  if(colors.size() == 0 || nb_retry < 1)
	    return random_color();
	    
	  auto color = cv::Scalar(0, 0, 0);
	  double d = 0;
	  for(unsigned int t = 0; t < nb_retry && d < min_dist; ++t) {
	    color = random_color();
	    d = 10; // big enough
	    for(auto& id_color : colors)
	      if(auto dd = distance(color, id_color.second); dd < d) {
		d = dd;
		if(d < min_dist)
		  break;
	      }
	  }

	  return color;
	}

      public:
	/**
	   This color map chooses random colors. Each color is
	   supposed do be very different from the others. To do so,
	   when a new random color is needed, we try at most
	   nb_retry times a random color, until the distance with
	   this color and an existing one is grather that
	   min_dist. The distance is the 3D euclidian distance, with
	   RGB values in [0,1]^3.
	*/
	random(RANDOM_DEVICE& rd, double min_dist=.01, unsigned int nb_retry=10)
	  : rd(rd), min_dist(min_dist*255.0), nb_retry(nb_retry), colors() {}
	  
	random()                         = delete;
	random(const random&)            = default;
	random(random&&)                 = default;
	random& operator=(const random&) = delete;
	random& operator=(random&&)      = delete;

	cv::Scalar operator()(unsigned int lbl) {
	  auto it = colors.find(lbl);
	  if(it != colors.end())
	    return it->second;
	  auto new_color = random_distinct_color();
	  colors[lbl] = new_color;
	  return new_color;
	}
	  
      };
    }

    class Frame {
    private:

      demo2d::Point cv_center;
      demo2d::Point cv_Ox;
      demo2d::Point cv_Oy;

      demo2d::Point to_cv_xline;
      demo2d::Point to_cv_yline;
	
      demo2d::Point to_demo_xline;
      demo2d::Point to_demo_yline;
	
    public:

      Frame()                        = default;
      Frame(const Frame&)            = default;
      Frame& operator=(const Frame&) = default;
      /**
       * @param cv_center The position of the frame origin (in pixel) in the CV image.
       * @param cv_Ox The abscissa unity vector of the frame (in pixel size).
       * @param cv_Oy The ordinate unity vector of the frame (in pixel size).
       */
      Frame(demo2d::Point cv_center,
	    demo2d::Point cv_Ox,
	    demo2d::Point cv_Oy)
	: cv_center(cv_center),
	  cv_Ox(cv_Ox),
	  cv_Oy(cv_Oy),
	  to_cv_xline(cv_Ox.x, cv_Oy.x),
	  to_cv_yline(cv_Ox.y, cv_Oy.y),
	  to_demo_xline( cv_Oy.y, -cv_Oy.x),
	  to_demo_yline(-cv_Ox.y,  cv_Ox.x) {
	double coef = 1/(cv_Ox.x * cv_Oy.y - cv_Ox.y * cv_Oy.x);
	to_demo_xline *= coef;
	to_demo_yline *= coef;
      }

      cv::Point operator()(const demo2d::Point& p) const {
	return cv::Point(p*to_cv_xline + cv_center.x,
			 p*to_cv_yline + cv_center.y);
      }

	
      demo2d::Point operator()(const cv::Point& p) const {
	demo2d::Point x = {double(p.x), double(p.y)};
	x -= cv_center;
	return {x*to_demo_xline, x*to_demo_yline};
      }

      /**
       * This converts a distance into an openCV length in
       * pixels. This may return silly values if the frame is not
       * orthogonal and normalized.
       */
      int operator()(double dist) const {
	auto o = (*this)(demo2d::Point(0, 0));
	auto m = (*this)(demo2d::Point(dist, 0));
	int dx = o.x - m.x;
	int dy = o.y - m.y;
	return int(std::sqrt(dx*dx + dy*dy) + .5);
      }
      
    };

    /**
     * @param cv_center The position of the frame origin (in pixel) in the CV image.
     * @param cv_Ox The abscissa unity vector of the frame (in pixel size).
     * @param cv_Oy The ordinate unity vector of the frame (in pixel size).
     */
    inline Frame frame(demo2d::Point cv_center,
		       demo2d::Point cv_Ox,
		       demo2d::Point cv_Oy) {
      return Frame(cv_center, cv_Ox, cv_Oy);
    }

    /**
     * @param x_unit_size The size of the horizontal unit vector in pixels.
     * @param y_unit_size The size of the vertical unit vector in pixels.
     * @param origin The center of the frame in the CV image.
     */
    inline Frame direct_orthogonal_frame(double x_unit_size,
					 double y_unit_size,
					 demo2d::Point origin) {
      demo2d::Point Ox(x_unit_size, 0);
      demo2d::Point Oy(0, -y_unit_size);
      return Frame(origin, Ox, Oy);
    }
      
    /**
     * @param size The image size in pixels.
     * @param unit_size The size of the vertical and horizontal unit vectors in pixels.
     * @param centered true puts the origin at the center, false puts it at the bottom left corner.
     */
    inline Frame direct_orthonormal_frame(const cv::Size& size,
					  double unit_size,
					  bool centered) {
      demo2d::Point Ox(unit_size, 0);
      demo2d::Point Oy(0, -unit_size);
      if(centered)
	return Frame(demo2d::Point(size.width*.5, size.height*.5), Ox, Oy);
      else
	return Frame(demo2d::Point(0, size.height-1), Ox, Oy);
      
    }

    /**
     * @param size The image size in pixels.
     * @param bbox The area that has to be displayed in the image.
     * @param pixel_margin The margin left between the area and the image border.
     */
    inline Frame direct_orthonormal_frame(const cv::Size& size,
					  const demo2d::sample::BBox& bbox,
					  unsigned int pixel_margin) {
      if((unsigned int)(size.width) < 2*pixel_margin)
	throw std::runtime_error("demo2d::opencv::direct_orthonormal_frame : img_width  < 2*pixel_margin");
      if((unsigned int)(size.height) < 2*pixel_margin)
	throw std::runtime_error("demo2d::opencv::direct_orthonormal_frame : img_height < 2*pixel_margin");

      unsigned int pix_width = size.width - 2 * pixel_margin;
      unsigned int pix_height = size.height - 2 * pixel_margin;
      auto bbox_size = bbox.size();

      double bbox_pix_width = 0;

      if(bbox_size.y/bbox_size.x < size.height/size.width) {
	bbox_pix_width = pix_width;
      }
      else {
	bbox_pix_width = pix_height * bbox_size.x / bbox_size.y;
      }

      demo2d::Point Ox = {bbox_pix_width / bbox_size.x, 0};
      demo2d::Point Oy = {0, -Ox.x};

      auto bbox_center  = .5*(bbox.bottom_left() + bbox.top_right());
      auto image_center = demo2d::Point(size.width, size.height)*.5;
      demo2d::Point O = image_center + ((bbox_center * Ox.x) & demo2d::Point(-1, 1));
      return Frame(O, Ox, Oy);
    }
      
    /**
     * This draws a bounding box.
     */
    inline void draw(cv::Mat& image, const Frame& frame, const demo2d::sample::BBox& bbox, const cv::Scalar& color, int thickness) {
      cv::rectangle(image, frame(bbox.bottom_left()), frame(bbox.top_right()), color, thickness);
    }


    template<typename OBJECT>
    class DotDrawer {

    private:
	
      cv::Mat image; // a share pointer.
      Frame frame;
      std::function<bool (const OBJECT&)>          do_draw;
      std::function<demo2d::Point (const OBJECT&)> point_of;
      std::function<int (const OBJECT&)>           radius_of;
      std::function<cv::Scalar (const OBJECT&)>    color_of;
      std::function<int (const OBJECT&)>           thickness_of;
	
	
    public:

      using difference_type   = long;
      using value_type        = OBJECT;
      using pointer           = OBJECT*;
      using reference         = OBJECT&;
      using iterator_category = std::output_iterator_tag;
	
      template<typename DO_DRAW, typename POINT_OF, typename RADIUS_OF, typename COLOR_OF, typename THICKNESS_OF>
      DotDrawer(cv::Mat& image,
		Frame frame,
		const DO_DRAW&      do_draw,
		const POINT_OF&     point_of,
		const RADIUS_OF&    radius_of,
		const COLOR_OF&     color_of,
		const THICKNESS_OF& thickness_of)
	: image(image),
	  frame(frame),
	  do_draw(do_draw),
	  point_of(point_of),
	  radius_of(radius_of),
	  color_of(color_of),
	  thickness_of(thickness_of) {}

      DotDrawer()                            = delete;
      DotDrawer(const DotDrawer&)            = default;
      DotDrawer& operator=(const DotDrawer&) = default; 

      DotDrawer& operator++()    {return *this;}
      DotDrawer& operator++(int) {return *this;}
      DotDrawer& operator*()     {return *this;}
      DotDrawer& operator=(const OBJECT& o) {
	if(do_draw(o))
	  cv::circle(image, frame(point_of(o)), radius_of(o), color_of(o), thickness_of(o));
	return *this;
      }
    };

    template<typename OBJECT, typename DO_DRAW, typename POINT_OF, typename RADIUS_OF, typename COLOR_OF, typename THICKNESS_OF>
    DotDrawer<OBJECT> dot_drawer(cv::Mat& image,
				 Frame frame,
				 const DO_DRAW&      do_draw,
				 const POINT_OF&     point_of,
				 const RADIUS_OF&    radius_of,
				 const COLOR_OF&     color_of,
				 const THICKNESS_OF& thickness_of) {
      return DotDrawer<OBJECT>(image, frame, do_draw, point_of, radius_of, color_of, thickness_of);
    }
      
    template<typename OBJECT>
    class SegmentDrawer {

    private:
	
      cv::Mat image; // a share pointer.
      Frame frame;
      std::function<bool (const OBJECT&)>                                    do_draw;
      std::function<std::pair<demo2d::Point, demo2d::Point> (const OBJECT&)> points_of;
      std::function<cv::Scalar (const OBJECT&)>                              color_of;
      std::function<int (const OBJECT&)>                                     thickness_of;
	
	
    public:
	
      using difference_type   = OBJECT;
      using value_type        = OBJECT;
      using pointer           = OBJECT*;
      using reference         = OBJECT&;
      using iterator_category = std::output_iterator_tag;

      template<typename DO_DRAW, typename POINTS_OF, typename COLOR_OF, typename THICKNESS_OF>
      SegmentDrawer(cv::Mat& image,
		    Frame frame,
		    const DO_DRAW&      do_draw,
		    const POINTS_OF&    points_of,
		    const COLOR_OF&     color_of,
		    const THICKNESS_OF& thickness_of)
	: image(image),
	  frame(frame),
	  do_draw(do_draw),
	  points_of(points_of),
	  color_of(color_of),
	  thickness_of(thickness_of) {}

      SegmentDrawer()                                = delete;
      SegmentDrawer(const SegmentDrawer&)            = default;
      SegmentDrawer& operator=(const SegmentDrawer&) = default; 

      SegmentDrawer& operator++()    {return *this;}
      SegmentDrawer& operator++(int) {return *this;}
      SegmentDrawer& operator*()     {return *this;}
      SegmentDrawer& operator=(const OBJECT& o) {
	if(do_draw(o)) {
	  auto pts = points_of(o);
	  cv::line(image, frame(pts.first), frame(pts.second), color_of(o), thickness_of(o));
	}
	return *this;
      }
    };

    template<typename OBJECT, typename DO_DRAW, typename POINTS_OF, typename COLOR_OF, typename THICKNESS_OF>
    SegmentDrawer<OBJECT> segment_drawer(cv::Mat& image,
					 Frame frame,
					 const DO_DRAW&      do_draw,
					 const POINTS_OF&    points_of,
					 const COLOR_OF&     color_of,
					 const THICKNESS_OF& thickness_of) {
      return SegmentDrawer<OBJECT>(image, frame, do_draw, points_of, color_of, thickness_of);
    }


    /**
     * Draws the line such as ax + by + c = 0 inside a bounding box.
     */
    inline void line(cv::Mat& image, Frame frame, const cv::Scalar& color, unsigned int thickness, const demo2d::sample::BBox& bbox, double a, double b, double c) {
      auto O  = bbox.bottom_left();
      auto W  = bbox.bottom_right();
      auto H  = bbox.top_left();
      auto WH = bbox.top_right();
      demo2d::Point ab {a, b};

      std::array<demo2d::Point, 2> pts;
      auto out = pts.begin();

      double s; // s for sign
      
      int sO = 0;
      int sW = 0;
      int sWH = 0;
      int sH = 0;
      
      s = O*ab + c;
      if(s < 0)       sO = -1;
      else if (s > 0) sO =  1;

      s = W*ab + c;
      if(s < 0)       sW = -1;
      else if (s > 0) sW =  1;
      
      s = WH*ab + c;
      if(s < 0)       sWH = -1;
      else if (s > 0) sWH =  1;
      
      s = H*ab + c;
      if(s < 0)       sH = -1;
      else if (s > 0) sH =  1;
      

      if(sO != sW)
	*(out++) = {(-c - O.y*b)/a, O.y};
      
      if(sW != sWH)
	*(out++) = {W.x, (-c - a*W.x)/b};

      if(out != pts.end() && sWH != sH) 
	*(out++) = {(-c - H.y*b)/a, H.y};

      if(out != pts.end() && sH != sO) 
	*(out++) = {O.x, (-c - a*O.x)/b};
      
      if(out != pts.end())
	return;

      auto U = frame(pts[0]);
      auto V = frame(pts[1]);
      cv::line(image, U, V, color, thickness);
    }


    inline void line(cv::Mat& image, Frame frame, const cv::Scalar& color, unsigned int thickness, double a, double b, double c) {
      demo2d::sample::BBox bbox {frame(cv::Point(0, 0)), frame(cv::Point(image.size().width, image.size().height))};
      line(image, frame, color, thickness, bbox, a, b, c);
    }

    namespace sample {

      /**
       * This handles image-based distributions information. This is
       * passed to an actual image based distribution for storing all the
       * required information.
       */
      struct ImageData {
	cv::Mat image; //!< a share pointer to the handled image.
	Frame frame;   //!< The frame for converting pixel positions into math coordinates.
	std::function<double (const unsigned char*)> pixel_to_density; //!< This converts pixel values (RGB) into density (in [0,1]).
	ImageData()                            = default;
	ImageData(const ImageData&)            = default;
	ImageData& operator=(const ImageData&) = default;
	ImageData(ImageData&&)                 = default;
	ImageData& operator=(ImageData&&)      = default;
	  
	template<typename PIXEL_TO_DENSITY>
	ImageData(const PIXEL_TO_DENSITY& pixel_to_density) : image(), frame(), pixel_to_density(pixel_to_density) {}
      };
	
      /**
       * Makes a data that handles image-based distributions
       * information. This is passed to an actual image distribution
       * for storing all the required information.
       */
      template<typename PIXEL_TO_DENSITY>
      ImageData image_data(const PIXEL_TO_DENSITY& pixel_to_density) {
	return ImageData(pixel_to_density);
      }
	
      /**
       * This handles video-based distributions information. This is
       * passed to an actual video-based distribution for storing all the
       * required information.
       */

      struct VideoData : ImageData {
	cv::VideoCapture cap;
	bool flipped = false;
	  
	VideoData()                            = default;
	VideoData(const VideoData&)            = default;
	VideoData& operator=(const VideoData&) = default;
	VideoData(VideoData&&)                 = default;
	VideoData& operator=(VideoData&&)      = default;
	  
	template<typename PIXEL_TO_DENSITY>
	VideoData(int device, const PIXEL_TO_DENSITY& pixel_to_density, bool flipped) : ImageData(pixel_to_density), cap(device), flipped(flipped) {
	  if(!cap.isOpened()) {
	    std::ostringstream ostr;
	    ostr << "Cannot open video from device " << device << " (/dev/video" << device << ").";
	    throw std::runtime_error(ostr.str());
	  }
	  image = cv::Mat((int)(cap.get(cv::CAP_PROP_FRAME_HEIGHT)),
			  (int)(cap.get(cv::CAP_PROP_FRAME_WIDTH)),
			  CV_8UC3, cv::Scalar(255,255,255));
	}

	/**
	 * This grabs the next frame.
	 */
	void operator++() {
	  cap >> image;
	  if(flipped) 
	    cv::flip(image, image, 1);
	  
		     
	}
      };
	
      /**
       * Makes a data that handles video-based distributions
       * information. This is passed to an actual video-based
       * distribution for storing all the required information.
       */
      template<typename PIXEL_TO_DENSITY>
      VideoData video_data(int device, const PIXEL_TO_DENSITY& pixel_to_density, bool flipped=false) {
	return VideoData(device, pixel_to_density, flipped);
      }
	

      class Webcam;

      /**
       * This is a density related to an image (see
       * demo2d::opencv::sample::image).
       */
      class Image : public demo2d::sample::Density {
      private:

	ImageData& image_data;
	  
	Image()                        = delete;
	Image(const Image&)            = delete;
	Image& operator=(const Image&) = delete;
	Image(ImageData& image_data) : image_data(image_data) {}

	friend demo2d::sample::density image(ImageData& image_data);
	friend class Webcam;
	  
      public:
	  
	virtual demo2d::sample::BBox bbox() const override {
	  return demo2d::sample::BBox(image_data.frame(cv::Point(0, image_data.image.size().height - 1)),
				      image_data.frame(cv::Point(image_data.image.size().width - 1, 0)));
	}
	  
	virtual double operator()(const Point& p) const override {
	  if(bbox().contains(p)) {
	    auto cv_pt = image_data.frame(p);
	    return image_data.pixel_to_density(reinterpret_cast<const unsigned char*>(&(image_data.image.at<unsigned char[3]>(cv_pt.y, cv_pt.x))));
	  }
	  else
	    return 0;
	}
	  
      };

      /**
       * This builds an image-based distribution.
       * @param image_data The image data for storing image-related information.
       */
      inline demo2d::sample::density image(ImageData& image_data) {
	return demo2d::sample::density(new Image(image_data));
      }


      /**
       * This is a density related to images taken from a webcam (see
       * demo2d::opencv::sample::webcam).
       */

      class Webcam : public demo2d::sample::Density {
	  
      private:
	  
	VideoData& video_data;
	Image image;
	  
	Webcam()                         = delete;
	Webcam(const Webcam&)            = delete;
	Webcam& operator=(const Webcam&) = delete;

	Webcam(VideoData& video_data) : demo2d::sample::Density(), video_data(video_data), image(video_data) {}

	friend demo2d::sample::density webcam(VideoData& video_data);
	
      public:
	  
	virtual demo2d::sample::BBox bbox() const override {
	  return image.bbox();
	}
	virtual double operator()(const Point& p) const override {
	  return image(p);
	}

      };
	
      /**
       * This builds webcam-based distribution.
       * @param video_data The video data for storing webcam-related information.
       */
      inline demo2d::sample::density webcam(VideoData& video_data) {
	return demo2d::sample::density(new Webcam(video_data));
      }
    }

    struct gui {
    private:

      std::map<std::string, // window name
	       std::map<int, // button
			std::vector<std::function<void (const demo2d::Point&)>> // callbacks
			>> mouse_callbacks;
      std::list<std::function<void (double)>> slider_callbacks; // List mandatory here, we need pushback to preserve element addresses.
      std::map<int, std::vector<std::function<void ()>>> keyboard_callbacks;
      
      static void on_mouse(int event, int x, int y, int, void* user_data) {
	auto win_that_ptr = reinterpret_cast<std::pair<std::string, gui*>*>(user_data);
	auto that = win_that_ptr->second;
	auto& window_name = win_that_ptr->first;
	auto& frame = that->frames[window_name];
	that->on_click(event, frame(cv::Point(x, y)), window_name);
      }
	
      static void on_trackbar(int slider, void* user_data) {
	(*(reinterpret_cast<std::function<void (double)>*>(user_data)))(slider*.001);
      }

      void on_click(int event, const demo2d::Point& pos, const std::string& win_name) {
	auto& cbs = mouse_callbacks[win_name];
	if(auto it = cbs.find(event); it != cbs.end())
	  for(auto& cb : it->second) cb(pos);
      }
	
	
      std::string window_name;
      std::map<std::string, Frame> frames;
      Frame the_frame;

      std::list<std::pair<std::string, gui*>> cb_userdata; // Do not use vector here !
	
    public:
	
      int loop_ms = 0;
      mutable int keycode = 0;
	
      gui() = delete;
      gui(const gui&) = delete;
      gui& operator=(const gui&) = delete;

      gui(const std::string& window_name,
	  const Frame& frame)
	: mouse_callbacks(), slider_callbacks(), keyboard_callbacks(), window_name(window_name), frames(), the_frame(frame), cb_userdata() {

	(*this)[window_name];
	frames.emplace(window_name, frame);
      }

      gui& operator=(const Frame& frame) {
	frames[window_name] = frame;
	this->the_frame = frame;
	return *this;
      }

      const Frame& frame() const {return the_frame;}
      
      gui& operator[](const std::string& window_name) {
	this->window_name = window_name;
	if(frames.find(window_name) == frames.end()) {
	  cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
	  auto& user_data = cb_userdata.emplace_back(window_name, this);
	  frames.emplace(window_name, the_frame);
	  cv::setMouseCallback(window_name, on_mouse, reinterpret_cast<void*>(&user_data));
	}
	the_frame = frames[window_name];
	return *this;
      }

      operator bool() const {
	keycode = cv::waitKey(loop_ms) & 0xFF;
	if(auto it = keyboard_callbacks.find(keycode); it != keyboard_callbacks.end())
	  for(auto& cb : it->second)
	    cb();
	return keycode != 27; // ESC stops.
      }

      void operator<<(cv::Mat image) {
	cv::imshow(window_name, image);
      }
	
      void operator+=(const std::tuple<int, std::function<void (const demo2d::Point&)>>& click_info) {
	mouse_callbacks[window_name][std::get<0>(click_info)].emplace_back(std::get<1>(click_info));
      }
	
      void operator+=(const std::tuple<int, std::function<void ()>>& keyboard_info) {
	keyboard_callbacks[std::get<0>(keyboard_info)].emplace_back(std::get<1>(keyboard_info));
      }
	
      void operator+=(const std::tuple<std::string, std::function<void (double)>>& trackbar_info) {
	cv::createTrackbar(std::get<0>(trackbar_info),
			   window_name,
			   nullptr, 1000, on_trackbar,
			   (void*)&slider_callbacks.emplace_back(std::get<1>(trackbar_info)));
      }
    };
  }
}
