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

#include <random>
#include <optional>
#include <utility>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <limits>

namespace demo2d {

  class Point {
  public:
    double x,y;
    
    Point() : x(0), y(0) {}
    Point(double xx, double yy) : x(xx), y(yy) {}
    Point(const Point&)            = default;
    Point& operator=(const Point&) = default;

    Point& operator=(double val) {
      x = val;
      y = val;
      return *this;
    }
  
    bool operator==(const Point& p) const {
      return x == p.x && y == p.y;
    }
  
    bool operator!=(const Point& p) const {
      return x != p.x || y != p.y;
    }

    Point operator+(const Point& p) const {
      return {x+p.x, y+p.y};
    }
  
    Point operator-() const {
      return {-x,-y};
    }

    Point operator+() const {
      return {x,y};
    }

    Point operator-(const Point& p) const {
      return {x-p.x, y-p.y};
    }

    double angle() const {
      return std::atan2(y, x);
    }

    /**
     * The dot product
     */
    double operator*(const Point& p) const {
      return x*p.x + y*p.y;
    }

    /**
     * Component-wise product.
     */
    Point operator&(const Point& p) const {
      return {x*p.x, y*p.y};
    }
      
    Point operator/(const Point& p) const {
      return {x/p.x, y/p.y};
    }

    /**
     * vector cross product
     */
    double operator^(const Point& p) const {
      return x*p.y - y*p.x;
    }

    /**
     * Unitary vector.
     */
    Point operator*() const {
      const Point& me = *this;
      return me/std::sqrt(me*me);
    }

    Point operator*(double a) const {
      return {x*a, y*a};
    }

    Point operator/(double a) const {
      return (*this)*(1/a);
    }

    Point& operator+=(const Point& p) {
      x += p.x;
      y += p.y;
      return *this;
    }

    Point& operator-=(const Point& p) {
      x -= p.x;
      y -= p.y;
      return *this;
    }
  
    Point& operator*=(double a) {
      x *= a;
      y *= a;
      return *this;
    }
  
    Point& operator/=(double a) {
      (*this)*=(1/a);
      return *this;
    }

    Point transpose() const {
      return {y, x};
    }

    Point rotate_left() const {
      return {-y, x};
    }

    Point rotate_right() const {
      return {y, -x};
    }

    double norm2() const {
      return x*x + y*y;
    }
    
    double norm() const {
      return std::sqrt(norm2());
    }

    static Point unitary(double angle) {
      return {std::cos(angle), std::sin(angle)};
    }
  };

  inline Point operator*(double a, const Point p) {
    return p*a;
  }

  inline std::ostream& operator<<(std::ostream& os, 
				  const Point& p) {
    os << '(' << p.x << ", " << p.y << ')';
    return os;
  }

  inline std::ostream& operator<<(std::ostream& os, 
				  const std::pair<Point, Point>& p) {
    os << '[' << p.first << ", " << p.second << ']';
    return os;
  }

  inline std::istream& operator>>(std::istream& is, 
				  Point& p) {
    char c;
    is >> c >> p.x >> c >> p.y >> c;
    return is;
  }

  template<typename RANDOM_ENGINE>
  Point uniform(RANDOM_ENGINE& rd, const Point& A, const Point& B) {
    std::uniform_real_distribution<double> uniform_dist(0., 1.);
    Point d = {uniform_dist(rd), uniform_dist(rd)};
    return A + (d & (B-A));
  }

  /**
   * Adds a value in [-r, +r[ to each components of A.
   */
  template<typename RANDOM_ENGINE>
  Point alter(RANDOM_ENGINE& rd, const Point& A, double r) {
    return A + uniform(rd, {-r, -r}, {r, r});
  }

    
  
  inline double d2(const Point& A, const Point& B) {
    Point tmp = B-A;
    return tmp*tmp;
  }

  inline double d(const Point& A, const Point& B) {
    return std::sqrt(d2(A,B));
  }
  
  inline double d2_point(const Point& A, const Point& B) {
    return d2(A, B);
  }
  
  inline double d_point(const Point& A, const Point& B) {
    return d(A, B);
  }


  inline Point min(const Point& A, const Point& B) {
    return {std::min(A.x,B.x),std::min(A.y,B.y)};
  }

  inline Point max(const Point& A, const Point& B) {
    return {std::max(A.x,B.x),std::max(A.y,B.y)};
  }


  /**
   * This returns (if it exists) the single point where the segments do intersect. If the segments overlap (i.e they share a common segment), no intersection is returned. If on segment is a point, no intersection is returned.
   */
  inline std::optional<Point> operator&&(const std::pair<Point, Point>& seg1, const std::pair<Point, Point>& seg2) {
    auto& P = seg1.first;
    auto  R = seg1.second - P;
    auto& Q = seg2.first;
    auto  S = seg2.second - Q;

    std::optional<Point> res;


    if(R == Point() || S == Point()) 
      return res;
	    
    double rs  = R^S;
    double qpr = (Q-P)^R;
	    
    if(rs != 0) {

      double t = ((Q-P)^S)/rs;
      double u = qpr/rs;
      if(0 <= t && t <= 1 && 0 <= u && u <= 1)
	res = P + t*R;
    }
      
    return res;
  }
    
  inline double d2(const Point& M, const std::pair<Point, Point>& S) {
    auto& [A, B] = S;
    // std::cout << "#### M = " << M << ", A = " << A << ", B = " << B << std::endl
    // 	      << "     (d displayed, but d2 is really computed)" << std::endl;
    if(A == B) {
      // std::cout << "---> A == B, return " << d(M, A) << std::endl;
      return d2(M, A);
    }
    auto AB_ = B - A;
    auto u   = *AB_;
    auto AH = (M - A) * u;
    // std::cout << "     AB_ = " << AB_ << ", u = " << u << ", AH = " << AH << std::endl;
    if(AH < 0) {
      // std::cout << "---> AH < 0, return " << d(M, A) << std::endl;
      return d2(M, A);
    }
    auto AH_ = u * AH;
    auto H   = A + AH_;
    auto BH_ = H - B;
    // std::cout << "     AH_ = " << AH_ << ", H = " << H << ", BH_ = " << BH_ << ", BH_ * u = " << BH_ * u << std::endl;
    if(BH_ * u > 0) {
      // std::cout << "---> BH_ * u > 0, return " << d(M, B) << std::endl;
      return d2(M, B);
    }
    // std::cout << "---> BH_ * u <= 0, return " << d(M, H) << std::endl;
    return d2(M, H);
  }
  
  inline double d(const Point& M, const std::pair<Point, Point>& S) {
    return std::sqrt(d2(M, S));
  }

  inline double d2_point_segment(const Point& M, const std::pair<Point, Point>& S) {
    return d2(M, S);
  }

  inline double d_point_segment(const Point& M, const std::pair<Point, Point>& S) {
    return d(M, S);
  }
    
  

  /**
   * This returns the distance between M and the segment S, but only
   * if M lies in a rectangle of which S is a median. It returns
   * infinity otherwise.
   */
  inline double cylinder_d2(const Point& M, const std::pair<Point, Point>& S) {
    auto& [A, B] = S;
    if(A == B) 
      return d2(M, A);
    auto AB_ = B - A;
    auto u   = *AB_;
    auto AH = (M - A) * u;
    if(AH < 0) 
      return std::numeric_limits<double>::max();
    auto AH_ = u * AH;
    auto H   = A + AH_;
    auto BH_ = H - B;
    if(BH_ * u > 0) 
      return std::numeric_limits<double>::max();
    return d2(M, H);
  }

  inline double cylinder_d(const Point& M, const std::pair<Point, Point>& S) {
    return std::sqrt(cylinder_d2(M, S));
  }
}
