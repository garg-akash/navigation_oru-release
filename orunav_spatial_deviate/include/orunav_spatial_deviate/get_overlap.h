#include <iostream>

#include <deque>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/geometries.hpp>

#include <boost/foreach.hpp>

using namespace std;
namespace bg = boost::geometry;

double cal_overlap(double** r1, double** r2) {
  double arr = 0;
  typedef boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<double> > polygon;
  
  typedef bg::model::d2::point_xy<double> point_type;
  bg::model::polygon<point_type> polygon1, polygon2;
  //To make the points in clockwise direction start from 3 to 0
  bg::append(bg::exterior_ring(polygon1), bg::make<point_type>(r1[3-0][0], r1[3-0][1]));
  bg::append(bg::exterior_ring(polygon1), bg::make<point_type>(r1[3-1][0], r1[3-1][1]));
  bg::append(bg::exterior_ring(polygon1), bg::make<point_type>(r1[3-2][0], r1[3-2][1]));
  bg::append(bg::exterior_ring(polygon1), bg::make<point_type>(r1[3-3][0], r1[3-3][1]));
  bg::append(bg::exterior_ring(polygon1), bg::make<point_type>(r1[3-0][0], r1[3-0][1]));

  bg::append(bg::exterior_ring(polygon2), bg::make<point_type>(r2[3-0][0], r2[3-0][1]));
  bg::append(bg::exterior_ring(polygon2), bg::make<point_type>(r2[3-1][0], r2[3-1][1]));
  bg::append(bg::exterior_ring(polygon2), bg::make<point_type>(r2[3-2][0], r2[3-2][1]));
  bg::append(bg::exterior_ring(polygon2), bg::make<point_type>(r2[3-3][0], r2[3-3][1]));
  bg::append(bg::exterior_ring(polygon2), bg::make<point_type>(r2[3-0][0], r2[3-0][1]));
  //cout << boost::geometry::wkt(polygon1) << std::endl;
  //cout << boost::geometry::wkt(polygon2) << std::endl;
  std::deque<polygon> output;
  boost::geometry::intersection(polygon1, polygon2, output);

  int i = 0;
  BOOST_FOREACH(polygon const& p, output)
  {
    //cout << i++ << ": " << boost::geometry::area(p) << std::endl;
    arr = boost::geometry::area(p);

  }
        
  return (arr);
}