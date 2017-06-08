#ifndef ELLIPSE
#define ELLIPSE

#include "laser_line_extraction/line.h"
#include "ellipse/EllipseList.h"
#include "ellipse/EllipseSeg.h"
#include "laser_line_extraction/LineSegment.h"
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include "tf/transform_datatypes.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace ellipse_extraction
{

class Ellipse
{

private:
  line_extraction::Line _line;
  double _height;
  double _width;
  double _p1;
  double _p2;
  double _rho;
  double _alpha;
  std::string _type;
  Eigen::Matrix2f _R;

  boost::array<double, 2>  _start;
  boost::array<double, 2>  _end;
  boost::array<double, 2>  _p_point;

  //tf::TransformListener _listener;


public:
  Ellipse(const line_extraction::Line &line, const double height, const double width);
  Ellipse(const laser_line_extraction::LineSegment &lineSeg);
  Ellipse(double posx,double posy, std::string type);
  ~Ellipse();
  std::vector<double[]>     calc_points();
  static double sBuffer;

  double  getAngle();
  double  getWidth();
  double  getHeight();
  double  getP1();
  double  getP2();
  double  getAlpha();
  double  getRho();
  std::string getType();
  Eigen::Matrix2f getR();
  line_extraction::Line getLine();

  boost::array<double, 2>&  getStart();
  boost::array<double, 2>&  getEnd();
  boost::array<double, 2>&  getPPoint();
  void setPPoint(double x, double y);
  bool                      onLine(boost::array<double, 2>& point);


};

}
#endif
